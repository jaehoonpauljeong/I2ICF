#!/usr/bin/env python3
"""Concurrent perception, intent ingestion and AI planning with one shared World State."""

import base64
from collections import deque
import copy
import json
import logging
import math
import os
import threading
import time
from pathlib import Path
import requests
from flask import Flask, jsonify, request
from common.config import (
    ENV_CFG,
    NET_CFG,
    WORLD_PORT,
    BIND_HOST,
    MAX_SYNC_SKEW_SEC,
    LIDAR_CFG,
)
from common.runtime_safety import (
    fresh,
    valid_pose,
    scan_health,
    apply_tf,
    OccupancyMap,
    synchronized_stamps,
    finite,
)
from common.navigation_utils import validate_and_score_path
from edge.mcp_ros2_server import MCPToolRegistry
from edge.mcp_client import MCPROS2Client
from edge.vlm_planner_agent import OllamaMCPPlannerAgent

from common.intent_contract import normalize_user_intent
from common.goal_completion import goal_evidence

log = logging.getLogger(__name__)


def localize_detections(detections, camera, scan):
    """Project measured LiDAR points into the calibrated optical camera, then associate boxes."""
    import numpy as np

    if not fresh(camera) or scan_health(scan):
        return []
    if not synchronized_stamps(
        camera.get("stamp_sec"), scan.get("stamp_sec"), MAX_SYNC_SKEW_SEC
    ):
        return []
    extrinsic = camera.get("laser_to_camera", {})
    intrinsics = camera.get("intrinsics", {})
    if not extrinsic.get("valid") or not fresh(extrinsic) or not intrinsics:
        return []
    if extrinsic.get("source_frame") != scan.get("frame_id"):
        return []
    qx, qy, qz, qw = extrinsic["quaternion"]
    rotation = np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ]
    )
    translation = np.array(extrinsic["translation"])
    projected = []
    for point in scan.get("points", []):
        # ViLaR's 12m limit applies to object association, not all navigation rays.
        if (
            not LIDAR_CFG.object_min_range_m
            < point["distance_m"]
            < LIDAR_CFG.object_max_range_m
        ):
            continue
        pc = (
            rotation @ np.array([point["x_laser"], point["y_laser"], 0.0]) + translation
        )
        if pc[2] <= 0:
            continue
        u = intrinsics["fx"] * pc[0] / pc[2] + intrinsics["cx"]
        v = intrinsics["fy"] * pc[1] / pc[2] + intrinsics["cy"]
        projected.append((u, v, point))
    objects = []
    for i, det in enumerate(detections):
        x1, y1, x2, y2 = det["bbox"]
        pad = 0.15 * (x2 - x1)
        hits = [
            p for u, v, p in projected if x1 + pad <= u <= x2 - pad and y1 <= v <= y2
        ]
        hits.sort(key=lambda p: p["distance_m"])
        selected = hits[: LIDAR_CFG.short_group_k]
        pt = None
        if selected:
            selected.sort(key=lambda p: p["distance_m"])
            pt = selected[len(selected) // 2]
        position = (
            apply_tf(pt["x_laser"], pt["y_laser"], scan["laser_tf_in_planning_frame"])
            if pt
            else None
        )
        objects.append(
            {
                "id": det.get("id", f"{det['class']}_{i}"),
                "class": det["class"],
                "confidence": det["confidence"],
                "bbox": det["bbox"],
                "observed_at": min(camera["timestamp"], scan["timestamp"]),
                "distance_m": pt["distance_m"] if pt else None,
                "position": position or {"valid": False},
                "angle_deg": pt["angle_deg"] if pt else None,
            }
        )
    return objects


def localization_error(camera, scan):
    error = scan_health(scan)
    if error:
        return error
    if not camera.get("intrinsics"):
        return "CAMERA_CALIBRATION_UNAVAILABLE"
    tf = camera.get("laser_to_camera", {})
    if not tf.get("valid") or not fresh(tf):
        return "CAMERA_TF_UNAVAILABLE"
    if tf.get("source_frame") != scan.get("frame_id"):
        return "CAMERA_LIDAR_FRAME_MISMATCH"
    if not synchronized_stamps(
        camera.get("stamp_sec"), scan.get("stamp_sec"), MAX_SYNC_SKEW_SEC
    ):
        return "CAMERA_LIDAR_TIMESTAMP_MISMATCH"
    return None


class EdgeServerOrchestrator:
    def __init__(self, model_path=None, model=None, occupancy_map=None):
        self.lock = threading.RLock()
        self.scan_condition = threading.Condition(self.lock)
        self.scan_epoch = 0
        self.sync_wait_sec = max(
            0.0, min(0.5, float(os.getenv("FUSION_SCAN_WAIT_SEC", "0.25")))
        )
        self.recovery_samples = []
        self.stop_event = threading.Event()
        self.intent_ingest_lock = threading.RLock()
        self.terminal_versions = set()
        self.failed_cycles = 0
        self.pending_closures = {}
        self.last_log_error = None
        self.last_log_retry = 0.0
        self.max_failed_cycles = max(
            1, int(os.getenv("MAX_FAILED_PLANNING_CYCLES", "3"))
        )
        self.model_path = Path(
            model_path
            or os.getenv(
                "YOLO_MODEL",
                str(Path(__file__).resolve().parents[1] / "detector/yolov8s.pt"),
            )
        )
        self.model = model
        self.grid = occupancy_map if occupancy_map is not None else OccupancyMap()
        self.latest_odom = {}
        self.latest_lidar = {}
        self.scan_history = deque(maxlen=60)
        self.perception_scan = None
        self.latest_camera = {}
        self.latest_image_b64 = None
        self.detected_objects = []
        self.perception = {
            "ready": False,
            "error": "PERCEPTION_NOT_STARTED",
            "timestamp": 0.0,
        }
        self.nav_feedback = {}
        self.active_intent = None
        self.active_intent_version = 0
        self.last_planned_intent_version = None
        self.last_event = -1
        self.last_health = None
        self.last_target_object_pos = None
        self.last_decision = None
        self.last_obstacle_signature = None
        self.intent_complete = False
        self.ollama_retry_at = 0.0
        self.ollama_retry_version = None
        self.mcp_registry = MCPToolRegistry(
            self.build_llm_world_state,
            occupancy_map=self.grid,
            report_provider=self.get_report,
        )
        # Local registry retains route memory and independent emergency-stop support.
        # The VLM has only the remote SDK client; there is no local tool fallback.
        self.mcp_client = MCPROS2Client(
            self.build_llm_world_state, self.record_mcp_result
        )
        self.planner_agent = OllamaMCPPlannerAgent(self.mcp_client)

    def record_mcp_result(self, name, result, version):
        """Retain collision evidence across separate T6 sessions and restarts."""
        with self.lock:
            if version != self.active_intent_version:
                return
            world = self.build_llm_world_state()
            memory = self.mcp_registry.route_memory
            memory.scope(world)
            inputs = result.get("audit_details") or {}
            memory.record(
                result.get("validation"),
                world.get("robot_state", {}).get("frame_id"),
                ("mcp", result.get("mcp_transport", {}).get("correlation_id")),
                inputs.get("lidar", {}).get("stamp_sec"),
            )

    def build_llm_world_state(self):
        with self.lock:
            robot = copy.deepcopy(self.latest_odom)
            robot["valid"] = valid_pose(robot)
            robot["status"] = self.nav_feedback.get("status", "unknown")
            perception = copy.deepcopy(self.perception)
            image_fresh = fresh(perception)
            perception["image_ready"] = bool(
                perception.get("image_ready") and image_fresh
            )
            perception["ready"] = bool(perception.get("ready") and image_fresh)
            if "localization_ready" in perception:
                # Validate the actual fusion pair, not an unrelated newer scan.
                paired_scan = (
                    self.perception_scan
                    if self.perception_scan is not None
                    else self.latest_lidar
                )
                current_error = scan_health(self.latest_lidar) or localization_error(
                    self.latest_camera, paired_scan
                )
                perception["localization_ready"] = bool(
                    perception["localization_ready"]
                    and image_fresh
                    and not current_error
                )
                perception["ready"] = (
                    perception["ready"] and perception["localization_ready"]
                )
                if current_error:
                    perception["error"] = perception["localization_error"] = (
                        current_error
                    )
            objects = [
                copy.deepcopy(o)
                for o in self.detected_objects
                if fresh({"timestamp": o.get("observed_at")})
            ]
            if not perception["ready"] or scan_health(self.latest_lidar):
                for obj in objects:
                    obj.update(
                        distance_m=None,
                        angle_deg=None,
                        position={"valid": False},
                        localization_error=perception.get("localization_error")
                        or "LOCALIZATION_UNAVAILABLE",
                    )
            world = {
                "timestamp": time.time(),
                "intent": copy.deepcopy(self.active_intent or {}),
                "intent_version": self.active_intent_version,
                "robot_state": robot,
                "environment": {
                    "map_available": bool(
                        ENV_CFG.map_available and self.grid.available
                    ),
                    "navigation_mode": ENV_CFG.navigation_mode,
                    "map": self.grid.info(),
                    "objects": objects,
                },
                "perception": perception,
                "sensor_health": {"scan_error": scan_health(self.latest_lidar)},
                "navigation": copy.deepcopy(self.nav_feedback),
                "raw_lidar": copy.deepcopy(self.latest_lidar),
                "intent_complete": self.intent_complete,
                "scenario_terminal": self.active_intent_version
                in self.terminal_versions,
            }
            world["route_constraints"] = self.mcp_registry.route_memory.snapshot(world)
            return world

    def obstacle_signature(self, world):
        scan = world["raw_lidar"]
        front = scan.get("front_min_m")
        return (
            round(front * 2) / 2 if isinstance(front, (float, int)) else None,
            tuple(
                sorted(
                    (
                        o.get("class"),
                        round(o["position"]["x"] * 2) / 2,
                        round(o["position"]["y"] * 2) / 2,
                    )
                    for o in world["environment"]["objects"]
                    if o.get("position", {}).get("valid")
                )
            ),
        )

    def evaluate_replan_triggers(self, world):
        if not world["intent"] or world.get("scenario_terminal"):
            return None
        if world["intent_version"] != self.last_planned_intent_version:
            return "NEW_USER_INTENT"
        if self.intent_complete:
            return None
        health = (
            world["robot_state"].get("valid"),
            scan_health(world["raw_lidar"]),
            world["perception"].get("ready"),
        )
        if self.last_health is not None and health != self.last_health:
            return "OBSERVATION_HEALTH_CHANGED"
        nav = world["navigation"]
        if nav.get("event_sequence", 0) != self.last_event and nav.get("status") in (
            "completed",
            "blocked",
            "failed",
            "canceled",
        ):
            return "OPERATION_FEEDBACK"
        if (
            nav.get("status") in ("stopped", "blocked", "failed", "canceled")
            and self.last_obstacle_signature is not None
            and self.obstacle_signature(world) != self.last_obstacle_signature
        ):
            return "OBSTACLE_OBSERVATION_CHANGED"
        intent = world["intent"]
        if intent.get("action") in ("find", "approach"):
            for obj in world["environment"]["objects"]:
                pos = obj.get("position", {})
                if obj.get("class") == intent.get("target_object") and pos.get("valid"):
                    target = (pos["x"], pos["y"])
                    if (
                        self.last_target_object_pos is None
                        or math.dist(target, self.last_target_object_pos) > 0.45
                    ):
                        self.last_target_object_pos = target
                        return "TARGET_OBSERVED_OR_MOVED"
        if nav.get("status") == "moving" and nav.get("current_path"):
            check = validate_and_score_path(
                nav["current_path"][nav.get("current_wp_index", 0) :],
                nav.get("frame_id"),
                world["robot_state"],
                world["environment"]["objects"],
                world["raw_lidar"].get("points", []),
                lidar_data=world["raw_lidar"],
                occupancy_map=self.grid if ENV_CFG.map_available else None,
                route_constraints=world.get("route_constraints"),
            )
            if not check["valid"]:
                return "CURRENT_PATH_BLOCKED"
        return None

    def try_finish_pose_goal(self, world):
        """Finish a measured final pose before invoking the model again.

        Intermediate segments, find/approach and user stop keep their own semantics.
        Robot verifies its own fresh pose and measured standstill before fencing
        the intent. A missing acknowledgement never becomes a success report.
        """
        intent = world.get("intent", {})
        if intent.get("action") not in ("move", "detour", "resume"):
            return None
        evidence = goal_evidence(
            world.get("robot_state", {}), intent.get("target_location")
        )
        if not evidence or not evidence["within_tolerance"]:
            return None
        version = world["intent_version"]
        nav = world.get("navigation", {})
        if (
            nav.get("intent_version") != version
            or nav.get("active_intent_id") != intent.get("id")
            or nav.get("e_stop")
        ):
            return None
        with self.lock:
            if (
                version != self.active_intent_version
                or version in self.terminal_versions
                or self.stop_event.is_set()
            ):
                return None
            data = {
                "intent_id": intent["id"],
                "intent_version": version,
                "target_location": intent["target_location"],
            }
            try:
                response = requests.post(
                    f"{NET_CFG.control_base_url}/control/finish", json=data, timeout=0.5
                )
                result = response.json()
                confirmed = (
                    response.status_code == 200
                    and result.get("ok")
                    and result.get("intent_closed")
                    and (result.get("completion_evidence") or {}).get(
                        "stopped_verified"
                    )
                )
            except (requests.RequestException, ValueError) as exc:
                result = {
                    "ok": False,
                    "error": "COMPLETION_ACK_UNAVAILABLE",
                    "message": str(exc),
                }
                confirmed = False
            decision = {
                "intent_id": intent["id"],
                "decision": "COMPLETE" if confirmed else "OBSERVE",
                "status": "completed" if confirmed else "verifying_completion",
                "decision_source": "goal_verifier",
                "reason": (
                    "Measured final goal reached and Robot intent closed"
                    if confirmed
                    else "Waiting for Robot goal/stop verification"
                ),
                "completion_evidence": result.get("completion_evidence") or evidence,
                "controller_result": result,
            }
            self.last_decision = decision
            if not confirmed:
                # Wait for the next fresh poll; do not issue another near-zero path
                # or rotation while closure/physical stopping is being verified.
                return decision
            self.intent_complete = True
            self.failed_cycles = 0
            self.ollama_retry_at = 0.0
            self.ollama_retry_version = None
            self.nav_feedback = copy.deepcopy(result["navigation_status"])
            writer = self.planner_agent.audit_writer
            sid = writer.scenario(intent, version, activate=True)["scenario_id"]
            writer.scenario_event(sid, "navigation_event", self.nav_feedback)
            writer.scenario_event(sid, "decision", decision)
            self.finish_scenario(
                "succeeded",
                decision["reason"],
                expected_version=version,
                completion_evidence=result["completion_evidence"],
            )
            return decision

    def step_planning_cycle(self):
        world = self.build_llm_world_state()
        version = world["intent_version"]
        if world.get("scenario_terminal") or self.stop_event.is_set():
            return None
        completion = self.try_finish_pose_goal(world)
        if completion is not None:
            return completion
        retrying = self.ollama_retry_version == version and self.ollama_retry_at > 0
        explicit_stop = world["intent"].get("action") == "stop"
        observation_retry = (
            retrying
            and (self.last_decision or {}).get("error", {}).get("code")
            == "OBSERVATION_UNAVAILABLE"
        )
        recovered = False
        if observation_retry:
            perception = world["perception"]
            healthy = (
                valid_pose(world["robot_state"])
                and scan_health(world["raw_lidar"]) is None
                and perception.get("ready")
                and fresh(perception)
            )
            sample = (
                perception.get("camera_stamp_sec", perception.get("timestamp")),
                world["raw_lidar"].get("stamp_sec"),
            )
            if not healthy:
                self.recovery_samples = []
            elif all(v is not None for v in sample) and (
                not self.recovery_samples
                or all(a != b for a, b in zip(sample, self.recovery_samples[-1]))
            ):
                self.recovery_samples = (self.recovery_samples + [sample])[-2:]
            recovered = len(self.recovery_samples) >= 2
        if (
            retrying
            and not explicit_stop
            and time.monotonic() < self.ollama_retry_at
            and not recovered
        ):
            return None
        trigger = (
            "OBSERVATION_RECOVERED"
            if recovered
            else self.evaluate_replan_triggers(world)
        )
        if not trigger and retrying:
            trigger = "OLLAMA_RETRY"
        if not trigger:
            return None
        try:
            with self.lock:
                if self.active_intent_version == version:
                    self.planner_agent.audit_writer.scenario(
                        world["intent"], version, activate=True
                    )
        except Exception as exc:
            log.error("Scenario audit failed: %s", exc)
        self.last_planned_intent_version = version
        self.last_health = (
            world["robot_state"].get("valid"),
            scan_health(world["raw_lidar"]),
            world["perception"].get("ready"),
        )
        self.last_event = world["navigation"].get("event_sequence", 0)
        self.last_obstacle_signature = self.obstacle_signature(world)
        prompt = {k: v for k, v in world.items() if k != "raw_lidar"}
        decision = self.planner_agent.plan_and_act(
            prompt,
            self.latest_image_b64 if world["perception"]["ready"] else None,
            trigger,
        )
        if decision.get("decision") == "COMPLETE" and world["intent"].get("action") in (
            "move",
            "detour",
            "resume",
        ):
            completion = self.try_finish_pose_goal(self.build_llm_world_state())
            if completion is not None:
                return completion
            decision = {
                **decision,
                "decision": "FAIL",
                "status": "failed",
                "error": {"code": "GOAL_NOT_VERIFIED"},
            }
        with self.lock:
            if self.active_intent_version != version:
                return decision
            self.last_decision = decision
            self.recovery_samples = []
            self.failed_cycles = (
                self.failed_cycles + 1 if decision.get("decision") == "FAIL" else 0
            )
            if decision.get("decision") == "FAIL":
                self.ollama_retry_at = time.monotonic() + 30.0
                self.ollama_retry_version = version
            else:
                self.ollama_retry_at = 0.0
                self.ollama_retry_version = None
            self.intent_complete = (
                decision.get("decision") == "COMPLETE"
                and decision.get("status") == "completed"
            )
        log.info("Decision: %s", decision.get("decision"))
        if self.intent_complete:
            self.finish_scenario(
                "succeeded", "Verified intent completion", expected_version=version
            )
        elif (
            decision.get("error", {}).get("code")
            in (
                "AI_REPORTED_FAILURE",
                "MCP_TRANSPORT_ERROR",
                "MCP_EXECUTION_UNCERTAIN",
                "MCP_REQUEST_EXPIRED",
                "MCP_CONTEXT_REQUIRED",
            )
            or self.failed_cycles >= self.max_failed_cycles
        ):
            self.finish_scenario(
                "failed",
                decision.get("error", {}).get("code"),
                expected_version=version,
            )
        return decision

    def accept_intent(self, raw, version):
        pending = normalize_user_intent(raw)
        if isinstance(version, bool) or not isinstance(version, int) or version <= 0:
            raise ValueError("version must be a positive integer")
        with self.intent_ingest_lock, self.lock:
            if version < self.active_intent_version:
                return {"ok": False, "error": "INTENT_CHANGED"}
            if version == self.active_intent_version:
                return {
                    "ok": bool(
                        self.active_intent
                        and self.active_intent.get("id") == pending["id"]
                    ),
                    "status": "already_accepted",
                }
            loc = pending.get("target_location")
            if isinstance(loc, dict) and loc.get("frame_id") == "base_link":
                if not valid_pose(self.latest_odom):
                    return {"ok": False, "error": "LOCALIZATION_INVALID"}
                pending["original_target_location"] = copy.deepcopy(loc)
                pending["target_location"] = apply_tf(
                    loc["x"], loc["y"], self.latest_odom
                )
            if pending.get("action") == "resume" and not pending.get("target_location"):
                path = self.nav_feedback.get("current_path", [])
                if path:
                    pending["target_location"] = {
                        **path[-1],
                        "frame_id": self.nav_feedback.get("frame_id"),
                    }
            # Cloud never calls Robot. Edge fences old executions before accepting a new intent.
            fence_data = {
                "intent_id": pending["id"],
                "intent_version": version,
                "action": pending["action"],
                "target_location": pending.get("target_location"),
                "speed_mps": pending.get("constraints", {}).get("speed_mps"),
            }
            fence = requests.post(
                f"{NET_CFG.control_base_url}/control/intent",
                json=fence_data,
                timeout=0.8,
            )
            if fence.status_code != 200 or not fence.json().get("ok"):
                return {"ok": False, "error": "CONTROLLER_REJECTED_INTENT"}
            self.finish_scenario(
                "cancelled" if pending["action"] == "stop" else "superseded",
                "New user intent",
            )
            self.active_intent = pending
            self.active_intent_version = version
            self.intent_complete = False
            self.last_target_object_pos = None
            self.failed_cycles = 0
            self.ollama_retry_at = 0.0
            self.ollama_retry_version = None
            self.planner_agent.audit_writer.scenario(pending, version, activate=True)
            if pending["action"] == "stop":
                # Fencing above already stopped motion, independently of model availability.
                self.intent_complete = True
                self.finish_scenario(
                    "succeeded", "Explicit user stop", expected_version=version
                )
            return {
                "ok": True,
                "status": "ok",
                "controller_acknowledged": True,
                "stop_confirmed": True if pending["action"] == "stop" else None,
                "report_requested": pending.get("report", False),
            }

    def finish_scenario(
        self, outcome, reason=None, expected_version=None, completion_evidence=None
    ):
        with self.lock:
            if not self.active_intent or (
                expected_version is not None
                and expected_version != self.active_intent_version
            ):
                return None
            writer = self.planner_agent.audit_writer
            info = writer.scenario(self.active_intent, self.active_intent_version)
            sid = info["scenario_id"]
            _, doc = writer.scenarios[sid]
            if self.active_intent_version in self.terminal_versions:
                # Retry only a previously failed final disk write, never rewrite an existing file.
                return self.persist_final(sid, outcome, doc.get("report"), reason)
            pose = copy.deepcopy(
                completion_evidence["robot_pose"]
                if completion_evidence
                else self.latest_odom
            )
            pose["valid"] = valid_pose(pose)
            report = {
                "intent_id": self.active_intent["id"],
                "intent_version": self.active_intent_version,
                "completed_at": writer.now(),
                "target_location": copy.deepcopy(
                    self.active_intent.get("target_location")
                ),
                "current_location": pose,
                "success": outcome == "succeeded",
                "status": outcome,
                "reason": reason,
                "report_requested": self.active_intent.get("report", False),
            }
            if completion_evidence:
                report["completion_evidence"] = copy.deepcopy(completion_evidence)
            self.terminal_versions.add(self.active_intent_version)
            if outcome not in ("superseded", "cancelled"):
                # Fence late/in-flight model calls at Robot, not only in Edge's World State.
                self.pending_closures[self.active_intent_version] = {
                    "intent_id": self.active_intent["id"],
                    "intent_version": self.active_intent_version,
                }
                self.retry_robot_closures()
                report["robot_intent_closed"] = (
                    self.active_intent_version not in self.pending_closures
                )
            self.ollama_retry_at = 0.0
            # Failure/interruption callers have stopped or fenced motion before closing.
            path = self.persist_final(sid, outcome, report, reason)
            # This branch runs only on the first terminal transition. Disk-write
            # retries and later shutdowns use the early return above.
            self.log_terminal_report(report, path)
            return path

    def log_terminal_report(self, report, path):
        evidence = report.get("completion_evidence") or {}
        pose = report.get("current_location") or {}
        summary = {
            "Intent ID": report["intent_id"],
            "Version": report["intent_version"],
            "Completion Time": report["completed_at"],
            "Status": report["status"],
            "Target Location": report.get("target_location"),
            "Final Position": {k: pose.get(k) for k in ("x", "y", "frame_id", "valid")},
            "Distance to Goal (m)": evidence.get("distance_m"),
            "Stop Verified": evidence.get("stopped_verified"),
            "Robot Intent Closed": report.get("robot_intent_closed"),
            "Reason": report.get("reason"),
            "JSON Log": (
                str(path) if path else "Save failed — automatic retry scheduled"
            ),
        }
        log.info(
            "\n========== Execution Result: %s ==========\n%s\n====================================",
            "SUCCESS" if report.get("success") else "INCOMPLETE",
            json.dumps(summary, ensure_ascii=False, indent=2),
        )

    def retry_robot_closures(self):
        with self.lock:
            for version, data in list(self.pending_closures.items()):
                try:
                    response = requests.post(
                        f"{NET_CFG.control_base_url}/control/finish",
                        json=data,
                        timeout=0.5,
                    )
                    result = response.json()
                    if result.get("ok") or result.get("error") == "INTENT_CHANGED":
                        self.pending_closures.pop(version, None)
                except (requests.RequestException, ValueError):
                    pass

    def persist_final(self, sid, outcome, report, reason):
        try:
            path = self.planner_agent.audit_writer.finalize(
                sid, outcome, report, reason
            )
            self.last_log_error = None
            return path
        except (OSError, ValueError) as exc:
            self.last_log_error = str(exc)
            log.error("Final maintenance log not saved: %s", exc)
            return None

    def retry_final_logs(self):
        if not self.last_log_error or time.monotonic() - self.last_log_retry < 1.0:
            return
        self.last_log_retry = time.monotonic()
        writer = self.planner_agent.audit_writer
        with writer.lock:
            pending = [
                (sid, copy.deepcopy(doc))
                for sid, (_, doc) in writer.scenarios.items()
                if doc["terminal"] and sid not in writer.saved
            ]
        for sid, doc in pending:
            self.persist_final(
                sid, doc["final_outcome"], doc.get("report"), "Retry final write"
            )

    def get_report(self, intent_id=None, intent_version=None):
        import json

        with self.lock:
            writer = self.planner_agent.audit_writer
            with writer.lock:
                candidates = [
                    copy.deepcopy(doc) for _, doc in writer.scenarios.values()
                ]
            # Read-only recovery of terminal reports; do not resume or modify old logs.
            for path in (writer.root / "scenarios").glob("*.json"):
                try:
                    doc = json.loads(path.read_text(encoding="utf-8"))
                    if doc.get("schema_version") == 2 and doc.get("terminal"):
                        candidates.append(doc)
                except (OSError, ValueError):
                    continue
            candidates = [
                d
                for d in candidates
                if (intent_id is None or d["intent_id"] == intent_id)
                and (intent_version is None or d["intent_version"] == intent_version)
            ]
            if not candidates:
                return {"ok": False, "error": "REPORT_NOT_FOUND"}
            doc = max(candidates, key=lambda d: d["intent_version"])
            if doc.get("report"):
                return {"ok": True, "report": copy.deepcopy(doc["report"])}
            pose = copy.deepcopy(self.latest_odom)
            pose["valid"] = valid_pose(pose)
            return {
                "ok": True,
                "report": {
                    "intent_id": doc["intent_id"],
                    "intent_version": doc["intent_version"],
                    "completed_at": None,
                    "target_location": doc["intent"].get("target_location"),
                    "current_location": pose,
                    "success": None,
                    "status": doc["status"],
                    "report_requested": doc["intent"].get("report", False),
                },
            }

    def shutdown(self):
        self.stop_event.set()
        try:
            result = self.mcp_registry._control(
                "stop", {"reason": "Edge shutting down"}
            )
            reason = (
                "Edge shutdown; stop acknowledged"
                if result.get("ok")
                else "Edge shutdown; stop unconfirmed"
            )
        except requests.RequestException:
            reason = "Edge shutdown; stop unconfirmed"
        self.finish_scenario("interrupted", reason)

    def record_scan(self, scan):
        with self.scan_condition:
            stamp = scan.get("stamp_sec")
            previous = self.latest_lidar.get("stamp_sec")
            if (
                isinstance(stamp, (int, float))
                and isinstance(previous, (int, float))
                and stamp < previous
            ):
                self.scan_epoch += 1
                self.scan_history.clear()
                self.perception_scan = {}
            self.latest_lidar = scan
            if isinstance(stamp, (int, float)) and math.isfinite(stamp):
                if (
                    not self.scan_history
                    or self.scan_history[-1].get("stamp_sec") != stamp
                ):
                    self.scan_history.append(copy.deepcopy(scan))
            self.scan_condition.notify_all()

    def select_scan(self, camera, fallback):
        """Choose by ROS stamp; retain the existing skew and wall-clock freshness limits."""
        stamp = camera.get("stamp_sec")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return fallback
        with self.lock:
            candidates = [
                s
                for s in [fallback, *self.scan_history]
                if fresh(s)
                and isinstance(s.get("stamp_sec"), (int, float))
                and math.isfinite(s["stamp_sec"])
            ]
            return (
                copy.deepcopy(
                    min(candidates, key=lambda s: abs(s["stamp_sec"] - stamp))
                )
                if candidates
                else fallback
            )

    def await_matching_scan(self, camera, fallback, epoch):
        """Wait briefly for a lagging scan; never restamp or widen the skew limit."""
        deadline = time.monotonic() + self.sync_wait_sec
        with self.scan_condition:
            while True:
                if epoch != self.scan_epoch or self.stop_event.is_set():
                    return {}
                scan = self.select_scan(camera, fallback)
                if (
                    localization_error(camera, scan)
                    != "CAMERA_LIDAR_TIMESTAMP_MISMATCH"
                ):
                    return scan
                # A camera older than every available scan will not benefit from waiting.
                if (
                    not finite(camera.get("stamp_sec"))
                    or not finite(scan.get("stamp_sec"))
                    or camera["stamp_sec"] <= scan["stamp_sec"]
                    or not fresh(camera)
                ):
                    return scan
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return scan
                self.scan_condition.wait(timeout=remaining)

    def poll_lidar(self):
        # Do not queue scans behind a slow odometry or navigation HTTP request.
        while not self.stop_event.is_set():
            try:
                response = requests.get(f"{NET_CFG.sensor_base_url}/lidar", timeout=0.3)
                response.raise_for_status()
                self.record_scan(response.json())
            except (requests.RequestException, ValueError):
                pass
            self.stop_event.wait(0.03)

    def log_navigation_event(self, value):
        key = (
            value.get("operation_id"),
            value.get("event_sequence"),
            value.get("status"),
            value.get("failure_code"),
        )
        if getattr(self, "_audit_nav_key", None) == key:
            return
        try:
            writer = self.planner_agent.audit_writer
            scenario_id = writer.navigation_scenario(value)
            writer.scenario_event(scenario_id, "navigation_event", value)
            self._audit_nav_key = key
        except Exception as exc:
            log.error("Navigation audit write failed: %s", exc)

    def poll_state(self):
        while not self.stop_event.is_set():
            self.retry_final_logs()
            self.retry_robot_closures()
            for endpoint, field in (
                (f"{NET_CFG.sensor_base_url}/odometry", "latest_odom"),
                (f"{NET_CFG.control_base_url}/navigation/status", "nav_feedback"),
            ):
                try:
                    response = requests.get(endpoint, timeout=0.3)
                    response.raise_for_status()
                    value = response.json()
                    with self.lock:
                        setattr(self, field, value)
                    if field == "nav_feedback":
                        self.log_navigation_event(value)
                except (requests.RequestException, ValueError):
                    pass
            time.sleep(0.05)

    def process_frame(self, frame, camera, scan):
        """Run image detection regardless of LiDAR readiness; gate localization separately."""
        with self.lock:
            epoch = self.scan_epoch
        predictions = self.model.predict(frame, verbose=False)
        detections = []
        for prediction in predictions:
            for box in prediction.boxes:
                detections.append(
                    {
                        "class": prediction.names[int(box.cls.item())],
                        "confidence": float(box.conf.item()),
                        "bbox": box.xyxy[0].tolist(),
                    }
                )
        scan = self.await_matching_scan(camera, scan, epoch)
        error = localization_error(camera, scan)
        if error:
            objects = [
                {
                    **det,
                    "id": f"{det['class']}_{i}",
                    "observed_at": camera["timestamp"],
                    "distance_m": None,
                    "angle_deg": None,
                    "position": {"valid": False},
                    "localization_error": error,
                }
                for i, det in enumerate(detections)
            ]
        else:
            objects = localize_detections(detections, camera, scan)
        with self.lock:
            if epoch != self.scan_epoch:
                # Discard in-flight fusion from the previous simulation timeline.
                return
            self.latest_camera = camera
            self.perception_scan = copy.deepcopy(scan)
            self.latest_image_b64 = camera["image"]
            self.detected_objects = objects
            self.perception = {
                # Keep existing ready as the conservative motion prerequisite.
                "ready": error is None,
                "image_ready": True,
                "localization_ready": error is None,
                "error": error,
                "localization_error": error,
                "detection_count": len(detections),
                "timestamp": camera["timestamp"],
                "camera_stamp_sec": camera.get("stamp_sec"),
                "lidar_stamp_sec": scan.get("stamp_sec"),
                "sync_skew_sec": (
                    abs(camera["stamp_sec"] - scan["stamp_sec"])
                    if isinstance(camera.get("stamp_sec"), (int, float))
                    and isinstance(scan.get("stamp_sec"), (int, float))
                    else None
                ),
                "max_sync_skew_sec": MAX_SYNC_SKEW_SEC,
            }

    def run_perception(self):
        with self.lock:
            self.perception = {
                "ready": False,
                "error": "MODEL_LOADING",
                "timestamp": time.time(),
            }
        try:
            import cv2
            import numpy as np

            if self.model is None:
                from ultralytics import YOLO

                if not self.model_path.is_file():
                    raise FileNotFoundError(f"YOLO weights missing: {self.model_path}")
                self.model = YOLO(str(self.model_path))
        except Exception as exc:
            with self.lock:
                self.perception = {
                    "ready": False,
                    "error": str(exc),
                    "timestamp": time.time(),
                }
            log.error("Perception unavailable: %s", exc)
            return
        previous = None
        while not self.stop_event.is_set():
            try:
                response = requests.get(
                    f"{NET_CFG.sensor_base_url}/camera", timeout=0.5
                )
                response.raise_for_status()
                camera = response.json()
                with self.lock:
                    scan = copy.deepcopy(self.latest_lidar)
                if not camera:
                    raise ValueError(
                        "CAMERA_NO_MESSAGES: check sensor /diagnostics and ROS image topic"
                    )
                if not fresh(camera):
                    raise ValueError("CAMERA_STALE_OR_INVALID_TIMESTAMP")
                if camera.get("timestamp") == previous:
                    time.sleep(0.03)
                    continue
                frame = cv2.imdecode(
                    np.frombuffer(base64.b64decode(camera["image"]), dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if frame is None:
                    raise ValueError("Camera JPEG decode failed")
                self.process_frame(frame, camera, scan)
                previous = camera["timestamp"]
            except Exception as exc:
                with self.lock:
                    self.perception = {
                        "ready": False,
                        "error": str(exc),
                        "timestamp": time.time(),
                    }
                log.warning("Perception update failed: %s", exc)
                time.sleep(0.2)

    def run_planning(self):
        while not self.stop_event.is_set():
            try:
                self.step_planning_cycle()
            except Exception:
                log.exception("Planner cycle failed")
                self.mcp_registry.call_tool(
                    "stop_robot", {"reason": "Planner exception"}
                )
            time.sleep(0.2)


def create_app(orchestrator):
    app = Flask(__name__)

    @app.post("/intents")
    def accept_intent():
        try:
            data = request.get_json(force=True)
            result = orchestrator.accept_intent(data["intent"], data["version"])
            return jsonify(result), 200 if result.get("ok") else 409
        except requests.RequestException as exc:
            return (
                jsonify(
                    {
                        "ok": False,
                        "error": "CONTROLLER_UNAVAILABLE",
                        "message": str(exc),
                    }
                ),
                503,
            )
        except (ValueError, TypeError, KeyError) as exc:
            return (
                jsonify({"ok": False, "error": "INVALID_INTENT", "message": str(exc)}),
                400,
            )

    @app.get("/reports")
    def report():
        try:
            version = (
                int(request.args["intent_version"])
                if "intent_version" in request.args
                else None
            )
            result = orchestrator.get_report(request.args.get("intent_id"), version)
            return jsonify(result), 200 if result.get("ok") else 404
        except ValueError:
            return jsonify({"ok": False, "error": "INVALID_VERSION"}), 400

    @app.get("/world_state")
    def world():
        return jsonify(orchestrator.build_llm_world_state())

    @app.get("/health")
    def health():
        world = orchestrator.build_llm_world_state()
        return jsonify(
            {
                "mcp": orchestrator.mcp_client.status(),
                "perception": world["perception"],
                "map": world["environment"]["map"],
                "pose_valid": world["robot_state"]["valid"],
                "scan_error": scan_health(world["raw_lidar"]),
                "scan_observation": {
                    "policy": world["raw_lidar"].get("no_return_policy", "strict"),
                    "front_observed": world["raw_lidar"].get("front_observed", False),
                    "front_no_return_ray_count": world["raw_lidar"].get(
                        "front_no_return_ray_count", 0
                    ),
                    "no_return_ray_count": world["raw_lidar"].get(
                        "no_return_ray_count", 0
                    ),
                },
                "planner_retry_in_sec": max(
                    0.0, round(orchestrator.ollama_retry_at - time.monotonic(), 1)
                ),
                "scenario_terminal": world["scenario_terminal"],
                "final_log_error": orchestrator.last_log_error,
                "pending_robot_closures": list(orchestrator.pending_closures),
                "requested_report": (
                    orchestrator.get_report(
                        world["intent"].get("id"), world["intent_version"]
                    ).get("report")
                    if world["intent"].get("report")
                    else None
                ),
                "last_decision": orchestrator.last_decision,
            }
        )

    return app


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    orchestrator = EdgeServerOrchestrator()
    workers = [
        orchestrator.poll_state,
        orchestrator.poll_lidar,
        orchestrator.run_perception,
    ]
    if os.getenv("INTERNAL_PLANNER_ENABLED", "true").lower() == "true":
        workers.append(orchestrator.run_planning)
    for worker in workers:
        threading.Thread(target=worker, daemon=True).start()
    import signal

    def terminate(signum, frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, terminate)
    try:
        create_app(orchestrator).run(host=BIND_HOST, port=WORLD_PORT, threaded=True)
    finally:
        orchestrator.shutdown()
