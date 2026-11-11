"""Bound unproductive model actions; never invent or execute a replacement route."""

import json
import math


class PlannerProgress:
    OBSERVATIONS = {
        "get_robot_state",
        "get_detected_objects",
        "get_local_obstacles",
        "get_map_info",
        "get_navigation_status",
    }
    COLLISIONS = {"PATH_COLLISION_DETECTED", "STATIC_MAP_COLLISION"}

    def __init__(self):
        self.key = None
        self.anchor = None
        self.rotations = 0
        self.replan_required = False
        self.initial_plan_ready = False

    def begin(self, world):
        intent = world.get("intent") or {}
        key = (world.get("intent_version"), intent.get("id") or intent.get("intent_id"))
        pose = world.get("robot_state") or {}
        self.enabled = intent.get("action") in (
            "move",
            "detour",
            "resume",
        ) and isinstance(intent.get("target_location"), dict)
        if key != self.key:
            self.key, self.anchor = key, None
            self.rotations, self.replan_required = 0, False
            self.initial_plan_ready = False
        if all(
            isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
            for k in ("x", "y")
        ):
            if self.anchor is None or (
                pose.get("frame_id") == self.anchor[2]
                and math.hypot(pose["x"] - self.anchor[0], pose["y"] - self.anchor[1])
                >= 0.5
            ):
                self.anchor = (pose["x"], pose["y"], pose.get("frame_id"))
                self.rotations = 0
        if world.get("route_constraints"):
            self.replan_required = True
        self.plan_attempted = False
        self.plan_ok = False
        self.queries = {}

    def snapshot(self):
        return {
            "enabled": self.enabled,
            "replan_required": self.replan_required,
            "initial_plan_required": self.enabled and not self.initial_plan_ready,
            "plan_attempted_this_cycle": self.plan_attempted,
            "plan_ok_this_cycle": self.plan_ok,
            "accepted_observation_rotations_since_progress": self.rotations,
            "max_observation_rotations": 2,
            "max_observation_angle_deg": 30,
            "progress_reset_distance_m": 0.5,
            "successful_queries_this_cycle": sum(self.queries.values()),
        }

    @staticmethod
    def query_key(name, args):
        return (name, json.dumps(args, sort_keys=True, allow_nan=False))

    def check(self, name, args):
        if not self.enabled:
            return
        if name == "execute_path" and self.replan_required:
            raise ValueError(
                "REPLAN_REQUIRED: a path collision remains unresolved. Request plan_path from the current pose, inspect its result in a new turn, then choose a validated detour. Nearby empty scans do not clear a distant collision."
            )
        if name == "execute_path" and not self.initial_plan_ready:
            raise ValueError(
                "PLAN_REQUIRED: before the first execution of this coordinate intent, request plan_path alone and inspect its successful path result in a new turn. Do not execute a direct goal waypoint based only on nearby observations."
            )
        if name == "rotate_robot":
            if not self.plan_attempted:
                raise ValueError(
                    "PLAN_BEFORE_ROTATION: request plan_path first. execute_path already aligns heading; rotation is not a substitute for route planning."
                )
            if self.plan_ok:
                raise ValueError(
                    "CANDIDATE_AVAILABLE: inspect the successful plan_path candidate and choose execute_path or revise it. No observation rotation is needed for heading alignment."
                )
            angle = args.get("angle_deg")
            if (
                isinstance(angle, bool)
                or not isinstance(angle, (float, int))
                or not math.isfinite(angle)
                or not 3.5 <= abs(angle) <= 30
            ):
                raise ValueError(
                    "OBSERVATION_ROTATION_LIMIT: after failed planning, choose a useful observation angle of 3.5 to 30 degrees. Do not turn around to align with the goal."
                )
            if self.rotations >= 2:
                raise ValueError(
                    "OBSERVATION_ROTATION_BUDGET: two observation rotations already accepted without 0.5m translational progress. Replan, stop, or report FAIL; do not rotate again."
                )
        if name in self.OBSERVATIONS:
            if (
                self.queries.get(self.query_key(name, args), 0) >= 2
                or sum(self.queries.values()) >= 4
            ):
                raise ValueError(
                    "OBSERVATION_BUDGET: use the returned evidence. Choose plan_path, a validated execution, stop, or FAIL instead of repeating queries. Empty short-range queries do not clear route constraints."
                )

    def record(self, name, args, result):
        if not self.enabled:
            return
        if name == "plan_path":
            self.plan_attempted = True
            self.plan_ok = bool(result.get("ok"))
            if (
                self.plan_ok
                and (result.get("path") or {}).get("points")
                and (result.get("validation") or {}).get("valid") is not False
            ):
                self.initial_plan_ready = True
            if self.plan_ok:
                self.replan_required = (
                    False  # Only the planning prerequisite; safety checks still apply.
                )
        if name == "execute_path" and result.get("error") in self.COLLISIONS:
            self.replan_required = True
            self.plan_ok = False
        if result.get("ok") and name == "rotate_robot":
            self.rotations += 1
        if result.get("ok") and name in self.OBSERVATIONS:
            key = self.query_key(name, args)
            self.queries[key] = self.queries.get(key, 0) + 1
