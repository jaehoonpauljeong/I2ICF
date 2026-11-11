#!/usr/bin/env python3
"""qwen3-vl:4b-instruct JSON tool-selection loop. No hidden rule-based driving fallback."""

import json
import copy
import logging
import math
import requests
from dataclasses import asdict
from edge.audit_log import AuditLog
from edge.mcp_client import MCPTransportError
from common.config import ENV_CFG, LLM_CFG
from common.runtime_safety import fresh, valid_pose
from common.navigation_utils import goal_geometry
from common.goal_completion import GOAL_TOLERANCE_M
from common.speed_policy import select_speed
from edge.prompt_context import compact_messages, context_tokens_needed
from edge.planner_progress import PlannerProgress

log = logging.getLogger(__name__)
SYSTEM_PROMPT = """You control a ROS2 robot through tools. Choose the next useful action autonomously.
Return ONLY one JSON object: decision, reason, and mcp_tool_calls (an array of name/arguments objects).
Keep explanations to one or two short sentences. Prefer one useful tool call per response.
Never repeat the supplied world state or tool schemas in your answer.
Use ACT for an action, OBSERVE for information, FAIL with error:{code,message} if unable to proceed,
or COMPLETE only when the user's goal is verified. Do not output schemas or repeat observations.
Use navigation_goal as the final destination; intermediate goals need a reason.
Honor intent constraints and measured coordinate frames. Never invent observations, TF or locations.
plan_path computes a candidate only: inspect its result in a NEW model turn before choosing execute_path.
Do not batch plan_path and a dependent execute_path in the same turn. Do not invent placeholder
waypoints while waiting for the result. Each waypoint must contain exactly x and y; yaw is forbidden.
The current/start pose (including map origin) is not a useful movement segment when the goal is elsewhere.
On NO_MOTION_REQUIRED, use the returned candidate or plan a useful remaining segment. This rejection
starts no operation and is not proof that the user's final goal has been reached.
You may instead choose your own waypoints. A mapless segment is at most 2.5m.
A motion tool starts an asynchronous operation; it does not prove goal completion.
For a pose goal, arrival is within 0.15 meters; final yaw is not required.
Do not request alignment or another move when already inside that tolerance.
On rejection, inspect the reason and revise, observe, stop or fail. Stop is always allowed.
stop_robot and cancel_navigation stop the current operation, not the user's pending goal.
After either succeeds, continue this decision with observation, replanning or explicit FAIL.
Neither clears e_stop. If e_stop is true, explicitly request reset_safety_stop when appropriate
before executing a newly validated route. A canceled operation does not mean a canceled intent.
Detecting a person does not by itself prove every route is blocked. Use measured positions and
path validation to assess clearance; consider a detour or further observation when appropriate.
planning_context contains historical candidates and the last accepted segment, not current clearance.
After segment completion, continue toward navigation_goal by choosing a remaining segment or replanning.
navigation.remaining_path is the unfinished suffix recorded by the controller for path_intent_version.
For the same intent, prefer that suffix after a pause, revalidate it with execute_path or plan a new
route from the measured current pose. Never return to completed waypoints just to resume.
If PATH_ALREADY_COMPLETED is returned, verify the final goal or plan a new segment; do not replay it.
Historical planning_context.candidate does not reset measured progress.
Never replay completed waypoints blindly; base_link coordinates refer to the pose at planning time.
navigation.last_safety_stop records a historical execution failure, including its collision location.
Use failures from the current intent to revise or investigate the route; they are not fresh obstacle measurements.
Use goal_geometry.heading_error_deg, not the sign of goal x, to interpret heading.
At yaw +/-180 degrees a goal along negative map x is already ahead. execute_path performs
alignment automatically. rotate_robot is relative counterclockwise and is only needed for
intentional new observations; a 180-degree turn does not remove a distant route obstacle.
For pose goals with valid observations, plan/execute first. Do not turn around at startup when
heading_error_deg is near zero. Do not rotate to align: the path controller already does that.
A necessary observation rotation must include purpose="observe" and reason describing the specific
missing information and why the existing observations are insufficient. Prefer a small useful angle.
Use the numeric goal_geometry.distance_m for arrival claims; never claim 0.15m without this evidence.
route_constraints are unresolved historical collisions in their named frame, not body-frame
coordinates or fresh detections. Detour around them; only positive newer scan evidence clears them.
If SAFETY_STOP_LATCHED is returned, a valid detour is NOT executable until an explicit
reset_safety_stop succeeds. Keep that detour, request reset if appropriate, then retry the detour.
Do not discard it and retry the known blocked straight path. Never auto-clear a latch.
For execute_path, include speed_mps (linear cruise speed, 0.01 to 0.35 m/s).
If intent.constraints.speed_mps exists, use that exact user value. Otherwise choose a speed
from current observations and include a non-empty top-level speed_reason string in the SAME JSON
response as execute_path. Explain why that numeric speed suits the observed clearance, visibility
or route, and mention relevant uncertainty; do not invent measurements. A generic reason for moving
is not a speed explanation. Example response shape: {"decision":"ACT","speed_reason":"Your
observation-based explanation for the selected speed","mcp_tool_calls":[...]}. speed_reason is
response metadata, NEVER a tool argument. Native tool calls also require JSON content containing
speed_reason when the user supplied no speed. No fixed default is substituted for missing speed.
Safety may slow or stop the robot regardless of the chosen speed. Rotation angle is independent.
Tool arguments must obey the supplied types, bounds and required fields; no extra arguments.
ACT/OBSERVE requires at least one tool call. Batch independent observations in one mcp_tool_calls array.
Use observations already supplied; avoid repeating unchanged queries. Reserve turns to plan and act.
An empty distance-filtered object list does not prove no person exists: distance may be unknown.
An empty local scan query covers ONLY its requested radius, not the full route. It cannot invalidate
a farther collision reported by execute_path. Replan around that collision or obtain relevant evidence.
The original-height LIMO has rear self-occlusion. Do not assume that blind region is empty.
Use forward travel; reverse is disabled. Resetting a safety latch is not proof of rotation clearance.
Rotation uses swept-footprint checks; static_map_only_simulation means rear dynamic clearance is unverified.
For each new coordinate move/detour/resume intent, first request plan_path alone using the target
coordinates and safety margin. Inspect its successful path result in a new turn before execute_path.
Use the returned route; do not begin with a direct goal waypoint just because nearby space is clear.
PLAN_REQUIRED is an error code, not a decision: use decision=PLAN with a single plan_path call.
An existing successful initial plan is retained across cycles of the same intent; fresh collision
checks still apply to every execution. For coordinate navigation, after collision, call plan_path before
another execute_path. REPLAN_REQUIRED is an error code, never a decision value. Use decision=REPLAN
with exactly one plan_path call; inspect its result in a new turn before selecting execute_path.
Each mcp_tool_calls entry must contain name and arguments, never a nested decision or call list.
Before any observation rotation, try plan_path in this cycle; if a candidate
exists, inspect and execute or revise it. Only after failed planning may you choose an observation
rotation of 3.5..30 degrees, at most twice without 0.5m translational progress. Heading alignment
belongs to execute_path. At most four successful observation queries per cycle, twice per identical
query. Use their results to plan instead of repeatedly querying a clear nearby radius.
"""


def prompt_view(value):
    """Bound repeated status history without dropping current state or altering source data."""
    if isinstance(value, dict):
        result = {
            key: prompt_view(item)
            for key, item in value.items()
            if key not in ("events", "audit_details")
        }
        # Current navigation is repeated in initial state and tool replies. Keep actionable
        # state and the last failure, while full forensic history remains in audit JSON.
        if "event_sequence" in value and "robot_pose" in value:
            result.pop("events", None)
            path = value.get("current_path", [])
            index = value.get("current_wp_index", 0)
            if (
                isinstance(path, list)
                and type(index) is int
                and 0 <= index <= len(path)
            ):
                result["remaining_path"] = prompt_view(path[index:])
                result["completed_waypoint_count"] = index
                result.pop("current_path", None)
            result["event_history_in_audit"] = True
            failure = result.get("last_safety_stop")
            if isinstance(failure, dict):
                keys = (
                    "code",
                    "action",
                    "status_before_stop",
                    "operation_id",
                    "timestamp",
                    "frame_id",
                    "robot_pose",
                    "remaining_path",
                    "rotate_target_yaw",
                    "angular_error_rad",
                    "validation",
                    "progress_clock",
                    "progress_elapsed_sec",
                    "rejected_ray_counts",
                    "command_before_stop",
                )
                result["last_safety_stop"] = {
                    k: failure[k] for k in keys if k in failure
                }
            pose = result.get("robot_pose")
            if isinstance(pose, dict):
                result["robot_pose"] = {k: v for k, v in pose.items() if k != "twist"}
            if result.get("motion_check") and result["motion_check"] == (
                result.get("last_safety_stop") or {}
            ).get("validation"):
                result["motion_check"] = {"same_as_last_safety_stop_validation": True}
        coverage = result.get("rotation_coverage")
        if isinstance(coverage, dict) and isinstance(coverage.get("sectors"), list):
            coverage["sector_accepted_ratios"] = [
                s.get("accepted_ratio") for s in coverage["sectors"]
            ]
            coverage["failed_sectors"] = [
                s.get("sector") for s in coverage["sectors"] if not s.get("coverage_ok")
            ]
            del coverage["sectors"]
        if "events" in value and not (
            "event_sequence" in value and "robot_pose" in value
        ):
            events = value["events"]
            if isinstance(events, list):
                result["events"] = [prompt_view(item) for item in events[-5:]]
                result["earlier_events_omitted"] = max(0, len(events) - 5)
            else:
                result["events"] = events
        return result
    if isinstance(value, list):
        return [prompt_view(item) for item in value]
    return value


def prompt_json(value):
    return json.dumps(
        prompt_view(value), ensure_ascii=False, separators=(",", ":"), allow_nan=False
    )


def prompt_tools(tools):
    """Remove schema wrapper repetition, keeping all tool choices and argument constraints."""
    return [
        {
            "name": t["name"],
            "description": t["description"],
            "arguments": t["inputSchema"]["properties"],
            "required": t["inputSchema"].get("required", []),
        }
        for t in tools
    ]


class OllamaMCPPlannerAgent:
    def __init__(self, mcp_registry):
        self.mcp = mcp_registry
        self.ollama_url = LLM_CFG.ollama_url
        self.model = LLM_CFG.model_name
        self.native_tools = LLM_CFG.native_tools
        self.last_error = None
        self.request_diagnostics = []
        self.context_key = None
        self.planning_context = {}
        self.progress_policy = PlannerProgress()
        self.audit_writer = AuditLog()
        self.audit_record = None
        self.audit_error = None

    def audit_event(self, kind, data):
        if self.audit_record is None:
            return
        try:
            self.audit_writer.event(self.audit_record, kind, data)
            return True
        except Exception as exc:
            self.audit_error = str(exc)
            log.error("Audit write failed: %s", exc)
            return False

    def _call_ollama(
        self,
        messages,
        use_tools=None,
        json_format=True,
        recovery=False,
        context_retry=False,
    ):
        native = self.native_tools if use_tools is None else use_tools
        num_ctx = getattr(self, "_active_num_ctx", LLM_CFG.num_ctx)
        original_chars = sum(len(str(m.get("content") or "")) for m in messages)
        history_compacted = False
        # Character count is only a trigger, NOT a tokenizer or safety guarantee.
        # Server token counts below drive bounded context recovery.
        if context_retry or original_chars > num_ctx * 2:
            compacted = compact_messages(messages)
            if (
                sum(len(str(m.get("content") or "")) for m in compacted)
                < original_chars
            ):
                messages = compacted
                history_compacted = True
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "format": "json",
            "options": {
                "temperature": LLM_CFG.temperature,
                "num_predict": LLM_CFG.num_predict,
                "num_ctx": num_ctx,
            },
        }
        if not json_format:
            payload.pop("format")
        if recovery:
            # A bounded generation retry, not a different planner or invented decision.
            payload.pop("format", None)
            payload["options"].update(
                temperature=0.2, repeat_penalty=1.1, repeat_last_n=64
            )
        if native:
            payload["tools"] = self.mcp.get_ollama_tools_schema()
        diagnostic = {
            "request": len(self.request_diagnostics) + 1,
            "recovery": recovery,
            "context_retry": context_retry,
            "original_text_chars": original_chars,
            "history_compacted": history_compacted,
            "format": payload.get("format", "prompt_only"),
            "text_chars": sum(len(str(m.get("content") or "")) for m in messages),
            "image_count": sum(len(m.get("images") or []) for m in messages),
            "options": dict(payload["options"]),
        }
        self.request_diagnostics.append(diagnostic)
        try:
            if self.audit_record is not None:
                self.audit_event(
                    "ollama_request",
                    {
                        "request": diagnostic["request"],
                        "url": self.ollama_url,
                        "payload": self.audit_writer.externalize_images(payload),
                    },
                )
        except Exception as exc:
            self.audit_error = str(exc)
            log.error("Audit image write failed: %s", exc)
        try:
            response = requests.post(
                self.ollama_url, json=payload, timeout=LLM_CFG.request_timeout_sec
            )
            diagnostic["http_status"] = response.status_code
            try:
                audit_body = response.json()
            except ValueError:
                audit_body = response.text
            self.audit_event(
                "ollama_response",
                {
                    "request": diagnostic["request"],
                    "http_status": response.status_code,
                    "body": audit_body,
                },
            )
            if (
                response.status_code == 400
                and native
                and "tool" in response.text.lower()
                and ("support" in response.text.lower())
            ):
                # Keep the model and its agency; switch only the tool-request encoding.
                self.native_tools = False
                return self._call_ollama(
                    messages,
                    use_tools=False,
                    json_format=json_format,
                    recovery=recovery,
                    context_retry=context_retry,
                )
            if response.status_code >= 400:
                try:
                    error_body = response.json()
                    detail = (
                        error_body.get("error", response.text)
                        if isinstance(error_body, dict)
                        else response.text
                    )
                except ValueError:
                    detail = response.text
                self.last_error = f"HTTP {response.status_code}: {str(detail)[:2000]}"
                diagnostic["error"] = self.last_error
                log.error("Ollama %s request rejected: %s", self.model, self.last_error)
                context_error = (
                    isinstance(detail, dict)
                    and detail.get("type") == "exceed_context_size_error"
                ) or "exceeds the available context size" in str(detail).lower()
                if response.status_code == 400 and context_error and not context_retry:
                    tokens = (
                        detail.get("n_prompt_tokens")
                        if isinstance(detail, dict)
                        else None
                    )
                    needed = context_tokens_needed(tokens, LLM_CFG.num_predict)
                    ceiling = max(LLM_CFG.num_ctx, LLM_CFG.max_num_ctx)
                    self._active_num_ctx = min(
                        ceiling, max(num_ctx, needed or num_ctx * 2)
                    )
                    self.audit_event(
                        "context_recovery",
                        {
                            "prompt_tokens": tokens,
                            "previous_num_ctx": num_ctx,
                            "next_num_ctx": self._active_num_ctx,
                            "output_reserve_tokens": LLM_CFG.num_predict,
                            "max_num_ctx": ceiling,
                        },
                    )
                    return self._call_ollama(
                        messages,
                        use_tools=native,
                        json_format=json_format,
                        recovery=recovery,
                        context_retry=True,
                    )
                if (
                    response.status_code == 500
                    and "token repeat limit reached" in str(detail).lower()
                    and not recovery
                ):
                    log.warning(
                        "Retrying repeated-token generation once with unconstrained JSON prompting and temperature=0.2"
                    )
                    return self._call_ollama(
                        messages,
                        use_tools=native,
                        json_format=False,
                        recovery=True,
                        context_retry=context_retry,
                    )
                return None
            response.raise_for_status()
            body = response.json()
            if not isinstance(body, dict):
                raise ValueError("Ollama response must be an object")
            if not isinstance(body.get("message"), dict):
                raise ValueError("Missing Ollama message")
            message = body["message"]
            content = message.get("content")
            diagnostic.update(
                {
                    key: body.get(key)
                    for key in ("done_reason", "prompt_eval_count", "eval_count")
                }
            )
            diagnostic["content_chars"] = len(str(content or ""))
            # Reserve room for the next answer using measured input, not a guessed
            # text-to-token ratio (images also consume context).
            needed = context_tokens_needed(
                body.get("prompt_eval_count"), LLM_CFG.num_predict
            )
            if needed and needed > num_ctx:
                self._active_num_ctx = min(
                    max(LLM_CFG.num_ctx, LLM_CFG.max_num_ctx), needed
                )
            if not message.get("tool_calls") and (
                content is None or isinstance(content, str) and not content.strip()
            ):
                # Some compatible servers return no text under constrained decoding.
                # Retry once without that constraint, retaining images and the JSON prompt.
                metadata = {
                    key: body.get(key)
                    for key in (
                        "model",
                        "done",
                        "done_reason",
                        "prompt_eval_count",
                        "eval_count",
                    )
                }
                metadata["response_keys"] = sorted(body)
                metadata["message_keys"] = sorted(message)
                metadata["thinking_chars"] = len(str(message.get("thinking") or ""))
                log.warning(
                    "Empty Ollama content (json_format=%s): %s", json_format, metadata
                )
                if json_format and not recovery:
                    return self._call_ollama(
                        messages,
                        use_tools=native,
                        json_format=False,
                        recovery=True,
                        context_retry=context_retry,
                    )
                self.last_error = (
                    "OLLAMA_EMPTY_RESPONSE: no content or tool calls after one retry; "
                    + prompt_json(metadata)
                )
                return None
            self.last_error = None
            return body
        except (requests.RequestException, ValueError) as exc:
            self.audit_event(
                "ollama_error", {"request": diagnostic["request"], "error": str(exc)}
            )
            self.last_error = str(exc)
            diagnostic["error"] = self.last_error
            log.error("Ollama %s request failed: %s", self.model, exc)
            return None

    def resolve_target_coordinates(self, world):
        intent, robot = world.get("intent", {}), world.get("robot_state", {})
        if intent.get("action") == "stop":
            return dict(robot)
        objects = world.get("environment", {}).get("objects", [])
        if intent.get("action") in ("find", "approach"):
            matches = [
                o
                for o in objects
                if o.get("class") == intent.get("target_object")
                and fresh(o.get("position", {}))
                and o["position"].get("valid")
                and o["position"].get("frame_id") == robot.get("frame_id")
            ]
            if matches:
                obj = min(
                    matches,
                    key=lambda o: math.hypot(
                        o["position"]["x"] - robot["x"], o["position"]["y"] - robot["y"]
                    ),
                )
                return obj["position"]
            return None
        loc = intent.get("target_location")
        return (
            loc
            if isinstance(loc, dict) and loc.get("frame_id") == robot.get("frame_id")
            else None
        )

    def completion_verified(self, world):
        intent, robot = world.get("intent", {}), world.get("robot_state", {})
        if intent.get("action") == "stop":
            return True
        if not valid_pose(robot):
            return False
        target = self.resolve_target_coordinates(world)
        if not target:
            return False
        if intent.get("action") == "find":
            return fresh(target)
        distance = math.hypot(target["x"] - robot["x"], target["y"] - robot["y"])
        if intent.get("action") == "approach":
            desired = intent.get("constraints", {}).get("stop_distance_m", 1.0)
            return desired <= distance <= desired + 0.20
        return distance <= GOAL_TOLERANCE_M

    def plan_and_act(self, world_state, image_b64=None, replan_trigger=None):
        self.request_diagnostics = []
        self._active_num_ctx = LLM_CFG.num_ctx
        self.audit_error = None
        self.audit_record = None
        discovery_error = None
        try:
            tools = self.mcp.list_tools()
        except MCPTransportError as exc:
            tools, discovery_error = [], str(exc)
        try:
            scenario = (
                self.audit_writer.scenario(
                    world_state.get("intent") or {}, world_state.get("intent_version")
                )
                or {}
            )
            self.audit_record = self.audit_writer.create(
                "planning_cycle",
                {
                    **scenario,
                    "trigger": replan_trigger,
                    "world_state": world_state,
                    "model": self.model,
                    "configuration": {
                        "llm": asdict(LLM_CFG),
                        "environment": asdict(ENV_CFG),
                    },
                    "tool_schemas": tools,
                    "explanation_scope": "Model-provided reasons, actual requests/results and deterministic helper decisions; no hidden internal reasoning is inferred.",
                },
            )
            self.audit_writer.scenario_event(
                scenario.get("scenario_id"),
                "planning_cycle_started",
                {
                    "path": str(self.audit_record[0]),
                    "trigger": replan_trigger,
                    "inputs": self.audit_record[1],
                },
                self.audit_record[1]["record_id"],
            )
        except Exception as exc:
            self.audit_error = str(exc)
            log.error("Audit creation failed: %s", exc)
        intent = world_state.get("intent") or {}
        version = world_state.get("intent_version")
        intent_id = intent.get("id") or intent.get("intent_id")
        context_key = (version, intent_id)
        if self.context_key != context_key or intent.get("action") == "stop":
            self.planning_context = {}
            self.context_key = context_key
        trace = []
        attempts = []
        speed_decisions = []
        self.progress_policy.begin(world_state)

        def call(name, args):
            self.audit_event(
                "tool_request",
                {"tool": name, "arguments": args, "intent_version": version},
            )
            result = self.mcp.call_tool(name, args, expected_intent_version=version)
            self.progress_policy.record(name, args, result)
            self.audit_event(
                "tool_result", {"tool": name, "arguments": args, "result": result}
            )
            result = {k: v for k, v in result.items() if k != "audit_details"}
            trace.append({"tool": name, "arguments": args, "result": result})
            if result.get("ok") and name == "plan_path" and result.get("path"):
                self.planning_context["candidate"] = copy.deepcopy(
                    {
                        "goal": args,
                        "frame_id": result.get("frame_id"),
                        "path": result["path"],
                        "planned_from": world_state.get("robot_state"),
                        "requires_fresh_validation": True,
                    }
                )
            if result.get("ok") and name == "execute_path":
                self.planning_context["last_accepted_segment"] = copy.deepcopy(
                    {
                        "frame_id": args.get("frame_id"),
                        "waypoints": result.get("path_progress", {}).get(
                            "points", args.get("waypoints")
                        ),
                        "operation_id": result.get("operation_id"),
                    }
                )
            return result

        def finish(decision, **extra):
            transport_failure = next(
                (
                    t["result"]
                    for t in trace
                    if t["result"].get("error")
                    in (
                        "MCP_TRANSPORT_ERROR",
                        "MCP_EXECUTION_UNCERTAIN",
                        "MCP_REQUEST_EXPIRED",
                        "MCP_CONTEXT_REQUIRED",
                    )
                ),
                None,
            )
            if transport_failure:
                decision = "FAIL"
                extra.update(
                    status="failed",
                    decision_source="mcp_transport",
                    error={
                        "code": transport_failure["error"],
                        "message": transport_failure.get("message"),
                    },
                )
            result = {
                "intent_id": intent_id,
                "decision": decision,
                "decision_source": "ollama",
                "model": self.model,
                "mcp_tool_trace": trace,
                "model_attempts": attempts,
                "speed_decisions": copy.deepcopy(speed_decisions),
                "progress_policy": self.progress_policy.snapshot(),
                "request_diagnostics": self.request_diagnostics,
                **extra,
            }
            result["audit_log"] = {
                "path": str(self.audit_record[0]) if self.audit_record else None,
                "error": self.audit_error,
            }
            if self.audit_record:
                result["audit_log"].update(
                    {
                        k: self.audit_record[1].get(k)
                        for k in ("scenario_id", "scenario_path")
                    }
                )
            if self.audit_record:
                self.audit_record[1]["status"] = "finished"
                self.audit_record[1]["decision"] = copy.deepcopy(result)
                self.audit_event("decision", result)
                result["audit_log"]["error"] = self.audit_error
            return result

        if discovery_error:
            return finish(
                "FAIL",
                status="failed",
                decision_source="mcp_transport",
                error={"code": "MCP_TRANSPORT_ERROR", "message": discovery_error},
            )
        # Deterministic user stop and safety interlocks are independent of model availability.
        if intent.get("action") == "stop":
            result = call("stop_robot", {"reason": "Explicit user stop"})
            return finish(
                "COMPLETE" if result.get("ok") else "FAIL",
                status="completed" if result.get("ok") else "failed",
                decision_source="user_stop",
            )
        if not valid_pose(world_state.get("robot_state", {})):
            call("stop_robot", {"reason": "Invalid localization"})
            return finish(
                "FAIL", error={"code": "LOCALIZATION_INVALID"}, decision_source="safety"
            )
        scan_error = world_state.get("sensor_health", {}).get("scan_error")
        perception = world_state.get("perception", {})
        if scan_error or not perception.get("ready") or not fresh(perception):
            call(
                "stop_robot",
                {"reason": "Waiting for valid camera and LiDAR observations"},
            )
            return finish(
                "FAIL",
                error={
                    "code": "OBSERVATION_UNAVAILABLE",
                    "scan_error": scan_error,
                    "perception_error": perception.get("error"),
                },
                status="failed",
                decision_source="safety",
            )
        # Hold position while reasoning, without discarding the retained path/intent.
        paused = call(
            "stop_robot", {"reason": "Planning from a stationary observation"}
        )
        if not paused.get("ok"):
            return finish(
                "FAIL",
                error={"code": "CONTROLLER_UNREACHABLE"},
                decision_source="safety",
            )
        user = {
            "role": "user",
            "content": prompt_json(
                {
                    "trigger": replan_trigger,
                    "navigation_goal": self.resolve_target_coordinates(world_state),
                    "planning_context": self.planning_context,
                    "world_state": world_state,
                    "goal_geometry": goal_geometry(
                        world_state.get("robot_state", {}),
                        self.resolve_target_coordinates(world_state),
                    ),
                    "progress_policy": self.progress_policy.snapshot(),
                    "planning_order": (
                        "Call plan_path alone now (decision=REPLAN), inspect its result, then choose motion."
                        if self.progress_policy.enabled
                        and self.progress_policy.replan_required
                        else (
                            "First call plan_path alone (decision=PLAN); inspect its returned path before execute_path."
                            if self.progress_policy.enabled
                            and not self.progress_policy.initial_plan_ready
                            else "Use current evidence and validated route progress to choose the next tool."
                        )
                    ),
                    "recovery_context": {
                        "safety_latched": world_state.get("navigation", {}).get(
                            "e_stop", False
                        ),
                        "unresolved_collisions": world_state.get(
                            "route_constraints", []
                        ),
                        "candidate_available": bool(
                            self.planning_context.get("candidate")
                        ),
                    },
                }
            ),
        }
        if image_b64:
            user["images"] = [image_b64]
        messages = [
            {
                "role": "system",
                "content": SYSTEM_PROMPT
                + "\nAvailable tools:\n"
                + prompt_json(prompt_tools(tools)),
            },
            user,
        ]
        system_content = messages[0]["content"]
        for iteration in range(LLM_CFG.max_mcp_tool_iterations):
            remaining = LLM_CFG.max_mcp_tool_iterations - iteration
            messages[0]["content"] = (
                system_content
                + f"\nModel turns remaining including this one: {remaining}. Choose a useful next action; never bypass validation to fit the budget."
            )
            response = self._call_ollama(messages)
            if response is None:
                call("stop_robot", {"reason": "Ollama request failed"})
                return finish(
                    "FAIL",
                    error={"code": "OLLAMA_UNAVAILABLE", "message": self.last_error},
                    status="failed",
                )
            msg = response["message"]
            content = msg.get("content", "")
            attempt = {
                "iteration": iteration + 1,
                "response_excerpt": str(content)[:1000],
                "response_truncated": len(str(content)) > 1000,
                "done_reason": response.get("done_reason"),
                "outcome": "received",
            }
            attempts.append(attempt)
            try:
                if (
                    response.get("done_reason") in ("length", "max_tokens")
                    or response.get("done") is False
                ):
                    raise ValueError(
                        "MODEL_RESPONSE_TRUNCATED: return a complete concise JSON object with one useful tool call and a short speed_reason when selecting speed; do not repeat observations or schemas"
                    )
                native_calls = msg.get("tool_calls") or []
                text = content.strip() if isinstance(content, str) else content
                if (
                    isinstance(text, str)
                    and text.startswith("```")
                    and text.endswith("```")
                ):
                    lines = text.splitlines()
                    if lines[0].strip() in ("```", "```json"):
                        text = "\n".join(lines[1:-1])
                if native_calls:
                    # Native function arguments carry motion parameters; companion content
                    # carries the model's explanation. Preserve old observation/user-speed calls.
                    try:
                        parsed = json.loads(text) if text else {}
                    except (ValueError, TypeError):
                        parsed = {}
                    if not isinstance(parsed, dict):
                        parsed = {}
                else:
                    parsed = json.loads(text)
                if not isinstance(parsed, dict):
                    raise ValueError("Decision must be an object")
                decision = parsed.get("decision")
                self.audit_event(
                    "parsed_decision",
                    {
                        "iteration": iteration + 1,
                        "decision": parsed,
                        "native_calls": native_calls,
                    },
                )
                attempt.update(decision=decision, outcome="parsed")
                # Preserve the raw response above. Repair only an unambiguous
                # planning label, never invent tools, arguments, or movement.
                repair_calls = parsed.get("mcp_tool_calls")
                if (
                    not native_calls
                    and decision in ("PLAN_REQUIRED", "REPLAN_REQUIRED")
                    and "path" not in parsed
                    and isinstance(repair_calls, list)
                    and len(repair_calls) == 1
                    and isinstance(repair_calls[0], dict)
                    and set(repair_calls[0]) <= {"name", "tool", "arguments"}
                    and repair_calls[0].get("name", repair_calls[0].get("tool"))
                    == "plan_path"
                    and repair_calls[0].get("tool", "plan_path") == "plan_path"
                    and isinstance(repair_calls[0].get("arguments"), dict)
                ):
                    parsed = copy.deepcopy(parsed)
                    original_decision = decision
                    parsed["decision"] = decision = (
                        "REPLAN" if decision == "REPLAN_REQUIRED" else "PLAN"
                    )
                    attempt.update(
                        decision=decision, original_decision=original_decision
                    )
                    self.audit_event(
                        "decision_normalized",
                        {
                            "iteration": iteration + 1,
                            "original_decision": original_decision,
                            "decision": decision,
                            "reason": "Single explicit plan_path call",
                        },
                    )
                # Keep executable choices as history, not repetitive natural-language reasoning.
                # Full original responses remain available in model_attempts diagnostics.
                if native_calls:
                    messages.append(msg)
                else:
                    messages.append(
                        {
                            "role": "assistant",
                            "content": prompt_json(
                                {k: v for k, v in parsed.items() if k != "reason"}
                            ),
                        }
                    )
                calls = [
                    {
                        "name": c["function"]["name"],
                        "arguments": c["function"].get("arguments", {}),
                    }
                    for c in native_calls
                ]
                if not native_calls:
                    if decision in ("PLAN", "REPLAN"):
                        if "mcp_tool_calls" in parsed:
                            if "path" in parsed:
                                raise ValueError(
                                    "Choose either mcp_tool_calls or a direct path, not both"
                                )
                            calls = parsed["mcp_tool_calls"]
                        else:
                            path = parsed.get("path")
                            if (
                                not isinstance(path, dict)
                                or not isinstance(path.get("points"), list)
                                or not path["points"]
                            ):
                                raise ValueError(
                                    "PLAN requires non-empty path.points or mcp_tool_calls to request plan_path"
                                )
                            if (
                                not isinstance(parsed.get("frame_id"), str)
                                or not parsed["frame_id"].strip()
                            ):
                                raise ValueError("A direct PLAN requires frame_id")
                            calls = [
                                {
                                    "name": "execute_path",
                                    "arguments": {
                                        "intent_id": intent_id,
                                        "action": parsed.get("action", "move"),
                                        "frame_id": parsed["frame_id"],
                                        "waypoints": path["points"],
                                        **(
                                            {"speed_mps": parsed["speed_mps"]}
                                            if "speed_mps" in parsed
                                            else {}
                                        ),
                                    },
                                }
                            ]
                    elif decision in ("ACT", "OBSERVE"):
                        calls = parsed.get("mcp_tool_calls", [])
                        if not calls:
                            raise ValueError("Tool calls are required")
                    elif decision == "FAIL":
                        result = call(
                            "stop_robot",
                            {"reason": parsed.get("reason", "AI reported failure")},
                        )
                        return finish(
                            "FAIL",
                            error=parsed.get("error", {"code": "AI_REPORTED_FAILURE"}),
                            reason=parsed.get("reason"),
                            stop_result=result,
                            status="failed",
                        )
                    elif decision == "COMPLETE":
                        # Query the current shared world; never accept completion against an old intent.
                        current = self.mcp._world_provider()
                        if current.get("intent_version") != version:
                            return finish("FAIL", error={"code": "INTENT_CHANGED"})
                        if not self.completion_verified(current):
                            attempt.update(
                                outcome="completion_rejected",
                                error="User goal is not verified",
                            )
                            messages.append(
                                {
                                    "role": "user",
                                    "content": "COMPLETE rejected: user goal is not verified. Observe, continue, or report FAIL.",
                                }
                            )
                            continue
                        stopped = call(
                            "stop_robot", {"reason": "Verified intent completion"}
                        )
                        return finish(
                            "COMPLETE" if stopped.get("ok") else "FAIL",
                            status="completed" if stopped.get("ok") else "failed",
                        )
                    else:
                        raise ValueError(
                            f"Unknown decision {decision!r}; expected PLAN, REPLAN, ACT, OBSERVE, COMPLETE or FAIL"
                        )
                if not isinstance(calls, list) or not 1 <= len(calls) <= 8:
                    raise ValueError("Tool call list must contain 1 to 8 calls")
                # Accept only an unambiguous alias for a discovered tool. Preserve
                # the original response in parsed_decision; never repair path values.
                calls = copy.deepcopy(calls)
                known_tools = {tool["name"] for tool in tools}
                for tc in calls:
                    if not isinstance(tc, dict):
                        raise ValueError(
                            "Each tool call must be a name/arguments object"
                        )
                    if "tool" in tc:
                        alias = tc["tool"]
                        if (
                            not isinstance(alias, str)
                            or alias not in known_tools
                            or ("name" in tc and tc["name"] != alias)
                        ):
                            raise ValueError(
                                "AMBIGUOUS_TOOL_NAME: use name with one available tool name"
                            )
                        tc["name"] = tc.pop("tool")
                        self.audit_event(
                            "tool_name_normalized",
                            {"iteration": iteration + 1, "name": alias},
                        )
                    if (
                        not isinstance(tc.get("name"), str)
                        or tc["name"] not in known_tools
                        or "decision" in tc
                        or "mcp_tool_calls" in tc
                    ):
                        raise ValueError(
                            "INVALID_TOOL_CALL: each entry requires an available name and arguments; nested decisions or call lists are not allowed"
                        )
                names = [tc.get("name") for tc in calls]
                if "plan_path" in names and any(
                    n in names for n in ("execute_path", "rotate_robot")
                ):
                    raise ValueError(
                        "DEPENDENT_TOOL_BATCH: request plan_path alone, inspect its result in a new model turn, then choose motion"
                    )
                for tool_index, tc in enumerate(calls):
                    if not isinstance(tc, dict):
                        raise ValueError(
                            "Each tool call must be a name/arguments object"
                        )
                    args = tc.get("arguments", {})
                    if isinstance(args, str):
                        args = json.loads(args)
                    if not isinstance(args, dict):
                        raise ValueError("Tool arguments must be an object")
                    try:
                        self.progress_policy.check(tc["name"], args)
                    except ValueError as exc:
                        self.audit_event(
                            "progress_policy_rejected",
                            {
                                "iteration": iteration + 1,
                                "tool": tc["name"],
                                "arguments": args,
                                "error": str(exc),
                                "progress_policy": self.progress_policy.snapshot(),
                            },
                        )
                        raise
                    speed_decision = None
                    speed_reason = parsed.get("speed_reason")
                    reason_location = "response.speed_reason"
                    if tc["name"] == "execute_path" and isinstance(args, dict):
                        # Models sometimes attach explanation metadata to the function.
                        # Preserve the original parsed response in audit, but never send
                        # this extra field through the strict MCP argument schema.
                        args = copy.deepcopy(args)
                        nested_reason = args.pop("speed_reason", None)
                        if isinstance(nested_reason, str) and nested_reason.strip():
                            speed_reason = nested_reason
                            reason_location = "arguments.speed_reason"
                    if (
                        tc["name"] == "execute_path"
                        and (intent.get("constraints") or {}).get("speed_mps") is None
                    ):
                        if not isinstance(args, dict):
                            raise ValueError("Tool arguments must be an object")
                        speed, source = select_speed(intent, args.get("speed_mps"))
                        if (
                            not isinstance(speed_reason, str)
                            or not speed_reason.strip()
                        ):
                            raise ValueError(
                                "SPEED_REASON_REQUIRED: include a non-empty top-level speed_reason explaining why the chosen speed_mps fits current observations, alongside execute_path; do not put it in tool arguments"
                            )
                        speed_decision = {
                            "intent_id": intent_id,
                            "intent_version": version,
                            "iteration": iteration + 1,
                            "tool_index": tool_index,
                            "tool": "execute_path",
                            "speed_mps": speed,
                            "speed_source": source,
                            "speed_reason": speed_reason,
                            "reason_source": "model",
                            "reason_location": reason_location,
                            "status": "proposed",
                        }
                        # Buffer before dispatch; the normal scenario finalizer saves once.
                        # Never execute a VLM speed whose explanation could not be recorded.
                        if not self.audit_event("speed_decision", speed_decision):
                            call(
                                "stop_robot",
                                {"reason": "Speed explanation audit unavailable"},
                            )
                            return finish(
                                "FAIL",
                                status="failed",
                                error={"code": "SPEED_AUDIT_UNAVAILABLE"},
                            )
                        speed_decisions.append(speed_decision)
                    result = call(tc["name"], args)
                    if speed_decision is not None:
                        speed_decision.update(
                            status="accepted" if result.get("ok") else "rejected",
                            operation_id=result.get("operation_id"),
                            error=result.get("error"),
                            applied_speed_mps=result.get("speed_mps"),
                            applied_speed_source=result.get("speed_source"),
                        )
                        self.audit_event("speed_decision_result", speed_decision)
                    attempt["outcome"] = (
                        "tool_executed" if result.get("ok") else "tool_rejected"
                    )
                    if native_calls:
                        messages.append(
                            {
                                "role": "tool",
                                "tool_name": tc["name"],
                                "content": prompt_json(result),
                            }
                        )
                    else:
                        # Replace older successful results for an identical query with a marker.
                        # Never erase a rejected action or its collision evidence.
                        for previous in messages[2:]:
                            if previous.get("role") != "user":
                                continue
                            try:
                                old = json.loads(previous["content"])
                            except (ValueError, TypeError):
                                continue
                            if (
                                isinstance(old, dict)
                                and old.get("tool") == tc["name"]
                                and old.get("arguments") == args
                                and old.get("result", {}).get("ok")
                                and tc["name"]
                                in (
                                    "get_robot_state",
                                    "get_detected_objects",
                                    "get_local_obstacles",
                                    "get_map_info",
                                    "get_navigation_status",
                                )
                            ):
                                previous["content"] = prompt_json(
                                    {
                                        "tool": tc["name"],
                                        "superseded_by_later_result": True,
                                    }
                                )
                        messages.append(
                            {
                                "role": "user",
                                "content": prompt_json(
                                    {
                                        "tool": tc["name"],
                                        "arguments": args,
                                        "result": result,
                                    }
                                ),
                            }
                        )
                    if result.get("error") in (
                        "MCP_TRANSPORT_ERROR",
                        "MCP_EXECUTION_UNCERTAIN",
                        "MCP_REQUEST_EXPIRED",
                        "MCP_CONTEXT_REQUIRED",
                    ):
                        return finish("FAIL")
                    if result.get("error") == "INTENT_CHANGED":
                        return finish("FAIL", error={"code": "INTENT_CHANGED"})
                    if result.get("ok") and tc["name"] in (
                        "execute_path",
                        "rotate_robot",
                    ):
                        return finish(
                            decision or "ACT",
                            status="executing",
                            operation_id=result.get("operation_id"),
                            reason=parsed.get("reason"),
                        )
                    # Pausing/canceling an operation is an intermediate tool action.
                    # Returning here strands a pending goal until an unrelated sensor
                    # event, and also discards later calls in the same model response.
                    # Keep the normal bounded reasoning loop and all motion checks.
            except (ValueError, TypeError, KeyError) as exc:
                self.audit_event(
                    "decision_rejected",
                    {"iteration": iteration + 1, "error": str(exc), "content": content},
                )
                attempt.update(outcome="invalid_response", error=str(exc))
                log.warning(
                    "Model response rejected (iteration %s): %s; response=%s",
                    iteration + 1,
                    exc,
                    attempt["response_excerpt"],
                )
                feedback = (
                    f"Invalid decision: {exc}. Return a single JSON object. Allowed decision values: "
                    "PLAN, REPLAN, ACT, OBSERVE, COMPLETE, FAIL. Error codes are not decision values. "
                    "Each mcp_tool_calls entry must be a name/arguments object, without nested decisions."
                )
                if self.progress_policy.enabled and (
                    self.progress_policy.replan_required
                    or not self.progress_policy.initial_plan_ready
                ):
                    target = intent.get("target_location") or {}
                    feedback += (
                        " Planning is required before motion. Request plan_path alone, inspect its result in a new turn, then choose execution or FAIL. Response shape: "
                        + prompt_json(
                            {
                                "decision": (
                                    "REPLAN"
                                    if self.progress_policy.replan_required
                                    else "PLAN"
                                ),
                                "mcp_tool_calls": [
                                    {
                                        "name": "plan_path",
                                        "arguments": {
                                            "goal_x": target.get("x"),
                                            "goal_y": target.get("y"),
                                            "frame_id": target.get("frame_id", "map"),
                                            "safety_margin_m": (
                                                intent.get("constraints") or {}
                                            ).get("safety_margin_m", 0.45),
                                        },
                                    }
                                ],
                            }
                        )
                    )
                messages.append(
                    {
                        "role": "user",
                        "content": feedback,
                    }
                )
        call(
            "stop_robot",
            {"reason": "Reasoning budget exhausted; awaiting new evidence"},
        )
        return finish(
            "FAIL",
            error={
                "code": "REASONING_LIMIT",
                "last_validation_error": (
                    attempts[-1].get("error") if attempts else None
                ),
            },
            status="failed",
        )
