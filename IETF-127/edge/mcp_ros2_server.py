#!/usr/bin/env python3
"""Common ROS2 capabilities. In-process and standard MCP clients share Edge world state."""
import asyncio
import copy
import json
import math
import sys
import time
import threading
import uuid
import requests
from jsonschema import validate, ValidationError
from common.config import ENV_CFG, MAP_CFG, NET_CFG, BIND_HOST, WORLD_URL, MIN_CLEARANCE_M, EDGE_BASE_URL
from common.navigation_utils import validate_and_score_path, generate_candidate_detour_path, goal_geometry
from common.runtime_safety import OccupancyMap, fresh, valid_pose, scan_health, finite

from edge.route_memory import RouteMemory
from common.path_progress import remaining_waypoints
from common.goal_completion import GOAL_TOLERANCE_M, path_requires_motion
from common.speed_policy import SPEED_SCHEMA, select_speed

NUMBER = {"type":"number"}
STRING = {"type":"string"}
POINTS = {"type":"array","minItems":1,"maxItems":128,"items":{"type":"object","properties":{"x":NUMBER,"y":NUMBER},"required":["x","y"],"additionalProperties":False}}
FRAME = {"type":"string","enum":["map","odom","base_link"]}


def tool(name, description, properties=None, required=None):
    return {"name":name,"description":description,"inputSchema":{"type":"object","properties":properties or {},
            "required":required or [],"additionalProperties":False}}


TOOLS = [
    tool("get_robot_state","Get measured robot pose, freshness, frame and navigation status."),
    tool("get_detected_objects","Get fresh camera/LiDAR objects with scan-time positions.",
         {"target_class":STRING,"max_distance_m":{"type":"number","minimum":0}}),
    tool("get_map_info","Get actual loaded-map availability and measured named locations."),
    tool("get_local_obstacles","Get scan health and nearby obstacle points in laser frame.",
         {"danger_radius_m":{"type":"number","minimum":0,"maximum":12}}),
    tool("plan_path","Optionally request a geometric candidate. The AI decides whether to use it; no execution occurs.",
         {"goal_x":NUMBER,"goal_y":NUMBER,"frame_id":FRAME,"safety_margin_m":{"type":"number","minimum":MIN_CLEARANCE_M}},["goal_x","goal_y"]),
    tool("execute_path","Validate and start one waypoint segment. Each waypoint contains only x and y, never yaw. Read plan_path results before choosing a path; do not send the current position as a placeholder. Returns an operation_id, not intent completion.",
         {"intent_id":STRING,"action":{"type":"string","enum":["move","detour","approach","resume"]},"frame_id":FRAME,"waypoints":POINTS,"speed_mps":SPEED_SCHEMA},["frame_id","waypoints"]),
    tool("stop_robot","Immediately pause motion. Always allowed, including during move intents.",{"reason":STRING}),
    tool("rotate_robot","Observation-only rotation: for pose goals specify purpose=observe and reason describing missing information. execute_path aligns automatically. Swept-footprint checks apply. Rear blind regions are not live observations. Completion requires another AI decision.",
         {"angle_deg":{"type":"number","minimum":-180,"maximum":180},
          "purpose":{"type":"string","enum":["observe"]},"reason":STRING},["angle_deg"]),
    tool("cancel_navigation","Cancel the current operation and stop motion. Does NOT clear a safety stop latch.",{"reason":STRING}),
    tool("get_navigation_status","Get operation status and sequenced completion/failure events."),
    tool("reset_safety_stop","Release the latch while remaining stopped if current localization and forward safety checks pass. Does not certify rear clearance or resume motion; every motion has independent checks. Cancel or a new intent cannot clear the latch.",{"reason":STRING}),
    tool("get_execution_report","Read measured scenario status at Edge. Never changes motion. Completion time/success are null while running.",
         {"intent_id":STRING,"intent_version":{"type":"integer","minimum":1}}),
]


def finite_tree(value):
    if isinstance(value,float): return math.isfinite(value)
    if isinstance(value,dict): return all(finite_tree(v) for v in value.values())
    if isinstance(value,list): return all(finite_tree(v) for v in value)
    return True


class MCPToolRegistry:
    def __init__(self, shared_world_state_provider=None, occupancy_map=None, report_provider=None):
        self.route_memory = RouteMemory()
        self._report_provider = report_provider
        self._world_provider = shared_world_state_provider or self._remote_world
        self.grid = occupancy_map if occupancy_map is not None else OccupancyMap()

    def _remote_world(self):
        response = requests.get(WORLD_URL,timeout=.6)
        response.raise_for_status()
        return response.json()

    def list_tools(self): return copy.deepcopy(TOOLS)

    def get_ollama_tools_schema(self):
        return [{"type":"function","function":{"name":t["name"],"description":t["description"],"parameters":t["inputSchema"]}} for t in TOOLS]

    def _control(self, endpoint, data=None):
        response = requests.post(f"{NET_CFG.control_base_url}/control/{endpoint}",json=data or {},timeout=1.)
        result = response.json()
        if response.status_code != 200: result["ok"] = False
        if endpoint in ("stop", "cancel") and result.get("ok"):
            result["operation_only"] = True
            result["continuation"] = "Motion is stopped. For a pending movement intent, continue planning or explicitly report FAIL; stopping alone does not complete the goal."
        if result.get("error") == "SAFETY_STOP_LATCHED" or (
                endpoint in ("stop", "cancel") and result.get("ok") and result.get("e_stop")):
            result["recovery"] = {"tool":"reset_safety_stop", "automatic":False,
                "instruction":"If current safety checks allow recovery, explicitly request reset_safety_stop, then resubmit the validated detour. Reset never starts motion and does not clear route constraints."}
        return result

    def call_tool(self, name, arguments=None, *, expected_intent_version=None, expires_at=None):
        args = arguments or {}
        fence = {}
        if expected_intent_version is not None:
            fence["expected_intent_version"] = expected_intent_version
        if expires_at is not None:
            if not finite(expires_at) or time.time() >= expires_at:
                return {"ok":False,"error":"MCP_REQUEST_EXPIRED"}
            fence["expires_at"] = expires_at
        spec = next((t for t in TOOLS if t["name"]==name),None)
        if spec is None: return {"ok":False,"error":"UNKNOWN_TOOL"}
        try:
            validate(args,spec["inputSchema"])
            if not finite_tree(args): raise ValueError("Non-finite input")
            if name == "get_execution_report":
                if self._report_provider:
                    return self._report_provider(args.get("intent_id"), args.get("intent_version"))
                response = requests.get(EDGE_BASE_URL + "/reports", params=args, timeout=1.)
                return response.json()
            # Stops remain available even when Edge, map, TF or sensors are offline.
            if name in ("stop_robot","cancel_navigation","reset_safety_stop"):
                return self._control({"stop_robot":"stop","cancel_navigation":"cancel","reset_safety_stop":"reset_stop"}[name],{**args,**fence})
            if name == "get_navigation_status":
                resp = requests.get(f"{NET_CFG.control_base_url}/navigation/status",timeout=.6)
                resp.raise_for_status()
                return {"ok":True,"navigation_status":resp.json()}
            if name == "get_map_info":
                return {"ok":True,**self.grid.info(),"map_available":bool(ENV_CFG.map_available and self.grid.available),"map_requested":ENV_CFG.map_available,
                        "planning_frame":ENV_CFG.planning_frame,"named_locations":MAP_CFG.named_locations if self.grid.available else {},
                        "occupancy_overview":self.grid.preview() if ENV_CFG.map_available else None}
            world = self._world_provider()
            if not fresh(world): return {"ok":False,"error":"WORLD_STATE_STALE"}
            constraints = self.route_memory.snapshot(world)
            for item in world.get("route_constraints", []):
                if item not in constraints:
                    constraints.append(item)
            robot = world.get("robot_state",{})
            scan = world.get("raw_lidar",{})
            objects = [o for o in world.get("environment",{}).get("objects",[]) if fresh({"timestamp":o.get("observed_at")})]
            if name == "get_robot_state":
                return {"ok":valid_pose(robot),"robot_state":robot,"goal_geometry":goal_geometry(robot,(world.get("intent") or {}).get("target_location"))}
            if name == "get_detected_objects":
                selected = [o for o in objects if (not args.get("target_class") or o.get("class")==args["target_class"])
                            and ("max_distance_m" not in args or finite(o.get("distance_m")) and o["distance_m"]<=args["max_distance_m"])]
                return {"ok":True,"objects":selected,"perception":world.get("perception",{})}
            if name == "get_local_obstacles":
                return {"ok":scan_health(scan) is None,"error":scan_health(scan),"frame_id":scan.get("frame_id"),
                        "timestamp":scan.get("timestamp"),"front_min_m":scan.get("front_min_m"),
                        "query_radius_m":args.get("danger_radius_m",2.5),
                        "route_constraints":constraints,
                        "scope_warning":"Empty points only describes this radius; unresolved route constraints remain outside it.",
                        "valid_ray_count":scan.get("valid_ray_count",0),"points":[p for p in scan.get("points",[]) if p["distance_m"]<=args.get("danger_radius_m",2.5)]}
            version = world.get("intent_version")
            if expected_intent_version is not None and version != expected_intent_version:
                return {"ok":False,"error":"INTENT_CHANGED"}
            if world.get("scenario_terminal"):
                return {"ok":False,"error":"SCENARIO_FINISHED"}
            intent = world.get("intent") or {}
            intent_id = intent.get("id") or intent.get("intent_id")
            if not intent_id or intent.get("action")=="stop":
                return {"ok":False,"error":"NO_MOVEMENT_INTENT"}
            if args.get("intent_id",intent_id) != intent_id:
                return {"ok":False,"error":"INTENT_CHANGED"}
            if name in ("execute_path","rotate_robot") and not world.get("perception",{}).get("ready"):
                return {"ok":False,"error":"PERCEPTION_UNAVAILABLE"}
            if not valid_pose(robot): return {"ok":False,"error":"LOCALIZATION_INVALID"}
            if scan_health(scan): return {"ok":False,"error":scan_health(scan)}
            if name == "rotate_robot":
                if abs(args["angle_deg"]) <= math.degrees(.06):
                    return {"ok":False,"error":"ROTATION_BELOW_TOLERANCE",
                            "message":"No useful rotation. execute_path aligns to its next waypoint automatically."}
                geometry = goal_geometry(robot, intent.get("target_location"))
                if intent.get("action") in ("move","detour","resume") and geometry:
                    if geometry["distance_m"] <= GOAL_TOLERANCE_M:
                        return {"ok":False,"error":"GOAL_ALREADY_IN_TOLERANCE","goal_geometry":geometry,
                                "message":"Wait for measured goal verification; final yaw is not required."}
                    if args.get("purpose") != "observe" or not (args.get("reason") or "").strip():
                        return {"ok":False,"error":"ROTATION_JUSTIFICATION_REQUIRED","goal_geometry":geometry,
                                "message":"execute_path aligns automatically. Plan/execute the remaining path. Only rotate for new observations: purpose=observe and reason must describe the missing information, not heading alignment."}
                return self._control("rotate",{**args,**fence,"intent_id":intent_id,"intent_version":version,"operation_id":str(uuid.uuid4())})
            margin = max(args.get("safety_margin_m",.45),intent.get("constraints",{}).get("safety_margin_m",.45))
            frame = args.get("frame_id",robot["frame_id"])
            audit_inputs={"requested_arguments":args,"intent":intent,"intent_version":version,
                          "effective_safety_margin_m":margin,"margin_rule":"max(requested_or_0.45, intent_or_0.45)",
                          "robot_state":robot,"detected_objects":objects,"lidar":scan,"route_constraints":constraints,
                          "map":self.grid.info() if hasattr(self.grid,"info") else {"enabled":ENV_CFG.map_available}}
            if name == "plan_path":
                result = generate_candidate_detour_path(0. if frame=="base_link" else robot["x"],0. if frame=="base_link" else robot["y"],
                           args["goal_x"],args["goal_y"],frame,robot,objects,scan.get("points",[]),margin,
                           lidar_data=scan,occupancy_map=self.grid if ENV_CFG.map_available else None,route_constraints=constraints)
                result.setdefault("audit_details",{})["inputs"]=audit_inputs
                return {"ok":result["success"],**result}
            resume = remaining_waypoints(args["waypoints"], frame, world.get("navigation",{}), version)
            audit_inputs["path_progress"] = resume
            if resume["matched_route"] and not resume["points"]:
                return {"ok":False,"error":"PATH_ALREADY_COMPLETED","path_progress":resume,
                        "message":"Verify the final goal or plan a new segment from the current pose.","audit_details":audit_inputs}
            check = validate_and_score_path(resume["points"],frame,robot,objects,scan.get("points",[]),margin,
                                           lidar_data=scan,occupancy_map=self.grid if ENV_CFG.map_available else None,route_constraints=constraints)
            if not check["valid"]:
                self.route_memory.record(check,robot["frame_id"],("tool",scan.get("stamp_sec"),str(args["waypoints"])),scan.get("stamp_sec"))
                return {"ok":False,"error":check["error_code"],"validation":check,"audit_details":audit_inputs,
                        "recovery":{"instruction":"Revise the path around the reported collision or request plan_path. A short local scan cannot clear this route constraint.",
                                    "route_constraints":self.route_memory.snapshot(world)}}
            if not path_requires_motion(check["points"], robot):
                return {"ok":False,"error":"NO_MOTION_REQUIRED", "validation":check,
                        "goal_geometry":goal_geometry(robot, intent.get("target_location")),
                        "message":"Every waypoint is already within arrival tolerance of the measured current pose. No motion was started and the final goal is not declared reached. Use the returned plan_path candidate or plan a remaining segment; never substitute the start pose.",
                        "audit_details":audit_inputs}
            # Enforce target standoff even when a model proposes a closer path.
            if intent.get("action")=="approach":
                stop = intent.get("constraints",{}).get("stop_distance_m",1.)
                targets = [o for o in objects if o.get("class")==intent.get("target_object")
                           and fresh(o.get("position",{})) and o["position"].get("valid")
                           and o["position"].get("frame_id")==robot["frame_id"]]
                if not targets: return {"ok":False,"error":"TARGET_NOT_LOCALIZED"}
                previous = robot
                for endpoint in check["points"]:
                    dx,dy = endpoint["x"]-previous["x"],endpoint["y"]-previous["y"]
                    length2 = dx*dx+dy*dy
                    for target in targets:
                        position = target["position"]
                        projection = ((position["x"]-previous["x"])*dx+(position["y"]-previous["y"])*dy)/length2 if length2 else 0.
                        ratio = max(0.,min(1.,projection))
                        distance = math.hypot(previous["x"]+ratio*dx-position["x"],previous["y"]+ratio*dy-position["y"])
                        if distance < stop:
                            return {"ok":False,"error":"APPROACH_STANDOFF_VIOLATION"}
                    previous = endpoint
            speed, speed_source = select_speed(intent, args.get("speed_mps"))
            control_request={**fence,"intent_id":intent_id,"intent_version":version,
                        "operation_id":str(uuid.uuid4()),"action":args.get("action","move"),"frame_id":check["frame_id"],
                        "safety_margin_m":margin,"speed_mps":speed,"speed_source":speed_source,"path":{"type":"waypoints","points":check["points"]}}
            if resume["matched_route"]:
                control_request["resume_operation_id"] = resume["source_operation_id"]
            result = self._control("execute_path",control_request)
            return {**result,"validation":check,"audit_details":{**audit_inputs,"controller_request":control_request}}
        except requests.RequestException as exc:
            uncertain = name in ("execute_path", "rotate_robot", "stop_robot", "cancel_navigation", "reset_safety_stop")
            return {"ok":False,"error":"MCP_EXECUTION_UNCERTAIN" if uncertain else "TOOL_ERROR",
                    "message":str(exc), "execution_uncertain":uncertain}
        except (ValueError, TypeError, KeyError, ValidationError) as exc:
            return {"ok":False,"error":"TOOL_ERROR","message":str(exc)}


def create_sdk_server(registry=None):
    """Official SDK owns protocol negotiation, notifications, schema errors and transports."""
    from mcp.server.fastmcp import FastMCP, Context
    from mcp.types import CallToolResult, TextContent
    class ContractMCP(FastMCP):
        async def list_tools(self):
            # Python dict[str,float] annotations lose x/y-only constraints. Publish
            # the exact contract enforced by the registry, without $ref indirection.
            contracts = {t["name"]:t["inputSchema"] for t in TOOLS}
            return [t.model_copy(update={"inputSchema":copy.deepcopy(contracts[t.name])})
                    for t in await super().list_tools()]

    server = ContractMCP("ietf127-ros2",host=BIND_HOST,port=NET_CFG.mcp_port,stateless_http=True,json_response=True)
    registry = registry or MCPToolRegistry()
    invocation_lock = threading.RLock()
    mutating = {"execute_path", "rotate_robot", "stop_robot", "cancel_navigation", "reset_safety_stop", "plan_path"}
    def invoke_sync(name,args,ctx):
        meta = ctx.request_context.meta
        data = meta.model_dump().get("ietf127", {}) if meta else {}
        if not isinstance(data, dict):
            data = {}
        version = data.get("intent_version")
        expiry = data.get("expires_at")
        if name in mutating and (type(version) is not int or version < 1
                                 or not isinstance(data.get("correlation_id"), str)
                                 or not finite(expiry)):
            result = {"ok":False, "error":"MCP_CONTEXT_REQUIRED"}
        else:
            with invocation_lock:
                result = registry.call_tool(name,{k:v for k,v in args.items() if k != "ctx" and v is not None},
                    expected_intent_version=version, expires_at=expiry)
        result["mcp_server"] = {"request_id":ctx.request_context.request_id,
                                "correlation_id":data.get("correlation_id"), "intent_version":version}
        return CallToolResult(content=[TextContent(type="text",text=json.dumps(result,allow_nan=False))],isError=not result.get("ok",False))
    async def invoke(name,args,ctx):
        # HTTP world/controller reads and geometric checks must not block the MCP loop.
        return await asyncio.to_thread(invoke_sync, name, args, ctx)
    @server.tool(description=TOOLS[0]["description"])
    async def get_robot_state(ctx: Context): return await invoke("get_robot_state",{},ctx)
    @server.tool(description=TOOLS[1]["description"])
    async def get_detected_objects(ctx: Context, target_class: str | None=None,max_distance_m: float | None=None):
        return await invoke("get_detected_objects",{"target_class":target_class,"max_distance_m":max_distance_m},ctx)
    @server.tool(description=TOOLS[2]["description"])
    async def get_map_info(ctx: Context): return await invoke("get_map_info",{},ctx)
    @server.tool(description=TOOLS[3]["description"])
    async def get_local_obstacles(ctx: Context, danger_radius_m: float=2.5): return await invoke("get_local_obstacles",{"danger_radius_m":danger_radius_m},ctx)
    @server.tool(description=TOOLS[4]["description"])
    async def plan_path(ctx: Context, goal_x: float,goal_y: float,frame_id: str | None=None,safety_margin_m: float=.45): return await invoke("plan_path",{"goal_x":goal_x,"goal_y":goal_y,"frame_id":frame_id,"safety_margin_m":safety_margin_m},ctx)
    @server.tool(description=TOOLS[5]["description"])
    async def execute_path(ctx: Context, frame_id: str,waypoints: list[dict[str,float]],intent_id: str | None=None,action: str="move",speed_mps: float | None=None):
        return await invoke("execute_path",{"frame_id":frame_id,"waypoints":waypoints,"intent_id":intent_id,"action":action,"speed_mps":speed_mps},ctx)
    @server.tool(description=TOOLS[6]["description"])
    async def stop_robot(ctx: Context, reason: str="AI requested stop"): return await invoke("stop_robot",{"reason":reason},ctx)
    @server.tool(description=TOOLS[7]["description"])
    async def rotate_robot(ctx: Context, angle_deg: float, purpose: str | None=None, reason: str | None=None):
        return await invoke("rotate_robot",{"angle_deg":angle_deg,"purpose":purpose,"reason":reason},ctx)
    @server.tool(description=TOOLS[8]["description"])
    async def cancel_navigation(ctx: Context, reason: str="AI canceled navigation"): return await invoke("cancel_navigation",{"reason":reason},ctx)
    @server.tool(description=TOOLS[9]["description"])
    async def get_navigation_status(ctx: Context): return await invoke("get_navigation_status",{},ctx)
    @server.tool(description=TOOLS[10]["description"])
    async def reset_safety_stop(ctx: Context, reason: str | None=None): return await invoke("reset_safety_stop",{"reason":reason},ctx)
    @server.tool(description=TOOLS[11]["description"])
    async def get_execution_report(ctx: Context, intent_id: str | None=None,intent_version: int | None=None):
        return await invoke("get_execution_report",{"intent_id":intent_id,"intent_version":intent_version},ctx)
    return server


if __name__ == "__main__":
    create_sdk_server().run(transport="stdio" if "--stdio" in sys.argv else "streamable-http")
