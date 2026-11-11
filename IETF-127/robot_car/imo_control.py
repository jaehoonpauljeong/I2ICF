#!/usr/bin/env python3
"""ROS2 execution boundary. Safety polling and watchdogs never wait for an LLM."""
import copy
import math
import os
import threading
import time
import requests
from flask import Flask, jsonify, request
from common.config import ENV_CFG, ROS_TOPICS, NET_CFG, BIND_HOST, STOP_DISTANCE_M
from common.navigation_utils import normalize_angle, validate_and_score_path
from common.runtime_safety import OccupancyMap, valid_pose, scan_health, finite, apply_tf, no_return_allowed
from robot_car.motion_observation import check_motion

from common.speed_policy import validate_speed
from common.path_progress import remaining_waypoints
from common.goal_completion import GOAL_TOLERANCE_M, goal_evidence, measured_stopped, path_requires_motion

PUBLISH_HZ = 20.0
YAW_TOLERANCE_RAD = 0.06
# Over the 0.5s checked horizon this is <=0.0005 rad (0.029 degrees),
# with sub-millimetre swept displacement at our footprint and speed limits.
SIM_FORWARD_ANGULAR_NOISE_RAD_S = 1e-3


def rotation_command(error):
    """Maintain a modest minimum turn command in Isaac to avoid near-target stalling."""
    speed = min(.8, 1.5*abs(error))
    if ENV_CFG.env_mode == "isaac_sim" and abs(error) > YAW_TOLERANCE_RAD:
        speed = max(.15, speed)
    return math.copysign(speed,error)
state_lock = threading.RLock()
app = Flask(__name__)


class Controller:
    def __init__(self, occupancy_map=None):
        self.lock = threading.RLock()
        self.grid = occupancy_map if occupancy_map is not None else OccupancyMap()
        self.scan = {}
        self.allow_map_blind = os.getenv("ISAAC_ALLOW_MAP_ONLY_REAR_SWEEP","false").lower()=="true"
        self.motion_check = None
        self.events = []
        self.seen_operations = {}
        self.progress_anchor = None
        self.progress_time = time.monotonic()
        self.state = {"status":"stopped", "e_stop":False, "linear_x":0., "angular_z":0.,
                      "safety_stop_code":None,"last_safety_stop":None,
                      "robot_pose":{}, "path_intent_version":None, "current_path":[], "current_wp_index":0, "progress_pct":0.,
                      "failure_code":None, "failure_reason":None, "goal_reached":False,
                      "operation_complete":False, "operation_id":None, "event_sequence":0,
                      "active_intent_id":None, "intent_version":0, "intent_closed":False, "user_speed_mps":None, "requested_speed_mps":None, "speed_source":None, "action":"stop", "rotate_target_yaw":None}

    def event(self, status, code=None):
        self.state.update(status=status, failure_code=code, failure_reason=code,
                          linear_x=0., angular_z=0.)
        self.state["event_sequence"] += 1
        self.events.append({"sequence":self.state["event_sequence"], "operation_id":self.state["operation_id"],
                            "intent_id":self.state["active_intent_id"], "status":status,
                            "code":code, "timestamp":time.time()})
        self.events = self.events[-128:]

    def stop(self, cancel=False, reason=None):
        with self.lock:
            completed = self.state['intent_closed'] and self.state['goal_reached']
            self.event("canceled" if cancel else "stopped")
            self.state.update(action="stop", rotate_target_yaw=None, goal_reached=completed, operation_complete=completed)
            if cancel:
                self.state.update(current_path=[], current_wp_index=0)
            return {"ok":True, "status":self.state["status"], "reason":reason,
                    "e_stop":self.state["e_stop"],"safety_stop_code":self.state["safety_stop_code"]}

    def rotation_coverage(self):
        ranges = self.scan.get("ranges", [])
        mask = self.scan.get("no_return_mask", [])
        allowed = no_return_allowed(self.scan)
        inc = abs(self.scan.get("angle_increment", 0.))
        sectors = []
        for i in range(8):
            start, end = i*len(ranges)//8, (i+1)*len(ranges)//8
            measured = sum(finite(r) and r > 0 for r in ranges[start:end])
            no_return = sum(mask[j] is True and ranges[j] is None for j in range(start,end)) if allowed and len(mask)==len(ranges) else 0
            ratio = (measured+no_return)/(end-start) if end>start else 0.
            sectors.append({"sector":i,"ray_count":end-start,"measured_count":measured,
                            "accepted_no_return_count":no_return,"accepted_ratio":ratio,
                            "coverage_ok":end>start and ratio>=.5})
        full_circle = bool(ranges) and len(ranges)*inc >= 2*math.pi-2*inc
        return {"full_circle":full_circle,"no_return_allowed":allowed,
                "required_sector_ratio":.5,"sectors":sectors,
                "coverage_ok":full_circle and all(s["coverage_ok"] for s in sectors)}

    def safety_error(self, rotating=False):
        if not valid_pose(self.state["robot_pose"]):
            return "LOCALIZATION_INVALID"
        error = scan_health(self.scan)
        if error:
            return error
        if self.scan.get("front_min_m") is None and not no_return_allowed(self.scan):
            return "SCAN_FRONT_UNKNOWN"
        if self.scan.get("front_min_m") is not None and self.scan["front_min_m"] <= STOP_DISTANCE_M:
            return "OBSTACLE_EMERGENCY_STOP"
        if rotating:
            for p in self.scan.get("points", []):
                bp = apply_tf(p["x_laser"], p["y_laser"], self.scan.get("laser_to_base", {}))
                if not bp or math.hypot(bp["x"],bp["y"]) <= STOP_DISTANCE_M:
                    return "ROTATION_BLOCKED"
        return None

    def update_sensors(self, pose, scan):
        with self.lock:
            self.state["robot_pose"] = copy.deepcopy(pose)
            self.scan = copy.deepcopy(scan)
            if self.state["status"] in ("moving", "rotating"):
                result = None
                error = self.safety_error(self.state["status"] == "rotating")
                if not error and self.state["status"] == "moving":
                    result = self.validate(self.state["current_path"][self.state["current_wp_index"]:], self.state["frame_id"])
                    if not result["valid"]: error = result["error_code"]
                if error: self.block(error, validation=result)

    def block(self, error, validation=None):
        if not self.state["e_stop"]:
            self.state["safety_stop_code"] = error
            self.state["last_safety_stop"] = copy.deepcopy({
                "code":error,"timestamp":time.time(),"operation_id":self.state["operation_id"],
                "status_before_stop":self.state["status"],"action":self.state["action"],
                "rotate_target_yaw":self.state["rotate_target_yaw"],
                "angular_error_rad":normalize_angle(self.state["rotate_target_yaw"]-self.state["robot_pose"]["yaw"])
                    if self.state["rotate_target_yaw"] is not None else None,
                "command_before_stop":{"linear_x":self.state["linear_x"],"angular_z":self.state["angular_z"]},
                "progress_clock":"simulation" if self.uses_sim_progress_clock() else "monotonic",
                "progress_elapsed_sec":self.progress_elapsed(),
                "intent_version":self.state["intent_version"],"robot_pose":self.state["robot_pose"],
                "remaining_path":self.state["current_path"][self.state["current_wp_index"]:],
                "frame_id":self.state.get("frame_id"),"validation":validation,
                "scan_stamp_sec":self.scan.get("stamp_sec"),"scan_timestamp":self.scan.get("timestamp"),
                "laser_tf_in_planning_frame":self.scan.get("laser_tf_in_planning_frame"),
                "rejected_ray_counts":self.scan.get("rejected_ray_counts")})
        self.state["e_stop"] = True
        self.event("blocked", error)

    def reset_stop(self):
        with self.lock:
            # Releasing a latch never starts motion; each subsequent command is checked independently.
            error = self.safety_error(rotating=False)
            if error: return {"ok":False,"error":error,"e_stop":self.state["e_stop"],
                              "safety_stop_code":self.state["safety_stop_code"],
                              "rotation_coverage":self.rotation_coverage()}
            self.state["e_stop"] = False
            self.state["safety_stop_code"] = None
            self.event("stopped")
            return {"ok":True,"status":"stopped"}

    def validate(self, points, frame, margin=.45):
        return validate_and_score_path(points, frame, self.state["robot_pose"], [], self.scan.get("points", []),
                                       margin, lidar_data=self.scan, occupancy_map=self.grid)

    def uses_sim_progress_clock(self):
        return (ENV_CFG.env_mode == "isaac_sim" and self.progress_anchor is not None
                and finite(self.progress_anchor.get("stamp_sec"))
                and finite(self.state["robot_pose"].get("stamp_sec")))

    def progress_elapsed(self):
        if self.uses_sim_progress_clock():
            return self.state["robot_pose"]["stamp_sec"]-self.progress_anchor["stamp_sec"]
        return time.monotonic()-self.progress_time

    def set_intent(self, data):
        with self.lock:
            try:
                user_speed = validate_speed(data["speed_mps"]) if data.get("speed_mps") is not None else None
            except ValueError as exc:
                return {"ok":False,"error":"INVALID_SPEED","message":str(exc)}
            key = (data.get("intent_id"), data.get("intent_version"))
            if not isinstance(key[0],str) or not key[0] or not isinstance(key[1],int):
                return {"ok":False,"error":"INVALID_INTENT"}
            if key[1] < self.state["intent_version"] or (key[1] == self.state["intent_version"] and key[0] != self.state["active_intent_id"]):
                return {"ok":False,"error":"INTENT_CHANGED"}
            if key != (self.state["active_intent_id"],self.state["intent_version"]):
                # Keep the paused path for a subsequent resume intent. A new move
                # still replaces it only after full validation.
                target = data.get("target_location") or {}
                path = self.state["current_path"]
                inherit_progress = (data.get("action") == "resume" and bool(path)
                    and target.get("frame_id") == self.state.get("frame_id")
                    and all(finite(target.get(k)) and abs(target[k]-path[-1][k]) <= 1e-6 for k in ("x","y")))
                self.stop(cancel=False, reason="Intent changed")
                self.state.update(active_intent_id=key[0], intent_version=key[1], user_speed_mps=user_speed, intent_closed=False,
                                  goal_reached=False, operation_complete=False, completion_evidence=None)
                if inherit_progress:
                    self.state["path_intent_version"] = key[1]
            return {"ok":True}

    def finish_intent(self, data):
        with self.lock:
            if (data.get("intent_id"), data.get("intent_version")) != (self.state["active_intent_id"], self.state["intent_version"]):
                return {"ok":False,"error":"INTENT_CHANGED"}
            if 'target_location' in data:
                # Atomic, version-fenced verification and closure. Retrying a lost
                # acknowledgement returns the original evidence without reopening.
                if self.state['intent_closed']:
                    if not self.state['goal_reached']:
                        return {'ok':False,'error':'SCENARIO_FINISHED'}
                    evidence = self.state['completion_evidence']
                    if data['target_location'] != evidence['target_location']:
                        return {'ok':False,'error':'COMPLETION_TARGET_CHANGED'}
                    return {'ok':True,'intent_closed':True,'completion_evidence':copy.deepcopy(evidence),
                            'navigation_status':self.snapshot()}
                evidence = goal_evidence(self.state['robot_pose'], data['target_location'])
                if not evidence or not evidence['within_tolerance']:
                    return {'ok':False,'error':'GOAL_NOT_VERIFIED','completion_evidence':evidence}
                if self.state['e_stop']:
                    return {'ok':False,'error':'SAFETY_STOP_LATCHED'}
                self.stop(reason='Verifying final goal arrival')
                if not measured_stopped(self.state['robot_pose']):
                    return {'ok':False,'error':'GOAL_STOPPING'}
                evidence['stopped_verified'] = True
                self.state.update(intent_closed=True, goal_reached=True, operation_complete=True,
                                  current_wp_index=len(self.state["current_path"]), progress_pct=100.,
                                  completion_evidence=copy.deepcopy(evidence))
                self.event('completed')
                return {'ok':True,'intent_closed':True,'completion_evidence':evidence,
                        'navigation_status':self.snapshot()}
            if not self.state["intent_closed"]:
                self.state["intent_closed"] = True
                self.stop(reason="Scenario terminated")
            return {"ok":True,"intent_closed":True}

    def request_fence_error(self, data):
        """Called while holding lock, immediately before a remote mutation."""
        if "expected_intent_version" in data:
            if data["expected_intent_version"] != self.state["intent_version"]:
                return {"ok":False,"error":"INTENT_CHANGED"}
            if self.state["intent_closed"]:
                return {"ok":False,"error":"SCENARIO_FINISHED"}
        if "expires_at" in data and (not finite(data["expires_at"]) or time.time() >= data["expires_at"]):
            return {"ok":False,"error":"MCP_REQUEST_EXPIRED"}
        return None

    def dispatch(self, data, rotate=False):
        with self.lock:
            error = self.request_fence_error(data)
            if error: return error
            if not data.get("intent_id") or (data.get("intent_id"),data.get("intent_version")) != (self.state["active_intent_id"],self.state["intent_version"]):
                return {"ok":False,"error":"INTENT_CHANGED"}
            if self.state["intent_closed"]:
                return {"ok":False,"error":"SCENARIO_FINISHED"}
            op = data.get("operation_id")
            if not isinstance(op,str) or not op: return {"ok":False,"error":"OPERATION_ID_REQUIRED"}
            if op in self.seen_operations: return copy.deepcopy(self.seen_operations[op])
            if self.state["e_stop"]: return {"ok":False,"error":"SAFETY_STOP_LATCHED"}
            error = self.safety_error(rotating=rotate)
            if error: return {"ok":False,"error":error}
            resume = {"matched_route":False, "skipped_waypoint_count":0}
            if rotate:
                angle = data.get("angle_deg")
                if not finite(angle) or abs(angle)>180: return {"ok":False,"error":"INVALID_ANGLE"}
                if abs(math.radians(angle))>YAW_TOLERANCE_RAD:
                    self.motion_check=check_motion(self.scan,self.state["robot_pose"],self.grid,0.,
                                                   rotation_command(math.radians(angle)),self.allow_map_blind)
                    if not self.motion_check["ok"]: return {"ok":False,"error":self.motion_check["error"],"motion_check":self.motion_check}
                self.state.update(rotate_target_yaw=normalize_angle(self.state["robot_pose"]["yaw"]+math.radians(angle)),
                                  action="rotate")
            else:
                try:
                    user_speed = self.state.get("user_speed_mps")
                    speed = validate_speed(user_speed if user_speed is not None else data.get("speed_mps"))
                except ValueError as exc:
                    return {"ok":False,"error":"INVALID_SPEED","message":str(exc)}
                path = data.get("path", {})
                points = path.get("points", [])
                frame = data.get("frame_id")
                if path.get("type") == "relative_motion":
                    distance, angle = path.get("distance_m"), path.get("angle_deg",0.)
                    if not finite(distance) or not finite(angle) or distance < 0:
                        return {"ok":False,"error":"INVALID_RELATIVE_MOTION"}
                    points = [{"x":distance*math.cos(math.radians(angle)),"y":distance*math.sin(math.radians(angle))}]
                    frame = "base_link"
                if data.get("resume_operation_id") is not None and data["resume_operation_id"] != self.state["operation_id"]:
                    return {"ok":False,"error":"RESUME_OPERATION_CHANGED"}
                resume = remaining_waypoints(points, frame, self.state, data["intent_version"])
                if resume["matched_route"] and not resume["points"]:
                    return {"ok":False,"error":"PATH_ALREADY_COMPLETED", "path_progress":resume,
                            "message":"This segment is finished. Verify final goal or plan a new remaining segment."}
                check = self.validate(resume["points"],frame,data.get("safety_margin_m",.45))
                if not check["valid"]: return {"ok":False,"validation":check,"error":check["error_code"]}
                if not path_requires_motion(check["points"], self.state["robot_pose"]):
                    return {"ok":False,"error":"NO_MOTION_REQUIRED","validation":check,
                            "message":"All waypoints are already within arrival tolerance; no operation was started."}
                self.state.update(requested_speed_mps=speed, speed_source="user" if user_speed is not None else "vlm")
                # Keep the original route and measured index when resuming. Validation
                # above and the live watchdog still inspect every remaining segment.
                route = self.state["current_path"] if resume["matched_route"] else check["points"]
                index = resume["resume_index"] if resume["matched_route"] else 0
                self.state.update(current_path=route, frame_id=check["frame_id"], current_wp_index=index,
                                  path_intent_version=data["intent_version"],
                                  action=data.get("action","move"), progress_pct=100*index/len(route), rotate_target_yaw=None)
            self.state.update(operation_id=op, operation_complete=False, goal_reached=False)
            self.progress_anchor = copy.deepcopy(self.state["robot_pose"])
            self.progress_time = time.monotonic()
            self.event("rotating" if rotate else "moving")
            result = {"ok":True,"operation_id":op,"status":self.state["status"],"accepted":True,"speed_mps":self.state.get("requested_speed_mps"),"speed_source":self.state.get("speed_source"), "path_progress":resume}
            self.seen_operations[op] = result
            if len(self.seen_operations)>256: self.seen_operations.pop(next(iter(self.seen_operations)))
            return result

    def tick(self):
        with self.lock:
            s = self.state
            if s["status"] not in ("moving","rotating") or s["e_stop"]:
                s.update(linear_x=0.,angular_z=0.)
                return 0.,0.
            error = self.safety_error(s["status"] == "rotating")
            if error:
                self.block(error)
                return 0.,0.
            pose = s["robot_pose"]
            if self.progress_anchor is not None and self.progress_elapsed() < 0:
                self.block("SIM_CLOCK_REWOUND")
                return 0.,0.
            if s["status"] == "rotating":
                err = normalize_angle(s["rotate_target_yaw"]-pose["yaw"])
                if abs(err) <= YAW_TOLERANCE_RAD:
                    self.event("completed")
                    s.update(operation_complete=True,rotate_target_yaw=None)
                    return 0.,0.
                v,w = 0.,rotation_command(err)
            else:
                path = s["current_path"]
                i = s["current_wp_index"]
                while i < len(path) and math.hypot(path[i]["x"]-pose["x"],path[i]["y"]-pose["y"]) <= GOAL_TOLERANCE_M:
                    i += 1
                s["current_wp_index"] = i
                s["progress_pct"] = 100*i/len(path)
                if i == len(path):
                    self.event("completed")
                    # Segment completion is not user-intent completion.
                    s["operation_complete"] = True
                    return 0.,0.
                dx,dy = path[i]["x"]-pose["x"],path[i]["y"]-pose["y"]
                c,t = math.cos(pose["yaw"]),math.sin(pose["yaw"])
                x,y = c*dx+t*dy,-t*dx+c*dy
                heading = math.atan2(y,x)
                if abs(heading)>math.radians(45):
                    error = self.safety_error(rotating=True)
                    if error:
                        self.block(error)
                        return 0.,0.
                    v,w = 0.,rotation_command(heading)
                else:
                    v = s["requested_speed_mps"]
                    w = max(-.8,min(.8,v*2*y/max(.04,x*x+y*y)))
            # Snap numerical drift in a forward command itself, before validating/publishing.
            # Do not classify an actually curved or in-place turn as a straight sweep.
            raw_w=w
            if ENV_CFG.env_mode == "isaac_sim" and v>0 and abs(w)<=SIM_FORWARD_ANGULAR_NOISE_RAD_S:
                w=0.
            self.motion_check=check_motion(self.scan,pose,self.grid,v,w,self.allow_map_blind)
            self.motion_check["candidate_command"]={"linear_x":v,"angular_z":w,"raw_angular_z":raw_w}
            if not self.motion_check["ok"]:
                self.block(self.motion_check["error"],validation=self.motion_check)
                return 0.,0.
            if self.motion_check.get("map_only_blind_ray_count"):
                # Slow both velocities together, retaining the checked arc curvature.
                factor=min(1.,.1/abs(v) if v else 1.,.15/abs(w) if w else 1.)
                v,w=v*factor,w*factor
            # Completion above takes priority; slow simulation is not a stuck robot.
            # Sensor freshness remains checked against wall time in safety_error.
            if self.progress_anchor is not None:
                moved = math.hypot(pose["x"]-self.progress_anchor["x"],pose["y"]-self.progress_anchor["y"]) > .03
                turned = abs(normalize_angle(pose["yaw"]-self.progress_anchor["yaw"])) > .03
                if moved or turned:
                    self.progress_anchor=copy.deepcopy(pose)
                    self.progress_time=time.monotonic()
                elif self.progress_elapsed() > 10.:
                    self.block("PROGRESS_TIMEOUT")
                    return 0.,0.
            s.update(linear_x=v,angular_z=w)
            return v,w

    def snapshot(self):
        with self.lock: return {**copy.deepcopy(self.state),"events":copy.deepcopy(self.events),
                               "remaining_path":copy.deepcopy(self.state["current_path"][self.state["current_wp_index"]:]),
                               "rotation_coverage":self.rotation_coverage(),
                               "motion_check":copy.deepcopy(self.motion_check),
                               "motion_policy":{"reverse_allowed":False,
                                   "allow_map_only_rear_sweep":self.allow_map_blind and ENV_CFG.env_mode=="isaac_sim"}}


controller = Controller()


def reply(result):
    return jsonify(result), 200 if result.get("ok") else 409


@app.post("/control/intent")
def set_intent(): return reply(controller.set_intent(request.get_json()))


@app.post("/control/finish")
def finish_intent(): return reply(controller.finish_intent(request.get_json()))


@app.post("/control/execute_path")
def execute_path(): return reply(controller.dispatch(request.get_json()))


@app.post("/control/rotate")
def rotate_robot(): return reply(controller.dispatch(request.get_json(),rotate=True))


@app.post("/control/stop")
def stop_robot():
    data=request.get_json(silent=True) or {}
    with controller.lock:
        error = controller.request_fence_error(data)
        if error: return reply(error)
        if "intent_version" in data:
            fenced=controller.set_intent(data)
            if not fenced.get("ok"): return reply(fenced)
        return reply(controller.stop(reason=data.get("reason")))


@app.post("/control/cancel")
def cancel_navigation():
    data = request.get_json(silent=True) or {}
    with controller.lock:
        error = controller.request_fence_error(data)
        return reply(error or controller.stop(cancel=True, reason=data.get("reason")))


@app.post("/control/reset_stop")
def reset_stop():
    data = request.get_json(silent=True) or {}
    with controller.lock:
        error = controller.request_fence_error(data)
        return reply(error or controller.reset_stop())


@app.post("/control/distance")
def control_distance():
    data = request.get_json()
    d = data.get("distance")
    with controller.lock:
        if not finite(d) or d <= 0: controller.block("EXTERNAL_DISTANCE_UNKNOWN")
        elif d <= STOP_DISTANCE_M: controller.block("OBSTACLE_EMERGENCY_STOP")
    # A distance report can assert a stop, never clear it.
    return jsonify({"ok":True,"e_stop":controller.state["e_stop"]})


@app.post("/control/pose_update")
@app.post("/control/drive")
@app.post("/control/cmd_vel")
def reject_unvalidated_command():
    return jsonify({"ok":False,"error":"USE_VALIDATED_EXECUTION_API"}),410


@app.get("/status")
@app.get("/navigation/status")
def get_navigation_status(): return jsonify(controller.snapshot())


def poll_sensors():
    while True:
        try:
            p = requests.get(f"{NET_CFG.sensor_base_url}/odometry",timeout=.25)
            l = requests.get(f"{NET_CFG.sensor_base_url}/lidar",timeout=.25)
            p.raise_for_status(); l.raise_for_status()
            controller.update_sensors(p.json(),l.json())
        except (requests.RequestException, ValueError):
            # Old acquisition timestamps remain unchanged; tick watchdog stops motion.
            pass
        time.sleep(.05)


def run_ros_controller():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    rclpy.init()
    node = Node("ietf127_controller")
    pub = node.create_publisher(Twist,ROS_TOPICS.cmd_vel_topic,10)
    def tick():
        v,w = controller.tick()
        msg = Twist(); msg.linear.x=v; msg.angular.z=w
        pub.publish(msg)
    node.create_timer(1/PUBLISH_HZ,tick)
    try: rclpy.spin(node)
    finally:
        pub.publish(Twist())
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    import rclpy  # No silent physical/kinematic backend substitution.
    threading.Thread(target=poll_sensors,daemon=True).start()
    threading.Thread(target=run_ros_controller,daemon=True).start()
    app.run(host=BIND_HOST,port=NET_CFG.control_port,threaded=True)
