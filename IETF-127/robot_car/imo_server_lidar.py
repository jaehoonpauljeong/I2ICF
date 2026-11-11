#!/usr/bin/env python3
"""ROS2 snapshot server. Exact stamped TF; immutable latest snapshots for all readers."""
import base64
import copy
import math
import os
import logging
import threading
import time
from flask import Flask, Response, jsonify
from common.config import ENV_CFG, LIDAR_CFG, ROS_TOPICS, NET_CFG, BIND_HOST
from common.navigation_utils import is_valid_lidar_range, is_self_detection, yaw_from_quaternion
from common.runtime_safety import apply_tf, fresh

app = Flask(__name__)
state_lock = threading.RLock()
latest = {"odometry": {}, "lidar": {}, "camera": {}}
worker_state = {"status": "not_started", "error": None}
received_messages = {}
CAMERA_RELIABILITY = os.getenv("CAMERA_RELIABILITY", "reliable" if ENV_CFG.env_mode == "isaac_sim" else "best_effort").lower()


def mark_received(name):
    with state_lock:
        entry = received_messages.setdefault(name, {"count": 0, "last_received_at": 0.})
        entry["count"] += 1
        entry["last_received_at"] = time.time()


def run_sensor_worker():
    with state_lock:
        worker_state.update(status="starting", error=None)
    try:
        ros_sensor_process()
        with state_lock:
            worker_state.update(status="stopped")
    except Exception as exc:
        logging.exception("ROS sensor worker failed")
        with state_lock:
            worker_state.update(status="failed", error=f"{type(exc).__name__}: {exc}")
    finally:
        with state_lock:
            for name in latest:
                latest[name] = {"valid": False, "timestamp": 0., "error": "SENSOR_WORKER_STOPPED"}



def store(name, value):
    with state_lock:
        previous = latest[name]
        if "stamp_sec" in value and "stamp_sec" in previous and value["stamp_sec"] <= previous["stamp_sec"]:
            return  # Repeated/out-of-order ROS messages cannot renew freshness.
        latest[name] = copy.deepcopy(value)


def snapshot(name):
    with state_lock:
        return copy.deepcopy(latest[name])


def camera_calibration(info, width, height, frame):
    """Accept only finite, undistorted intrinsics for this exact image geometry."""
    if not info or (info["width"], info["height"], info["frame_id"]) != (width, height, frame):
        return None
    k, distortion = info["k"], info["d"]
    if len(k) != 9 or not all(math.isfinite(v) for v in [*k, *distortion]):
        return None
    if k[0] <= 0 or k[4] <= 0 or any(abs(v) > 1e-9 for v in distortion):
        return None
    # This projection supports the standard pinhole K, without skew.
    if any(abs(k[i]) > 1e-9 for i in (1, 3, 6, 7)) or abs(k[8]-1.) > 1e-9:
        return None
    return {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}


def make_scan(ranges, angle_min, angle_increment, range_min, range_max,
              source_frame, stamp_sec, timestamp, laser_tf, base_tf):
    """Pure scan conversion used by ROS and replay tests. Unknown returns remain unknown."""
    filtered, points, front, front_valid = [], [], 0, 0
    front_min = None
    no_return_mask = []
    self_detection_mask = []
    front_no_return = 0
    rejected = {"sensor_invalid": 0, "configured_range": 0, "tf_invalid": 0, "self_detection": 0}
    front_rejected = dict.fromkeys(rejected, 0)
    transform_valid = (fresh(laser_tf) and fresh(base_tf) and laser_tf.get("valid")
                       and base_tf.get("valid") and laser_tf.get("source_frame") == source_frame
                       and base_tf.get("source_frame") == source_frame and base_tf.get("frame_id") == ENV_CFG.base_frame)
    valid_geometry = all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max)) and abs(angle_increment) > 1e-9
    for i, raw in enumerate(ranges):
        angle = angle_min + i*angle_increment
        # Direction is expressed in base_link, including the sensor's actual yaw.
        base_angle = math.atan2(math.sin(angle+base_tf.get("yaw", 0.)), math.cos(angle+base_tf.get("yaw", 0.)))
        in_front = abs(base_angle) <= math.radians(30)
        front += int(in_front)
        distance = float(raw)
        no_return = ENV_CFG.env_mode == "isaac_sim" and distance == -1.0
        no_return_mask.append(no_return)
        if in_front and no_return:
            front_no_return += 1
        reason = None
        bp = None
        if not math.isfinite(distance) or distance <= 0 or not range_min <= distance < range_max:
            reason = "sensor_invalid"
        elif not is_valid_lidar_range(distance):
            reason = "configured_range"
        elif not transform_valid:
            reason = "tf_invalid"
        else:
            bp = apply_tf(distance*math.cos(angle), distance*math.sin(angle), base_tf)
            if not bp:
                reason = "tf_invalid"
            elif is_self_detection(bp["x"], bp["y"]):
                reason = "self_detection"
        self_detection_mask.append(reason == "self_detection")
        if reason:
            rejected[reason] += 1
            if in_front: front_rejected[reason] += 1
            filtered.append(None)
            continue
        filtered.append(distance)
        points.append({"x_laser": distance*math.cos(angle), "y_laser": distance*math.sin(angle),
                       "distance_m": distance, "angle_deg": math.degrees(angle)})
        if in_front:
            front_valid += 1
            d = math.hypot(bp["x"], bp["y"])
            front_min = d if front_min is None else min(front_min, d)
    return {"timestamp": timestamp, "stamp_sec": stamp_sec, "frame_id": source_frame,
            "valid": bool(transform_valid and valid_geometry), "unit": "m", "ranges": filtered,
            "angle_min": angle_min, "angle_increment": angle_increment,
            "range_min": range_min, "range_max": range_max,
            "valid_ray_count": len(points), "total_ray_count": len(ranges),
            "no_return_mask": no_return_mask, "no_return_ray_count": sum(no_return_mask),
            "self_detection_mask":self_detection_mask,
            "front_no_return_ray_count": front_no_return,
            "observation_state": "NO_RETURN" if not points and any(no_return_mask) else "RETURNS_OR_INVALID",
            "no_return_policy": "isaac_minus_one" if LIDAR_CFG.allow_no_return and ENV_CFG.env_mode == "isaac_sim" else "strict",
            "front_rejected_ray_counts": front_rejected, "rejected_ray_counts": rejected, "front_ray_count": front, "front_valid_ray_count": front_valid,
            "configured_max_range_m": LIDAR_CFG.max_valid_range_m,
            "front_observed": front > 0 and front_valid/front >= 0.5,
            "front_min_m": front_min, "points": points,
            "laser_tf_in_planning_frame": laser_tf, "laser_to_base": base_tf}


def ros_sensor_process():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan, CameraInfo
    from tf2_ros import Buffer, TransformListener, TransformException

    class Sensors(Node):
        def __init__(self):
            super().__init__("ietf127_sensors", parameter_overrides=[Parameter("use_sim_time", value=ENV_CFG.env_mode == "isaac_sim")])
            self.buffer = Buffer(node=self)
            self.listener = TransformListener(self.buffer, self)
            # Different sensors must not block one another or /clock during stamped TF waits.
            self.sensor_groups = [MutuallyExclusiveCallbackGroup() for _ in range(4)]
            if CAMERA_RELIABILITY not in ("reliable", "best_effort"):
                raise ValueError("CAMERA_RELIABILITY must be reliable or best_effort")
            camera_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=5,
                durability=DurabilityPolicy.VOLATILE,
                reliability=(ReliabilityPolicy.RELIABLE if CAMERA_RELIABILITY == "reliable"
                             else ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(f"RGB subscription: {ROS_TOPICS.camera_topic}, reliability={CAMERA_RELIABILITY}")
            self.create_subscription(Odometry, ROS_TOPICS.odom_topic, self.odom, qos_profile_sensor_data, callback_group=self.sensor_groups[0])
            self.create_subscription(LaserScan, ROS_TOPICS.scan_topic, self.scan, qos_profile_sensor_data, callback_group=self.sensor_groups[1])
            self.create_subscription(Image, ROS_TOPICS.camera_topic, self.camera, camera_qos, callback_group=self.sensor_groups[2])
            self.calibration = None
            self.create_subscription(CameraInfo, ROS_TOPICS.camera_info_topic, self.camera_info, qos_profile_sensor_data, callback_group=self.sensor_groups[3])

        def observation_time(self, stamp):
            sensor_time = stamp.sec + stamp.nanosec*1e-9
            ros_now = self.get_clock().now().nanoseconds*1e-9
            age = ros_now-sensor_time
            if sensor_time <= 0 or not -0.05 <= age <= LIDAR_CFG.max_sensor_age_sec:
                return 0.0
            return time.time()-max(age,0.)

        def tf(self, target, source, stamp, received):
            try:
                transform = self.buffer.lookup_transform(target, source, Time.from_msg(stamp),
                                                        timeout=Duration(seconds=LIDAR_CFG.tf_lookup_timeout_sec))
                t, q = transform.transform.translation, transform.transform.rotation
                return {"x": t.x, "y": t.y, "yaw": yaw_from_quaternion(q.x,q.y,q.z,q.w),
                        "frame_id": target, "source_frame": source, "timestamp": received,
                        "stamp_sec": stamp.sec+stamp.nanosec*1e-9, "valid": True}
            except TransformException as exc:
                self.get_logger().warning(str(exc))
                return {"valid": False, "frame_id": target, "source_frame": source, "timestamp": received}

        def odom(self, msg):
            mark_received("odom")
            received = self.observation_time(msg.header.stamp)
            # TF defines the actual base pose in BOTH map-based and mapless modes.
            pose = self.tf(ENV_CFG.planning_frame, ENV_CFG.base_frame, msg.header.stamp, received)
            pose.update(tf_valid=pose["valid"], unit="m", twist={
                "linear": {"x": msg.twist.twist.linear.x}, "angular": {"z": msg.twist.twist.angular.z}})
            store("odometry", pose)

        def scan(self, msg):
            mark_received("scan")
            received = self.observation_time(msg.header.stamp)
            frame = msg.header.frame_id
            world_tf = self.tf(ENV_CFG.planning_frame, frame, msg.header.stamp, received)
            base_tf = self.tf(ENV_CFG.base_frame, frame, msg.header.stamp, received)
            store("lidar", make_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max, frame,
                                     msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                     received, world_tf, base_tf))

        def camera_info(self, msg):
            mark_received("camera_info")
            self.calibration = {"width": msg.width, "height": msg.height,
                                "frame_id": msg.header.frame_id, "k": list(msg.k), "d": list(msg.d)}

        def camera(self, msg):
            mark_received("camera")
            received = self.observation_time(msg.header.stamp)
            try:
                channels = {"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4}[msg.encoding]
                # Honor per-row padding (sensor_msgs/Image.step).
                rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                image = rows[:, :msg.width*channels].reshape(msg.height,msg.width,channels)
                if msg.encoding == "rgb8": image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                if msg.encoding == "rgba8": image = cv2.cvtColor(image,cv2.COLOR_RGBA2BGR)
                if msg.encoding == "bgra8": image = cv2.cvtColor(image,cv2.COLOR_BGRA2BGR)
                ok, jpg = cv2.imencode(".jpg", image)
                if not ok:
                    raise ValueError("JPEG_ENCODING_FAILED")
                if ok:
                    laser_frame = snapshot("lidar").get("frame_id", ENV_CFG.laser_frame)
                    optical_frame = ENV_CFG.camera_optical_frame or msg.header.frame_id
                    intrinsics = camera_calibration(self.calibration, msg.width, msg.height, msg.header.frame_id)
                    try:
                        extrinsic = self.buffer.lookup_transform(optical_frame, laser_frame, Time.from_msg(msg.header.stamp),
                                                                 timeout=Duration(seconds=LIDAR_CFG.tf_lookup_timeout_sec))
                        t,q = extrinsic.transform.translation,extrinsic.transform.rotation
                        camera_tf = {"translation":[t.x,t.y,t.z],"quaternion":[q.x,q.y,q.z,q.w],
                                     "source_frame":laser_frame,"frame_id":optical_frame,"timestamp":received,"valid":True}
                    except TransformException:
                        camera_tf = {"valid":False,"timestamp":received}
                    store("camera", {"timestamp": received, "stamp_sec":msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                     "frame_id":optical_frame, "image_frame_id":msg.header.frame_id,
                                     "valid":bool(intrinsics and camera_tf.get("valid")),
                                     "image":base64.b64encode(jpg).decode(), "width":msg.width, "height":msg.height,
                                     "intrinsics":intrinsics,"laser_to_camera":camera_tf})
            except (KeyError, ValueError, cv2.error) as exc:
                store("camera", {"valid":False,"timestamp":received,"error":str(exc)})

    rclpy.init()
    node = Sensors()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    with state_lock:
        worker_state.update(status="running", error=None)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


@app.get("/odometry")
def get_odometry():
    return jsonify(snapshot("odometry"))


@app.get("/lidar")
def get_lidar():
    return jsonify(snapshot("lidar"))


@app.get("/camera")
def get_camera():
    return jsonify(snapshot("camera"))


@app.get("/video")
def video():
    def frames():
        previous = None
        while True:
            frame = snapshot("camera")
            if fresh(frame) and frame.get("image") and frame.get("timestamp") != previous:
                previous = frame["timestamp"]
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"+base64.b64decode(frame["image"])+b"\r\n"
            time.sleep(.03)
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/diagnostics")
def diagnostics():
    with state_lock:
        result = {"worker": copy.deepcopy(worker_state), "received": copy.deepcopy(received_messages)}
    result["ros_environment"] = {k: os.getenv(k, "(default)") for k in
                                 ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY")}
    result["camera_reliability"] = CAMERA_RELIABILITY
    result["topics"] = {"scan": ROS_TOPICS.scan_topic, "odom": ROS_TOPICS.odom_topic,
                        "camera": ROS_TOPICS.camera_topic, "camera_info": ROS_TOPICS.camera_info_topic}
    result["lidar"] = {k: v for k, v in snapshot("lidar").items() if k not in ("ranges", "points", "no_return_mask")}
    result["camera"] = {k: v for k, v in snapshot("camera").items() if k != "image"}
    return jsonify(result)


@app.get("/health")
def health():
    return jsonify({name: bool(fresh(snapshot(name)) and snapshot(name).get("valid")) for name in latest})


if __name__ == "__main__":
    # Missing ROS2 is a startup error, never an implicit simulation backend.
    import rclpy
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    threading.Thread(target=run_sensor_worker, daemon=True).start()
    app.run(host=BIND_HOST, port=NET_CFG.sensor_port, threaded=True)
