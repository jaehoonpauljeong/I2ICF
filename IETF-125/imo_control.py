from flask import Flask, request, jsonify
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# -------------------------
# 설정값
# -------------------------
STOP_THRESHOLD   = 1.0   # 이 거리 이하면 강제 정지
DEFAULT_SPEED    = 0.2   # 기본 직진 속도
PUBLISH_HZ       = 10.0  # cmd_vel publish 주기


# -------------------------
# 전역 상태 (edge_control와 공유되는 상태)
# -------------------------
state_lock = threading.Lock()

drive_state = {
    "e_stop": False,
    "last_distance": None,

    # edge_control.py 에서 보내는 주행 명령
    "linear_x": DEFAULT_SPEED,
    "angular_z": 0.0
}


# -------------------------
# ROS2 Controller
# -------------------------
class LimoController(Node):

    def __init__(self):
        super().__init__('imo_limo_controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.get_logger().info(
            f"[IMO] Controller started | default_speed={DEFAULT_SPEED}"
        )

    def _tick(self):

        with state_lock:
            stop = drive_state["e_stop"]
            linear_x = drive_state["linear_x"]
            angular_z = drive_state["angular_z"]

        msg = Twist()

        # -------------------------
        # 안전 정지 우선
        # -------------------------
        if stop:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        else:
            msg.linear.x = linear_x
            msg.angular.z = angular_z

        self.pub.publish(msg)


# -------------------------
# Flask Server
# -------------------------
app = Flask(__name__)


# -------------------------
# 거리 기반 안전 정지
# -------------------------
@app.route("/control/distance", methods=["POST"])
def control_distance():

    """
    edge_control.py 에서 distance 전송

    JSON:
    {
        "distance": 1.23
    }
    """

    try:
        data = request.get_json(force=True)
        d = float(data["distance"])
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400

    with state_lock:

        drive_state["last_distance"] = d

        # threshold 이하 → 강제 정지
        drive_state["e_stop"] = (d <= STOP_THRESHOLD)

        estop = drive_state["e_stop"]

    return jsonify({
        "ok": True,
        "distance": d,
        "e_stop": estop
    })


# -------------------------
# edge_control → 주행 명령
# -------------------------
@app.route("/control/cmd_vel", methods=["POST"])
def control_cmd_vel():

    """
    edge_control.py 에서 회피 명령 전송

    JSON:
    {
        "linear_x": 0.2,
        "angular_z": -0.35
    }
    """

    try:
        data = request.get_json(force=True)

        linear_x = float(data.get("linear_x", DEFAULT_SPEED))
        angular_z = float(data.get("angular_z", 0.0))

    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400

    with state_lock:

        drive_state["linear_x"] = linear_x
        drive_state["angular_z"] = angular_z

    return jsonify({
        "ok": True,
        "linear_x": linear_x,
        "angular_z": angular_z
    })


# -------------------------
# 현재 상태 조회
# -------------------------
@app.route("/control/state", methods=["GET"])
def get_state():

    with state_lock:
        return jsonify(drive_state)


# -------------------------
# ROS Thread
# -------------------------
def ros_thread():

    rclpy.init()

    node = LimoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


# -------------------------
# Main
# -------------------------
if __name__ == "__main__":

    # ROS2 실행
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()

    # Flask 실행
    try:
        app.run(host="0.0.0.0", port=8001, threaded=True)
    except KeyboardInterrupt:
        pass