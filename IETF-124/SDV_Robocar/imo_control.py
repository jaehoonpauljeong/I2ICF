from flask import Flask, request, jsonify
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# -------------------------
# 설정값
# -------------------------
STOP_THRESHOLD   = 1.0   # 이하면 정지
DEFAULT_SPEED    = 0.2   # 주행 속도 (m/s)
PUBLISH_HZ       = 10.0  # cmd_vel 퍼블리시 주기

# -------------------------
# 전역 상태 (스레드 공유)
# -------------------------
state_lock   = threading.Lock()
drive_state  = {
    "e_stop": False,        # True면 정지, False면 주행
    "last_distance": None,  # 최근 수신 거리
}

# -------------------------
# ROS2 노드: 주행/정지 명령 퍼블리시
# -------------------------
class LimoController(Node):
    def __init__(self):
        super().__init__('imo_limo_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(f"[IMO] Controller started: default_speed={DEFAULT_SPEED} m/s")

    def _tick(self):
        with state_lock:
            stop = drive_state["e_stop"]
        msg = Twist()
        if stop:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = DEFAULT_SPEED
            msg.angular.z = 0.0
        self.pub.publish(msg)

# -------------------------
# Flask 서버: 거리 수신 → 정지/재개
# -------------------------
app = Flask(__name__)

@app.route("/control/distance", methods=["POST"])
def control_distance():
    """
    요청 예시(JSON):
      {"distance": 1.23}
    규칙:
      distance <= 1.4  -> 정지(e_stop=True)
      distance >  1.4  -> 주행(e_stop=False)
    """
    try:
        data = request.get_json(force=True)
        d = float(data["distance"])
    except Exception as e:
        return jsonify({"ok": False, "error": f"invalid payload: {e}"}), 400

    with state_lock:
        drive_state["last_distance"] = d
        drive_state["e_stop"] = (d <= STOP_THRESHOLD)
        estop = drive_state["e_stop"]

    return jsonify({"ok": True, "distance": d, "e_stop": estop,
                    "rule": {"stop_thr": STOP_THRESHOLD}}), 200

@app.route("/control/state", methods=["GET"])
def get_state():
    with state_lock:
        return jsonify(drive_state), 200

def ros_thread_fn():
    rclpy.init()
    node = LimoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    # ROS 스레드 시작(즉시 0.2m/s 주행 시작)
    t = threading.Thread(target=ros_thread_fn, daemon=True)
    t.start()

    # Flask(수신 전용) 시작: 8001
    # threaded=True → 동시 요청 처리
    try:
        app.run(host="0.0.0.0", port=8001, threaded=True)
    except KeyboardInterrupt:
        pass
