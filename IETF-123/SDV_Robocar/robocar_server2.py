from flask import Flask, Response, jsonify
import multiprocessing
import time
import queue
import cv2

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# --- Flask 서버 정의 ---
app = Flask(__name__)
cap = cv2.VideoCapture(0)  # 0 또는 1
'''
# 해상도 낮추기 (320 x 240)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
'''

shared_queue = multiprocessing.Queue(maxsize=1)

# --- MJPEG 영상 스트리밍 함수 ---
def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


# --- Flask 라우트: 영상 스트리밍 ---
@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Flask 라우트: 최신 Odometry 정보 반환 (HTTP JSON) ---
@app.route('/odometry')
def stream_odometry():
    try:
        data = shared_queue.get(timeout=1.0)  # 최신 메시지 수신 대기
        return jsonify(data)
    except queue.Empty:
        return jsonify({"error": "No odometry data available"}),   # No Content

# --- ROS2 노드 정의 ---
def ros_process(q):
    class OdometryStreamer(Node):
        def __init__(self):
            super().__init__('odometry_streamer')
            self.subscription = self.create_subscription(
                Odometry,
                '/wheel/odom',
                self.listener_callback,
                10
            )

        def listener_callback(self, msg):
            pose = msg.pose.pose
            twist = msg.twist.twist
            data = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "pose": {
                    "position": {
                        "x": pose.position.x,
                        "y": pose.position.y,
                        "z": pose.position.z
                    },
                    "orientation": {
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w
                    }
                },
                "twist": {
                    "linear": {
                        "x": twist.linear.x,
                        "y": twist.linear.y,
                        "z": twist.linear.z
                    },
                    "angular": {
                        "x": twist.angular.x,
                        "y": twist.angular.y,
                        "z": twist.angular.z
                    }
                }
            }

            if not q.full():
                q.put(data)

    # --- ROS2 노드 실행 함수 (Flask와 병렬) ---
    rclpy.init()
    node = OdometryStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# --- Flask 서버 실행 ---
if __name__ == "__main__":
    ros_proc = multiprocessing.Process(target=ros_process, args=(shared_queue,))    # ROS process 병렬 처리
    ros_proc.start()
    app.run(host='0.0.0.0', port=8000)
    ros_proc.join()
