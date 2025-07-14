import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import datetime
import os

class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/wheel/odom',  # 또는 '/odom'
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        os.makedirs("odom_logs", exist_ok=True)

    def listener_callback(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist

        data = {
            "timestamp": datetime.datetime.now().isoformat(),
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

        filename = f"odom_logs/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')}.json"
        with open(filename, "w") as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"[저장됨] {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
