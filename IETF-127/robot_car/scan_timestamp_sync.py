#!/usr/bin/env python3
"""ViLaR-compatible /sim/scan -> /sim/scan_sync receive-time restamping.

This is a simulation compatibility adapter, not measurement-time recovery.
Ranges, angles and frame_id are unchanged. Start only one adapter.
"""
import copy


def restamp_scan(message, stamp):
    result = copy.deepcopy(message)
    result.header.stamp = copy.deepcopy(stamp)
    return result


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class ScanTimestampSync(Node):
        def __init__(self):
            super().__init__('scan_timestamp_sync', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.declare_parameter('input_topic', '/sim/scan')
            self.declare_parameter('output_topic', '/sim/scan_sync')
            input_topic = self.get_parameter('input_topic').value
            output_topic = self.get_parameter('output_topic').value
            if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
                raise ValueError('Input and output topics must differ')
            self.publisher = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
            self.subscription = self.create_subscription(LaserScan, input_topic, self.scan, qos_profile_sensor_data)
            self.last_source_stamp = None
            self.last_clock = None
            self.get_logger().info(f'{input_topic} -> {output_topic}: stamp replaced with reception ROS time; ranges unchanged')

        def scan(self, message):
            now = self.get_clock().now()
            if now.nanoseconds <= 0:
                return
            if self.last_clock is not None and now.nanoseconds < self.last_clock:
                self.last_source_stamp = None
            self.last_clock = now.nanoseconds
            original = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
            # Do not make duplicate or out-of-order observations look fresh.
            if self.last_source_stamp is not None and original <= self.last_source_stamp:
                return
            self.last_source_stamp = original
            self.publisher.publish(restamp_scan(message, now.to_msg()))

    rclpy.init(args=args)
    node = None
    try:
        node = ScanTimestampSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
