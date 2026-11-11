"""Additional TF for the verified ViLaR warehouse and supplied LIMO USD only.

Run on the ROS2 computer. Isaac owns odom/base/sensor transforms.
Disable publish_map_tf when localization already owns map -> odom.
"""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("publish_map_tf", default_value="true"),
        DeclareLaunchArgument("start_scan_sync", default_value="true"),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "robot_car.scan_timestamp_sync"],
            cwd=str(Path(__file__).resolve().parents[1]),
            output="screen", condition=IfCondition(LaunchConfiguration("start_scan_sync"))),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="ietf127_map_to_odom",
             condition=IfCondition(LaunchConfiguration("publish_map_tf")),
             arguments=["--x", "0", "--y", "0", "--z", "0",
                        "--roll", "0", "--pitch", "0", "--yaw", "3.141592653589793",
                        "--frame-id", "map", "--child-frame-id", "odom"]),
        # USD camera has Rx(pi) relative to depth_link (OpenGL camera axes).
        # OpenGL -> ROS optical also has Rx(pi), so optical orientation is identity.
        # Preserve the camera prim's measured translation, instead of using depth_link's origin.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="ietf127_camera_optical",
             arguments=["--x", "0.01", "--y", "-0.0000033935893158521324", "--z", "0",
                        "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                        "--frame-id", "depth_link", "--child-frame-id", "limo_camera_optical"]),
    ])
