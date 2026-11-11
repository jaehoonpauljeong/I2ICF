#!/usr/bin/env python3
"""Optional ROS2 publication of the canonical Edge-localized objects.

All consumers use the same scan-time transforms and base-frame self filter.
This node does not create a second independently localized object database.
"""
import requests
from common.config import WORLD_URL, ROS_TOPICS
from common.runtime_safety import fresh


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PointStamped
    from visualization_msgs.msg import Marker
    from std_msgs.msg import String
    import json
    rclpy.init()
    node=Node("ietf127_semantic_objects")
    points=node.create_publisher(PointStamped,ROS_TOPICS.dynamic_object_point_topic,10)
    markers=node.create_publisher(Marker,ROS_TOPICS.dynamic_object_marker_topic,10)
    objects=node.create_publisher(String,"/localized_objects_json",10)
    def tick():
        try:
            resp=requests.get(WORLD_URL,timeout=.3);resp.raise_for_status()
            world=resp.json()
            if not fresh(world):return
            accepted=[]
            for i,obj in enumerate(world.get("environment",{}).get("objects",[])):
                pos=obj.get("position",{})
                if not fresh(pos) or not pos.get("valid") or "stamp_sec" not in pos:continue
                point=PointStamped()
                point.header.frame_id=pos["frame_id"]
                point.header.stamp.sec=int(pos["stamp_sec"])
                point.header.stamp.nanosec=int((pos["stamp_sec"]%1)*1e9)
                point.point.x=pos["x"];point.point.y=pos["y"]
                points.publish(point)
                marker=Marker();marker.header=point.header;marker.ns="objects";marker.id=i
                marker.type=Marker.CYLINDER;marker.action=Marker.ADD
                marker.pose.position=point.point;marker.pose.orientation.w=1.
                marker.scale.x=.35;marker.scale.y=.35;marker.scale.z=.5
                marker.color.r=1.;marker.color.a=.8;marker.lifetime.nanosec=500000000
                markers.publish(marker);accepted.append(obj)
            out=String();out.data=json.dumps({"objects":accepted});objects.publish(out)
        except (requests.RequestException,ValueError,KeyError):pass
    node.create_timer(.2,tick)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
