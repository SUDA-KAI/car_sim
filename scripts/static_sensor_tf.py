#!/usr/bin/python3
"""Publish the complete measured/calibrated sensor tree from one ROS node."""
import argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
import yaml
from build_sensor_description import world_transforms


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mounting',required=True);args,ros_args=p.parse_known_args()
    frames=yaml.safe_load(Path(args.mounting).read_text());world_transforms(frames)
    rclpy.init(args=ros_args);node=Node('sim_calibrated_sensor_tf');broadcaster=StaticTransformBroadcaster(node)
    transforms=[]
    for child,pose in frames.items():
        msg=TransformStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id=pose['parent'];msg.child_frame_id=child
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=[float(pose.get(k,0)) for k in ('x','y','z')]
        q=Rotation.from_euler('xyz',[float(pose.get(k,0)) for k in ('roll','pitch','yaw')]).as_quat()
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,q);transforms.append(msg)
    broadcaster.sendTransform(transforms)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
