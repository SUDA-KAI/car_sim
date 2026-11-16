#!/usr/bin/python3
"""Bridge ideal Gazebo horizontal scan fans to the real sensor Range interface."""
import argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from ultrasonic_config import load_mounting, fan_distance


class UltrasonicSim(Node):
    def __init__(self,mounting):
        super().__init__('sim_ultrasonic_ranges')
        self.subscriptions_=[]
        for sensor in mounting:
            publisher=self.create_publisher(Range,sensor['topic'],10)
            def receive(scan,s=sensor,pub=publisher):
                distance=fan_distance(scan.ranges,s['min_range'],s['max_range'])
                if distance is None:
                    return
                message=Range();message.header.stamp=scan.header.stamp
                message.header.frame_id=s['frame'];message.radiation_type=Range.ULTRASOUND
                message.field_of_view=s['field_of_view'];message.min_range=s['min_range']
                message.max_range=s['max_range'];message.range=distance
                pub.publish(message)
            self.subscriptions_.append(self.create_subscription(LaserScan,f"/sim/{sensor['frame']}/scan",receive,qos_profile_sensor_data))
        self.get_logger().info('Four ultrasonic Range topics active: ideal horizontal fans, not an acoustic model.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings',type=Path,required=True)
    parser.add_argument('--height',type=float,default=.49)
    args,ros_args=parser.parse_known_args()
    mounting=load_mounting(args.settings,args.height)
    rclpy.init(args=ros_args);node=UltrasonicSim(mounting)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
