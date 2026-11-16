#!/usr/bin/python3
"""Republish Gazebo D6 scans with the deployed driver topics and point cloud."""
import math
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py import point_cloud2
from d6_sensor_config import load_config


def convert_scan(source,frame,bins,minimum,maximum):
    scan=LaserScan()
    scan.header=deepcopy(source.header)
    scan.header.frame_id=frame
    scan.angle_min=0.
    scan.angle_increment=2*math.pi/bins
    scan.angle_max=(bins-1)*scan.angle_increment
    scan.range_min=minimum
    scan.range_max=maximum
    scan.scan_time=source.scan_time
    scan.time_increment=source.time_increment
    scan.ranges=[math.inf]*bins
    scan.intensities=[0.]*bins
    for index,distance in enumerate(source.ranges):
        if not math.isfinite(distance) or not minimum<=distance<=maximum:continue
        angle=(source.angle_min+index*source.angle_increment)%(2*math.pi)
        target=round(angle/scan.angle_increment)%bins
        if distance<scan.ranges[target]:scan.ranges[target]=distance
    points=[(distance*math.cos(index*scan.angle_increment),
             distance*math.sin(index*scan.angle_increment),0.)
            for index,distance in enumerate(scan.ranges) if math.isfinite(distance)]
    cloud=point_cloud2.create_cloud_xyz32(scan.header,points)
    return scan,cloud


def obstacle_cloud(scan,sensor):
    """Apply the real navigation chain's vehicle self mask in base_link."""
    yaw=sensor['yaw'];c=math.cos(yaw);s=math.sin(yaw)
    points=[]
    for index,distance in enumerate(scan.ranges):
        if not math.isfinite(distance):continue
        angle=index*scan.angle_increment
        x=distance*math.cos(angle);y=distance*math.sin(angle)
        bx=sensor['x']+c*x-s*y
        by=sensor['y']+s*x+c*y
        bz=sensor['z']
        if -.66<=bx<=.60 and -.55<=by<=.55 and .05<=bz<=.80:continue
        points.append((bx,by,bz))
    header=deepcopy(scan.header)
    header.frame_id='base_link'
    return point_cloud2.create_cloud_xyz32(header,points)


class D6Sim(Node):
    def __init__(self):
        super().__init__('sim_d6_driver')
        config=load_config()
        self.subs=[]
        self.pubs=[]
        for key,sensor in config['sensors'].items():
            scan_pub=self.create_publisher(LaserScan,f'/d6_{key}/scan',qos_profile_sensor_data)
            cloud_pub=self.create_publisher(PointCloud2,f'/d6_{key}/points',qos_profile_sensor_data)
            nav_pub=self.create_publisher(PointCloud2,
                f'/sim/d6_{"front" if key=="b" else "rear"}_obstacle_cloud',qos_profile_sensor_data)
            self.pubs.extend((scan_pub,cloud_pub,nav_pub))
            def callback(msg,frame=sensor['frame'],spec=sensor,scan_pub=scan_pub,cloud_pub=cloud_pub,nav_pub=nav_pub):
                scan,cloud=convert_scan(msg,frame,config['bins'],config['min_range_m'],config['max_range_m'])
                scan_pub.publish(scan)
                cloud_pub.publish(cloud)
                nav_pub.publish(obstacle_cloud(scan,spec))
            self.subs.append(self.create_subscription(LaserScan,f'/sim/d6_{key}/scan',callback,qos_profile_sensor_data))


def main():
    rclpy.init()
    node=D6Sim()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
