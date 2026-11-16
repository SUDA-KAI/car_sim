#!/usr/bin/python3
"""Adapt Gazebo snapshot clouds to Unitree fields and generate a Nav2 scan."""
import argparse
import copy
import math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField,LaserScan,Imu
from l2_sensor_config import load_config,DEFAULT_CONFIG

POINT_DTYPE=np.dtype(dict(names=['x','y','z','intensity','ring','time'],
    formats=['<f4','<f4','<f4','<f4','<u2','<f4'],offsets=[0,4,8,12,16,20],itemsize=24))


def read_xyz(message):
    """Read organized clouds including row padding and arbitrary field offsets."""
    fields={f.name:f for f in message.fields}
    if not all(k in fields and fields[k].datatype==PointField.FLOAT32 and fields[k].count==1 for k in ('x','y','z')):
        raise ValueError('Expected scalar FLOAT32 x/y/z')
    if message.row_step<message.width*message.point_step or len(message.data)<message.height*message.row_step:
        raise ValueError('Incomplete PointCloud2 buffer')
    order='>' if message.is_bigendian else '<'
    values=[]
    for key in ('x','y','z'):
        offset=fields[key].offset
        if offset+4>message.point_step:raise ValueError('Point field exceeds point_step')
        view=np.ndarray((message.height,message.width),dtype=order+'f4',buffer=message.data,
                        offset=offset,strides=(message.row_step,message.point_step))
        values.append(view.reshape(-1))
    return np.stack(values,axis=1)


def convert_points(xyz,config):
    lidar=config['lidar'];distance=np.linalg.norm(xyz,axis=1)
    valid=np.isfinite(xyz).all(axis=1)&(distance>=lidar['range_min_m'])&(distance<=lidar['range_max_m'])
    xyz=xyz[valid];points=np.zeros(len(xyz),dtype=POINT_DTYPE)
    for i,k in enumerate(('x','y','z')):points[k]=xyz[:,i]
    # Virtual elevation rows, not the real L2's rotating-prism ring identifiers.
    elevation=np.degrees(np.arctan2(xyz[:,2],np.hypot(xyz[:,0],xyz[:,1])))
    row=(elevation-lidar['elevation_min_deg'])/(lidar['elevation_max_deg']-lidar['elevation_min_deg'])*(lidar['vertical_samples']-1)
    points['ring']=np.clip(np.rint(row),0,lidar['vertical_samples']-1).astype(np.uint16)
    # No material-return model, so intensity is zero. All points share the snapshot
    # timestamp; never invent nonzero timing and induce false motion deskew.
    return points


def project_scan(points,config):
    s=config['scan'];l=config['lidar'];bins=int(s['bins']);ranges=np.full(bins,np.inf,dtype=np.float32)
    valid=(points['z']>=s['min_height_m'])&(points['z']<=s['max_height_m'])
    x=points['x'][valid];y=points['y'][valid];r=np.hypot(x,y)
    angles=np.arctan2(y,x)
    # Round to the nearest ray; wrap +pi back to the -pi seam.
    indices=np.rint((angles+math.pi)*bins/(2*math.pi)).astype(np.int64)%bins
    valid=(r>=l['range_min_m'])&(r<=l['range_max_m'])
    np.minimum.at(ranges,indices[valid],r[valid])
    return ranges


class L2Sim(Node):
    def __init__(self,config,no_world_orientation=False):
        self.no_world_orientation=no_world_orientation
        super().__init__('sim_l2_and_imu');self.config=config
        # Reliable publishers serve both Point-LIO's reliable IMU subscription
        # and consumers requesting best-effort sensor data.
        self.cloud_pub=self.create_publisher(PointCloud2,'/unilidar/cloud',10)
        self.scan_pub=self.create_publisher(LaserScan,'/scan',10)
        self.create_subscription(PointCloud2,'/sim/l2/points',self.cloud,qos_profile_sensor_data)
        self.imu_pubs={}
        for source,topic,frame in [('/sim/l2/imu','/unilidar/imu','unilidar_imu'),('/sim/external/imu','/imu/data_raw','imu_link')]:
            pub=self.create_publisher(Imu,topic,10);self.imu_pubs[topic]=pub
            self.create_subscription(Imu,source,lambda m,p=pub,f=frame:self.imu(m,p,f),qos_profile_sensor_data)
        self.get_logger().info('L2 snapshot cloud + builtin/external IMU active; /scan projected from 3D. No camera dependency.')

    def cloud(self,message):
        try:points=convert_points(read_xyz(message),self.config)
        except (ValueError,TypeError) as error:
            self.get_logger().error(str(error));return
        cloud=PointCloud2();cloud.header=copy.deepcopy(message.header);cloud.header.frame_id='unilidar_lidar'
        cloud.height=1;cloud.width=len(points);cloud.is_bigendian=False;cloud.is_dense=True
        cloud.fields=[PointField(name=k,offset=POINT_DTYPE.fields[k][1],datatype=PointField.UINT16 if k=='ring' else PointField.FLOAT32,count=1) for k in POINT_DTYPE.names]
        cloud.point_step=24;cloud.row_step=cloud.width*24;cloud.data=points.tobytes();self.cloud_pub.publish(cloud)
        scan=LaserScan();scan.header=cloud.header;scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/self.config['scan']['bins']
        scan.angle_max=scan.angle_min+(self.config['scan']['bins']-1)*scan.angle_increment
        scan.scan_time=1/self.config['lidar']['rate_hz'];scan.time_increment=0.
        scan.range_min=self.config['lidar']['range_min_m'];scan.range_max=self.config['lidar']['range_max_m']
        scan.ranges=project_scan(points,self.config).tolist();self.scan_pub.publish(scan)

    def imu(self,message,publisher,frame):
        message.header.frame_id=frame
        if self.no_world_orientation:
            message.orientation.x=message.orientation.y=message.orientation.z=0.
            message.orientation.w=1.
            message.orientation_covariance=[-1.]+[0.]*8
        # Gazebo orientations are ideal world-referenced quaternions; no hardware
        # fusion/calibration accuracy is claimed. Zero covariance means unknown.
        for field,noise in [('angular_velocity_covariance','angular_velocity_noise_stddev'),('linear_acceleration_covariance','acceleration_noise_stddev')]:
            variance=self.config['imu'][noise]**2;setattr(message,field,[variance if k in (0,4,8) else 0. for k in range(9)])
        publisher.publish(message)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--no-world-orientation',action='store_true')
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=L2Sim(load_config(args.config),args.no_world_orientation)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
