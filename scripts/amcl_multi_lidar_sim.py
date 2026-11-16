#!/usr/bin/python3
"""Simulation counterpart of robot_navigation multi_lidar_localization_scan.

Align L2 and two D6 endpoints by acquisition-time odom TF. The virtual scan
only updates AMCL; it is never used as free-space or collision evidence.
"""
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

BINS=360
MAX_AGE=.45
MAX_SKEW=.20
L2_HISTORY=.4
RANGE_MIN=.35
RANGE_MAX=6.


def pose2(transform):
    t=transform.transform.translation;q=transform.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return np.array([t.x,t.y,yaw],dtype=np.float64)


def apply(pose,points):
    c=math.cos(pose[2]);s=math.sin(pose[2])
    x=pose[0]+c*points[:,0]-s*points[:,1]
    y=pose[1]+s*points[:,0]+c*points[:,1]
    return np.column_stack((x,y))


def inverse_apply(pose,points):
    dx=points[:,0]-pose[0];dy=points[:,1]-pose[1]
    c=math.cos(pose[2]);s=math.sin(pose[2])
    return np.column_stack((c*dx+s*dy,-s*dx+c*dy))


def scan_points(scan):
    ranges=np.asarray(scan.ranges,dtype=np.float64)
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<scan.range_max)
    angle=scan.angle_min+np.nonzero(valid)[0]*scan.angle_increment
    return np.column_stack((ranges[valid]*np.cos(angle),ranges[valid]*np.sin(angle)))


def cloud_points(cloud):
    if (cloud.height!=1 or cloud.point_step!=12 or cloud.row_step!=cloud.width*12
            or cloud.is_bigendian or len(cloud.data)<cloud.row_step):
        return np.empty((0,2),dtype=np.float64)
    fields={field.name:field.offset for field in cloud.fields}
    if any(fields.get(key)!=index*4 for index,key in enumerate(('x','y','z'))):
        return np.empty((0,2),dtype=np.float64)
    xyz=np.frombuffer(cloud.data,dtype='<f4',count=cloud.width*3).reshape(-1,3)
    valid=np.isfinite(xyz).all(axis=1)&(xyz[:,2]>=.05)&(xyz[:,2]<=.80)
    points=xyz[valid,:2].astype(np.float64)
    return points[(np.abs(points[:,0])>.60)|(np.abs(points[:,1])>.45)]


def ranges_for(points,neighbors=False,min_support=1):
    ranges=np.full(BINS,np.inf,dtype=np.float64)
    if len(points)==0:return ranges
    valid=(np.abs(points[:,0])>.60)|(np.abs(points[:,1])>.45)
    points=points[valid]
    distance=np.hypot(points[:,0],points[:,1])
    valid=(distance>=RANGE_MIN)&(distance<RANGE_MAX)
    points=points[valid];distance=distance[valid]
    indices=np.rint((np.arctan2(points[:,1],points[:,0])+math.pi)*BINS/(2*math.pi)).astype(np.int64)%BINS
    np.minimum.at(ranges,indices,distance)
    if min_support>1:
        counts=np.bincount(indices,minlength=BINS)
        ranges[counts<min_support]=np.inf
    if neighbors:
        raw=ranges.copy();accepted=np.zeros(BINS,dtype=bool)
        for delta in (-2,-1,1,2):
            other=np.roll(raw,-delta)
            with np.errstate(invalid='ignore'):
                separation=raw*raw+other*other-2*raw*other*math.cos(delta*2*math.pi/BINS)
            accepted|=np.isfinite(other)&(separation<=.18**2)
        ranges[~accepted]=np.inf
    return ranges


class MultiLidarAMCL(Node):
    def __init__(self):
        super().__init__('sim_multi_lidar_localization_scan')
        self.buffer=Buffer(cache_time=Duration(seconds=3.))
        self.listener=TransformListener(self.buffer,self)
        self.pending=[deque(maxlen=5) for _ in range(3)]
        self.latest=[None]*3
        self.l2_history=deque(maxlen=5)
        self.last_stamp=-1.
        self.pub=self.create_publisher(LaserScan,'/localization/multi_lidar_scan',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/localization/scan_fusion_status',10)
        self.subs=[self.create_subscription(LaserScan,'/scan',lambda msg:self.enqueue(0,msg),qos_profile_sensor_data)]
        for index,topic in enumerate(('/sim/d6_front_obstacle_cloud','/sim/d6_rear_obstacle_cloud'),1):
            self.subs.append(self.create_subscription(PointCloud2,topic,
                lambda msg,i=index:self.enqueue(i,msg),qos_profile_sensor_data))
        self.create_timer(.05,self.process)

    def stamp(self,msg):
        return Time.from_msg(msg.header.stamp).nanoseconds*1e-9

    def enqueue(self,index,msg):
        stamp=self.stamp(msg)
        if stamp<=0 or (self.get_clock().now().nanoseconds*1e-9-stamp)>MAX_AGE:return
        if self.pending[index] and stamp<=self.stamp(self.pending[index][-1]):return
        self.pending[index].append(msg)

    def lookup(self,target,source,stamp):
        return pose2(self.buffer.lookup_transform(target,source,Time.from_msg(stamp)))

    def process(self):
        now=self.get_clock().now().nanoseconds*1e-9
        for index,queue in enumerate(self.pending):
            while queue:
                msg=queue[0];stamp=self.stamp(msg)
                if now-stamp>MAX_AGE:queue.popleft();continue
                try:
                    base=self.lookup('odom','base_link',msg.header.stamp)
                    sensor=self.lookup('base_link',msg.header.frame_id,msg.header.stamp) if index==0 else None
                except TransformException:break
                queue.popleft()
                points=scan_points(msg) if index==0 else cloud_points(msg)
                if index==0:points=apply(sensor,points)
                frame=(stamp,base,apply(base,points))
                self.latest[index]=frame
                if index==0:self.l2_history.append(frame)
        valid=[(i,f) for i,f in enumerate(self.latest) if f and -.05<=now-f[0]<=MAX_AGE]
        if not valid:return
        _,reference=max(valid,key=lambda item:item[1][0])
        stamp,base,_=reference
        if stamp<=self.last_stamp:return
        candidates=[];used=[];supported=[];contributed=[]
        for index,frame in enumerate(self.latest):
            included=bool(frame and -.05<=now-frame[0]<=MAX_AGE and 0<=stamp-frame[0]<=MAX_SKEW)
            used.append(included)
            if not included:candidates.append(np.full(BINS,np.inf));supported.append(0);continue
            if index==0:
                points=[item[2] for item in self.l2_history if 0<=stamp-item[0]<=L2_HISTORY]
                current=inverse_apply(base,np.concatenate(points)) if points else np.empty((0,2))
                ranges=ranges_for(current,min_support=2)
            else:ranges=ranges_for(inverse_apply(base,frame[2]),neighbors=True)
            candidates.append(ranges);supported.append(int(np.isfinite(ranges).sum()))
        output=np.full(BINS,np.inf,dtype=np.float32)
        intensity=np.zeros(BINS,dtype=np.float32)
        contributed=[0,0,0]
        for bin_index in range(BINS):
            chosen=1 if candidates[1][bin_index]<=candidates[2][bin_index] else 2
            if not math.isfinite(candidates[chosen][bin_index]):chosen=0
            if math.isfinite(candidates[chosen][bin_index]):
                output[bin_index]=candidates[chosen][bin_index]
                intensity[bin_index]=chosen+1
                contributed[chosen]+=1
        scan=LaserScan();scan.header.frame_id='base_link'
        scan.header.stamp=Time(seconds=stamp).to_msg()
        scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/BINS
        scan.angle_max=scan.angle_min+(BINS-1)*scan.angle_increment
        scan.range_min=RANGE_MIN;scan.range_max=RANGE_MAX
        scan.scan_time=.1;scan.ranges=output.tolist();scan.intensities=intensity.tolist()
        self.pub.publish(scan);self.last_stamp=stamp
        names=('l2','d6_front','d6_rear')
        status={'stamp':stamp,'sources':[{'name':names[i],'included':used[i],
            'supported_bins':supported[i],'used_bins':contributed[i]} for i in range(3)],
            'purpose':'AMCL pose correction only'}
        self.status.publish(String(data=json.dumps(status)))


def main():
    rclpy.init();node=MultiLidarAMCL()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
