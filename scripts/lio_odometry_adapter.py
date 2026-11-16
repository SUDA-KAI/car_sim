#!/usr/bin/python3
"""Rebase estimated LIO base poses to a startup-local odom frame; no truth input."""
from collections import deque
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scipy.spatial.transform import Rotation


def fresh_stamp(stamp,now,max_age=.75):
    return math.isfinite(stamp) and math.isfinite(now) and -.05<=now-stamp<=max_age


def visual_consistent(lio,visual):
    distance=float(np.linalg.norm(lio[:2,3]-visual[:2,3]))
    a=math.atan2(lio[1,0],lio[0,0]);b=math.atan2(visual[1,0],visual[0,0])
    angle=abs(math.atan2(math.sin(a-b),math.cos(a-b)))
    return distance<=.35 and angle<=.12


def pose_matrix(pose):
    t=np.eye(4);q=pose.orientation
    if not np.isfinite([pose.position.x,pose.position.y,pose.position.z,q.x,q.y,q.z,q.w]).all():raise ValueError('Nonfinite pose')
    t[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();t[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
    return t


class LioAdapter(Node):
    def __init__(self):
        super().__init__('sim_lio_odometry_adapter');self.origin=None;self.last_stamp=None;self.last_input=0.;self.count=0
        self.source_age=None;self.rejected_stale=0;self.last_message_stamp=None;self.visual=deque(maxlen=60);self.rejected_inconsistent=0;self.inconsistent=False
        self.pub=self.create_publisher(Odometry,'/localization/lio_odom',10)
        self.status=self.create_publisher(String,'/localization/lio_status',10)
        self.visual_sub=self.create_subscription(Odometry,'/localization/visual_odom',self.receive_visual,20)
        self.sub=self.create_subscription(Odometry,'/pointlio/odometry',self.receive,20)
        self.create_timer(1.,self.watchdog,clock=rclpy.clock.Clock())

    def receive_visual(self,msg):
        try:self.visual.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,pose_matrix(msg.pose.pose)))
        except ValueError:pass

    def watchdog(self):
        age=None if not self.last_input else time.monotonic()-self.last_input
        if self.last_message_stamp is not None:self.source_age=self.get_clock().now().nanoseconds*1e-9-self.last_message_stamp
        state='ready' if age is not None and age<3 else 'waiting_for_lio'
        if self.inconsistent:state='inconsistent_lio'
        if self.source_age is not None and self.source_age>.75:state='stale_lio'
        self.status.publish(String(data=json.dumps(dict(state=state,received=self.count,age=age,source_age_s=self.source_age,
            rejected_stale=self.rejected_stale,rejected_inconsistent=self.rejected_inconsistent,origin='first estimated base pose; no simulator pose'))))

    def receive(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.last_message_stamp=stamp
        now=self.get_clock().now().nanoseconds*1e-9;self.source_age=now-stamp
        if not fresh_stamp(stamp,now):self.rejected_stale+=1;return
        if self.last_stamp is not None and stamp<=self.last_stamp:return
        try:pose=pose_matrix(msg.pose.pose)
        except ValueError:return
        if self.origin is None:
            # Use measured mounting geometry already applied by Point-LIO and
            # its first *estimated* base pose. No spawn coordinate or yaw.
            yaw=math.atan2(pose[1,0],pose[0,0]);self.origin=np.eye(4)
            self.origin[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();self.origin[:3,3]=pose[:3,3]
        relative=np.linalg.inv(self.origin)@pose
        reference=min(self.visual,key=lambda item:abs(item[0]-stamp)) if self.visual else None
        if reference is not None and abs(reference[0]-stamp)<.2:
            self.inconsistent=not visual_consistent(relative,reference[1])
        if self.inconsistent:
            self.rejected_inconsistent+=1;return
        result=copy.deepcopy(msg);result.header.frame_id='odom';result.child_frame_id='base_link'
        result.pose.pose.position.x,result.pose.pose.position.y,result.pose.pose.position.z=map(float,relative[:3,3])
        q=Rotation.from_matrix(relative[:3,:3]).as_quat()
        result.pose.pose.orientation.x,result.pose.pose.orientation.y,result.pose.pose.orientation.z,result.pose.pose.orientation.w=map(float,q)
        # Point-LIO's default full odometry covariance may be all zeros. Use
        # explicit, conservative simulation tuning, not invented certainty.
        result.pose.covariance=[0.]*36;result.twist.covariance=[0.]*36
        for i in range(6):
            result.pose.covariance[i*7]=.05**2 if i<3 else .04**2
            result.twist.covariance[i*7]=.1**2
        self.pub.publish(result);self.last_stamp=stamp;self.last_input=time.monotonic();self.count+=1


def main():
    rclpy.init();node=LioAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
