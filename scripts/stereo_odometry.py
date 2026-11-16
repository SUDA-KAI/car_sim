#!/usr/bin/python3
"""Quality-gated stereo depth + temporal KLT + PnP odometry frontend."""
import argparse
import json
import time
import threading
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
import yaml
from build_sensor_description import world_transforms
from stereo_config import load_calibration,DEFAULT_CALIBRATION
from stereo_sim import image_array
from stereo_geometry import estimate_delta,stereo_features


class VisualOdometry(Node):
    def __init__(self,calibration,mounting):
        super().__init__('sim_stereo_odometry')
        self.c=calibration;self.k=calibration['P'][0][:,:3]
        self.baseline=abs(calibration['P'][1][0,3]/self.k[0,0])
        self.base_camera=world_transforms(mounting)['camera_left_rect_optical']
        self.pending={};self.previous=None;self.pose=np.eye(4);self.good=0;self.lost=0;self.last_input=0.
        self.condition=threading.Condition();self.latest_pair=None;self.stopping=False;self.dropped_pairs=0
        self.pub=self.create_publisher(Odometry,'/localization/visual_odom',10)
        self.status=self.create_publisher(String,'/localization/visual_status',10)
        self.subs=[self.create_subscription(Image,f'/camera_{side}/image_rect',lambda msg,n=i:self.receive(n,msg),qos_profile_sensor_data) for i,side in enumerate(('left','right'))]
        self.create_timer(1.,self.watchdog,clock=rclpy.clock.Clock())
        self.worker=threading.Thread(target=self.work,daemon=True);self.worker.start()

    def report(self,state,**values):
        self.status.publish(String(data=json.dumps(dict(state=state,good_frames=self.good,lost_frames=self.lost,dropped_pairs=self.dropped_pairs,**values))))

    def watchdog(self):
        if time.monotonic()-self.last_input>3.:self.report('waiting_for_images')

    def receive(self,index,msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        pair=self.pending.setdefault(stamp,{})
        pair[index]=msg
        # Gazebo cameras can render on adjacent 2 ms physics steps. Accept only
        # this small exposure skew; do not pair arbitrary stale frames.
        if len(pair)!=2:
            candidates=[s for s,p in self.pending.items() if s!=stamp and 1-index in p and abs(s-stamp)<=2000000]
            if candidates:
                other=min(candidates,key=lambda s:abs(s-stamp));pair[1-index]=self.pending[other].pop(1-index)
                if not self.pending[other]:del self.pending[other]
        if len(self.pending)>16:self.pending.pop(min(self.pending))
        if len(pair)!=2:return
        del self.pending[stamp];self.last_input=time.monotonic()
        with self.condition:
            if self.latest_pair is not None:self.dropped_pairs+=1
            self.latest_pair=pair;self.condition.notify()

    def work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.latest_pair is not None or self.stopping)
                if self.stopping:return
                pair=self.latest_pair;self.latest_pair=None
            try:self.process(pair)
            except (ValueError,cv2.error) as error:
                self.lost+=1;self.previous=None;self.report('processing_error',reason=str(error))

    def stop(self):
        with self.condition:self.stopping=True;self.condition.notify()
        self.worker.join(timeout=3.)

    def process(self,pair):
        msg=pair[0]
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if self.previous is not None and stamp<=self.previous[0]:return
        left=cv2.cvtColor(image_array(pair[0]),cv2.COLOR_BGR2GRAY);right=cv2.cvtColor(image_array(pair[1]),cv2.COLOR_BGR2GRAY)
        status='initializing';values={};processing_start=time.monotonic()
        if self.previous is not None:
            old_stamp,old_left,old_points,objects=self.previous;dt=(stamp-old_stamp)*1e-9
            if len(old_points)>=20 and 0<dt<2.:
                tracked,valid,_=cv2.calcOpticalFlowPyrLK(old_left,left,old_points.reshape(-1,1,2),None,winSize=(21,21),maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
                back,back_valid,_=cv2.calcOpticalFlowPyrLK(left,old_left,tracked,None,winSize=(21,21),maxLevel=3)
                good=(valid[:,0]>0)&(back_valid[:,0]>0)&(np.linalg.norm(back[:,0]-old_points,axis=1)<1.)
                pixels=tracked[:,0][good];points=objects[good]
                try:
                    delta,inliers,error=estimate_delta(points,pixels,self.k,self.base_camera,dt)
                    self.pose=self.pose@delta;self.good+=1;status='tracking';values=dict(inliers=inliers,reprojection_rmse_px=error,tracked_points=len(points),frame_dt_s=dt)
                    odom=Odometry();odom.header.stamp=msg.header.stamp;odom.header.frame_id='visual_odom';odom.child_frame_id='base_link'
                    position=self.pose[:3,3];q=Rotation.from_matrix(self.pose[:3,:3]).as_quat()
                    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,position)
                    odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q)
                    linear=delta[:3,:3].T@delta[:3,3]/dt;angular=delta[:3,:3].T@cv2.Rodrigues(delta[:3,:3])[0][:,0]/dt
                    odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,linear)
                    odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=map(float,angular)
                    variance=max(.03**2,(error*.02)**2)*(1+30/inliers)
                    for axis in range(6):
                        odom.pose.covariance[axis*7]=.02+.005*self.lost
                        odom.twist.covariance[axis*7]=variance if axis<3 else max(.02**2,variance/4)
                    self.pub.publish(odom)
                except (ValueError,cv2.error) as error:
                    status='tracking_lost';values={'reason':str(error)};self.lost+=1
            else:
                status='frame_gap' if dt>=2. else 'insufficient_stereo_features'
                values=dict(frame_dt_s=dt,previous_stereo_features=len(old_points));self.lost+=1
        points,objects=stereo_features(left,right,self.k,self.baseline)
        self.previous=(stamp,left,points,objects);self.report(status,stereo_features=len(points),processing_ms=1000*(time.monotonic()-processing_start),**values)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));p.add_argument('--width',type=int,default=640);p.add_argument('--mounting',required=True)
    args,ros_args=p.parse_known_args();cv2.setNumThreads(1);rclpy.init(args=ros_args)
    node=VisualOdometry(load_calibration(args.calibration,args.width),yaml.safe_load(Path(args.mounting).read_text()))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
