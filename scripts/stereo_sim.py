#!/usr/bin/python3
"""Publish calibrated distorted raw and rectified stereo images from ideal renders."""
import argparse
import array as array_module
import copy
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from stereo_config import load_calibration,DEFAULT_CALIBRATION


def image_array(msg):
    channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}.get(msg.encoding)
    if channels is None:raise ValueError('Unsupported image encoding '+msg.encoding)
    array=np.ndarray((msg.height,msg.width,channels),np.uint8,buffer=msg.data,strides=(msg.step,channels,1)).copy()
    if channels==1:return cv2.cvtColor(array[:,:,0],cv2.COLOR_GRAY2BGR)
    if msg.encoding=='rgb8':return cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    if msg.encoding=='rgba8':return cv2.cvtColor(array,cv2.COLOR_RGBA2BGR)
    if msg.encoding=='bgra8':return cv2.cvtColor(array,cv2.COLOR_BGRA2BGR)
    return array


def image_message(array,header):
    msg=Image();msg.header=copy.deepcopy(header);msg.height,msg.width=array.shape[:2]
    msg.encoding='bgr8';msg.step=msg.width*3
    # rclpy's bytes setter validates every pixel in Python. array('B') uses
    # the generated message's buffer fast path and avoids asymmetric drops.
    msg.data=array_module.array('B',np.ascontiguousarray(array).tobytes());return msg


class StereoSim(Node):
    def __init__(self,calibration):
        super().__init__('sim_calibrated_stereo');self.calibration=calibration;self.maps=[];self.pubs=[];self.subs=[]
        w,h=calibration['width'],calibration['height']
        pixels=np.stack(np.meshgrid(np.arange(w,dtype=np.float32),np.arange(h,dtype=np.float32)),axis=-1)
        for i,side in enumerate(('left','right')):
            k,d=calibration['K'][i],calibration['D'][i]
            ideal=cv2.undistortPoints(pixels.reshape(-1,1,2),k,d,P=k).reshape(h,w,2)
            rectify=cv2.initUndistortRectifyMap(k,d,calibration['R'][i],calibration['P'][i],(w,h),cv2.CV_32FC1)
            self.maps.append((ideal[:,:,0],ideal[:,:,1],*rectify))
            pubs=[self.create_publisher(kind,f'/camera_{side}/{topic}',10) for kind,topic in [(Image,'image_raw'),(CameraInfo,'camera_info'),(Image,'image_rect'),(CameraInfo,'camera_info_rect')]]
            self.pubs.append(pubs)
            self.subs.append(self.create_subscription(Image,f'/sim/camera_{side}/image_ideal',lambda msg,n=i:self.receive(n,msg),qos_profile_sensor_data))
        self.get_logger().info(f'Kalibr stereo active: {w}x{h}, baseline {calibration["baseline_m"]:.6f} m; optical frames calibrated, body mounts measured.')

    def receive(self,i,msg):
        c=self.calibration;side=('left','right')[i]
        try:ideal=image_array(msg)
        except (ValueError,TypeError) as error:self.get_logger().warning(str(error));return
        if ideal.shape[:2]!=(c['height'],c['width']):
            self.get_logger().error('Rendered resolution does not match calibration');return
        a,b,x,y=self.maps[i];raw=cv2.remap(ideal,a,b,cv2.INTER_LINEAR)
        rect=cv2.remap(raw,x,y,cv2.INTER_LINEAR)
        header=copy.deepcopy(msg.header);header.frame_id=f'camera_{side}_link_optical'
        self.pubs[i][0].publish(image_message(raw,header))
        info=CameraInfo();info.header=header;info.width=c['width'];info.height=c['height'];info.distortion_model='plumb_bob'
        info.k=c['K'][i].reshape(-1).tolist();info.d=c['D'][i].tolist();info.r=c['R'][i].reshape(-1).tolist();info.p=c['P'][i].reshape(-1).tolist()
        self.pubs[i][1].publish(info)
        header.frame_id=f'camera_{side}_rect_optical';self.pubs[i][2].publish(image_message(rect,header))
        info.header=header;info.k=c['P'][i][:,:3].reshape(-1).tolist();info.r=np.eye(3).reshape(-1).tolist();info.d=[0.]*5
        self.pubs[i][3].publish(info)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));parser.add_argument('--width',type=int,default=640)
    args,ros_args=parser.parse_known_args();cv2.setNumThreads(1);rclpy.init(args=ros_args);node=StereoSim(load_calibration(args.calibration,args.width))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
