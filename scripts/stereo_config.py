"""Kalibr stereo geometry. T_1_0 maps left optical coordinates into right."""
from pathlib import Path
import copy
import hashlib
import math
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml
from build_sensor_description import ROOT,world_transforms
from sensor_visuals import pose_text

DEFAULT_CALIBRATION=ROOT/'calibration/stereo_20261102_135012.yaml'
OPTICAL_FROM_LINK=np.array([[0.,0.,1.,0.],[-1.,0.,0.,0.],[0.,-1.,0.,0.],[0.,0.,0.,1.]])


def pose_dict(transform,parent):
    values=list(map(float,pose_text(transform).split()))
    return dict(parent=parent,**dict(zip(('x','y','z','roll','pitch','yaw'),values)))


def load_calibration(path=DEFAULT_CALIBRATION,width=640):
    path=Path(path);data=yaml.safe_load(path.read_text())
    matrices=[];distortion=[];size=None
    for name in ('cam0','cam1'):
        camera=data[name]
        if camera['camera_model']!='pinhole' or camera['distortion_model']!='radtan':raise ValueError('Expected pinhole-radtan stereo')
        resolution=tuple(camera['resolution'])
        if size is not None and resolution!=size:raise ValueError('Stereo resolutions must agree')
        size=resolution
        fx,fy,cx,cy=map(float,camera['intrinsics']);d=list(map(float,camera['distortion_coeffs']))
        if len(d)!=4 or not np.isfinite([fx,fy,cx,cy,*d]).all() or min(fx,fy)<=0:raise ValueError('Invalid camera intrinsics')
        matrices.append(np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]],float));distortion.append(np.array(d+[0.]))
    if type(width) is not int or not 160<=width<=size[0]:raise ValueError('Invalid simulation camera width')
    scale=width/size[0];height=round(size[1]*scale)
    if abs(height-size[1]*scale)>1e-6:raise ValueError('Camera size must preserve original aspect ratio')
    for k in matrices:k[:2]*=scale
    t=np.asarray(data['cam1']['T_cn_cnm1'],float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]):raise ValueError('Invalid stereo homogeneous transform')
    if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-5):raise ValueError('Invalid stereo rotation')
    baseline=float(np.linalg.norm(t[:3,3]))
    if not .01<baseline<1.:raise ValueError('Unexpected stereo baseline in metres')
    r0,r1,p0,p1,q,_,_=cv2.stereoRectify(matrices[0],distortion[0],matrices[1],distortion[1],(width,height),t[:3,:3],t[:3,3],flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    return dict(source=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        width=width,height=height,native_resolution=size,K=matrices,D=distortion,
        T_right_left=t,R=[r0,r1],P=[p0,p1],Q=q,baseline_m=baseline)


def calibrated_mounts(mounting,calibration):
    frames=copy.deepcopy(mounting)
    # Preserve the user-confirmed housings/brackets; calibrate the optical frames.
    frames['camera_right_link_optical']=pose_dict(np.linalg.inv(calibration['T_right_left']),'camera_left_link_optical')
    for i,side in enumerate(('left','right')):
        t=np.eye(4);t[:3,:3]=calibration['R'][i].T
        frames[f'camera_{side}_rect_optical']=pose_dict(t,f'camera_{side}_link_optical')
    return frames


def attach_stereo(source,output,mounting,calibration,rate=6.):
    if not math.isfinite(rate) or not 0<rate<=30:raise ValueError('Invalid camera rate')
    tree=ET.parse(source);base=tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
    if base is None:raise ValueError('Missing car')
    transforms=world_transforms(mounting)
    for i,side in enumerate(('left','right')):
        k=calibration['K'][i];name=f'calibrated_camera_{side}'
        sensor=ET.SubElement(base,'sensor',name=name,type='camera')
        def add(parent,key,value):ET.SubElement(parent,key).text=str(value)
        add(sensor,'pose',pose_text(transforms[f'camera_{side}_link_optical']@np.linalg.inv(OPTICAL_FROM_LINK)))
        add(sensor,'topic',f'/sim/camera_{side}/image_ideal');add(sensor,'update_rate',rate);add(sensor,'always_on','true')
        add(sensor,'ignition_frame_id',f'camera_{side}_link_optical')
        camera=ET.SubElement(sensor,'camera');add(camera,'horizontal_fov',2*math.atan(calibration['width']/(2*k[0,0])))
        image=ET.SubElement(camera,'image');add(image,'width',calibration['width']);add(image,'height',calibration['height']);add(image,'format','R8G8B8')
        clip=ET.SubElement(camera,'clip');add(clip,'near',.05);add(clip,'far',30.)
        lens=ET.SubElement(camera,'lens');add(lens,'type','gnomonical');add(lens,'scale_to_hfov','false')
        intrinsic=ET.SubElement(lens,'intrinsics')
        for key,value in [('fx',k[0,0]),('fy',k[1,1]),('cx',k[0,2]),('cy',k[1,2]),('s',0.)]:add(intrinsic,key,value)
        # Brown distortion is applied once in the ROS adapter, with OpenCV's
        # exact Kalibr convention, avoiding renderer-specific distortion models.
    ET.indent(tree,space='  ');tree.write(output,encoding='utf-8',xml_declaration=True)


def bridges():
    return [dict(ros_topic_name=f'/sim/camera_{side}/image_ideal',gz_topic_name=f'/sim/camera_{side}/image_ideal',
        ros_type_name='sensor_msgs/msg/Image',gz_type_name='ignition.msgs.Image',direction='GZ_TO_ROS') for side in ('left','right')]
