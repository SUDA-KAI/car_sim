"""Add real Gazebo 3D range / inertial sensors at the measured mount frames."""
from pathlib import Path
import copy
import math
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from build_sensor_description import ROOT,world_transforms
from sensor_visuals import pose_text

DEFAULT_CONFIG=ROOT/'config/l2_sim.yaml'
BUILTIN_IMU_OFFSET=[-.007698,-.014655,.00667]


def load_config(path=DEFAULT_CONFIG):
    config=yaml.safe_load(Path(path).read_text())
    l=config['lidar'];i=config['imu'];s=config['scan']
    if not all(math.isfinite(float(v)) for group in config.values() for v in group.values()):
        raise ValueError('Sensor settings must be finite')
    if not (2<=l['horizontal_samples']<=4096 and 2<=l['vertical_samples']<=256 and
            int(l['horizontal_samples'])==l['horizontal_samples'] and int(l['vertical_samples'])==l['vertical_samples']):
        raise ValueError('Invalid scan sample counts')
    if not (-90<=l['elevation_min_deg']<l['elevation_max_deg']<90):raise ValueError('Invalid elevation limits')
    if not (0<l['range_min_m']<l['range_max_m'] and l['rate_hz']>0 and i['rate_hz']>0):raise ValueError('Invalid range or rate')
    if s['min_height_m']>=s['max_height_m'] or s['bins']<2 or int(s['bins'])!=s['bins']:raise ValueError('Invalid scan projection')
    if any(value<0 for group in (l,i) for key,value in group.items() if 'stddev' in key):raise ValueError('Noise cannot be negative')
    return config


def add(parent,tag,text=None,**attributes):
    element=ET.SubElement(parent,tag,**attributes)
    if text is not None:element.text=str(text)
    return element


def attach_inertial_lidar(source,output,mounting_path,config_path=DEFAULT_CONFIG):
    config=load_config(config_path);mounting=yaml.safe_load(Path(mounting_path).read_text())
    transforms=world_transforms(mounting);tree=ET.parse(source)
    world=tree.find('world');base=world.find("model[@name='patrol_car']/link[@name='base_link']")
    if base is None:raise ValueError('Missing patrol_car/base_link')
    # The 2D navigation scan will come from this 3D sensor, not a second lidar.
    for sensor in list(base.findall('sensor')):
        if sensor.findtext('topic')=='/sim/scan':base.remove(sensor)
    if world.find("plugin[@name='ignition::gazebo::systems::Imu']") is None:
        add(world,'plugin',filename='ignition-gazebo-imu-system',name='ignition::gazebo::systems::Imu')
    if world.find("plugin[@name='ignition::gazebo::systems::Sensors']") is None:
        add(add(world,'plugin',filename='ignition-gazebo-sensors-system',name='ignition::gazebo::systems::Sensors'),'render_engine','ogre2')
    # Fortress can render the directional-light icon into upward GPU scans.
    # Its position has no effect on directional illumination; place the icon
    # beyond sensing range and suppress its GUI visualization.
    for light in world.findall("light[@type='directional']"):
        visualize=light.find('visualize')
        if visualize is None:visualize=add(light,'visualize')
        visualize.text='false'
        pose=light.find('pose')
        if pose is None:pose=add(light,'pose')
        values=(pose.text or '0 0 0 0 0 0').split()
        values[2]=str(max(float(values[2]),config['lidar']['range_max_m']+100))
        pose.text=' '.join(values)
    # The official cloud origin is on the mounting plane, inside the visual mesh.
    # Exclude only that mesh and debug axes from GPU rays; keep them visible in GUI.
    for visual in base.findall('visual'):
        name=visual.get('name','')
        if name=='measured_l2_mesh' or '_axis_' in name:
            flags=visual.find('visibility_flags')
            if flags is None:flags=add(visual,'visibility_flags')
            flags.text='2'
    l=config['lidar'];sensor=add(base,'sensor',name='l2_lidar_3d',type='gpu_lidar')
    add(sensor,'pose',pose_text(transforms['unilidar_lidar']))
    add(sensor,'topic','/sim/l2/scan');add(sensor,'update_rate',l['rate_hz']);add(sensor,'always_on','true')
    add(sensor,'ignition_frame_id','unilidar_lidar')
    # Bit 0 is Gazebo GUI-only geometry (e.g. the sun gizmo); bit 1 is our
    # excluded sensor/debug mesh. Bit 2 still sees ordinary scene visuals.
    lidar=add(sensor,'lidar');add(lidar,'visibility_mask',4);scan=add(lidar,'scan')
    h=add(scan,'horizontal');add(h,'samples',l['horizontal_samples']);add(h,'resolution',1)
    add(h,'min_angle',-math.pi);add(h,'max_angle',math.pi-2*math.pi/l['horizontal_samples'])
    v=add(scan,'vertical');add(v,'samples',l['vertical_samples']);add(v,'resolution',1)
    add(v,'min_angle',math.radians(l['elevation_min_deg']));add(v,'max_angle',math.radians(l['elevation_max_deg']))
    r=add(lidar,'range');add(r,'min',l['range_min_m']);add(r,'max',l['range_max_m']);add(r,'resolution',.001)
    noise=add(lidar,'noise');add(noise,'type','gaussian');add(noise,'mean',0);add(noise,'stddev',l['range_noise_stddev_m'])
    builtin=transforms['unilidar_lidar'].copy();offset=np.eye(4);offset[:3,3]=BUILTIN_IMU_OFFSET;builtin=builtin@offset
    if 'unilidar_imu' in transforms:
        # Follow the commissioned base_link -> IMU transform.  The factory
        # offset remains a consistency check, not an alternate TF parent.
        if not np.allclose(transforms['unilidar_imu'],builtin,atol=1e-6):
            raise ValueError('L2 lidar and IMU mounting disagree with factory offset')
        builtin=transforms['unilidar_imu']
    for name,frame,transform,topic in [('l2_builtin_imu','unilidar_imu',builtin,'/sim/l2/imu'),
            ('external_imu','imu_link',transforms['imu_link'],'/sim/external/imu')]:
        sensor=add(base,'sensor',name=name,type='imu');add(sensor,'pose',pose_text(transform))
        add(sensor,'always_on','true');add(sensor,'update_rate',config['imu']['rate_hz']);add(sensor,'topic',topic)
        add(sensor,'ignition_frame_id',frame);imu=add(sensor,'imu')
        reference=add(imu,'orientation_reference_frame');add(reference,'localization','CUSTOM')
        add(reference,'custom_rpy','0 0 0',parent_frame='world')
        for measure,key in [('angular_velocity','angular_velocity_noise_stddev'),('linear_acceleration','acceleration_noise_stddev')]:
            group=add(imu,measure)
            for axis in ('x','y','z'):
                noise=add(add(group,axis),'noise',type='gaussian');add(noise,'mean',0);add(noise,'stddev',config['imu'][key])
    ET.indent(tree,space='  ');tree.write(output,encoding='utf-8',xml_declaration=True)
    frames=copy.deepcopy(mounting)
    if 'unilidar_imu' not in frames:
        frames['unilidar_imu']=dict(parent='unilidar_lidar',**dict(zip(('x','y','z'),BUILTIN_IMU_OFFSET)),roll=0,pitch=0,yaw=0)
    return frames


def bridges():
    return [dict(ros_topic_name='/sim/l2/points',gz_topic_name='/sim/l2/scan/points',ros_type_name='sensor_msgs/msg/PointCloud2',gz_type_name='ignition.msgs.PointCloudPacked',direction='GZ_TO_ROS'),
            *[dict(ros_topic_name=topic,gz_topic_name=topic,ros_type_name='sensor_msgs/msg/Imu',gz_type_name='ignition.msgs.IMU',direction='GZ_TO_ROS') for topic in ('/sim/l2/imu','/sim/external/imu')]]
