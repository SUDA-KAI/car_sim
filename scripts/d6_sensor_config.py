"""Attach both low-mounted COIN-D6 scan planes to Gazebo."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml

DEFAULT_CONFIG=Path(__file__).resolve().parents[1]/'config/d6_sim.yaml'


def load_config(path=DEFAULT_CONFIG):
    config=yaml.safe_load(Path(path).read_text())
    if not isinstance(config,dict) or set(config.get('sensors',{}))!={'a','b'}:
        raise ValueError('Expected front D6 b and rear D6 a')
    if (not 0<config['rate_hz']<=50 or config['bins']<90 or
            not 0<=config['min_range_m']<config['max_range_m'] or
            config['range_noise_stddev_m']<0):
        raise ValueError('Invalid D6 scan configuration')
    for sensor in config['sensors'].values():
        if not all(math.isfinite(float(sensor[key])) for key in ('x','y','z','yaw')):
            raise ValueError('Nonfinite D6 mounting')
    return config


def add_mounts(mounting,config):
    mounting=dict(mounting)
    for sensor in config['sensors'].values():
        mounting[sensor['frame']]=dict(parent='base_link',
            x=sensor['x'],y=sensor['y'],z=sensor['z'],
            roll=0,pitch=0,yaw=sensor['yaw'])
    return mounting


def attach_sensors(source,output,config):
    tree=ET.parse(source)
    world=tree.find('world')
    base=world.find("model[@name='patrol_car']/link[@name='base_link']")
    if base is None:raise ValueError('Missing patrol_car/base_link')
    if world.find("plugin[@name='ignition::gazebo::systems::Sensors']") is None:
        plugin=ET.SubElement(world,'plugin',filename='ignition-gazebo-sensors-system',
                            name='ignition::gazebo::systems::Sensors')
        ET.SubElement(plugin,'render_engine').text='ogre2'
    for key,spec in config['sensors'].items():
        sensor=ET.SubElement(base,'sensor',name=f'd6_{key}_lidar',type='gpu_lidar')
        values=[spec[k] for k in ('x','y','z')]+[0,0,spec['yaw']]
        ET.SubElement(sensor,'pose').text=' '.join(map(str,values))
        ET.SubElement(sensor,'topic').text=f'/sim/d6_{key}/scan'
        ET.SubElement(sensor,'update_rate').text=str(config['rate_hz'])
        ET.SubElement(sensor,'always_on').text='true'
        ET.SubElement(sensor,'ignition_frame_id').text=spec['frame']
        lidar=ET.SubElement(sensor,'lidar')
        scan=ET.SubElement(lidar,'scan')
        horizontal=ET.SubElement(scan,'horizontal')
        for name,value in [('samples',config['bins']),('resolution',1),
                           ('min_angle',-math.pi),
                           ('max_angle',math.pi-2*math.pi/config['bins'])]:
            ET.SubElement(horizontal,name).text=str(value)
        limits=ET.SubElement(lidar,'range')
        for name,value in [('min',config['min_range_m']),('max',config['max_range_m']),('resolution',.001)]:
            ET.SubElement(limits,name).text=str(value)
        noise=ET.SubElement(lidar,'noise')
        ET.SubElement(noise,'type').text='gaussian'
        ET.SubElement(noise,'mean').text='0'
        ET.SubElement(noise,'stddev').text=str(config['range_noise_stddev_m'])
    ET.indent(tree,space='  ')
    tree.write(output,encoding='utf-8',xml_declaration=True)


def bridges():
    return [dict(ros_topic_name=f'/sim/d6_{key}/scan',gz_topic_name=f'/sim/d6_{key}/scan',
                 ros_type_name='sensor_msgs/msg/LaserScan',gz_type_name='ignition.msgs.LaserScan',
                 direction='GZ_TO_ROS') for key in ('a','b')]
