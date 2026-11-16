"""Replace Fortress' coupled Ackermann actuator at launch time."""
import xml.etree.ElementTree as ET
import math
from chassis_config import load_chassis, inner_wheel_limit


def install_split_drive(source, destination):
    tree = ET.parse(source)
    robot = tree.find(".//model[@name='patrol_car']")
    if robot is None:
        raise ValueError('patrol_car model is missing')
    old = robot.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']")
    if old is None:
        raise ValueError('AckermannSteering plugin is missing')
    robot.remove(old)
    chassis=load_chassis()
    half_base=chassis['wheelbase_m']/2.
    for end,x in (('front',half_base),('rear',-half_base)):
        for side in ('left','right'):
            name=f'{end}_{side}_{"steer" if end=="front" else "roll"}'
            joint=robot.find(f"joint[@name='{name}']")
            if joint is None:raise ValueError(f'{name} joint is missing')
            pose=joint.find('pose')
            values=(pose.text or '').split()
            if len(values)!=6:raise ValueError(f'{name} joint pose is invalid')
            values[0]=str(x)
            pose.text=' '.join(values)
    for name in ('front_left_steer','front_right_steer'):
        limit=robot.find(f"joint[@name='{name}']/axis/limit")
        if limit is None:raise ValueError(f'{name} joint limit is missing')
        limit.find('lower').text=str(-inner_wheel_limit(chassis))
        limit.find('upper').text=str(inner_wheel_limit(chassis))
    for joint, topic in [('front_left_steer', 'front_left'), ('front_right_steer', 'front_right')]:
        plugin = ET.SubElement(robot, 'plugin', filename='ignition-gazebo-joint-position-controller-system',
                               name='ignition::gazebo::systems::JointPositionController')
        ET.SubElement(plugin, 'joint_name').text = joint
        ET.SubElement(plugin, 'topic').text = '/sim/drive/' + topic
        ET.SubElement(plugin, 'use_velocity_commands').text = 'true'
        ET.SubElement(plugin, 'cmd_max').text = '1.0'
    for joint, topic in [('rear_left_roll', 'rear_left'), ('rear_right_roll', 'rear_right')]:
        plugin = ET.SubElement(robot, 'plugin', filename='ignition-gazebo-joint-controller-system',
                               name='ignition::gazebo::systems::JointController')
        ET.SubElement(plugin, 'joint_name').text = joint
        ET.SubElement(plugin, 'topic').text = '/sim/drive/' + topic
    plugin = ET.SubElement(robot, 'plugin', filename='ignition-gazebo-joint-state-publisher-system',
                           name='ignition::gazebo::systems::JointStatePublisher')
    ET.SubElement(plugin, 'topic').text = '/sim/drive/joint_state'
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='utf-8', xml_declaration=True)


def bridges():
    result = []
    for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        topic = '/sim/drive/' + name
        result.append(dict(ros_topic_name=topic, gz_topic_name=topic,
                           ros_type_name='std_msgs/msg/Float64', gz_type_name='ignition.msgs.Double',
                           direction='ROS_TO_GZ'))
    result.append(dict(ros_topic_name='/sim/drive/joint_state', gz_topic_name='/sim/drive/joint_state',
                       ros_type_name='sensor_msgs/msg/JointState', gz_type_name='ignition.msgs.Model',
                       direction='GZ_TO_ROS'))
    return result
