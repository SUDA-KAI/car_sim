#!/usr/bin/python3
"""Generate a synthetic map and a simplified Ackermann car (no measured CAD)."""
from pathlib import Path
import xml.etree.ElementTree as E
import yaml
from PIL import Image, ImageDraw
from build_world import tag, box, ROOT, build

im = Image.new('L', (400, 400), 254)
draw = ImageDraw.Draw(im)
draw.rectangle((0, 0, 399, 399), outline=0, width=5)
draw.rectangle((170, 170, 230, 230), fill=0)
im.save(ROOT/'maps/demo.pgm')
(ROOT/'maps/demo.yaml').write_text(yaml.safe_dump(dict(image='demo.pgm', resolution=0.05, origin=[-10.,-10.,0.], negate=0, occupied_thresh=0.65, free_thresh=0.196)))
sdf = E.Element('sdf', version='1.8')
model = tag(sdf, 'model', name='patrol_car')
tag(model, 'pose', '-5 -5 0 0 0 0')
base = box(model, 'base_link', '0 0 0.45 0 0 0', '1.2 0.70 0.5', '0.1 0.4 0.8 1')
inertial = tag(base, 'inertial')
tag(inertial, 'mass', 40)
inertia = tag(inertial, 'inertia')
for key, value in [('ixx', 3), ('iyy', 6), ('izz', 7)]: tag(inertia, key, value)
# Wheelbase 0.5 m follows the existing chassis config; wheel radius/track are provisional.
for side, y in [('left', 0.39), ('right', -0.39)]:
    for end, x in [('front', 0.25), ('rear', -0.25)]:
        name = f'{end}_{side}'
        parent = 'base_link'
        if end == 'front':
            parent = name+'_knuckle'
            link = tag(model, 'link', name=parent)
            tag(link, 'pose', f'{x} {y} 0.16 0 0 0')
            inertial = tag(link, 'inertial'); tag(inertial, 'mass', 0.2)
            inertia = tag(inertial, 'inertia')
            for k in ('ixx','iyy','izz'): tag(inertia,k,0.001)
            joint = tag(model, 'joint', name=name+'_steer', type='revolute')
            tag(joint,'parent','base_link'); tag(joint,'child',parent)
            axis = tag(joint,'axis'); tag(axis,'xyz','0 0 1')
            limit = tag(axis,'limit'); tag(limit,'lower',-0.7); tag(limit,'upper',0.7)
        wheel = tag(model, 'link', name=name+'_wheel')
        tag(wheel, 'pose', f'{x} {y} 0.16 -1.57079632679 0 0')
        inertial=tag(wheel,'inertial'); tag(inertial,'mass',1)
        inertia=tag(inertial,'inertia')
        for k in ('ixx','iyy','izz'): tag(inertia,k,0.015)
        for kind in ('collision','visual'):
            part=tag(wheel,kind,name=kind); cylinder=tag(tag(part,'geometry'),'cylinder')
            tag(cylinder,'radius',0.16); tag(cylinder,'length',0.12)
            if kind == 'visual':
                mat=tag(part,'material'); tag(mat,'ambient','0.1 0.1 0.1 1'); tag(mat,'diffuse','0.1 0.1 0.1 1')
        joint=tag(model,'joint',name=name+'_roll',type='revolute')
        tag(joint,'parent',parent); tag(joint,'child',name+'_wheel')
        axis=tag(joint,'axis'); tag(axis,'xyz','0 0 1')
plugin=tag(model,'plugin',filename='ignition-gazebo-ackermann-steering-system',name='ignition::gazebo::systems::AckermannSteering')
for key,value in [('left_joint','rear_left_roll'),('right_joint','rear_right_roll'),('left_steering_joint','front_left_steer'),('right_steering_joint','front_right_steer'),('wheel_base',0.5),('wheel_separation',0.78),('kingpin_width',0.78),('wheel_radius',0.16),('steering_limit',0.46365),('topic','/sim/cmd_vel'),('odom_topic','/sim/odom'),('tf_topic','/sim/tf'),('frame_id','odom'),('child_frame_id','base_link'),('odom_publish_frequency',30),('min_velocity',-0.2),('max_velocity',0.8)]: tag(plugin,key,value)
sensor=tag(base,'sensor',name='lidar',type='gpu_lidar')
tag(sensor,'pose','0 0 0.40 0 0 0')
tag(sensor,'topic','/sim/scan'); tag(sensor,'update_rate',10)
tag(sensor,'always_on','true'); tag(sensor,'ignition_frame_id','laser_frame')
lidar=tag(sensor,'lidar'); scan=tag(lidar,'scan'); horizontal=tag(scan,'horizontal')
for key,value in [('samples',720),('resolution',1),('min_angle',-3.14159265),('max_angle',3.14159265)]:tag(horizontal,key,value)
rng=tag(lidar,'range')
for key,value in [('min',0.15),('max',25),('resolution',0.01)]:tag(rng,key,value)
E.indent(sdf); E.ElementTree(sdf).write(ROOT/'models/demo_car.sdf',encoding='utf-8',xml_declaration=True)
build(ROOT/'maps/demo.yaml',ROOT/'worlds/demo.sdf',1.5,[-5,-5,0],robot_path=ROOT/'models/demo_car.sdf')
