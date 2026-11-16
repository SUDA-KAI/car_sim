#!/usr/bin/python3
"""Build an open inspection world showing the measured sensor mountings."""
import copy
import re
from pathlib import Path
import xml.etree.ElementTree as ET
from sensor_visuals import ROOT,add_visuals
from assembly_config import APPLIED,load_applied,apply_visuals


def add(parent,tag_name,text=None,**attributes):
    element=ET.SubElement(parent,tag_name,attributes)
    if text is not None:element.text=str(text)
    return element


def build():
    tree=ET.ElementTree(ET.Element('sdf',version='1.8'));world=add(tree.getroot(),'world',name='sensor_mounting_preview')
    physics=add(world,'physics',name='physics',type='ignored');add(physics,'max_step_size',.002);add(physics,'real_time_factor',1)
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        add(world,'plugin',filename=f'ignition-gazebo-{filename}-system',name=f'ignition::gazebo::systems::{name}')
    scene=add(world,'scene');add(scene,'ambient','.65 .65 .65 1');add(scene,'background','.85 .89 .92 1')
    light=add(world,'light',name='sun',type='directional');add(light,'pose','0 0 10 0 0 0');add(light,'diffuse','.85 .85 .85 1');add(light,'direction','-.3 .2 -1')
    floor=add(world,'model',name='ground');add(floor,'static','true');link=add(floor,'link',name='ground')
    for kind in ('visual','collision'):
        part=add(link,kind,name=kind);geometry=add(part,'geometry');plane=add(geometry,'plane');add(plane,'normal','0 0 1');add(plane,'size','8 8')
        if kind=='visual':
            material=add(part,'material');add(material,'ambient','.65 .69 .71 1');add(material,'diffuse','.65 .69 .71 1')
    robot=copy.deepcopy(ET.parse(ROOT/'models/patrol_car.sdf').find('model'))
    add(robot,'pose','0 0 .002 0 0 0')
    for plugin in list(robot.findall('plugin')):robot.remove(plugin)
    for body in robot.findall('link'):
        for sensor in list(body.findall('sensor')):body.remove(sensor)
    # Resolve existing resources relative to the model directory.
    for uri in robot.findall('.//uri'):
        if uri.text and '://' not in uri.text:uri.text=(ROOT/'models'/uri.text).resolve().as_uri()
    world.append(robot)
    if APPLIED.exists():
        state,assets,mapping=load_applied()
        mounting=APPLIED.parent/'sensor_mounting_tf.yaml'
        add_visuals(tree,mounting,axes=False,l2_mesh=assets['l2']['path'])
        apply_visuals(tree,state,assets,mapping)
    else:add_visuals(tree,ROOT/'sensor_mounting_tf.yaml',axes=True)
    output=ROOT/'worlds/sensor_mounting_preview.sdf';ET.indent(tree,space='  ');tree.write(output,encoding='utf-8',xml_declaration=True)
    # Retain Gazebo's full standard UI and only adjust its initial inspection view.
    # Ignition GUI configs contain several top-level XML elements.
    gui=Path('/usr/share/ignition/ignition-gazebo6/gui/gui.config').read_text()
    gui=re.sub(r'<camera_pose>[^<]*</camera_pose>','<camera_pose>2.2 -2.6 1.9 0 0.36 2.27</camera_pose>',gui,count=1)
    gui=gui.replace('<width>1000</width>','<width>1280</width>',1).replace('<height>845</height>','<height>900</height>',1)
    gui_path=ROOT/'config/sensor_preview_gui.config';gui_path.write_text(gui)
    print(output)


if __name__=='__main__':build()
