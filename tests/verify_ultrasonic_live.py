#!/usr/bin/python3
"""Read sensors/TF and inject a removable obstacle in an isolated simulation."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Range
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from map_msgs.msg import OccupancyGridUpdate
from tf2_ros import Buffer,TransformListener

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from ultrasonic_config import load_mounting


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','42') or os.environ.get('IGN_PARTITION')!='ultrasonic_verify':
        raise RuntimeError('Use an isolated ROS domain and IGN_PARTITION=ultrasonic_verify')
    rclpy.init();node=Node('verify_ultrasonic_live');messages={};costmap=[None]
    buffer=Buffer();listener=TransformListener(buffer,node)
    subs=[node.create_subscription(Range,f'/ultrasonic_{i}',lambda m,i=i:messages.__setitem__(i,m),10) for i in range(1,5)]
    subs.append(node.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:costmap.__setitem__(0,m),10))
    def update_costmap(update):
        m=costmap[0]
        if m is None:return
        for row in range(update.height):
            for col in range(update.width):
                value=update.data[row*update.width+col]
                cost=254 if value==100 else (253 if value==99 else (255 if value<0 else round(value*252/98)))
                m.data[(update.y+row)*m.metadata.size_x+update.x+col]=cost
    subs.append(node.create_subscription(OccupancyGridUpdate,'/local_costmap/costmap_updates',update_costmap,10))
    map_client=node.create_client(GetCostmap,'/local_costmap/get_costmap')
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
            if predicate():return
        raise RuntimeError('Timed out waiting for simulation evidence')
    result={};created=False;wall_removed=False
    wall_model=ET.parse(ROOT/'test_scenes/worlds/narrow_17.sdf').find("world/model[@name='right_wall']")
    wall_sdf='<sdf version="1.8">'+ET.tostring(wall_model,encoding='unicode')+'</sdf>'
    def service(name,request_type,request):
        proc=subprocess.run(['ign','service','-s',f'/world/patrol/{name}','--reqtype',request_type,
            '--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],text=True,capture_output=True,timeout=10)
        if proc.returncode or 'data: true' not in proc.stdout:
            raise RuntimeError(f'Gazebo service failed: {proc.stdout} {proc.stderr}')
    def cell_value(x,y):
        m=costmap[0]
        if m is None:return None
        col=int((x-m.metadata.origin.position.x)/m.metadata.resolution);row=int((y-m.metadata.origin.position.y)/m.metadata.resolution)
        if 0<=col<m.metadata.size_x and 0<=row<m.metadata.size_y:return m.data[row*m.metadata.size_x+col]
    try:
        wait(lambda:len(messages)==4 and costmap[0] is not None,55)
        wait(lambda:buffer.can_transform('map','base_link',rclpy.time.Time()),10)
        vehicle=buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
        q=vehicle.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        def world_point(x,y):
            return (vehicle.translation.x+math.cos(yaw)*x-math.sin(yaw)*y,
                    vehicle.translation.y+math.sin(yaw)*x+math.cos(yaw)*y)
        arc_x,arc_y=world_point(.49,-.67)
        box_x,box_y=world_point(.49,-.72)
        expected={i:(.85-abs(world_point(s['x'],s['y'])[1]) if i>2 else .85+world_point(s['x'],s['y'])[1])
                  for i,s in enumerate(load_mounting(),1)}
        result['expected_wall_ranges_m']=expected
        result['initial_ranges_m']={str(i):m.range for i,m in messages.items()}
        for i,m in messages.items():
            assert m.header.frame_id==f'ultrasonic_{i}' and m.radiation_type==Range.ULTRASOUND
            assert abs(m.range-expected[i])<.015,(i,m.range,expected[i])
            assert abs(m.field_of_view-math.pi/3)<1e-6 and abs(m.max_range-.55)<1e-6
        result['transforms']={}
        for i in range(1,5):
            tf=buffer.lookup_transform('base_link',f'ultrasonic_{i}',rclpy.time.Time()).transform
            assert abs(tf.translation.z-.49)<1e-6
            result['transforms'][str(i)]=dict(x=tf.translation.x,y=tf.translation.y,z=tf.translation.z,qz=tf.rotation.z,qw=tf.rotation.w)
        result['baseline_obstacle_cell']=cell_value(arc_x,arc_y)
        sdf=f"""<sdf version='1.8'><model name='ultrasonic_verify_box'><static>true</static>
        <pose>{box_x} {box_y} 0.49 0 0 {yaw}</pose><link name='body'>
        <collision name='collision'><geometry><box><size>0.12 0.10 0.14</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>0.12 0.10 0.14</size></box></geometry></visual>
        </link></model></sdf>"""
        service('create','ignition.msgs.EntityFactory','sdf: '+json.dumps(sdf));created=True
        wait(lambda:abs(messages[2].range-.195)<.02,30)
        result['obstacle_ranges_m']={str(i):m.range for i,m in messages.items()}
        # Sonar is angularly ambiguous; verify a lethal mark near its return arc.
        def maximum_arc_cost():
            future=map_client.call_async(GetCostmap.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            if future.done() and future.result() is not None:
                costmap[0]=future.result().map
            values=[cell_value(*world_point(x,y)) for x in (.43,.45,.47,.49,.51,.53,.55)
                       for y in (-.65,-.67,-.69)]
            result['arc_cell_values']=values
            return max(value or 0 for value in values)
        wait(lambda:maximum_arc_cost()==254,30)
        result['obstacle_arc_max_cost']=maximum_arc_cost()
        service('remove','ignition.msgs.Entity','name: "ultrasonic_verify_box" type: MODEL');created=False
        wait(lambda:abs(messages[2].range-expected[2])<.015,30)
        result['removed_ranges_m']={str(i):m.range for i,m in messages.items()}
        wait(lambda:maximum_arc_cost()<254,30)
        result['removed_arc_max_cost']=maximum_arc_cost()
        service('remove','ignition.msgs.Entity','name: "right_wall" type: MODEL');wall_removed=True
        wait(lambda:all(math.isinf(messages[i].range) and messages[i].range>0 for i in (1,2)),30)
        result['right_wall_removed_no_echo_channels']=[1,2]
        service('create','ignition.msgs.EntityFactory','sdf: '+json.dumps(wall_sdf));wall_removed=False
        wait(lambda:all(abs(messages[i].range-expected[i])<.015 for i in (1,2)),30)
        result['result']='passed'
    finally:
        if created:
            service('remove','ignition.msgs.Entity','name: "ultrasonic_verify_box" type: MODEL')
        if wall_removed:
            service('create','ignition.msgs.EntityFactory','sdf: '+json.dumps(wall_sdf))
        (ROOT/'artifacts/ultrasonic_live_validation.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
