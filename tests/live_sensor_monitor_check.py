#!/usr/bin/python3
"""Synthetic topic/UI check; refuses to run alongside any simulation or robot nodes."""
import base64
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan, PointCloud2, PointField
from rosgraph_msgs.msg import Clock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'test'))
from verify_sensor_mounting_browser import Browser


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='42'
    rclpy.init();node=Node('sim_sensor_monitor_test')
    publishers=[node.create_publisher(Range,f'/ultrasonic_{i}',10) for i in range(1,5)]
    clock=node.create_publisher(Clock,'/clock',10)
    scan=node.create_publisher(LaserScan,'/scan',10)
    cloud=node.create_publisher(PointCloud2,'/unilidar/cloud',10)
    registered=node.create_publisher(PointCloud2,'/pointlio/cloud_registered',10)
    alarm=[]
    from std_msgs.msg import Bool
    sub=node.create_subscription(Bool,'/sim/proximity_alarm',lambda msg:alarm.append(msg.data),10)
    time.sleep(1.)
    others=set(node.get_node_names())-{'sim_sensor_monitor','sim_sensor_monitor_test'}
    if others:
        node.destroy_node();rclpy.shutdown()
        raise RuntimeError(f'Refusing synthetic sensor/clock traffic alongside nodes: {others}')
    browser=Browser(2835)
    results=[]
    sim_time=100.
    def state():return json.load(urlopen('http://127.0.0.1:8768/api/state',timeout=3))
    def send(distances, duration=1.2, stale=False):
        nonlocal sim_time
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            sim_time+=.05
            t=Clock();t.clock.sec=int(sim_time);t.clock.nanosec=int((sim_time%1)*1e9);clock.publish(t)
            for pub,distance in zip(publishers,distances):
                msg=Range();msg.header.stamp=t.clock
                if stale:msg.header.stamp.sec-=10
                msg.min_range=.03;msg.max_range=.55;msg.field_of_view=math.pi/3;msg.range=distance;pub.publish(msg)
            msg=LaserScan();msg.header.frame_id='unilidar_lidar';msg.angle_min=-1.;msg.angle_increment=.5
            msg.range_min=.1;msg.range_max=30.;msg.ranges=[1.,2.,3.,2.,1.];scan.publish(msg)
            msg=PointCloud2();msg.header.frame_id='unilidar_lidar';msg.height=1;msg.width=3
            msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z'])]
            msg.point_step=12;msg.row_step=36;msg.data=struct.pack('<9f',1.,2.,0.,-1.,1.,1.,0.,-1.,2.);cloud.publish(msg)
            msg.header.stamp=t.clock;msg.header.frame_id='pointlio_map';registered.publish(msg)
            rclpy.spin_once(node,timeout_sec=.05)
        return state()
    try:
        browser.call('WebDriver:SetWindowRect',dict(width=1250,height=1000))
        browser.call('WebDriver:Navigate',dict(url='http://127.0.0.1:8768'))
        browser.wait('document.getElementById("alarm").textContent.includes("数据未就绪")')
        results.append('Missing sensor data is shown as unknown')
        s=send([.3]*4);assert not s['unknown'] and not s['alarm']
        browser.wait('document.getElementById("alarm").className === "clear"')
        s=send([.3,.1,.3,.3]);assert s['alarm'] and s['channels'][1]['near']
        browser.wait('document.getElementById("alarm").className === "near"')
        assert any(alarm)
        results.append('10 cm on channel 2 triggers ROS Bool alarm and red browser warning')
        s=send([.3,.11,.3,.3]);assert s['alarm']
        s=send([.3,.13,.3,.3]);assert not s['alarm']
        results.append('11 cm retains alarm; 13 cm clears hysteresis')
        s=send([-math.inf,math.inf,math.nan,.3]);assert s['alarm'] and s['unknown']
        results.append('Too close, no echo and invalid data remain distinct')
        s=send([.3]*4,stale=True);assert s['alarm']
        time.sleep(2.2);s=state();assert s['channels'][0]['kind']=='stale' and s['alarm']
        browser.wait('document.getElementById("cards").textContent.includes("数据过期")')
        results.append('Stale measurements cannot clear previous near alarm; UI flags stale')
        send([.3]*4)
        assert len(state()['views']['/scan']['points'])==5
        assert len(state()['views']['/unilidar/cloud']['points'])==3
        assert len(state()['views']['/viewer/accumulated_map']['points'])==3
        assert state()['views']['/viewer/accumulated_map']['frame']=='pointlio_map'
        browser.js('var e=document.getElementById("view");e.value="/unilidar/cloud";e.dispatchEvent(new Event("change"));')
        browser.wait('document.getElementById("viewInfo").textContent.includes("3 点")')
        element=browser.call('WebDriver:FindElement',{'using':'css selector','value':'#sound'})
        browser.call('WebDriver:ElementClick',{'id':element['element-6066-11e4-a52e-4f735466cecf']})
        browser.wait('document.getElementById("sound").textContent.includes("关闭")')
        results.append('Scan/cloud decode, registered cloud accumulation and browser selector work; audio context enables')
        image=browser.call('WebDriver:TakeScreenshot',{})
        (ROOT/'artifacts/sensor_monitor_desktop.png').write_bytes(base64.b64decode(image))
        browser.call('WebDriver:SetWindowRect',dict(width=430,height=900))
        assert browser.js('return document.documentElement.scrollWidth <= window.innerWidth')
        results.append('Narrow browser layout has no horizontal overflow')
        send([.3]*4)
        output=dict(type='synthetic ROS topics and headless Firefox; no Gazebo physics',checks=results,passed=True)
        (ROOT/'artifacts/sensor_monitor_validation.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(output,ensure_ascii=False))
    finally:
        try:browser.call('Marionette:Quit',{'flags':['eForceQuit']})
        except Exception:pass
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
