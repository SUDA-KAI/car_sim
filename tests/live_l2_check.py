#!/usr/bin/python3
"""Bounded physics check; run only in domain 42 with narrow_17 world-only mode."""
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu,PointCloud2,LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from l2_sim import read_xyz

assert os.environ.get('ROS_DOMAIN_ID')=='42','Simulation domain 42 required'
rclpy.init();node=rclpy.create_node('bounded_l2_imu_physics_check');latest={};records={};sim=[None];frames={};cmd=node.create_publisher(Twist,'/cmd_vel',10)
def receive(topic,m):
    latest[topic]=m;stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    row=dict(stamp=stamp,frame=m.header.frame_id)
    if isinstance(m,Odometry):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        row.update(x=p.x,y=p.y,z=p.z,v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,
                   yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    elif isinstance(m,Imu):
        a=m.linear_acceleration;w=m.angular_velocity;q=m.orientation
        row.update(acc=[a.x,a.y,a.z],gyro=[w.x,w.y,w.z],q=[q.x,q.y,q.z,q.w])
    elif isinstance(m,PointCloud2):
        xyz=read_xyz(m)
        row.update(points=len(xyz),z_min=float(xyz[:,2].min()) if len(xyz) else None,z_max=float(xyz[:,2].max()) if len(xyz) else None,fields=[f.name for f in m.fields])
    elif isinstance(m,LaserScan):row.update(left=m.ranges[270],right=m.ranges[90],finite=sum(math.isfinite(x) for x in m.ranges))
    records.setdefault(topic,[]).append(row)
for topic,kind in [('/ground_truth/odom',Odometry),('/pointlio/odometry',Odometry),('/unilidar/cloud',PointCloud2),('/scan',LaserScan),('/unilidar/imu',Imu),('/imu/data_raw',Imu)]:
    node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),rclpy.qos.qos_profile_sensor_data)
node.create_subscription(Clock,'/clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage,'/tf_static',lambda m:frames.update({t.child_frame_id:t.header.frame_id for t in m.transforms}),qos)
def publish(v=0.,w=0.):
    m=Twist();m.linear.x=v;m.angular.z=w;cmd.publish(m)
def spin_phase(duration,v=0.,w=0.):
    begin=sim[0];deadline=time.monotonic()+60
    while sim[0]-begin<duration:
        if time.monotonic()>deadline:raise TimeoutError('Physics too slow or paused')
        if 'controller_server' in [name for name,_ in node.get_node_names_and_namespaces()]:raise RuntimeError('Navigation must be disabled for this check')
        odom=latest['/ground_truth/odom'];p=odom.pose.pose.position
        if abs(p.y)>.27:raise RuntimeError('Check exceeded its lateral bound')
        publish(v,w);rclpy.spin_once(node,timeout_sec=.02)
report={}
try:
    deadline=time.monotonic()+45
    while len(latest)<6 or sim[0] is None:
        if time.monotonic()>deadline:raise TimeoutError('Missing sensor/LIO data: '+str(list(latest)))
        rclpy.spin_once(node,timeout_sec=.05)
    p=latest['/ground_truth/odom'].pose.pose.position
    assert abs(p.x+3)<.1 and abs(p.y-.1)<.05,'Expected narrow_17 starting position'
    spin_phase(2.0);static={t:records[t][-1] for t in records};start=static['/ground_truth/odom']
    assert static['/unilidar/cloud']['z_max']<.65,'GUI/sky ghost returns'
    spin_phase(4.0,.2,.04);spin_phase(1.5);finish={t:records[t][-1] for t in records}
    report.update(static=static,finish=finish,frames=frames)
    report['displacement_m']=math.hypot(finish['/ground_truth/odom']['x']-start['x'],finish['/ground_truth/odom']['y']-start['y'])
    report['max_abs_gyro_z']=max(abs(r['gyro'][2]) for r in records['/unilidar/imu'])
    for t in ('/unilidar/imu','/imu/data_raw'):
        assert abs(static[t]['acc'][2]-9.8)<.05,(t,'gravity')
        assert max(abs(v) for v in static[t]['gyro'])<.001,(t,'stationary gyro')
    assert static['/unilidar/imu']['frame']=='unilidar_imu'
    assert static['/imu/data_raw']['frame']=='imu_link'
    assert static['/unilidar/cloud']['points']>100
    assert static['/unilidar/cloud']['z_max']<.65,'GUI/sky ghost returns'
    assert report['displacement_m']>.5 and report['max_abs_gyro_z']>.015
    assert frames['unilidar_imu']=='unilidar_lidar'
    assert frames['unilidar_lidar']=='third_crossbar'
    report['sim_rates_hz']={t:(len(rows)-1)/(rows[-1]['stamp']-rows[0]['stamp']) for t,rows in records.items() if len(rows)>1 and rows[-1]['stamp']>rows[0]['stamp']}
    report['result']='passed'
except Exception as error:
    report.update(result='failed',error=str(error),available=list(latest));raise
finally:
    for _ in range(12):publish();rclpy.spin_once(node,timeout_sec=.02)
    (ROOT/'artifacts/l2_imu_live_validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2));node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
