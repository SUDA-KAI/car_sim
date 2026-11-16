#!/usr/bin/python3
"""Explicit simulation-only throttle/brake/steer profile for friction A/B checks."""
import os,time,json,math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','42'), 'Use a separate test-only domain'
rclpy.init();node=rclpy.create_node('surface_physics_profile');latest={};truth_samples=[]
pub=node.create_publisher(Twist,'/cmd_vel',10)
subs=[node.create_subscription(Clock,'/clock',lambda m:latest.__setitem__('time',m.clock.sec+m.clock.nanosec/1e9),qos_profile_sensor_data),
      node.create_subscription(Odometry,'/ground_truth/odom',lambda m:latest.__setitem__('truth',m),qos_profile_sensor_data),
      node.create_subscription(Odometry,'/wheel/odom',lambda m:latest.__setitem__('wheel',m),qos_profile_sensor_data)]
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
try:
 deadline=time.monotonic()+90
 while len(latest)<3 and time.monotonic()<deadline:spin(.1)
 assert len(latest)==3,'Simulation topics not ready'
 assert 'controller_server' not in node.get_node_names() and 'sim_joystick_control' not in node.get_node_names(),'Profile needs world-only mode'
 profiles=[('accelerate',12.,.6,0.),('brake',3.,0.,0.),('steer',8.,.6,.2),('stop',3.,0.,0.)]
 result={}
 for name,duration,speed,yaw in profiles:
  start=latest['time'];gaps=[];max_lateral=0.;x0=latest['truth'].pose.pose.position.x;y0=latest['truth'].pose.pose.position.y
  wall_deadline=time.monotonic()+120
  while latest['time']-start<duration:
   if time.monotonic()>wall_deadline:raise RuntimeError('Simulation stalled')
   m=Twist();m.linear.x=speed;m.angular.z=yaw;pub.publish(m);spin(.05)
   t=latest['truth'];w=latest['wheel'];gaps.append(w.twist.twist.linear.x-t.twist.twist.linear.x)
   max_lateral=max(max_lateral,abs(t.twist.twist.linear.y))
  p=latest['truth'].pose.pose.position
  result[name]=dict(max_abs_wheel_speed_gap=max(map(abs,gaps)),rms_gap=math.sqrt(sum(g*g for g in gaps)/len(gaps)),
                    max_abs_lateral_speed=max_lateral,true_displacement_m=math.hypot(p.x-x0,p.y-y0),finish_xyz=[p.x,p.y,p.z])
 print(json.dumps(result,indent=2),flush=True)
finally:
 for _ in range(5):pub.publish(Twist());spin(.05)
 node.destroy_node();rclpy.shutdown()
