#!/usr/bin/python3
"""Explicit integration check: synthetic Joy -> cmd_vel -> Gazebo motion.
Run only with the isolated joystick simulation and its ROS_DOMAIN_ID.
"""
import json,math,os,time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy,LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0'), 'Use an isolated simulation domain'
rclpy.init();node=rclpy.create_node('verify_sim_joystick');received={}
subs=[]
for topic,kind in [('/cmd_vel',Twist),('/chassis/odom',Odometry),('/scan',LaserScan)]:
    subs.append(node.create_subscription(kind,topic,lambda msg,t=topic:received.__setitem__(t,msg),qos_profile_sensor_data))
pub=node.create_publisher(Joy,'/joy',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
def send(speed=0.,turn=0.,enable=False,seconds=.6):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        msg=Joy();msg.axes=[0.,speed,1.,turn,0.,1.,0.,0.];msg.buttons=[0,0,0,0,int(enable),0]
        msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg);spin(.05)
def command_zero():
    cmd=received['/cmd_vel'];assert cmd.linear.x==0 and cmd.angular.z==0
try:
    end=time.monotonic()+60
    while len(received)<3 and time.monotonic()<end:spin(.1)
    assert len(received)==3, f'Missing topics: {list(received)}'
    assert 'controller_server' not in node.get_node_names(), 'Navigation must be disabled'
    command_zero()
    send(speed=.6,turn=.3,enable=True);command_zero() # cannot start with a held button
    send();start=received['/chassis/odom'].pose.pose
    send(speed=.6,turn=.3,enable=True,seconds=3.)
    cmd=received['/cmd_vel'];assert cmd.linear.x>0 and cmd.angular.z>0
    finish=received['/chassis/odom'].pose.pose
    moved=math.hypot(finish.position.x-start.position.x,finish.position.y-start.position.y)
    assert moved>.05,f'Gazebo did not move: {moved}'
    yaw=lambda q:2*math.atan2(q.z,q.w)
    yaw_change=abs(yaw(finish.orientation)-yaw(start.orientation))
    assert yaw_change>.005,f'Gazebo did not turn: {yaw_change}'
    send(speed=.6,enable=False);command_zero()
    send();send(speed=.4,enable=True);assert received['/cmd_vel'].linear.x>0
    spin(.7);command_zero() # publisher stops; wall-clock watchdog must stop commands
    send(speed=.4,enable=True);command_zero() # timeout requires neutral/release again
    print(json.dumps(dict(result='PASS',synthetic_Joy=True,moved_m=moved,yaw_change_rad=yaw_change,
        checks=['manual_mode_without_nav2','startup_interlock','positive_speed_and_turn',
                'Gazebo_movement','enable_release_stop','input_timeout_stop','reconnect_interlock']),indent=2))
finally:
    send();node.destroy_node();rclpy.shutdown()
