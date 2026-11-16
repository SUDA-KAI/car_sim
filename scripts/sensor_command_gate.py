#!/usr/bin/python3
"""Simulation velocity gate: do not drive on prediction without fresh observations."""
import copy
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String


def input_health(samples,now,wall):
    def fresh(name,limit):
        if name not in samples:return False
        stamp,received=samples[name]
        return all(map(math.isfinite,(stamp,received,now,wall))) and -.05<=now-stamp<=limit and 0<=wall-received<=4.
    return dict(position=fresh('lio',.75) or fresh('visual',.75),fused=fresh('fused',.3),
                sonar=all(fresh(f'sonar_{i}',1.) for i in range(1,5)))


class CommandGate(Node):
    def __init__(self):
        super().__init__('sim_sensor_command_gate');self.samples={};self.command=Twist();self.command_received=None;self.previous=None;self.last_status=0.
        self.pub=self.create_publisher(Twist,'/localization/safe_cmd_vel',10)
        self.status=self.create_publisher(String,'/localization/gate_status',10)
        self.subs=[self.create_subscription(Twist,'/cmd_vel',self.receive_command,10)]
        for source,topic in [('lio','/localization/lidar_odom'),('visual','/localization/visual_odom'),('fused','/chassis/odom')]:
            self.subs.append(self.create_subscription(Odometry,topic,lambda msg,s=source:self.receive_stamp(s,msg),qos_profile_sensor_data))
        for i in range(1,5):
            self.subs.append(self.create_subscription(Range,f'/ultrasonic_{i}',lambda msg,s=f'sonar_{i}':self.receive_range(s,msg),qos_profile_sensor_data))
        self.create_timer(.05,self.tick,clock=rclpy.clock.Clock())

    def receive_stamp(self,name,msg):
        self.samples[name]=(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,time.monotonic())

    def receive_range(self,name,msg):
        if math.isnan(msg.range) or not 0<=msg.min_range<msg.max_range:
            self.samples.pop(name,None);return
        self.receive_stamp(name,msg)

    def receive_command(self,msg):
        values=[getattr(vector,key) for vector in (msg.linear,msg.angular) for key in ('x','y','z')]
        if not all(map(math.isfinite,values)):self.command_received=None;return
        self.command=copy.deepcopy(msg);self.command_received=time.monotonic()

    def tick(self):
        wall=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        health=input_health(self.samples,now,wall)
        command_fresh=self.command_received is not None and wall-self.command_received<.5
        enabled=all(health.values())
        self.pub.publish(self.command if enabled and command_fresh else Twist())
        state=dict(state='enabled' if enabled else 'blocked',inputs=health,command_fresh=command_fresh)
        if self.previous!=state or wall-self.last_status>.5:
            self.status.publish(String(data=json.dumps(state)));self.previous=state;self.last_status=wall


def main():
    rclpy.init();node=CommandGate()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
