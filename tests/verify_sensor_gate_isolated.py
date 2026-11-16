#!/usr/bin/python3
"""Fault injection in unused ROS domain 74; never starts Gazebo or hardware."""
import json
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from sensor_command_gate import CommandGate


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='74','Fault messages must stay out of robot/simulation domains'
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    gate=CommandGate();test=rclpy.create_node('isolated_gate_fault_test');executor=SingleThreadedExecutor();executor.add_node(gate);executor.add_node(test)
    cmd=test.create_publisher(Twist,'/cmd_vel',10);fused=test.create_publisher(Odometry,'/chassis/odom',10)
    lio=test.create_publisher(Odometry,'/localization/lio_odom',10);clock=test.create_publisher(Clock,'/clock',10)
    sonar=[test.create_publisher(Range,f'/ultrasonic_{i}',10) for i in range(1,5)]
    output=[];status=[];subs=[test.create_subscription(Twist,'/localization/safe_cmd_vel',lambda msg:output.append(msg.linear.x),10),
        test.create_subscription(String,'/localization/gate_status',lambda msg:status.append(json.loads(msg.data)),10)]
    start=time.monotonic()
    while time.monotonic()-start<1.:executor.spin_once(timeout_sec=.05)
    names=set(test.get_node_names())-{'sim_sensor_command_gate','isolated_gate_fault_test'}
    assert not names,names
    sim_time=10.
    def phase(healthy,duration):
        nonlocal sim_time
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            sim_time+=.04;tick=Clock();tick.clock.sec=int(sim_time);tick.clock.nanosec=round((sim_time%1)*1e9);clock.publish(tick)
            odom=Odometry();odom.header.stamp=tick.clock;odom.header.frame_id='odom';odom.child_frame_id='base_link';fused.publish(odom)
            if healthy:lio.publish(odom)
            for pub in sonar:
                msg=Range();msg.header.stamp=tick.clock;msg.min_range=.03;msg.max_range=.55;msg.range=.3;pub.publish(msg)
            twist=Twist();twist.linear.x=.2;cmd.publish(twist)
            for _ in range(15):executor.spin_once(timeout_sec=.002)
        for _ in range(30):executor.spin_once(timeout_sec=.002)
    try:
        phase(True,2.);assert any(x>.15 for x in output),output
        output.clear();status.clear();phase(False,2.5)
        assert output and all(abs(x)<1e-6 for x in output[-5:]),output[-10:]
        assert any(not x['inputs']['position'] and x['inputs']['fused'] and x['inputs']['sonar'] for x in status),status
        report=dict(passed=True,domain=74,healthy_command_forwarded=True,lost_position_sources_stop_motion=True,
            fresh_ekf_prediction_does_not_bypass_gate=True,no_gazebo_or_hardware=True)
        (ROOT/'artifacts/sensor_gate_fault_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:
        executor.remove_node(gate);executor.remove_node(test);gate.destroy_node();test.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
