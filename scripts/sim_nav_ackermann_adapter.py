#!/usr/bin/env python3
"""Simulation counterpart of the commissioned Nav Ackermann command adapter.

It consumes Nav2 Twist, uses measured simulated steering and wheel speed, and
requires four fresh A22 Range messages before allowing rear-wheel motion.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, Range
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from chassis_config import load_chassis

CHASSIS=load_chassis()
STEERING_RATE=0.20  # rad/s, commissioned adapter launch
STOP_ERROR=0.08
FULL_SPEED_ERROR=0.035
INPUT_TIMEOUT=0.50
SOURCE_MAX_AGE=0.50
LOCALIZATION_MAX_AGE=1.0
DIRECTION_HOLD=0.20
PERIOD=0.05


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def age_valid(now, stamp, limit=SOURCE_MAX_AGE):
    return stamp is not None and -0.05 <= now-stamp <= limit


def range_valid(message, now):
    if message is None or not age_valid(now,stamp_seconds(message.header.stamp)):
        return False
    if not (math.isfinite(message.min_range) and math.isfinite(message.max_range)
            and 0 <= message.min_range < message.max_range):
        return False
    value=message.range
    return value == math.inf or math.isfinite(value) and message.min_range <= value <= message.max_range


def center_from_wheels(left,right):
    lt,rt=math.tan(left),math.tan(right)
    return math.atan(2*lt*rt/(lt+rt)) if abs(lt+rt)>1e-9 else 0.0


class SimNavAckermannAdapter(Node):
    def __init__(self):
        super().__init__('sim_nav_ackermann_adapter')
        self.declare_parameter('require_a22',True)
        self.declare_parameter('require_amcl',True)
        self.require_a22=bool(self.get_parameter('require_a22').value)
        self.require_amcl=bool(self.get_parameter('require_amcl').value)
        self.declare_parameter('input_topic','/cmd_vel')
        self.cmd=None;self.cmd_receipt=None
        self.joints={};self.joint_stamp=None
        self.odom_stamp=None;self.actual_speed=0.0
        self.pose_stamp=None
        self.scan_stamp=None
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.ranges={}
        self.target=0.0;self.commanded=0.0
        self.last_direction=0;self.stationary_since=None
        self.last_reason=None
        self.command_pub=self.create_publisher(AckermannDriveStamped,'/sim/ackermann_cmd',10)
        self.target_pub=self.create_publisher(Float64,'/nav/steering_target',10)
        self.scale_pub=self.create_publisher(Float64,'/nav/steering_speed_scale',10)
        self.status_pub=self.create_publisher(String,'/sim/nav_drive_gate_status',10)
        self.create_subscription(Twist,self.get_parameter('input_topic').value,self.on_cmd,10)
        self.create_subscription(JointState,'/sim/drive/joint_state',self.on_joint,10)
        self.create_subscription(Odometry,'/wheel/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
        self.create_subscription(LaserScan,'/localization/multi_lidar_scan',self.on_scan,qos_profile_sensor_data)
        for index in range(1,5):
            name=f'ultrasonic_{index}'
            self.create_subscription(Range,'/'+name,
                lambda msg,key=name:self.ranges.__setitem__(key,msg),qos_profile_sensor_data)
        self.create_timer(PERIOD,self.tick)

    def on_cmd(self,msg):
        if (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)
                and abs(msg.linear.y)<1e-6):
            self.cmd=(msg.linear.x,msg.angular.z)
            self.cmd_receipt=time.monotonic()

    def on_joint(self,msg):
        first=not self.joints
        for name,value in zip(msg.name,msg.position):
            if math.isfinite(value):self.joints[name]=value
        if first and all(key in self.joints for key in ('front_left_steer','front_right_steer')):
            self.commanded=center_from_wheels(self.joints['front_left_steer'],self.joints['front_right_steer'])
            self.target=self.commanded
        self.joint_stamp=self.get_clock().now().nanoseconds*1e-9

    def on_odom(self,msg):
        self.odom_stamp=stamp_seconds(msg.header.stamp)
        self.actual_speed=msg.twist.twist.linear.x

    def on_pose(self,msg):
        if msg.header.frame_id=='map':self.pose_stamp=stamp_seconds(msg.header.stamp)

    def on_scan(self,msg):
        if msg.header.frame_id=='base_link':self.scan_stamp=stamp_seconds(msg.header.stamp)

    def publish(self,speed,scale,reason):
        msg=AckermannDriveStamped()
        msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.drive.speed=float(speed);msg.drive.steering_angle=float(self.commanded)
        self.command_pub.publish(msg)
        self.target_pub.publish(Float64(data=float(self.target)))
        self.scale_pub.publish(Float64(data=float(scale)))
        if reason!=self.last_reason:
            self.get_logger().info(f'Navigation drive gate: {reason}')
            self.last_reason=reason
        status={'reason':reason,'allowed':reason=='ready','speed_mps':speed,
                'requested_steering_rad':self.target,'commanded_steering_rad':self.commanded,
                'actual_speed_mps':self.actual_speed}
        self.status_pub.publish(String(data=json.dumps(status)))

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if not age_valid(now,self.joint_stamp) or not all(
                key in self.joints for key in ('front_left_steer','front_right_steer')):
            self.publish(0.,0.,'steering_feedback_missing');return
        actual=center_from_wheels(self.joints['front_left_steer'],self.joints['front_right_steer'])
        if not age_valid(now,self.odom_stamp) or not math.isfinite(self.actual_speed):
            self.publish(0.,0.,'wheel_odometry_stale');return
        if self.require_amcl:
            # AMCL may publish /amcl_pose only after motion, while rebroadcasting
            # map->odom for fresh scans.  Requiring a new pose would deadlock
            # departure from rest.  Require an initialized pose, live fused
            # scan and recent AMCL transform instead.
            if self.pose_stamp is None:
                self.cmd=None
                self.publish(0.,0.,'amcl_initial_pose_missing');return
            if not age_valid(now,self.scan_stamp,LOCALIZATION_MAX_AGE):
                self.cmd=None
                self.publish(0.,0.,'fused_scan_stale');return
            try:
                transform=self.tf_buffer.lookup_transform('map','odom',Time())
                tf_age=now-stamp_seconds(transform.header.stamp)
            except Exception:
                tf_age=math.inf
            if not -0.30 <= tf_age <= LOCALIZATION_MAX_AGE:
                self.cmd=None
                self.publish(0.,0.,'amcl_tf_stale');return
        if self.require_a22:
            for index in range(1,5):
                name=f'ultrasonic_{index}';reading=self.ranges.get(name)
                if reading is None or reading.header.frame_id!=name or not range_valid(reading,now):
                    self.cmd=None
                    self.publish(0.,0.,name+'_missing_or_invalid');return
        if self.cmd is None or self.cmd_receipt is None or time.monotonic()-self.cmd_receipt>INPUT_TIMEOUT:
            self.publish(0.,0.,'command_timeout');return
        speed,yaw_rate=self.cmd
        requested=max(-CHASSIS['max_reverse_mps'],min(CHASSIS['max_forward_mps'],speed))
        if abs(speed)>=0.02:
            self.target=max(-math.radians(CHASSIS['max_center_steer_deg']),
                            min(math.radians(CHASSIS['max_center_steer_deg']),
                                math.atan(CHASSIS['wheelbase_m']*yaw_rate/speed)))
        step=STEERING_RATE*PERIOD
        self.commanded+=max(-step,min(step,self.target-self.commanded))
        curvature=max(abs(math.tan(self.commanded)),abs(math.tan(actual)))/CHASSIS['wheelbase_m']
        if curvature>1e-9:
            limit=min(abs(requested),0.35/curvature,math.sqrt(0.35/curvature))
            requested=math.copysign(limit,requested)
        error=abs(self.target-actual)
        scale=0. if error>=STOP_ERROR else (STOP_ERROR-error)/(STOP_ERROR-FULL_SPEED_ERROR) if error>FULL_SPEED_ERROR else 1.
        if abs(self.actual_speed)<=0.015:
            if self.stationary_since is None:self.stationary_since=now
        else:self.stationary_since=None
        direction=1 if requested>1e-4 else -1 if requested<-1e-4 else 0
        reversing=direction and ((self.last_direction and direction!=self.last_direction)
                                 or direction*self.actual_speed < -0.015)
        if reversing and (self.stationary_since is None or now-self.stationary_since<DIRECTION_HOLD):
            scale=0.
        output=requested*scale
        if abs(output)>1e-4:self.last_direction=direction
        self.publish(output,scale,'ready')


def main():
    rclpy.init();node=SimNavAckermannAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.publish(0.,0.,'shutdown')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
