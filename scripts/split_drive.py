#!/usr/bin/python3
"""Rear drive and front steering with measured pre-steer interlock."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from chassis_config import load_chassis, inner_wheel_limit

CHASSIS = load_chassis()
WHEELBASE = CHASSIS['wheelbase_m']
TRACK = CHASSIS['track_m']
RADIUS = CHASSIS['wheel_radius_m']
MAX_FORWARD = CHASSIS['max_forward_mps']
MAX_REVERSE = CHASSIS['max_reverse_mps']
MAX_CENTRE_STEER = math.radians(CHASSIS['max_center_steer_deg'])
MAX_WHEEL_STEER = inner_wheel_limit(CHASSIS)


def targets(v, yaw_rate, current_angle):
    v=max(-MAX_REVERSE,min(MAX_FORWARD,v))
    if abs(v) < .015:
        return 0., current_angle, current_angle
    center = max(-MAX_CENTRE_STEER, min(MAX_CENTRE_STEER, math.atan(WHEELBASE * yaw_rate / v)))
    curvature = math.tan(center) / WHEELBASE
    left = math.atan2(WHEELBASE * curvature, 1. - TRACK * curvature / 2.)
    right = math.atan2(WHEELBASE * curvature, 1. + TRACK * curvature / 2.)
    return v, max(-MAX_WHEEL_STEER, min(MAX_WHEEL_STEER, left)), max(-MAX_WHEEL_STEER, min(MAX_WHEEL_STEER, right))


def steering_targets(center):
    center=max(-MAX_CENTRE_STEER,min(MAX_CENTRE_STEER,center))
    curvature=math.tan(center)/WHEELBASE
    left=math.atan2(WHEELBASE*curvature,1.-TRACK*curvature/2.)
    right=math.atan2(WHEELBASE*curvature,1.+TRACK*curvature/2.)
    return center,left,right


class SplitDrive(Node):
    def __init__(self):
        super().__init__('sim_split_drive')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('ackermann_input_topic', '/sim/ackermann_cmd')
        self.declare_parameter('publish_nav_odom', False)
        self.declare_parameter('publish_wheel_odom', True)
        self.declare_parameter('publish_wheel_tf', False)
        self.command = Twist()
        self.ackermann_command = None
        self.received = 0.
        self.feedback = {}
        self.feedback_time = 0.
        self.last_angle = 0.
        self.last_targets = (0., 0.)
        self.steering = False
        self.wheel_pose = [0., 0., 0.]
        self.last_wheel = None
        self.last_odom_time = None
        self.wheel_speed = 0.
        self.wheel_yaw_rate = 0.
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10) if self.get_parameter('publish_wheel_odom').value else None
        self.nav_odom_pub = self.create_publisher(Odometry, '/chassis/odom', 10) if self.get_parameter('publish_nav_odom').value else None
        self.tf_pub = TransformBroadcaster(self) if self.get_parameter('publish_wheel_tf').value else None
        self.pubs = {
            name: self.create_publisher(Float64, '/sim/drive/' + name, 10)
            for name in ('front_left', 'front_right', 'rear_left', 'rear_right')
        }
        self.create_subscription(Twist, self.get_parameter('input_topic').value, self.on_command, 10)
        ackermann_topic=self.get_parameter('ackermann_input_topic').value
        if ackermann_topic:
            self.create_subscription(AckermannDriveStamped, ackermann_topic,
                self.on_ackermann_command, 10)
        self.create_subscription(JointState, '/sim/drive/joint_state', self.on_feedback, 10)
        self.create_timer(.02, self.tick, clock=rclpy.clock.Clock())

    def on_command(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.command = msg
            self.received = time.monotonic()
            self.ackermann_command = None

    def on_ackermann_command(self, msg):
        if math.isfinite(msg.drive.speed) and math.isfinite(msg.drive.steering_angle):
            self.ackermann_command = (msg.drive.speed, msg.drive.steering_angle)
            self.received = time.monotonic()

    def on_feedback(self, msg):
        self.feedback.update(zip(msg.name, msg.position))
        self.feedback_time = time.monotonic()
        if all(name in self.feedback for name in ('rear_left_roll','rear_right_roll','front_left_steer','front_right_steer')):
            wheels=(self.feedback['rear_left_roll'],self.feedback['rear_right_roll'])
            sim_time=self.get_clock().now().nanoseconds*1e-9
            if self.last_wheel is not None:
                ds=RADIUS*((wheels[0]-self.last_wheel[0])+(wheels[1]-self.last_wheel[1]))/2.
                left_tan=math.tan(self.feedback['front_left_steer'])
                right_tan=math.tan(self.feedback['front_right_steer'])
                front=math.atan(2*left_tan*right_tan/(left_tan+right_tan)) if abs(left_tan+right_tan)>1e-9 else 0.
                dtheta=ds*math.tan(front)/WHEELBASE
                dt=sim_time-self.last_odom_time if self.last_odom_time is not None else 0.
                if dt>0:
                    self.wheel_speed=ds/dt;self.wheel_yaw_rate=dtheta/dt
                heading=self.wheel_pose[2]+dtheta/2.
                self.wheel_pose[0]+=ds*math.cos(heading)
                self.wheel_pose[1]+=ds*math.sin(heading)
                self.wheel_pose[2]+=dtheta
            self.last_wheel=wheels
            self.last_odom_time=sim_time
            self.publish_odom()

    def publish_odom(self):
        x,y,yaw=self.wheel_pose
        stamp=self.get_clock().now().to_msg()
        z,w=math.sin(yaw/2),math.cos(yaw/2)
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
        odom.pose.pose.orientation.z=z;odom.pose.pose.orientation.w=w
        odom.pose.covariance[0]=odom.pose.covariance[7]=.02
        odom.pose.covariance[35]=.04
        odom.twist.twist.linear.x=self.wheel_speed
        odom.twist.twist.angular.z=self.wheel_yaw_rate
        if self.odom_pub:self.odom_pub.publish(odom)
        if self.nav_odom_pub:self.nav_odom_pub.publish(odom)
        if self.tf_pub:
            transform=TransformStamped();transform.header=odom.header;transform.child_frame_id='base_link'
            transform.transform.translation.x=x;transform.transform.translation.y=y
            transform.transform.rotation.z=z;transform.transform.rotation.w=w
            self.tf_pub.sendTransform(transform)

    def publish(self, name, value):
        self.pubs[name].publish(Float64(data=float(value)))

    def tick(self):
        now = time.monotonic()
        fresh = now - self.received < CHASSIS['command_timeout_sec']
        if fresh and self.ackermann_command is not None:
            speed,angle=self.ackermann_command
            v=max(-MAX_REVERSE,min(MAX_FORWARD,speed))
            self.last_angle,left,right=steering_targets(angle)
            self.last_targets=(left,right)
        else:
            v = self.command.linear.x if fresh else 0.
            w = self.command.angular.z if fresh else 0.
            v, left, right = targets(v, w, self.last_angle)
            if fresh and abs(v) >= .015:
                self.last_angle = max(-MAX_CENTRE_STEER,min(MAX_CENTRE_STEER,math.atan(WHEELBASE*w/v)))
                self.last_targets = (left, right)
            else:
                left, right = self.last_targets
        self.publish('front_left', left)
        self.publish('front_right', right)
        left_actual = self.feedback.get('front_left_steer')
        right_actual = self.feedback.get('front_right_steer')
        settled = (now - self.feedback_time < .3 and left_actual is not None and right_actual is not None
                   and abs(left_actual-left) < CHASSIS['steering_settle_rad']
                   and abs(right_actual-right) < CHASSIS['steering_settle_rad'])
        # Only interrupt motion for a material change; small continuous
        # corrections keep the navigation controller from stop-go cycling.
        error = max(abs((left_actual or 0.)-left), abs((right_actual or 0.)-right))
        if error > .10:
            self.steering = True
        if self.steering and settled:
            self.steering = False
        drive = v if settled and not self.steering else 0.
        curvature = math.tan(self.last_angle) / WHEELBASE if abs(drive) >= .015 else 0.
        self.publish('rear_left', drive * (1. - TRACK * curvature / 2.) / RADIUS)
        self.publish('rear_right', drive * (1. + TRACK * curvature / 2.) / RADIUS)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SplitDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish('rear_left', 0.)
        node.publish('rear_right', 0.)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
