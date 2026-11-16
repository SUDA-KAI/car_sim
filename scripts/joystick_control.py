#!/usr/bin/python3
"""Simulation joystick control with Ackermann kinematics and a wall-clock watchdog."""
import math
import time
from dataclasses import dataclass

@dataclass(frozen=True)
class Settings:
    axis_speed: int=1
    axis_steering: int=3
    speed_sign: float=1.0
    steering_sign: float=1.0
    enable_button: int=4
    deadzone: float=.12
    max_forward_speed: float=.5
    max_reverse_speed: float=.2
    max_steering_angle: float=.532217358
    wheelbase: float=.90
    max_yaw_rate: float=.45
    input_timeout: float=.35
    publish_rate: float=20.

    def validate(self):
        if min(self.axis_speed,self.axis_steering,self.enable_button)<0:
            raise ValueError('Axis and button indices must be nonnegative')
        if not 0<=self.deadzone<1:raise ValueError('deadzone must be in [0,1)')
        if self.speed_sign not in (-1,1) or self.steering_sign not in (-1,1):
            raise ValueError('Axis signs must be +1 or -1')
        for key in ('max_forward_speed','max_reverse_speed','max_steering_angle','wheelbase',
                    'max_yaw_rate','input_timeout','publish_rate'):
            value=getattr(self,key)
            if not math.isfinite(value) or value<=0:raise ValueError(f'{key} must be positive and finite')
        if self.max_steering_angle>=math.pi/2:raise ValueError('steering angle must be < pi/2')

class JoystickState:
    def __init__(self,settings=None):
        self.cfg=settings or Settings();self.cfg.validate()
        self.armed=False;self.last_input=None;self.command=(0.,0.);self.status='waiting_for_joy'

    def axis(self,value):
        value=max(-1.,min(1.,value));d=self.cfg.deadzone
        return 0. if abs(value)<=d else math.copysign((abs(value)-d)/(1-d),value)

    def receive(self,axes,buttons,now):
        c=self.cfg
        if (len(axes)<=max(c.axis_speed,c.axis_steering) or len(buttons)<=c.enable_button
                or not all(math.isfinite(float(a)) for a in axes)):
            self.armed=False;self.command=(0.,0.);self.status='invalid_or_disconnected_joy';self.last_input=None
            return
        if self.last_input is not None and now-self.last_input>c.input_timeout:self.armed=False
        self.last_input=now
        speed=self.axis(axes[c.axis_speed]*c.speed_sign)
        steering=self.axis(axes[c.axis_steering]*c.steering_sign)
        enabled=bool(buttons[c.enable_button])
        if not enabled:
            self.command=(0.,0.)
            if speed==0 and steering==0:self.armed=True
            self.status='ready_hold_enable' if self.armed else 'release_enable_and_center_stick'
            return
        if not self.armed:
            self.command=(0.,0.);self.status='release_enable_and_center_stick';return
        velocity=speed*(c.max_forward_speed if speed>=0 else c.max_reverse_speed)
        # angular.z is yaw rate, not steering angle. Reversing changes yaw sign.
        yaw=velocity*math.tan(steering*c.max_steering_angle)/c.wheelbase
        self.command=(velocity,max(-c.max_yaw_rate,min(c.max_yaw_rate,yaw)))
        self.status='enabled'

    def output(self,now):
        if self.last_input is None:return (0.,0.)
        if now-self.last_input>self.cfg.input_timeout:
            self.command=(0.,0.);self.armed=False;self.status='joy_timeout_release_and_center'
        return self.command

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String

    class JoystickControl(Node):
        def __init__(self):
            super().__init__('sim_joystick_control')
            defaults=Settings()
            values={key:self.declare_parameter(key,getattr(defaults,key)).value for key in defaults.__dataclass_fields__}
            self.state=JoystickState(Settings(**values))
            self.pub=self.create_publisher(Twist,'/cmd_vel',10)
            self.status_pub=self.create_publisher(String,'/sim/teleop_status',10)
            self.sub=self.create_subscription(Joy,'/joy',self.on_joy,qos_profile_sensor_data)
            self.timer=self.create_timer(1/self.state.cfg.publish_rate,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.previous_status=None
            self.get_logger().info('Simulation control: release enable and center stick, then hold enable to drive.')
        def on_joy(self,msg):
            self.state.receive(msg.axes,msg.buttons,time.monotonic())
        def stop(self):self.pub.publish(Twist())
        def tick(self):
            speed,yaw=self.state.output(time.monotonic())
            command=Twist();command.linear.x=speed;command.angular.z=yaw;self.pub.publish(command)
            status=String();status.data=self.state.status;self.status_pub.publish(status)
            if self.state.status!=self.previous_status:
                self.get_logger().info(self.state.status);self.previous_status=self.state.status

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None
    try:
        node=JoystickControl();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:
            if rclpy.ok():node.stop()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
