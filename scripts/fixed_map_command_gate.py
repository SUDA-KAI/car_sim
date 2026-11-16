#!/usr/bin/python3
"""Stop fixed-map navigation when its obstacle or odometry inputs go stale."""
import copy
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String


def input_health(samples, sim_now, wall_now):
    def fresh(name, sim_limit):
        if name not in samples:
            return False
        stamp, received = samples[name]
        return (math.isfinite(stamp) and -.05 <= sim_now-stamp <= sim_limit
                and 0 <= wall_now-received <= 2.5)
    return dict(scan=fresh('scan', .75), odom=fresh('odom', .3),
                sonar=all(fresh(f'sonar_{i}', 1.) for i in range(1, 5)))


class FixedMapCommandGate(Node):
    def __init__(self):
        super().__init__('fixed_map_command_gate')
        self.samples = {}
        self.command = Twist()
        self.command_received = None
        self.previous_status = None
        self.last_status = 0.
        self.safe = self.create_publisher(Twist, '/localization/safe_cmd_vel', 10)
        self.status = self.create_publisher(String, '/localization/gate_status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(LaserScan, '/scan', lambda msg: self.on_sample('scan', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/chassis/odom', lambda msg: self.on_sample('odom', msg), qos_profile_sensor_data)
        for index in range(1, 5):
            self.create_subscription(Range, f'/ultrasonic_{index}',
                lambda msg, i=index: self.on_range(i, msg), qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=rclpy.clock.Clock())

    def on_sample(self, name, msg):
        self.samples[name] = (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, time.monotonic())

    def on_range(self, index, msg):
        name = f'sonar_{index}'
        if math.isnan(msg.range) or not 0 <= msg.min_range < msg.max_range:
            self.samples.pop(name, None)
        else:
            self.on_sample(name, msg)

    def on_command(self, msg):
        values = [getattr(vector, axis) for vector in (msg.linear, msg.angular) for axis in ('x', 'y', 'z')]
        if not all(map(math.isfinite, values)):
            self.command_received = None
            return
        self.command = copy.deepcopy(msg)
        self.command_received = time.monotonic()

    def tick(self):
        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds*1e-9
        health = input_health(self.samples, sim, wall)
        command_fresh = self.command_received is not None and wall-self.command_received < .5
        enabled = all(health.values())
        self.safe.publish(self.command if enabled and command_fresh else Twist())
        status = dict(state='enabled' if enabled else 'blocked', inputs=health, command_fresh=command_fresh)
        if status != self.previous_status or wall-self.last_status > .5:
            self.status.publish(String(data=json.dumps(status)))
            self.previous_status, self.last_status = status, wall


def main():
    rclpy.init()
    node = FixedMapCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.safe.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
