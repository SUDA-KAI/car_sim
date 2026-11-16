"""Gymnasium environment for the headless Patrol Car Gazebo simulation."""
from __future__ import annotations

import math
import os
import subprocess
import threading
import time
from typing import Any

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PatrolCarEnv(gym.Env):
    """Continuous Ackermann navigation task.

    Observation: normalized lidar sectors followed by goal x/y in the vehicle
    frame, cos/sin(goal bearing), longitudinal velocity and yaw rate.
    Action: [normalized speed, normalized steering angle].
    """

    metadata = {"render_modes": []}

    def __init__(self, *, goal=(2.0, 0.0), spawn=(0.4164, 12.0758, -0.52),
                 lidar_bins=90, step_seconds=0.10, max_steps=600,
                 scan_max=10.0, collision_distance=0.48,
                 max_forward=0.8, max_reverse=0.2, max_steering=0.60,
                 wheelbase=0.79, topic_prefix="", world="patrol",
                 entity="patrol_car"):
        super().__init__()
        # AckermannSteering publishes odometry in a frame whose origin and yaw
        # are the model's initial world pose.  Goals are therefore expressed in
        # that local odom frame, not in map/world coordinates.
        self.goal = np.asarray(goal, dtype=np.float32)
        self.spawn = tuple(map(float, spawn))
        self.lidar_bins = int(lidar_bins)
        self.step_seconds = float(step_seconds)
        self.max_steps = int(max_steps)
        self.scan_max = float(scan_max)
        self.collision_distance = float(collision_distance)
        self.max_forward = float(max_forward)
        self.max_reverse = float(max_reverse)
        self.max_steering = float(max_steering)
        self.wheelbase = float(wheelbase)
        self.world = world
        self.entity = entity
        prefix = topic_prefix.rstrip("/")
        self._topic = lambda name: f"{prefix}/{name}" if prefix else f"/{name}"

        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self.lidar_bins + 6,), dtype=np.float32)

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node(f"patrol_gym_{os.getpid()}_{id(self) & 0xffff:x}")
        self.pub = self.node.create_publisher(Twist, self._topic("cmd_vel"), 10)
        self.node.create_subscription(Odometry, self._topic("chassis/odom"),
                                      self._on_odom, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, self._topic("scan"),
                                      self._on_scan, qos_profile_sensor_data)
        self._condition = threading.Condition()
        self._odom = None
        self._scan = None
        self._odom_seq = 0
        self._scan_seq = 0
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._steps = 0
        self._previous_distance = 0.0

    def _on_odom(self, msg):
        with self._condition:
            self._odom = msg
            self._odom_seq += 1
            self._condition.notify_all()

    def _on_scan(self, msg):
        with self._condition:
            self._scan = msg
            self._scan_seq += 1
            self._condition.notify_all()

    def _wait_for_data(self, old_odom=-1, old_scan=-1, timeout=8.0):
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._odom is not None and self._scan is not None
                and self._odom_seq > old_odom and self._scan_seq > old_scan,
                timeout=timeout)
        if not ready:
            raise RuntimeError(
                "Timed out waiting for /chassis/odom and /scan. Start run_rl_sim.sh "
                "with the same ROS_DOMAIN_ID before constructing the environment.")

    def _publish_action(self, action):
        speed_cmd = float(np.clip(action[0], -1.0, 1.0))
        speed = speed_cmd * (self.max_forward if speed_cmd >= 0 else self.max_reverse)
        steering = float(np.clip(action[1], -1.0, 1.0)) * self.max_steering
        msg = Twist()
        msg.linear.x = speed
        # The Gazebo Ackermann plugin consumes desired yaw rate in Twist.
        msg.angular.z = speed * math.tan(steering) / self.wheelbase
        self.pub.publish(msg)

    def _set_pose(self, x, y, yaw):
        zq, wq = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        request = (f'name: "{self.entity}", position: {{x: {x}, y: {y}, z: 0}}, '
                   f'orientation: {{z: {zq}, w: {wq}}}')
        command = ["ign", "service", "-s", f"/world/{self.world}/set_pose",
                   "--reqtype", "ignition.msgs.Pose", "--reptype",
                   "ignition.msgs.Boolean", "--timeout", "3000", "--req", request]
        last = None
        for _ in range(20):
            last = subprocess.run(command, text=True, capture_output=True)
            if last.returncode == 0 and "data: true" in last.stdout.lower():
                return
            time.sleep(0.25)
        detail = ((last.stdout or "") + (last.stderr or "")).strip() if last else ""
        raise RuntimeError(f"Gazebo set_pose failed: {detail}")

    def _observation(self):
        odom, scan = self._odom, self._scan
        raw = np.asarray(scan.ranges, dtype=np.float32)
        raw = np.nan_to_num(raw, nan=self.scan_max, posinf=self.scan_max, neginf=0.0)
        raw = np.clip(raw, 0.0, self.scan_max)
        chunks = np.array_split(raw, self.lidar_bins)
        lidar = np.asarray([c.min() if len(c) else self.scan_max for c in chunks],
                           dtype=np.float32) / self.scan_max
        p = odom.pose.pose.position
        heading = _yaw(odom.pose.pose.orientation)
        dx, dy = float(self.goal[0] - p.x), float(self.goal[1] - p.y)
        gx = math.cos(heading) * dx + math.sin(heading) * dy
        gy = -math.sin(heading) * dx + math.cos(heading) * dy
        bearing = math.atan2(gy, gx)
        extra = np.asarray([
            np.clip(gx / 10.0, -1, 1), np.clip(gy / 10.0, -1, 1),
            math.cos(bearing), math.sin(bearing),
            np.clip(odom.twist.twist.linear.x / self.max_forward, -1, 1),
            np.clip(odom.twist.twist.angular.z, -1, 1),
        ], dtype=np.float32)
        return np.concatenate((lidar, extra)).astype(np.float32), math.hypot(dx, dy), float(raw.min())

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        if "goal" in options:
            self.goal = np.asarray(options["goal"], dtype=np.float32)
        pose = options.get("spawn", self.spawn)
        self._publish_action((0.0, 0.0))
        self._set_pose(*map(float, pose))
        old_o, old_s = self._odom_seq, self._scan_seq
        # Several zero commands remove residual controller demand after teleport.
        for _ in range(3):
            self._publish_action((0.0, 0.0))
            time.sleep(0.05)
        self._wait_for_data(old_o, old_s)
        obs, distance, clearance = self._observation()
        self._steps = 0
        self._previous_distance = distance
        return obs, {"distance": distance, "clearance": clearance}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        old_o, old_s = self._odom_seq, self._scan_seq
        self._publish_action(action)
        deadline = time.monotonic() + self.step_seconds
        while time.monotonic() < deadline:
            time.sleep(min(0.01, deadline - time.monotonic()))
        self._wait_for_data(old_o, old_s, timeout=max(2.0, self.step_seconds * 10))
        obs, distance, clearance = self._observation()
        self._steps += 1
        progress = self._previous_distance - distance
        self._previous_distance = distance
        reached = distance < 0.45
        collision = clearance < self.collision_distance
        truncated = self._steps >= self.max_steps
        reward = 5.0 * progress - 0.01 - 0.02 * abs(float(action[1]))
        if reached:
            reward += 30.0
        if collision:
            reward -= 20.0
        terminated = reached or collision
        if terminated or truncated:
            self._publish_action((0.0, 0.0))
        return obs, float(reward), terminated, truncated, {
            "distance": distance, "clearance": clearance,
            "reached_goal": reached, "collision": collision,
        }

    def close(self):
        try:
            self._publish_action((0.0, 0.0))
            self._executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if self._thread.is_alive():
                self._thread.join(timeout=1.0)

