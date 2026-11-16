"""ROS runs in one executor thread; HTTP commands enter through a queue."""
from collections import deque
from concurrent.futures import Future
import math
import os
import queue
import threading
import time


class RosBridge:
    def __init__(self, log, domain):
        self.log = log
        self.domain = domain
        self.lock = threading.RLock()
        self.commands = queue.Queue()
        self.quit = threading.Event()
        self.active = False
        self.available = False
        self.error = None
        self.reset_data()
        self.thread = threading.Thread(target=self.run, daemon=True, name='console-ros')
        self.thread.start()

    def reset_data(self):
        self.data = dict(pose=None, speed=None, angular_speed=None, sim_time=None,
                         scan=[], path=[], trail=[], ranges=[None] * 4,
                         ready=False, mission='idle', goal=None, remaining=None,
                         completed=0, total=0, elapsed=0, recoveries=0, targets=[], mode='waypoints',
                         preview=dict(id=0, state='idle', path=[], points=[], message=''))
        self.seen = {}

    def snapshot(self):
        with self.lock:
            data = dict(self.data)
            data['preview'] = dict(self.data['preview'])
            now = time.monotonic()
            data['connected'] = self.active and now - self.seen.get('clock', 0) < 3
            data['pose_fresh'] = self.active and now - self.seen.get('pose', 0) < 3
            data['ready'] = bool(data['ready'] and data['connected'] and data['pose_fresh'])
            if now - self.seen.get('odom', 0) >= 3:
                data['speed'] = data['angular_speed'] = None
            data['available'] = self.available
            data['error'] = self.error
            data['scan'] = data['scan'] if now - self.seen.get('scan', 0) < 3 else []
            data['ranges'] = [v if now - self.seen.get(f'range{i}', 0) < 3 else None for i, v in enumerate(data['ranges'])]
            return data

    def command(self, name, *args):
        if not self.available:
            raise ValueError('ROS 连接不可用：' + (self.error or '正在初始化'))
        future = Future()
        self.commands.put((name, args, future))
        return future.result(timeout=5)

    def run(self):
        try:
            self._run_ros()
        except Exception as error:
            self.error = str(error)
            self.log('error', 'ROS 连接异常：' + str(error))
        finally:
            self.available = False

    def _run_ros(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry, Path
        from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, ComputePathToPose, ComputePathThroughPoses
        from lifecycle_msgs.srv import GetState
        from sensor_msgs.msg import LaserScan, Range
        from rosgraph_msgs.msg import Clock
        from tf2_ros import Buffer, TransformListener, TransformException

        # Explicit domain and own context: never join the running hardware graph.
        context = Context()
        rclpy.init(context=context, domain_id=self.domain, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('gazebo_web_console', context=context)
        node.set_parameters([Parameter('use_sim_time', value=True)])
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        self.node = node
        self.buffer = Buffer(node=node)
        listener = TransformListener(self.buffer, node)
        self.clients = {'waypoints': ActionClient(node, NavigateToPose, '/navigate_to_pose'),
                        'through_poses': ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')}
        self.actions = {'waypoints': NavigateToPose, 'through_poses': NavigateThroughPoses}
        self.plan_actions = (ComputePathToPose, ComputePathThroughPoses)
        self.plan_clients = (ActionClient(node, ComputePathToPose, '/compute_path_to_pose'),
                             ActionClient(node, ComputePathThroughPoses, '/compute_path_through_poses'))
        self.preview_handle = None
        self.preview_sequence = 0
        self.state_client = node.create_client(GetState, '/bt_navigator/get_state')
        self.initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.handle = None
        self.generation = 0
        self.pending = deque()
        self.canceling = False
        self.state_future = None
        self.next_state_check = 0

        def clock(msg):
            if not self.active:
                return
            with self.lock:
                self.data['sim_time'] = msg.clock.sec + msg.clock.nanosec / 1e9
                self.seen['clock'] = time.monotonic()

        def odometry(msg):
            if not self.active:
                return
            with self.lock:
                self.data['speed'] = msg.twist.twist.linear.x
                self.data['angular_speed'] = msg.twist.twist.angular.z
                self.seen['odom'] = time.monotonic()

        def path(msg):
            if self.active and msg.header.frame_id == 'map':
                poses = msg.poses
                stride = max(1, len(poses) // 1200)
                with self.lock:
                    self.data['path'] = [[p.pose.position.x, p.pose.position.y] for p in poses[::stride]]

        def scan(msg):
            if not self.active:
                return
            try:
                transform = self.buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            except TransformException:
                return
            t, q = transform.transform.translation, transform.transform.rotation
            # Project the full 3D transform onto map XY, including ramp tilt.
            r00, r01 = 1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z)
            r10, r11 = 2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z)
            values = []
            stride = max(1, len(msg.ranges) // 450)
            for i in range(0, len(msg.ranges), stride):
                distance = msg.ranges[i]
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    angle = msg.angle_min + i * msg.angle_increment
                    lx, ly = distance * math.cos(angle), distance * math.sin(angle)
                    values.append([t.x + r00 * lx + r01 * ly, t.y + r10 * lx + r11 * ly])
            with self.lock:
                self.data['scan'] = values
                self.seen['scan'] = time.monotonic()

        def sensor(msg, i):
            if self.active:
                with self.lock:
                    # JSON null = no echo. Never serialize infinity / NaN.
                    self.data['ranges'][i] = float(msg.range) if math.isfinite(msg.range) else None
                    self.seen[f'range{i}'] = time.monotonic()

        subscriptions = [node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
                         node.create_subscription(Odometry, '/chassis/odom', odometry, qos_profile_sensor_data),
                         node.create_subscription(Path, '/plan', path, 10),
                         node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)]
        subscriptions += [node.create_subscription(Range, f'/ultrasonic_{i+1}', lambda msg, i=i: sensor(msg, i), qos_profile_sensor_data) for i in range(4)]

        def pose_stamped(point):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = float(point['x'])
            pose.pose.position.y = float(point['y'])
            pose.pose.orientation.z = math.sin(float(point['yaw']) / 2)
            pose.pose.orientation.w = math.cos(float(point['yaw']) / 2)
            return pose
        self.pose_stamped = pose_stamped

        def tick():
            if not self.active:
                return
            now = time.monotonic()
            if now - self.seen.get('odom', 0) < 3:
                try:
                    transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    t, q = transform.transform.translation, transform.transform.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    with self.lock:
                        self.data['pose'] = dict(x=t.x, y=t.y, z=t.z, yaw=yaw)
                        self.seen['pose'] = now
                        trail = self.data['trail']
                        if not trail or math.hypot(t.x - trail[-1][0], t.y - trail[-1][1]) > .04:
                            self.data['trail'] = (trail + [[t.x, t.y]])[-2500:]
                except TransformException:
                    pass
            if now >= self.next_state_check:
                self.next_state_check = now + 1
                if self.state_future and not self.state_future.done():
                    self.state_future.cancel()
                if self.state_client.service_is_ready():
                    gen = self.generation
                    self.state_future = self.state_client.call_async(GetState.Request())
                    def state_done(future):
                        if not future.cancelled() and gen == self.generation:
                            try:
                                with self.lock:
                                    self.data['ready'] = future.result().current_state.id == 3
                            except Exception:
                                self.data['ready'] = False
                    self.state_future.add_done_callback(state_done)
                else:
                    self.data['ready'] = False
        node.create_timer(.2, tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.available = True
        self.log('info', f'ROS 2 桥接已就绪 · 仿真域 {self.domain}')
        try:
            while not self.quit.is_set():
                while True:
                    try:
                        name, args, future = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        result = self._command(name, *args)
                        future.set_result(result)
                    except Exception as error:
                        future.set_exception(error)
                executor.spin_once(timeout_sec=.05)
        finally:
            executor.shutdown()
            node.destroy_node()
            context.shutdown()

    def _command(self, name, *args):
        if name == 'session':
            if args[0]:
                peers = [name for name in self.node.get_node_names() if name != self.node.get_name()]
                if peers:
                    raise ValueError(f'仿真域 {self.domain} 已被其他 ROS 节点占用，请使用 --domain 指定独立域')
            with self.lock:
                self.generation += 1
                self.active = args[0]
                self.handle = None
                self.preview_handle = None
                self.pending.clear()
                self.canceling = False
                self.buffer.clear()
                self.reset_data()
            return {'ok': True}
        if name in ('navigate', 'preview'):
            points, mode = args
            if not self.active or not self.snapshot()['ready']:
                raise ValueError('请等待 Gazebo 时钟、定位和 Nav2 全部就绪后再导航')
            if self.data['mission'] in ('sending', 'running', 'canceling'):
                raise ValueError('已有导航任务，请先取消当前任务')
            if self.data['preview']['state'] == 'planning':
                raise ValueError('路径检查尚未完成，请稍候')
            if name == 'preview':
                return self._preview(points)
            if not self.clients[mode].server_is_ready():
                raise ValueError('导航 Action 服务尚未就绪')
            self.mode = mode
            self.pending = deque(points)
            self.canceling = False
            with self.lock:
                self.data.update(mission='sending', total=len(points), completed=0, remaining=None,
                                 path=[], elapsed=0, recoveries=0, targets=points, mode=mode)
            self._send_next()
            return {'ok': True}
        if name == 'cancel':
            if self.data['preview']['state'] == 'planning':
                self.data['preview'].update(state='canceled', path=[], message='路径检查已取消')
                if self.preview_handle:
                    self.preview_handle.cancel_goal_async()
            self.pending.clear()
            if self.data['mission'] in ('sending', 'running', 'canceling'):
                self.canceling = True
                self.data['mission'] = 'canceling'
                if self.handle:
                    self.handle.cancel_goal_async()
                self.log('warning', '已请求取消导航，等待 Nav2 停止任务')
            return {'ok': True}
        if name == 'initial_pose':
            from geometry_msgs.msg import PoseWithCovarianceStamped
            if not self.active or self.data['mission'] in ('sending', 'running', 'canceling') or self.data['preview']['state'] == 'planning':
                raise ValueError('请先启动仿真并取消导航，再设置初始位置')
            pose = self.pose_stamped(args[0])
            msg = PoseWithCovarianceStamped()
            msg.header = pose.header
            msg.pose.pose = pose.pose
            msg.pose.covariance[0] = msg.pose.covariance[7] = .25
            msg.pose.covariance[35] = .0685
            self.initial_pub.publish(msg)
            self.log('info', '已向 AMCL 发送初始位姿')
            return {'ok': True}
        raise ValueError('未知 ROS 命令')

    def _preview(self, points):
        index = int(len(points) > 1)
        client = self.plan_clients[index]
        if not client.server_is_ready():
            raise ValueError('Nav2 路径规划服务尚未就绪')
        self.preview_sequence += 1
        ident, generation = self.preview_sequence, self.generation
        started = time.monotonic()
        self.data['preview'] = dict(id=ident, state='planning', path=[], points=points, message='正在调用 Nav2 检查实际路径')
        goal = self.plan_actions[index].Goal()
        goal.planner_id = 'GridBased'
        goal.use_start = False
        if index:
            goal.goals = [self.pose_stamped(point) for point in points]
        else:
            goal.goal = self.pose_stamped(points[0])

        def current():
            return generation == self.generation and self.data['preview']['id'] == ident and self.data['preview']['state'] == 'planning'

        def failed(message):
            self.data['preview'].update(state='failed', path=[], message=message, seconds=time.monotonic() - started)
            self.preview_handle = None
            self.log('warning', message)

        def finished(future):
            if not current():
                return
            try:
                result = future.result()
                path = result.result.path
                if result.status != 4 or not path.poses or path.header.frame_id != 'map':
                    failed('Nav2 在当前搜索时限内未返回可行路径；请检查目标朝向、通道与搜索时限。尚未启动导航。')
                    return
                poses = path.poses
                stride = max(1, math.ceil(len(poses) / 2000))
                sampled = [[p.pose.position.x, p.pose.position.y] for p in poses[::stride]]
                last = [poses[-1].pose.position.x, poses[-1].pose.position.y]
                if sampled[-1] != last:
                    sampled.append(last)
                length = sum(math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y) for a, b in zip(poses, poses[1:]))
                with self.lock:
                    self.data['preview'].update(state='ready', path=sampled, length=length,
                                                seconds=time.monotonic() - started, message=f'路径检查通过 · {length:.1f} m')
                self.preview_handle = None
                self.log('success', f'Nav2 路径检查通过 · {length:.1f} m · 尚未启动运动')
            except Exception as error:
                failed('路径检查失败：' + str(error))

        def accepted(future):
            try:
                handle = future.result()
                if not current():
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    failed('Nav2 拒绝路径检查请求，请检查规划器是否就绪')
                    return
                self.preview_handle = handle
                handle.get_result_async().add_done_callback(finished)
            except Exception as error:
                if current():
                    failed('路径检查请求失败：' + str(error))
        client.send_goal_async(goal).add_done_callback(accepted)
        return {'ok': True, 'preview_id': ident}

    def _send_next(self):
        if not self.pending:
            self.data['mission'] = 'succeeded'
            self.data['remaining'] = 0
            self.data['path'] = []
            self.log('success', '导航完成，已到达全部目标点')
            return
        gen = self.generation
        points = list(self.pending) if self.mode == 'through_poses' else [self.pending.popleft()]
        if self.mode == 'through_poses':
            self.pending.clear()
        goal = self.actions[self.mode].Goal()
        poses = [self.pose_stamped(p) for p in points]
        if self.mode == 'through_poses':
            goal.poses = poses
        else:
            goal.pose = poses[0]
        self.data['goal'] = points[-1]
        self.data['mission'] = 'sending'

        def feedback(msg):
            if gen != self.generation:
                return
            fb = msg.feedback
            with self.lock:
                self.data.update(remaining=float(fb.distance_remaining),
                                 elapsed=fb.navigation_time.sec + fb.navigation_time.nanosec / 1e9,
                                 recoveries=fb.number_of_recoveries)
                if self.mode == 'through_poses':
                    self.data['completed'] = self.data['total'] - fb.number_of_poses_remaining

        def accepted(future):
            if gen != self.generation:
                return
            try:
                self.handle = future.result()
                if not self.handle.accepted:
                    self.pending.clear()
                    self.data['mission'] = 'failed'
                    self.log('error', 'Nav2 拒绝了导航目标')
                    return
                if self.canceling:
                    self.handle.cancel_goal_async()
                else:
                    self.data['mission'] = 'running'
                    self.log('info', f"Nav2 已接受目标 ({points[-1]['x']:.2f}, {points[-1]['y']:.2f})，朝向 {math.degrees(points[-1]['yaw']):.1f}°")
                self.handle.get_result_async().add_done_callback(finished)
            except Exception as error:
                self.pending.clear()
                self.data['mission'] = 'failed'
                self.log('error', '发送导航失败：' + str(error))

        def finished(future):
            if gen != self.generation:
                return
            self.handle = None
            try:
                status = future.result().status
            except Exception as error:
                status = 6
                self.log('error', str(error))
            if self.canceling or status == 5:
                self.data.update(mission='canceled', path=[])
                self.canceling = False
                self.log('warning', '导航任务已取消')
            elif status != 4:
                self.pending.clear()
                self.data.update(mission='failed', path=[])
                self.log('error', f'导航失败（Action 状态 {status}），请检查目标和运行日志')
            else:
                self.data['completed'] += len(points) if self.mode == 'waypoints' else 0
                if self.mode == 'through_poses':
                    self.data['completed'] = self.data['total']
                self._send_next()
        self.clients[self.mode].send_goal_async(goal, feedback_callback=feedback).add_done_callback(accepted)

    def close(self):
        self.quit.set()
        self.thread.join(timeout=3)
