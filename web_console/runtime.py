"""Own exactly one isolated simulation process group and its log stream."""
from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from .catalog import ROOT, finite
from .ros_bridge import RosBridge


class Runtime:
    def __init__(self, catalog, domain=73, partition='patrol_web_console'):
        if not 1 <= domain <= 232:
            raise ValueError('仿真域必须在 1–232 之间；不允许使用实车默认域 0')
        self.catalog, self.domain, self.partition = catalog, domain, partition
        self.lock = threading.RLock()
        self.events = deque(maxlen=400)
        self.sequence = 0
        self.process = None
        self.state = 'idle'
        self.map_id = None
        self.settings = {}
        self.started = None
        self.exit_code = None
        self.bridge = RosBridge(self.log, domain)
        self.log('info', '控制台已连接本机，选择地图即可启动仿真')

    def log(self, level, message):
        with self.lock:
            self.sequence += 1
            self.events.append(dict(id=self.sequence, time=time.time(), level=level, message=message[:1400]))

    def snapshot(self, after=0):
        with self.lock:
            return dict(state=self.state, map_id=self.map_id, domain=self.domain,
                        partition=self.partition, settings=self.settings, exit_code=self.exit_code,
                        uptime=round(time.monotonic() - self.started, 1) if self.started and self.state in ('starting', 'running', 'stopping') else 0,
                        telemetry=self.bridge.snapshot(),
                        events=[e for e in self.events if e['id'] > after])

    def start(self, payload):
        entry = self.catalog.get(payload['map_id'])
        controller = payload.get('controller', 'scene')
        localization = payload.get('localization', 'auto')
        speed = finite(payload.get('max_speed', .3))
        planning_timeout = finite(payload.get('planning_timeout', 20))
        if not 1 <= planning_timeout <= 60:
            raise ValueError('路径搜索时限必须在 1–60 秒之间')
        if controller not in ('scene', 'teb', 'mppi') or localization not in ('auto', 'amcl', 'ground_truth'):
            raise ValueError('控制器或定位模式无效')
        if localization == 'ground_truth' and entry['localization'] != 'ground_truth':
            raise ValueError('当前地图不提供真值定位，请选择自动或 AMCL')
        if not 0 < speed <= .8:
            raise ValueError('速度范围必须在 0–0.8 m/s 之间')
        for key in ('gui', 'nav', 'use_l2', 'use_ultrasonic'):
            if key in payload and not isinstance(payload[key], bool):
                raise ValueError('开关值必须是布尔值')
        with self.lock:
            if self.process is not None or self.state == 'stopping':
                raise ValueError('请先停止当前仿真，再切换地图或配置')
        # Never wait on the ROS thread while holding the event/state lock:
        # action callbacks on that thread also append runtime events.
        self.bridge.command('session', True)
        with self.lock:
            settings = dict(controller=controller, localization=localization, max_speed=speed, planning_timeout=planning_timeout,
                            gui=payload.get('gui', True), nav=payload.get('nav', True),
                            use_l2=payload.get('use_l2', True), use_ultrasonic=payload.get('use_ultrasonic', True))
            args = [str(ROOT / 'run_sim.sh'), 'world:=' + entry['world'], 'sensor_monitor:=false']
            args += [key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value)) for key, value in settings.items()]
            env = dict(os.environ, SIM_ROS_DOMAIN_ID=str(self.domain), ROS_DOMAIN_ID=str(self.domain),
                       SIM_IGN_PARTITION=self.partition, IGN_PARTITION=self.partition,
                       PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
            try:
                process = subprocess.Popen(args, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                           stderr=subprocess.STDOUT, text=True, errors='replace', start_new_session=True)
            except OSError:
                self.bridge.active = False
                raise
            self.process = process
            self.map_id, self.settings = entry['id'], settings
            self.state, self.exit_code, self.started = 'starting', None, time.monotonic()
            self.log('info', f"启动 Gazebo · {entry['title']} · ROS 域 {self.domain}")
            threading.Thread(target=self._watch, args=(process,), daemon=True).start()
        return {'ok': True}

    def _watch(self, process):
        import re
        ansi = re.compile(r'\x1b\[[0-9;]*m')
        for line in process.stdout:
            line = ansi.sub('', line.strip())
            if line:
                lower = line.lower()
                level = 'error' if '[error]' in lower or 'traceback' in lower else 'warning' if '[warn' in lower else 'info'
                self.log(level, line)
                with self.lock:
                    if self.state == 'starting' and self.bridge.snapshot()['connected']:
                        self.state = 'running'
        code = process.wait()
        # A failed launch can leave children behind. Signal only our own group.
        self._signal_group(process.pid, signal.SIGTERM)
        with self.lock:
            stopping = self.state == 'stopping'
            self.exit_code = code
            self.state = 'stopping'
        # Retain ownership until all group members have received a hard stop.
        time.sleep(.4)
        self._signal_group(process.pid, signal.SIGKILL)
        try:
            self.bridge.command('session', False)
        except Exception as error:
            self.log('error', str(error))
        with self.lock:
            if self.process is process:
                self.process = None
                self.state = 'idle' if stopping or code == 0 else 'error'
        self.log('info' if stopping or code == 0 else 'error', f'仿真已停止 · 退出码 {code}')

    @staticmethod
    def _signal_group(pid, sig):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            pass

    def stop(self):
        with self.lock:
            process = self.process
            if process is None:
                return {'ok': True}
            if self.state == 'stopping':
                return {'ok': True}
            self.state = 'stopping'
        try:
            self.bridge.command('cancel')
        except Exception:
            pass
        self.log('warning', '正在停止当前仿真进程组')
        self._signal_group(process.pid, signal.SIGINT)
        def escalate():
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self._signal_group(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self._signal_group(process.pid, signal.SIGKILL)
        threading.Thread(target=escalate, daemon=True).start()
        return {'ok': True}

    def navigate(self, payload, preview=False):
        with self.lock:
            if not self.process or self.state not in ('starting', 'running') or not self.settings.get('nav'):
                raise ValueError('请先启动启用导航的仿真')
            if payload.get('map_id') != self.map_id:
                raise ValueError('选中地图与正在运行的地图不同')
            entry = self.catalog.get(self.map_id)
            points = payload['points']
            if not isinstance(points, list) or not 1 <= len(points) <= 64:
                raise ValueError('请设置 1–64 个导航目标')
            checked = []
            for point in points:
                x, y = self.catalog.check_point(entry, point['x'], point['y'])
                checked.append(dict(x=x, y=y, yaw=finite(point.get('yaw', 0))))
            mode = payload.get('mode', 'waypoints')
            if mode not in ('waypoints', 'through_poses'):
                raise ValueError('导航模式无效')
        return self.bridge.command('preview' if preview else 'navigate', checked, mode)

    def initial_pose(self, payload):
        with self.lock:
            if not self.process or payload.get('map_id') != self.map_id or self.state not in ('starting', 'running'):
                raise ValueError('请先启动选中的地图')
            entry = self.catalog.get(self.map_id)
            localization = self.settings['localization']
            if localization == 'ground_truth' or localization == 'auto' and entry['localization'] == 'ground_truth':
                raise ValueError('真值定位无需设置 AMCL 初始位姿')
            x, y = self.catalog.check_point(entry, payload['x'], payload['y'])
            point = dict(x=x, y=y, yaw=finite(payload.get('yaw', 0)))
        return self.bridge.command('initial_pose', point)

    def close(self):
        self.stop()
        process = self.process
        if process:
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                self._signal_group(process.pid, signal.SIGKILL)
            deadline = time.monotonic() + 3
            while self.process and time.monotonic() < deadline:
                time.sleep(.05)
        self.bridge.close()
