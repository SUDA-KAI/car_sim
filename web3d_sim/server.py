#!/usr/bin/python3
"""web3d_sim — 纯 Web 3D 仿真控制台后端.

Gazebo Fortress 无界面跑物理仿真, 浏览器 three.js 渲染三维过程, 不弹原生窗口.
- REST: 场景列表 / 仿真启停 / 地图上传(转 3D 墙体世界) / Nav2 目标
- rclpy 后台线程: NavigateToPose action client + /clock
- rosbridge_websocket (9090) 由本服务随仿真一起拉起, 前端订阅实时话题
"""
import base64
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml
from PIL import Image

HERE = Path(__file__).resolve().parent
# 优先取仓库内布局 (car_sim/web3d_sim/), 否则用环境变量/本机默认位置
CAR_SIM = Path(os.environ.get('CAR_SIM_ROOT') or
               (HERE.parent if (HERE.parent / 'run_sim.sh').exists() else '/home/a/car_sim'))
DATA = HERE / 'data'
MAPS_DIR = DATA / 'maps'
WORLDS_DIR = DATA / 'worlds'
LOGS = HERE / 'logs'

WEB_PORT = int(os.environ.get('WEB3D_PORT', '8780'))
ROSBRIDGE_PORT = int(os.environ.get('WEB3D_BRIDGE_PORT', '9090'))
SIM_DOMAIN = os.environ.get('WEB3D_DOMAIN', '74')
SIM_PARTITION = f'web3d_sim_{SIM_DOMAIN}'

# 服务自身的 rclpy 节点必须与仿真同域 (rclpy.init 时读取)
os.environ['ROS_DOMAIN_ID'] = SIM_DOMAIN

STATIC = HERE / 'static'

_state_lock = threading.Lock()
_sim_proc = None      # run_sim.sh process
_bridge_proc = None   # rosbridge_websocket process
_clock = {'sec': 0, 'nanosec': 0}
_nav = {'state': 'idle', 'detail': '', 'distance': None}
_frames = {'left': None, 'right': None}  # 最新编码后的 JPEG 相机帧


def _encode_frame(side, msg):
    """sensor_msgs/Image -> throttled JPEG cache (~8 fps max)."""
    import cv2
    import numpy as np
    now = time.time()
    frame = _frames.get(side)
    if frame is not None and now - frame['ts'] < 0.12:
        return
    w, h, enc = msg.width, msg.height, msg.encoding
    if enc not in ('bgr8', 'rgb8', 'mono8'):
        return
    ch = 1 if enc == 'mono8' else 3
    step = msg.step if msg.step else w * ch
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if arr.size < h * step:
        return
    arr = arr.reshape(h, step)[:, :w * ch]
    if enc == 'bgr8':
        img = arr.reshape(h, w, 3)
    elif enc == 'rgb8':
        img = cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    else:
        img = arr.reshape(h, w)
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 72])
    if ok:
        _frames[side] = {'jpeg': buf.tobytes(), 'ts': now}

# ---------------------------------------------------------------- rclpy side
_ros_ready = False
_ros_error = ''
_node = None
_goal_handle = None


def ros_spin():
    global _ros_ready, _ros_error, _node, _goal_handle
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from rosgraph_msgs.msg import Clock
    except Exception as exc:  # pragma: no cover
        _ros_error = f'rclpy import failed: {exc}'
        return

    rclpy.init()
    node = Node('web3d_sim_server')
    _node = node

    def on_clock(msg):
        _clock['sec'] = msg.clock.sec
        _clock['nanosec'] = msg.clock.nanosec

    node.create_subscription(Clock, '/clock', on_clock, 10)

    from sensor_msgs.msg import Image

    def make_image_cb(side):
        def cb(msg):
            try:
                _encode_frame(side, msg)
            except Exception:
                pass
        return cb

    node.create_subscription(Image, '/camera_left/image_rect', make_image_cb('left'), 2)
    node.create_subscription(Image, '/camera_right/image_rect', make_image_cb('right'), 2)

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    node._web3d_action_client = client
    _ros_ready = True
    try:
        rclpy.spin(node)
    except Exception:
        pass


def send_nav_goal(x, y, yaw):
    """Send NavigateToPose goal; returns (ok, message)."""
    global _goal_handle
    if not _ros_ready:
        return False, f'ROS 后端未就绪 ({_ros_error or "初始化中"})'
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose

    client = _node._web3d_action_client
    if not client.wait_for_server(timeout_sec=5.0):
        return False, 'NavigateToPose action 不可用 (Nav2 未激活或仿真未启动)'

    goal = NavigateToPose.Goal()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # stamp with the Gazebo sim clock (cached from /clock); 0 = latest TF
    pose.header.stamp.sec = int(_clock.get('sec', 0))
    pose.header.stamp.nanosec = int(_clock.get('nanosec', 0))
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    goal.pose = pose

    with _state_lock:
        _nav.update(state='pending', detail=f'目标 ({x:.2f}, {y:.2f})', distance=None)

    def on_feedback(msg):
        with _state_lock:
            _nav['distance'] = round(float(msg.feedback.distance_remaining), 2)

    def on_result(future):
        result = future.result()
        code = result.status
        label = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(code, f'status_{code}')
        with _state_lock:
            _nav.update(state=label, detail='到达目标' if code == 4 else '未完成')

    send_future = client.send_goal_async(goal, feedback_callback=on_feedback)

    def on_accept(future):
        global _goal_handle
        handle = future.result()
        if not handle.accepted:
            with _state_lock:
                _nav.update(state='rejected', detail='目标被拒绝')
            return
        _goal_handle = handle
        with _state_lock:
            _nav.update(state='active', detail='导航中')
        handle.get_result_async().add_done_callback(on_result)

    send_future.add_done_callback(on_accept)
    return True, '目标已发送'


def cancel_nav():
    global _goal_handle
    with _state_lock:
        handle, _goal_handle = _goal_handle, None
        _nav.update(state='canceling', detail='取消中')
    if handle is not None:
        handle.cancel_goal_async()
        return True
    return False


# ------------------------------------------------------------- map utilities
def parse_map_yaml(yaml_path: Path):
    cfg = yaml.safe_load(yaml_path.read_text())
    img_path = (yaml_path.parent / cfg['image']).resolve()
    return cfg, img_path


def map_meta_and_occupancy(yaml_path: Path):
    """Return meta + occupied runs for wall extrusion, mirroring map_server rules."""
    cfg, img_path = parse_map_yaml(yaml_path)
    img = Image.open(img_path).convert('L')
    w, h = img.size
    pixels = list(img.getdata())
    negate = bool(cfg.get('negate', 0))
    occ_th = float(cfg.get('occupied_thresh', 0.65))
    free_th = float(cfg.get('free_thresh', 0.25))
    # map_server: p = (255 - value)/255 unless negate; occupied if p > occ_th
    runs = []  # [start_index, length, ...] of occupied cells, row-major (row 0 = top)
    run_start = None
    for i, v in enumerate(pixels):
        p = (v / 255.0) if negate else ((255 - v) / 255.0)
        occ = p > occ_th
        if occ and run_start is None:
            run_start = i
        elif not occ and run_start is not None:
            runs.append([run_start, i - run_start])
            run_start = None
    if run_start is not None:
        runs.append([run_start, len(pixels) - run_start])
    origin = cfg.get('origin', [0, 0, 0])
    meta = dict(
        width=w, height=h,
        resolution=float(cfg['resolution']),
        origin_x=float(origin[0]), origin_y=float(origin[1]),
        origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
        negate=negate, occupied_thresh=occ_th, free_thresh=free_th,
        image=cfg['image'],
    )
    return meta, runs


def find_spawn(meta, runs):
    """Pick a spawn cell with >=0.8 m clearance; fall back to map centre."""
    w, h, res = meta['width'], meta['height'], meta['resolution']
    occ = [False] * (w * h)
    for start, length in runs:
        for i in range(start, start + length):
            occ[i] = True

    def free_cell(cx, cy):
        r = max(1, int(0.8 / res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < w and 0 <= y < h and occ[y * w + x]:
                    return False
                # treat map borders as blocked
                if not (0 <= x < w and 0 <= y < h):
                    return False
        return True

    candidates = [(w // 2, h // 2)]
    step = max(1, min(w, h) // 20)
    for cy in range(h // 4, 3 * h // 4, step):
        for cx in range(w // 4, 3 * w // 4, step):
            candidates.append((cx, cy))
    for cx, cy in candidates:
        if free_cell(cx, cy):
            # cell -> world (row 0 is top of image = max y)
            x = meta['origin_x'] + (cx + 0.5) * res
            y = meta['origin_y'] + (h - cy - 0.5) * res
            return [round(x, 3), round(y, 3), 0.0]
    return [0.0, 0.0, 0.0]


def list_scenes():
    scenes = []
    for folder, kind in ((CAR_SIM / 'worlds', 'builtin'),
                         (CAR_SIM / 'test_scenes' / 'worlds', 'test'),
                         (WORLDS_DIR, 'custom')):
        if not folder.is_dir():
            continue
        for sdf in sorted(folder.glob('*.sdf')):
            meta_file = sdf.with_suffix('.json')
            if not meta_file.is_file():
                continue
            try:
                meta = json.loads(meta_file.read_text())
            except Exception:
                continue
            map_path = Path(meta.get('map', ''))
            if not map_path.is_absolute():
                map_path = meta_file.parent / map_path
            if not map_path.is_file():
                continue  # 地图缺失的场景无法显示 (历史元数据含作者机器绝对路径)
            scenes.append(dict(
                id=f'{kind}:{sdf.stem}',
                name=sdf.stem,
                kind=kind,
                world=str(sdf),
                meta_file=str(meta_file),
                map_path=str(map_path.resolve()),
                spawn=meta.get('spawn', [0, 0, 0]),
            ))
    return scenes


def image_to_png(yaml_path: Path, dest: Path):
    cfg, img_path = parse_map_yaml(yaml_path)
    Image.open(img_path).convert('L').save(dest)


# ------------------------------------------------------------- robot model
ASSEMBLED = CAR_SIM / 'models' / 'assembled'
MESH_DIR = ASSEMBLED / 'meshes'


def _rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis RPY: R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Returns 3x3 rows."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec(m, v):
    return [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]]


def robot_model_data():
    """Assembled car visuals from the project's own snapshot + asset manifest.

    Mirrors assembly_config.apply_visuals: mesh pose = frame_world @ offset_m,
    mesh vertices scaled by asset['scale'].
    """
    snapshot = json.loads((ASSEMBLED / 'assembly_snapshot.json').read_text())
    manifest = json.loads((ASSEMBLED / 'asset_manifest.json').read_text())
    assets = manifest['assets']

    parts = {p['id']: p for p in snapshot['parts']}

    def world_frame(pid, _depth=0):
        if _depth > 32 or pid not in parts:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        p = parts[pid]
        xyz, rpy = list(map(float, p['xyz_m'])), list(map(float, p['rpy_deg']))
        rpy = [a * math.pi / 180.0 for a in rpy]
        if not p.get('parent') or p['parent'] == pid:
            return xyz, rpy
        pxyz, prpy = world_frame(p['parent'], _depth + 1)
        rot = _rpy_matrix(*prpy)
        composed_xyz = [a + b for a, b in zip(pxyz, _mat_vec(rot, xyz))]
        composed_rpy = [a + b for a, b in zip(prpy, rpy)]
        return composed_xyz, composed_rpy

    visuals = []
    for pid, p in parts.items():
        asset = assets.get(p.get('asset'))
        if asset is None:
            continue
        xyz, rpy = world_frame(pid)
        offset = asset.get('offset_m', [0, 0, 0])
        rot = _rpy_matrix(*rpy)
        final_xyz = [a + b for a, b in zip(xyz, _mat_vec(rot, offset))]
        visuals.append(dict(
            id=pid, mesh=Path(asset['mesh']).name,
            xyz=[round(v, 5) for v in final_xyz],
            rpy=[round(v, 5) for v in rpy],
            scale=float(asset.get('scale', 1.0)),
            color=asset.get('color', '#888888'),
        ))

    markers = []
    for pid in ('camera_left_link', 'camera_right_link', 'imu_link', 'third_crossbar'):
        if pid in parts:
            xyz, rpy = world_frame(pid)
            markers.append(dict(id=pid,
                                xyz=[round(v, 5) for v in xyz],
                                rpy=[round(v, 5) for v in rpy]))

    laser_xyz, laser_rpy = world_frame('unilidar_lidar')

    sonars = []
    for sid in ('ultrasonic_1', 'ultrasonic_2', 'ultrasonic_3', 'ultrasonic_4'):
        if sid in parts:
            sxyz, srpy = world_frame(sid)
            sonars.append(dict(id=sid,
                               x=round(sxyz[0], 4), y=round(sxyz[1], 4),
                               yaw=round(srpy[2], 4)))

    return dict(
        parts=visuals,
        markers=markers,
        wheels=[[0.4, 0.4, 0.2], [0.4, -0.4, 0.2], [-0.4, 0.4, 0.2], [-0.4, -0.4, 0.2]],
        wheel_radius=0.2, wheel_length=0.1,
        laser=dict(xyz=[round(v, 5) for v in laser_xyz],
                   rpy=[round(v, 5) for v in laser_rpy]),
        sonars=sonars,
    )


# ----------------------------------------------------------- sim process mgmt
def child_env():
    env = dict(os.environ)
    env.update(
        ROS_DOMAIN_ID=SIM_DOMAIN,
        SIM_ROS_DOMAIN_ID=SIM_DOMAIN,
        SIM_IGN_PARTITION=SIM_PARTITION,
        GAZEBO_MODEL_DATABASE_URI='',
    )
    env['PATH'] = '/usr/bin:' + env.get('PATH', '')
    return env


def sim_status():
    with _state_lock:
        sim_alive = _sim_proc is not None and _sim_proc.poll() is None
        bridge_alive = _bridge_proc is not None and _bridge_proc.poll() is None
        clock = dict(_clock)
        nav = dict(_nav)
    detail = ''
    log_file = LOGS / 'sim.log'
    if log_file.is_file():
        lines = log_file.read_text(errors='replace').splitlines()
        detail = '\n'.join(lines[-40:])
    return dict(
        sim_running=sim_alive,
        bridge_running=bridge_alive,
        domain=SIM_DOMAIN,
        rosbridge_port=ROSBRIDGE_PORT,
        clock=clock,
        nav=nav,
        ros_ready=_ros_ready,
        ros_error=_ros_error,
        log_tail=detail,
    )


def _kill_strays():
    """Kill orphaned sim/bridge process groups (e.g. after a server restart)."""
    def run_sim_pids():
        out = subprocess.run(['pgrep', '-f', 'run_sim.sh'],
                             capture_output=True, text=True).stdout.split()
        return [int(p) for p in out]

    for pid in run_sim_pids():
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except Exception:
            pass
    subprocess.run(['pkill', '-f', 'rosbridge_websocket'], capture_output=True)
    time.sleep(1.0)
    for pid in run_sim_pids():
        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except Exception:
            pass


def start_sim(world: Path, controller: str, max_speed: float):
    global _sim_proc, _bridge_proc
    with _state_lock:
        if _sim_proc is not None and _sim_proc.poll() is None:
            return False, '仿真已在运行, 请先停止'
        if subprocess.run(['pgrep', '-f', 'run_sim.sh'], capture_output=True).returncode == 0:
            return False, '检测到已有仿真进程残留 (服务曾重启?), 请先点击「停止」清理'
        if not world.is_file():
            return False, f'世界文件不存在: {world}'
        env = child_env()
        LOGS.mkdir(exist_ok=True)
        log = open(LOGS / 'sim.log', 'ab', buffering=0)
        _bridge_proc = subprocess.Popen(
            ['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
             '--ros-args', '-p', f'port:={ROSBRIDGE_PORT}'],
            env=env, stdout=log, stderr=log, start_new_session=True)
        _sim_proc = subprocess.Popen(
            ['bash', str(CAR_SIM / 'run_sim.sh'), 'gui:=false', f'world:={world}',
             f'controller:={controller}', f'max_speed:={max_speed}',
             'use_cameras:=true'],
            env=env, stdout=log, stderr=log, start_new_session=True)
        _nav.update(state='idle', detail='')
    return True, '仿真启动中 (约 30-60 秒)'


def stop_sim():
    global _sim_proc, _bridge_proc
    with _state_lock:
        procs = [_sim_proc, _bridge_proc]
        _sim_proc = _bridge_proc = None
        _nav.update(state='idle', detail='')
    for proc in procs:
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
    time.sleep(1.5)
    for proc in procs:
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    _kill_strays()
    return True, '已停止'


def handle_upload(payload):
    name = re.sub(r'[^\w\-]', '_', str(payload.get('name', 'custom'))).strip('_') or 'custom'
    yaml_text = payload.get('yaml', '')
    image_b64 = payload.get('image_b64', '')
    if not yaml_text or not image_b64:
        return False, '缺少 yaml 或 image'
    try:
        raw = base64.b64decode(image_b64)
        if len(raw) > 12 * 1024 * 1024:
            return False, '地图图像超过 12 MB'
        cfg = yaml.safe_load(yaml_text)
        # canonicalise image filename so yaml and file agree
        suffix = '.png' if raw[:8] == b'\x89PNG\r\n\x1a\n' else '.pgm'
        img_name = f'map_image{suffix}'
        folder = MAPS_DIR / f'{name}_{int(time.time())}'
        folder.mkdir(parents=True, exist_ok=True)
        (folder / img_name).write_bytes(raw)
        cfg['image'] = img_name
        map_yaml = folder / 'map.yaml'
        map_yaml.write_text(yaml.safe_dump(cfg, allow_unicode=True))
    except Exception as exc:
        return False, f'保存失败: {exc}'
    return build_world_from_map(map_yaml, name)


def build_world_from_map(map_yaml: Path, name: str):
    """Run the project's build_world.py: 2D map -> extruded-wall 3D world."""
    safe_name = re.sub(r'[^\w\-]', '_', name)
    out = WORLDS_DIR / f'{safe_name}_{int(time.time())}'
    try:
        meta, runs = map_meta_and_occupancy(map_yaml)
        spawn = find_spawn(meta, runs)
    except Exception as exc:
        return False, f'地图解析失败: {exc}'
    cmd = ['/usr/bin/python3', str(CAR_SIM / 'scripts' / 'build_world.py'),
           '--map', str(map_yaml), '--output', f'{out}.sdf', '--height', '1.5',
           '--spawn', str(spawn[0]), str(spawn[1]), str(spawn[2])]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120,
                              cwd=str(CAR_SIM / 'scripts'))
    except Exception as exc:
        return False, f'生成世界失败: {exc}'
    if proc.returncode != 0:
        return False, f'生成世界失败: {proc.stderr[-400:]}'
    meta_file = Path(f'{out}.json')
    if meta_file.is_file():
        meta = json.loads(meta_file.read_text())
        meta['map'] = str(map_yaml)  # keep absolute so the launch finds it
        meta_file.write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    return True, f'已生成场景 {out.name}'


def available_maps():
    """Map yaml files already on the server (project maps + previously uploaded)."""
    items = []
    allowed = [(CAR_SIM / 'maps', '项目地图'), (MAPS_DIR, '已上传')]
    for folder, kind in allowed:
        if not folder.is_dir():
            continue
        for y in sorted(folder.rglob('*.yaml')):
            try:
                cfg = yaml.safe_load(y.read_text())
                if not isinstance(cfg, dict) or 'image' not in cfg or 'resolution' not in cfg:
                    continue
                if not (y.parent / cfg['image']).is_file():
                    continue
            except Exception:
                continue
            items.append(dict(name=y.stem, yaml=str(y.resolve()), kind=kind))
    return items


def handle_use_map(payload):
    """Load a server-side map yaml as a new 3D scene."""
    raw = str(payload.get('yaml', ''))
    target = Path(raw).resolve()
    allowed_roots = [(CAR_SIM / 'maps').resolve(), MAPS_DIR.resolve()]
    if not any(str(target).startswith(str(r)) for r in allowed_roots):
        return False, '非法地图路径'
    if not target.is_file():
        return False, f'地图不存在: {target}'
    return build_world_from_map(target, target.stem)


# ----------------------------------------------------------------- HTTP layer
class Handler(BaseHTTPRequestHandler):
    server_version = 'web3d_sim/1.0'

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype='application/json; charset=utf-8'):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/' or path == '/index.html':
            return self._send_file(STATIC / 'index.html', 'text/html; charset=utf-8')
        if path.startswith('/static/'):
            rel = path[len('/static/'):]
            target = (STATIC / rel).resolve()
            if not str(target).startswith(str(STATIC.resolve())) or not target.is_file():
                return self._send(404, {'error': 'not found'})
            ctype = 'application/javascript' if target.suffix == '.js' else \
                'text/css' if target.suffix == '.css' else 'application/octet-stream'
            return self._send_file(target, ctype)
        if path == '/api/scenes':
            return self._send(200, {'scenes': list_scenes()})
        if path == '/api/maps/available':
            return self._send(200, {'maps': available_maps()})
        if path == '/api/sim/status':
            return self._send(200, sim_status())
        if path == '/api/nav/status':
            with _state_lock:
                return self._send(200, dict(_nav))
        if path == '/api/map':
            return self._serve_map_meta()
        if path == '/api/map/image.png':
            return self._serve_map_image()
        if path == '/api/robot':
            return self._send(200, robot_model_data())
        if path in ('/api/camera/left/stream', '/api/camera/right/stream'):
            return self._serve_mjpeg(path.split('/')[3])
        if path.startswith('/api/robot/mesh/'):
            name = Path(path[len('/api/robot/mesh/'):]).name  # strip any traversal
            target = (MESH_DIR / name).resolve()
            if not str(target).startswith(str(MESH_DIR.resolve())) or not target.is_file():
                return self._send(404, {'error': 'mesh not found'})
            return self._send(200, target.read_bytes(), 'model/stl')
        return self._send(404, {'error': 'not found'})

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        if length > 20 * 1024 * 1024:
            return self._send(413, {'ok': False, 'message': 'body too large'})
        try:
            payload = json.loads(self.rfile.read(length) or b'{}')
        except Exception:
            return self._send(400, {'ok': False, 'message': 'bad json'})
        path = self.path.split('?')[0]

        if path == '/api/sim/start':
            world = Path(payload.get('world', ''))
            ok, msg = start_sim(world,
                                str(payload.get('controller', 'mppi')),
                                float(payload.get('max_speed', 0.5)))
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        if path == '/api/sim/stop':
            ok, msg = stop_sim()
            return self._send(200, {'ok': ok, 'message': msg})
        if path == '/api/nav/goal':
            try:
                ok, msg = send_nav_goal(payload['x'], payload['y'], payload.get('yaw', 0.0))
            except KeyError:
                return self._send(400, {'ok': False, 'message': '需要 x/y'})
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        if path == '/api/nav/cancel':
            cancel_nav()
            return self._send(200, {'ok': True, 'message': 'canceling'})
        if path == '/api/maps/upload':
            ok, msg = handle_upload(payload)
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        if path == '/api/maps/use':
            ok, msg = handle_use_map(payload)
            return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
        return self._send(404, {'ok': False, 'message': 'not found'})

    # ---- helpers
    def _serve_mjpeg(self, side):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        last = None
        last_sent = 0.0
        try:
            while True:
                frame = _frames.get(side)
                now = time.time()
                # 帧内容变化时立即发送; 静止场景也每 0.5 秒补发心跳帧
                if frame and (frame['jpeg'] != last or now - last_sent > 0.5):
                    last = frame['jpeg']
                    last_sent = now
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                        + str(len(last)).encode() + b'\r\n\r\n' + last + b'\r\n')
                    self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def _send_file(self, path: Path, ctype):
        try:
            data = path.read_bytes()
        except Exception:
            return self._send(404, {'error': 'not found'})
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _current_map_yaml(self):
        """Resolve the map yaml for the selected/current scene."""
        scene_id = self.headers.get('X-Scene-Id', '')
        scenes = {s['id']: s for s in list_scenes()}
        scene = scenes.get(scene_id)
        if scene is None:
            scene = scenes.get('builtin:map_20260817_154310_filtered') or next(iter(scenes.values()), None)
        if scene is None:
            return None, None
        meta = json.loads(Path(scene['meta_file']).read_text())
        map_path = Path(meta.get('map', ''))
        if not map_path.is_absolute():
            map_path = Path(scene['meta_file']).parent / map_path
        return scene, map_path.resolve()

    def _serve_map_meta(self):
        scene, map_yaml = self._current_map_yaml()
        if map_yaml is None or not map_yaml.is_file():
            return self._send(404, {'error': 'no map'})
        meta, runs = map_meta_and_occupancy(map_yaml)
        meta['scene'] = scene['name']
        meta['occupied_runs'] = runs
        return self._send(200, meta)

    def _serve_map_image(self):
        scene, map_yaml = self._current_map_yaml()
        if map_yaml is None or not map_yaml.is_file():
            return self._send(404, {'error': 'no map'})
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.png') as tmp:
            image_to_png(map_yaml, Path(tmp.name))
            return self._send(200, Path(tmp.name).read_bytes(), 'image/png')


def main():
    for folder in (MAPS_DIR, WORLDS_DIR, LOGS):
        folder.mkdir(parents=True, exist_ok=True)
    threading.Thread(target=ros_spin, daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', WEB_PORT), Handler)
    print(f'[web3d_sim] http://127.0.0.1:{WEB_PORT}  (rosbridge :{ROSBRIDGE_PORT}, '
          f'ROS_DOMAIN_ID={SIM_DOMAIN})')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        stop_sim()


if __name__ == '__main__':
    main()
