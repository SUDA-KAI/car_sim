"""Discover existing world bundles and prepare user supplied occupancy maps."""
import base64
import io
import json
import math
from pathlib import Path
import sys
import threading
import uuid

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from build_world import build

LABELS = {
    'map_20260817_154310_filtered': ('现场地图 · 滤波版', '现场', '现场扫描地图，保留未知区域；AMCL 定位。'),
    'map_20260817_154310': ('现场地图 · 原始版', '现场', '现场原始占据地图。'),
    'demo': ('室内导航实验场', '基础', '开放式室内场景，适合体验地图选点与路径规划。'),
    'narrow_17': ('1.7 m 直线窄道', '窄道', '真实车宽 0.9 m，直线通行与倒车测试。'),
    'narrow_17_baseline': ('直线窄道 · 参数对照', '窄道', '保留原始参数的基准对照场景。'),
    'narrow_turns': ('环形窄道 · 连续转弯', '窄道', '1.7 m 净宽，5 m 中心线转弯半径；历史窄弯测试尚未通过。'),
    'narrow_ramp_6deg': ('窄道坡面 · 6°', '坡道', '6° 三维坡面；二维导航地图不表达地面高程。'),
    'narrow_ramp_12deg': ('窄道坡面 · 12°', '坡道', '12° 三维坡面，用于爬坡及车身姿态观察。'),
    'narrow_slip_normal': ('窄道 · 标准摩擦', '摩擦', '标准摩擦地面测试。'),
    'narrow_slip_low': ('窄道 · 低摩擦', '摩擦', '低摩擦地面，观察滑移与制动效果。'),
    'narrow_slip_ice': ('窄道 · 极低摩擦', '摩擦', '极低摩擦地面，导航可能失败。'),
    'turns': ('宽道连续转弯', '基础', '宽版历史转弯场景。'),
    'ramp_6deg': ('宽道坡面 · 6°', '坡道', '宽版 6° 坡道。'),
    'ramp_12deg': ('宽道坡面 · 12°', '坡道', '宽版 12° 坡道。'),
    'slip_normal': ('宽道 · 标准摩擦', '摩擦', '宽版标准摩擦场景。'),
    'slip_low': ('宽道 · 低摩擦', '摩擦', '宽版低摩擦场景。'),
    'slip_ice': ('宽道 · 极低摩擦', '摩擦', '宽版极低摩擦场景。'),
}


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('坐标与参数必须是有限数值')
    return value


def read_map(path):
    cfg = yaml.safe_load(path.read_text())
    if not isinstance(cfg, dict) or cfg.get('mode', 'trinary') != 'trinary':
        raise ValueError('仅支持 ROS trinary 占据地图')
    resolution = finite(cfg['resolution'])
    origin = [finite(v) for v in cfg['origin']]
    if len(origin) != 3 or not 0 < resolution <= 10:
        raise ValueError('地图分辨率或原点无效')
    free, occupied = finite(cfg['free_thresh']), finite(cfg['occupied_thresh'])
    if not 0 <= free < occupied <= 1 or cfg.get('negate', 0) not in (0, 1):
        raise ValueError('地图阈值或 negate 无效')
    image_path = (path.parent / cfg['image']).resolve()
    with Image.open(image_path) as image:
        if image.width * image.height > 6_000_000:
            raise ValueError('地图最多支持 600 万像素')
        pixels = np.asarray(image.convert('L'))
    probability = pixels / 255.0 if cfg.get('negate', 0) else 1.0 - pixels / 255.0
    return cfg, probability < free, probability > occupied, resolution, origin


class Catalog:
    def __init__(self, root=ROOT, data=None):
        self.root = Path(root)
        self.data = Path(data or self.root / 'web_console/data/maps')
        self.lock = threading.RLock()
        self.entries = {}
        self.refresh()

    def refresh(self):
        with self.lock:
            entries = {}
            paths = sorted((self.root / 'worlds').glob('*.sdf'))
            paths += sorted((self.root / 'test_scenes/worlds').glob('*.sdf'))
            paths += sorted(self.data.glob('*/world.sdf'))
            for world in paths:
                try:
                    meta = json.loads(world.with_suffix('.json').read_text())
                    map_path = (world.parent / meta['map']).resolve()
                    cfg, free, occupied, resolution, origin = read_map(map_path)
                    ident = world.parent.name if world.name == 'world.sdf' else world.stem
                    title, category, description = LABELS.get(ident, (meta.get('title', ident), '导入', '由二维地图挤出生成的 Gazebo 场景。'))
                    route_path = meta.get('route')
                    if not route_path and ident in ('demo', 'map_20260817_154310_filtered', 'map_20260817_154310'):
                        route_path = str(self.root / 'config' / f'patrol_{ident}.yaml')
                    route = {}
                    if route_path:
                        route_file = (world.parent / route_path).resolve()
                        if route_file.exists():
                            route = yaml.safe_load(route_file.read_text()) or {}
                    spawn = [finite(v) for v in meta['spawn']]
                    if len(spawn) != 3:
                        continue
                    h, w = free.shape
                    colors = np.empty((h, w, 3), dtype=np.uint8)
                    colors[:] = (20, 30, 40)
                    colors[free] = (39, 57, 66)
                    colors[occupied] = (126, 161, 167)
                    output = io.BytesIO()
                    Image.fromarray(colors).save(output, format='PNG')
                    entry = dict(id=ident, title=title, category=category, description=description,
                                 width=w, height=h, resolution=resolution, origin=origin, spawn=spawn,
                                 size=[round(w * resolution, 2), round(h * resolution, 2)],
                                 localization=meta.get('localization', 'amcl'),
                                 waypoints=route.get('waypoints', []),
                                 navigation_mode=route.get('navigation_mode', 'waypoints'),
                                 world=str(world), map=str(map_path), png=output.getvalue(), free=free,
                                 cfg=cfg)
                    entries[ident] = entry
                except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
                    continue
            self.entries = entries

    def get(self, ident):
        with self.lock:
            if ident not in self.entries:
                raise ValueError('地图不存在，请刷新地图列表')
            return self.entries[ident]

    def public(self, entry):
        return {k: v for k, v in entry.items() if k not in ('world', 'map', 'png', 'free', 'cfg')}

    def list(self):
        with self.lock:
            return [self.public(e) for e in sorted(self.entries.values(), key=lambda e: (e['id'] != 'map_20260817_154310_filtered', e['category'] != '基础', e['title']))]

    def check_point(self, entry, x, y):
        x, y = finite(x), finite(y)
        ox, oy, yaw = entry['origin']
        dx, dy = x - ox, y - oy
        col = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / entry['resolution'])
        row = entry['height'] - 1 - math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / entry['resolution'])
        if not (0 <= row < entry['height'] and 0 <= col < entry['width'] and entry['free'][row, col]):
            raise ValueError('目标点必须位于地图内的已知空闲区域')
        return x, y

    def import_map(self, payload):
        title = str(payload.get('title', '自定义地图')).strip()[:60] or '自定义地图'
        cfg = yaml.safe_load(payload['yaml'])
        if not isinstance(cfg, dict):
            raise ValueError('YAML 必须包含地图配置')
        image_name = cfg.get('image', '')
        supplied_name = payload.get('image_name', '')
        if not isinstance(image_name, str) or Path(image_name).name != supplied_name:
            raise ValueError('图片文件名必须与 YAML 中的 image 一致')
        raw = base64.b64decode(payload['image'], validate=True)
        if len(raw) > 12_000_000:
            raise ValueError('地图图片不能超过 12 MB')
        with Image.open(io.BytesIO(raw)) as image:
            if image.format not in ('PNG', 'PPM') or image.width * image.height > 6_000_000:
                raise ValueError('请选择 PGM / PNG 地图，最多 600 万像素')
            image = image.convert('L')
        ident = 'custom_' + uuid.uuid4().hex[:12]
        folder = self.data / ident
        folder.mkdir(parents=True)
        try:
            image.save(folder / 'image.pgm')
            cfg['image'] = 'image.pgm'
            map_path = folder / 'map.yaml'
            map_path.write_text(yaml.safe_dump(cfg))
            cfg, free, _, resolution, origin = read_map(map_path)
            # Match build_world's square clearance check, including image edges.
            margin = math.ceil(.80 / resolution)
            window = 2 * margin + 1
            if free.shape[0] < window or free.shape[1] < window:
                raise ValueError('地图没有可容纳小车的已知空闲区域（需要 0.8 m 出生余量）')
            sums = np.pad(free.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
            totals = sums[window:, window:] - sums[:-window, window:] - sums[window:, :-window] + sums[:-window, :-window]
            rows, cols = np.where(totals == window * window)
            if not len(rows):
                raise ValueError('未找到具有 0.8 m 出生余量的位置，请扩大已知空闲区')
            best = np.argmin((rows + margin - free.shape[0] / 2) ** 2 + (cols + margin - free.shape[1] / 2) ** 2)
            lx = (cols[best] + margin + .5) * resolution
            ly = (free.shape[0] - rows[best] - margin - .5) * resolution
            ox, oy, yaw = origin
            spawn = [ox + math.cos(yaw) * lx - math.sin(yaw) * ly,
                     oy + math.sin(yaw) * lx + math.cos(yaw) * ly, yaw]
            height = finite(payload.get('wall_height', 1.5))
            if not .2 <= height <= 5:
                raise ValueError('墙体高度必须在 0.2–5 m 之间')
            build(map_path.resolve(), (folder / 'world.sdf').resolve(), height, spawn)
            metadata = json.loads((folder / 'world.json').read_text())
            metadata['title'] = title
            (folder / 'world.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
        except Exception:
            import shutil
            shutil.rmtree(folder)
            raise
        self.refresh()
        return self.public(self.get(ident))
