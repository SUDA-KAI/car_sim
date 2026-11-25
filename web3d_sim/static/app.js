// Web3D 仿真导航台 — three.js 三维渲染 + roslib 实时数据
// 坐标约定: ROS map 坐标 (x, y, z-up) 直接映射到 three.js (x, y, z), camera.up = Z
import * as THREE from 'three';
import { STLLoader } from '/static/vendor/STLLoader.js';

const $ = (id) => document.getElementById(id);
const view = $('view');

// ---------------------------------------------------------------- three.js
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
view.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
scene.fog = new THREE.Fog(0x14171c, 60, 220);

const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 500);
camera.up.set(0, 0, 1);
const camTarget = new THREE.Vector3(0, 0, 0);
let camDist = 22, camAz = Math.PI * 0.35, camEl = 0.9;

function applyCamera() {
  const el = Math.max(0.05, Math.min(1.52, camEl));
  camera.position.set(
    camTarget.x + camDist * Math.cos(el) * Math.cos(camAz),
    camTarget.y + camDist * Math.cos(el) * Math.sin(camAz),
    camTarget.z + camDist * Math.sin(el),
  );
  camera.lookAt(camTarget);
}
applyCamera();

scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x22262c, 1.0));
const sun = new THREE.DirectionalLight(0xffffff, 0.9);
sun.position.set(30, -20, 50);
scene.add(sun);

const grid = new THREE.GridHelper(120, 60, 0x2c333c, 0x22282f);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.002;
scene.add(grid);

// ---------------------------------------------------------------- map layer
const mapGroup = new THREE.Group();
scene.add(mapGroup);
const map2d = { meta: null, img: null };   // 2D 栅格地图面板数据
const planWorld = [];                       // 规划路径世界坐标 (2D 叠加用)
const scanWorld = [];                       // 激光点世界坐标 (2D 叠加用)
const scanLocal = [];                       // 激光点车体坐标 (全景面板用)
const sonarMounts = [];                     // 超声波安装位姿 (x, y, yaw)
const sonarRanges = {};                     // id -> {range, max}

function cellToWorld(meta, idx) {
  const w = meta.width;
  const col = idx % w;
  const row = Math.floor(idx / w);
  return [
    meta.origin_x + (col + 0.5) * meta.resolution,
    meta.origin_y + (meta.height - row - 0.5) * meta.resolution,
  ];
}

async function loadMap(sceneId) {
  const [metaRes, imgRes] = await Promise.all([
    fetch('/api/map', { headers: { 'X-Scene-Id': sceneId } }),
    fetch('/api/map/image.png', { headers: { 'X-Scene-Id': sceneId } }),
  ]);
  if (!metaRes.ok) throw new Error('地图元数据获取失败');
  const meta = await metaRes.json();
  mapGroup.clear();

  // floor texture
  const blob = await imgRes.blob();
  const img = await createImageBitmap(blob);
  map2d.meta = meta;
  map2d.img = img;
  const cnv = document.createElement('canvas');
  cnv.width = img.width; cnv.height = img.height;
  const ctx = cnv.getContext('2d');
  ctx.drawImage(img, 0, 0);
  // dark theme tint: occupied dark, free lighter
  const tex = new THREE.CanvasTexture(cnv);
  tex.magFilter = THREE.LinearFilter;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  const wMeters = meta.width * meta.resolution;
  const hMeters = meta.height * meta.resolution;
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(wMeters, hMeters),
    new THREE.MeshStandardMaterial({ map: tex, roughness: 1, metalness: 0 }),
  );
  floor.position.set(meta.origin_x + wMeters / 2, meta.origin_y + hMeters / 2, 0.01);
  mapGroup.add(floor);

  // extruded walls from occupied runs
  const runs = meta.occupied_runs || [];
  let total = 0;
  for (const [, len] of runs) total += len;
  const res = meta.resolution;
  const box = new THREE.BoxGeometry(res * 0.97, res * 0.97, 1.5);
  const walls = new THREE.InstancedMesh(
    box,
    new THREE.MeshStandardMaterial({ color: 0x59636e, roughness: 0.9 }),
    Math.max(1, total),
  );
  const m4 = new THREE.Matrix4();
  let i = 0;
  for (const [start, len] of runs) {
    for (let k = 0; k < len; k++, i++) {
      const [x, y] = cellToWorld(meta, start + k);
      m4.makeTranslation(x, y, 0.75);
      walls.setMatrixAt(i, m4);
    }
  }
  walls.count = i;
  walls.instanceMatrix.needsUpdate = true;
  mapGroup.add(walls);
  return meta;
}

// ---------------------------------------------------------------- robot
const robot = new THREE.Group();
scene.add(robot);
const stlLoader = new STLLoader();

// URDF 固定轴 RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll), 与服务端一致
function rpyToMatrix4([r, p, y]) {
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return new THREE.Matrix4().set(
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0,
    -sp, cp * sr, cp * cr, 0,
    0, 0, 0, 1,
  );
}

async function loadRobot() {
  const res = await fetch('/api/robot');
  if (!res.ok) throw new Error('车模接口不可用');
  const model = await res.json();
  robot.clear();

  // 真实车体/雷达/支架 STL 装配件
  for (const part of model.parts) {
    try {
      const geo = await stlLoader.loadAsync(`/api/robot/mesh/${part.mesh}`);
      if (part.scale && part.scale !== 1) geo.scale(part.scale, part.scale, part.scale);
      const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
        color: new THREE.Color(part.color), roughness: 0.6, metalness: 0.12,
      }));
      mesh.position.set(part.xyz[0], part.xyz[1], part.xyz[2]);
      mesh.quaternion.setFromRotationMatrix(rpyToMatrix4(part.rpy));
      robot.add(mesh);
    } catch (e) {
      console.warn('网格加载失败:', part.mesh, e);
    }
  }

  // 车轮 (URDF 圆柱几何; three.js 圆柱轴向为 Y = 轮轴方向, 无需旋转)
  const wgeo = new THREE.CylinderGeometry(model.wheel_radius, model.wheel_radius, model.wheel_length, 24);
  const wmat = new THREE.MeshStandardMaterial({ color: 0x15181c, roughness: 0.95 });
  for (const [wx, wy, wz] of model.wheels) {
    const wheel = new THREE.Mesh(wgeo, wmat);
    wheel.position.set(wx, wy, wz);
    robot.add(wheel);
  }

  // 相机 / IMU 标记块
  for (const mk of model.markers || []) {
    const box = new THREE.Mesh(
      new THREE.BoxGeometry(0.09, 0.06, 0.05),
      new THREE.MeshStandardMaterial({ color: 0x2b3a4a, roughness: 0.45, metalness: 0.3 }),
    );
    box.position.set(mk.xyz[0], mk.xyz[1], mk.xyz[2]);
    robot.add(box);
  }

  // 激光挂载位姿 (扫描点云变换用)
  if (model.laser) {
    state.laserMount = {
      x: model.laser.xyz[0], y: model.laser.xyz[1], z: model.laser.xyz[2],
      roll: model.laser.rpy[0], pitch: model.laser.rpy[1], yaw: model.laser.rpy[2],
    };
  }
  sonarMounts.length = 0;
  for (const s of model.sonars || []) {
    sonarMounts.push({ id: s.id, x: s.x, y: s.y, yaw: s.yaw });
  }
}

// ---------------------------------------------------------------- overlays
const SCAN_MAX = 2048;
const scanGeo = new THREE.BufferGeometry();
scanGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(SCAN_MAX * 3), 3));
const scanPts = new THREE.Points(scanGeo, new THREE.PointsMaterial({
  color: 0x3ecf8e, size: 0.075, sizeAttenuation: true,
}));
scanPts.frustumCulled = false;
scene.add(scanPts);

const planLine = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0xf2b04a }),
);
scene.add(planLine);

const trailMax = 4000;
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(trailMax * 3), 3));
let trailCount = 0;
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x4aa3ff }));
trail.frustumCulled = false;
scene.add(trail);

const goalMarker = new THREE.Mesh(
  new THREE.ConeGeometry(0.28, 0.8, 4),
  new THREE.MeshStandardMaterial({ color: 0xef6b73, emissive: 0x551518 }),
);
goalMarker.rotation.x = Math.PI / 2;
goalMarker.visible = false;
scene.add(goalMarker);

// ---------------------------------------------------------------- robot state
const state = {
  pose: null,          // {x, y, yaw}
  laserMount: { x: 0.6, y: 0.0, z: 0.5 },  // L2 雷达安装位 (sensor_mounting_tf.yaml)
};

function setRobotPose(x, y, yaw) {
  state.pose = { x, y, yaw };
  robot.position.set(x, y, 0);
  robot.rotation.set(0, 0, yaw);
  // trail
  const attr = trailGeo.getAttribute('position');
  if (trailCount < trailMax) {
    attr.setXYZ(trailCount, x, y, 0.04);
    trailCount++;
    trailGeo.setDrawRange(0, trailCount);
    attr.needsUpdate = true;
  }
}

function setScan(msg) {
  const attr = scanGeo.getAttribute('position');
  const pose = state.pose || { x: 0, y: 0, yaw: 0 };
  const cy = Math.cos(pose.yaw), sy = Math.sin(pose.yaw);
  const mount = state.laserMount;
  const mx = pose.x + mount.x * cy - mount.y * sy;
  const my = pose.y + mount.x * sy + mount.y * cy;
  let n = 0;
  scanWorld.length = 0;
  scanLocal.length = 0;
  for (let i = 0; i < msg.ranges.length && n < SCAN_MAX; i++) {
    const r = msg.ranges[i];
    if (!isFinite(r) || r < 0.06 || r > 30) continue;
    const a = msg.angle_min + i * msg.angle_increment;
    const lx = r * Math.cos(a), ly = r * Math.sin(a);
    const wx = mx + lx * cy - ly * sy, wy = my + lx * sy + ly * cy;
    attr.setXYZ(n++, wx, wy, mount.z);
    if (i % 3 === 0) {
      scanWorld.push(wx, wy);
      const lc = Math.cos(mount.yaw || 0), ls = Math.sin(mount.yaw || 0);
      scanLocal.push(mount.x + lx * lc - ly * ls, mount.y + lx * ls + ly * lc);
    }
  }
  scanGeo.setDrawRange(0, n);
  attr.needsUpdate = true;
  $('sb-scan').textContent = `${n} 回波`;
}

function setPlan(msg) {
  const pts = [];
  planWorld.length = 0;
  for (const p of msg.poses || []) {
    pts.push(new THREE.Vector3(p.pose.position.x, p.pose.position.y, 0.06));
    planWorld.push(p.pose.position.x, p.pose.position.y);
  }
  if (pts.length > 1) {
    planLine.geometry.dispose();
    planLine.geometry = new THREE.BufferGeometry().setFromPoints(pts);
  }
}

// ---------------------------------------------------------------- panoramic panel
const panoCanvas = $('pano');
const panoCtx = panoCanvas.getContext('2d');
const PANO_RADIUS_M = 4;  // 视野半径 4m

function sizePano() {
  const rect = panoCanvas.getBoundingClientRect();
  panoCanvas.width = Math.round(rect.width * devicePixelRatio);
  panoCanvas.height = Math.round(rect.width * devicePixelRatio);
}

function drawPano() {
  const ctx = panoCtx;
  const W = panoCanvas.width, H = panoCanvas.height;
  if (!W) return;
  const cx = W / 2, cy = H / 2;
  const scale = (W / 2 - 8 * devicePixelRatio) / PANO_RADIUS_M;
  ctx.fillStyle = '#0d1014';
  ctx.fillRect(0, 0, W, H);

  // 距离圈 (1m 间距)
  for (let r = 1; r <= PANO_RADIUS_M; r++) {
    ctx.strokeStyle = 'rgba(120,135,150,0.35)';
    ctx.lineWidth = devicePixelRatio;
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = 'rgba(120,135,150,0.6)';
    ctx.font = `${10 * devicePixelRatio}px sans-serif`;
    ctx.fillText(`${r}m`, cx + 4 * devicePixelRatio, cy - r * scale + 12 * devicePixelRatio);
  }

  // 激光点云 (车体坐标, 车头朝上 → 世界 x 为屏幕 -y)
  ctx.fillStyle = 'rgba(62,207,142,0.8)';
  const dot = Math.max(1.5, devicePixelRatio);
  for (let i = 0; i < scanLocal.length; i += 2) {
    const px = cx + scanLocal[i + 1] * scale;   // 车体 y → 屏幕 x (左为正)
    const py = cy - scanLocal[i] * scale;       // 车体 x (前) → 屏幕 -y
    if (Math.hypot(px - cx, py - cy) > W / 2) continue;
    ctx.fillRect(px - dot / 2, py - dot / 2, dot, dot);
  }

  // 超声波楔形 (车体坐标; 车头朝上)
  for (const s of sonarMounts) {
    const rec = sonarRanges[s.id];
    const max = (rec && rec.max) || 0.55;
    const range = rec && isFinite(rec.range) ? Math.min(rec.range, max) : max;
    const fov = (rec && rec.fov) || 1.0472;
    const px = cx + s.y * scale;
    const py = cy - s.x * scale;
    // 车体 yaw=+x 朝上: 屏幕角度 = -(yaw) - π/2
    const dir = -s.yaw - Math.PI / 2;
    const near = rec && isFinite(rec.range) ? rec.range / max : 1;
    const hue = 120 - 120 * Math.max(0, Math.min(1, 1 - near));  // 远绿近红
    ctx.fillStyle = `hsla(${hue}, 85%, 55%, 0.4)`;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.arc(px, py, Math.max(4, range * scale), dir - fov / 2, dir + fov / 2);
    ctx.closePath();
    ctx.fill();
    // 探头位置
    ctx.fillStyle = '#d8dee6';
    ctx.fillRect(px - 2 * devicePixelRatio, py - 2 * devicePixelRatio, 4 * devicePixelRatio, 4 * devicePixelRatio);
  }

  // 小车轮廓 (车头朝上)
  ctx.strokeStyle = '#4aa3ff';
  ctx.lineWidth = 1.5 * devicePixelRatio;
  const carL = 0.6 * scale, carW = 0.45 * scale;
  ctx.strokeRect(cx - carW, cy - carL, carW * 2, carL * 2);
  ctx.fillStyle = '#4aa3ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - carL - 6 * devicePixelRatio);
  ctx.lineTo(cx - 5 * devicePixelRatio, cy - carL + 2 * devicePixelRatio);
  ctx.lineTo(cx + 5 * devicePixelRatio, cy - carL + 2 * devicePixelRatio);
  ctx.closePath();
  ctx.fill();
}

// ---------------------------------------------------------------- 2D map panel
const map2dCanvas = $('map2d');
const map2dCtx = map2dCanvas.getContext('2d');

function sizeMap2d() {
  const rect = map2dCanvas.getBoundingClientRect();
  map2dCanvas.width = Math.round(rect.width * devicePixelRatio);
  map2dCanvas.height = Math.round(rect.width * devicePixelRatio);
}

function worldToMap2d(x, y) {
  const m = map2d.meta;
  const W = map2dCanvas.width, H = map2dCanvas.height;
  return [
    (x - m.origin_x) / (m.width * m.resolution) * W,
    (1 - (y - m.origin_y) / (m.height * m.resolution)) * H,
  ];
}

function drawMap2d() {
  const m = map2d.meta;
  if (!m || !map2d.img) return;
  const ctx = map2dCtx;
  const W = map2dCanvas.width, H = map2dCanvas.height;
  ctx.fillStyle = '#0d1014';
  ctx.fillRect(0, 0, W, H);
  ctx.drawImage(map2d.img, 0, 0, W, H);

  // 规划路径
  if (planWorld.length > 3) {
    ctx.strokeStyle = '#f2b04a';
    ctx.lineWidth = Math.max(2, devicePixelRatio);
    ctx.beginPath();
    for (let i = 0; i < planWorld.length; i += 2) {
      const [px, py] = worldToMap2d(planWorld[i], planWorld[i + 1]);
      i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    }
    ctx.stroke();
  }

  // 激光点
  ctx.fillStyle = 'rgba(62,207,142,0.75)';
  const dot = Math.max(1.5, devicePixelRatio);
  for (let i = 0; i < scanWorld.length; i += 2) {
    const [px, py] = worldToMap2d(scanWorld[i], scanWorld[i + 1]);
    ctx.fillRect(px - dot / 2, py - dot / 2, dot, dot);
  }

  // 目标标记 (带朝向的箭头)
  if (goalPose.active) {
    const [gx, gy] = worldToMap2d(goalPose.x, goalPose.y);
    const s = 10 * devicePixelRatio;
    ctx.save();
    ctx.translate(gx, gy);
    ctx.rotate(-goalPose.yaw);
    ctx.fillStyle = '#ef6b73';
    ctx.beginPath();
    ctx.moveTo(s * 1.3, 0);
    ctx.lineTo(-s, -s * 0.8);
    ctx.lineTo(-s * 0.5, 0);
    ctx.lineTo(-s, s * 0.8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // 拖拽方向预览线 (设置目标朝向)
  if (map2dDown && map2dDrag) {
    const [ax, ay] = map2dToWorldEvt(map2dDown);
    const [bx, by] = map2dToWorldEvt(map2dDrag);
    const [px, py] = worldToMap2d(ax, ay);
    const [qx, qy] = worldToMap2d(bx, by);
    ctx.strokeStyle = 'rgba(239,107,115,0.85)';
    ctx.lineWidth = 2 * devicePixelRatio;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(qx, qy);
    ctx.stroke();
  }

  // 小车 (箭头)
  if (state.pose) {
    const [rx, ry] = worldToMap2d(state.pose.x, state.pose.y);
    const s = 11 * devicePixelRatio;
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-state.pose.yaw);  // 画布 y 向下, 取反偏航
    ctx.fillStyle = '#4aa3ff';
    ctx.beginPath();
    ctx.moveTo(s, 0);
    ctx.lineTo(-s * 0.7, -s * 0.62);
    ctx.lineTo(-s * 0.35, 0);
    ctx.lineTo(-s * 0.7, s * 0.62);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

// 单击 = 目标点(沿用当前朝向); 按住拖拽 = 目标点 + 拖拽方向即目标朝向
let map2dDown = null, map2dDrag = null;

function map2dToWorldEvt(e) {
  const m = map2d.meta;
  const rect = map2dCanvas.getBoundingClientRect();
  const nx = (e.clientX - rect.left) / rect.width;
  const ny = (e.clientY - rect.top) / rect.height;
  return [
    m.origin_x + nx * (m.width * m.resolution),
    m.origin_y + (1 - ny) * (m.height * m.resolution),
  ];
}

map2dCanvas.addEventListener('mousedown', (e) => {
  if (e.button !== 0 || !map2d.meta) return;
  map2dDown = { x: e.clientX, y: e.clientY };
  map2dDrag = null;
});
map2dCanvas.addEventListener('mousemove', (e) => {
  if (map2dDown) map2dDrag = { x: e.clientX, y: e.clientY };
});
map2dCanvas.addEventListener('mouseup', (e) => {
  if (!map2dDown || !map2d.meta) return;
  const dx = e.clientX - map2dDown.x, dy = e.clientY - map2dDown.y;
  const [x, y] = map2dToWorldEvt({ clientX: map2dDown.x, clientY: map2dDown.y });
  const yaw = Math.hypot(dx, dy) > 6
    ? Math.atan2(-dy, dx)            // 画布 y 向下 → 世界 y 向上
    : (state.pose ? state.pose.yaw : 0);
  sendGoal(x, y, yaw);
  map2dDown = null; map2dDrag = null;
});
map2dCanvas.addEventListener('mouseleave', () => { map2dDown = null; map2dDrag = null; });

// ---------------------------------------------------------------- camera input
let dragging = 0, lastX = 0, lastY = 0, moved = 0;
renderer.domElement.addEventListener('pointerdown', (e) => {
  dragging = e.button === 0 ? 1 : 2;
  lastX = e.clientX; lastY = e.clientY; moved = 0;
  renderer.domElement.setPointerCapture(e.pointerId);
});
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!dragging) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY;
  moved += Math.abs(dx) + Math.abs(dy);
  if (dragging === 1) {
    camAz -= dx * 0.005;
    camEl += dy * 0.005;
    applyCamera();
  } else {
    const panScale = camDist * 0.0016;
    const fwd = new THREE.Vector3().subVectors(camTarget, camera.position); fwd.z = 0; fwd.normalize();
    const right = new THREE.Vector3(-fwd.y, fwd.x, 0);
    camTarget.addScaledVector(right, -dx * panScale).addScaledVector(fwd, dy * panScale);
    applyCamera();
  }
});
renderer.domElement.addEventListener('pointerup', (e) => {
  const wasDrag = moved > 6;
  const btn = dragging;
  dragging = 0;
  if (e.button === 0 && !wasDrag) sendGoalAt(e);
});
renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());
renderer.domElement.addEventListener('wheel', (e) => {
  e.preventDefault();
  camDist = Math.max(3, Math.min(120, camDist * (e.deltaY > 0 ? 1.12 : 0.9)));
  applyCamera();
}, { passive: false });

const raycaster = new THREE.Raycaster();
const goalPose = { x: 0, y: 0, yaw: 0, active: false };

function sendGoal(x, y, yaw = null) {
  if (yaw == null) yaw = state.pose ? state.pose.yaw : 0;
  goalMarker.position.set(x, y, 0.45);
  goalMarker.visible = true;
  goalPose.x = x; goalPose.y = y; goalPose.yaw = yaw; goalPose.active = true;
  fetch('/api/nav/goal', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ x, y, yaw }),
  }).then((r) => r.json()).then((j) => toast(j.message || (j.ok ? '目标已发送' : '失败')))
    .catch(() => toast('目标发送失败'));
}

function sendGoalAt(e) {
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(ndc, camera);
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  const hit = new THREE.Vector3();
  if (!raycaster.ray.intersectPlane(plane, hit)) return;
  sendGoal(hit.x, hit.y);
}

// ---------------------------------------------------------------- rosbridge
let ros = null, cmdVel = null;
function connectRos() {
  const url = `ws://${location.hostname}:${window.ROSBridgePort || 9090}`;
  ros = new ROSLIB.Ros({ url });
  ros.on('connection', () => {
    $('b-bridge').textContent = 'rosbridge 已连接';
    $('b-bridge').className = 'badge ok';
    subscribeTopics();
  });
  ros.on('error', () => {
    $('b-bridge').textContent = 'rosbridge 连接错误';
    $('b-bridge').className = 'badge err';
  });
  ros.on('close', () => {
    $('b-bridge').textContent = 'rosbridge 未连接';
    $('b-bridge').className = 'badge warn';
    setTimeout(connectRos, 3000);
  });
}

function subscribeTopics() {
  const poseTopic = new ROSLIB.Topic({ ros, name: '/amcl_pose', messageType: 'geometry_msgs/PoseWithCovarianceStamped' });
  poseTopic.subscribe((msg) => {
    const p = msg.pose.pose;
    const yaw = Math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
      1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2));
    setRobotPose(p.position.x, p.position.y, yaw);
    $('sb-pose').textContent = `(${p.position.x.toFixed(2)}, ${p.position.y.toFixed(2)}) ${((yaw * 180 / Math.PI).toFixed(0))}°`;
  });

  const scanTopic = new ROSLIB.Topic({ ros, name: '/scan', messageType: 'sensor_msgs/LaserScan' });
  scanTopic.subscribe(setScan);

  const planTopic = new ROSLIB.Topic({ ros, name: '/plan', messageType: 'nav_msgs/Path' });
  planTopic.subscribe(setPlan);

  for (const id of ['ultrasonic_1', 'ultrasonic_2', 'ultrasonic_3', 'ultrasonic_4']) {
    const topic = new ROSLIB.Topic({ ros, name: `/${id}`, messageType: 'sensor_msgs/Range' });
    topic.subscribe((msg) => {
      sonarRanges[id] = { range: msg.range, max: msg.max_range, fov: msg.field_of_view };
    });
  }

  cmdVel = new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
}

// ---------------------------------------------------------------- teleop
const keys = new Set();
addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
  keys.add(e.code);
  if (e.code === 'Space') { publishCmd(0, 0); e.preventDefault(); }
});
addEventListener('keyup', (e) => keys.delete(e.code));

function publishCmd(v, w) {
  if (!cmdVel) return;
  cmdVel.publish(new ROSLIB.Message({
    linear: { x: v, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: w },
  }));
}
setInterval(() => {
  if (!cmdVel) return;
  let v = 0, w = 0;
  if (keys.has('KeyW') || keys.has('ArrowUp')) v += 0.5;
  if (keys.has('KeyS') || keys.has('ArrowDown')) v -= 0.3;
  if (keys.has('KeyA') || keys.has('ArrowLeft')) w += 0.5;
  if (keys.has('KeyD') || keys.has('ArrowRight')) w -= 0.5;
  if (v || w) publishCmd(v, w);
}, 90);

// ---------------------------------------------------------------- UI wiring
function toast(text) {
  const el = $('toast');
  el.textContent = text;
  el.style.opacity = 1;
  clearTimeout(el._t);
  el._t = setTimeout(() => { el.style.opacity = 0; }, 3200);
}

async function refreshScenes() {
  const res = await fetch('/api/scenes');
  const { scenes } = await res.json();
  const sel = $('scene');
  sel.innerHTML = '';
  const groups = { builtin: '内置场景', test: '测试场景', custom: '自定义地图' };
  for (const kind of ['builtin', 'test', 'custom']) {
    const items = scenes.filter((s) => s.kind === kind);
    if (!items.length) continue;
    const og = document.createElement('optgroup');
    og.label = groups[kind];
    for (const s of items) {
      const opt = document.createElement('option');
      opt.value = s.id;
      opt.textContent = s.name;
      opt.dataset.world = s.world;
      opt.dataset.spawn = JSON.stringify(s.spawn || [0, 0, 0]);
      og.appendChild(opt);
    }
    sel.appendChild(og);
  }
  return scenes;
}

async function refreshServerMaps() {
  try {
    const res = await fetch('/api/maps/available');
    const { maps } = await res.json();
    const sel = $('server-map');
    sel.innerHTML = '';
    const groups = {};
    for (const m of maps) {
      (groups[m.kind] = groups[m.kind] || []).push(m);
    }
    for (const [kind, items] of Object.entries(groups)) {
      const og = document.createElement('optgroup');
      og.label = kind;
      for (const m of items) {
        const opt = document.createElement('option');
        opt.value = m.yaml;
        opt.textContent = m.name;
        og.appendChild(opt);
      }
      sel.appendChild(og);
    }
  } catch (_) { /* server restarting */ }
}

function applySpawnPose() {
  // 小车图标初始位置 = 仿真出生点 (实时定位到达后自动覆盖)
  const opt = $('scene').selectedOptions[0];
  if (!opt || !opt.dataset.spawn) return;
  const [x, y, yaw] = JSON.parse(opt.dataset.spawn);
  trailCount = 0;
  trailGeo.setDrawRange(0, 0);
  setRobotPose(x, y, yaw);
}

async function reloadMap() {
  try {
    await loadMap($('scene').value);
    applySpawnPose();
    toast('地图已加载');
  } catch (err) {
    toast(`地图加载失败: ${err.message}`);
  }
}

$('scene').addEventListener('change', reloadMap);
$('btn-start').addEventListener('click', async () => {
  const opt = $('scene').selectedOptions[0];
  if (!opt) return toast('请先选择场景');
  reloadMap();  // 确保三维视图显示所选场景的地图
  const r = await fetch('/api/sim/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ world: opt.dataset.world, controller: 'mppi' }),
  });
  const j = await r.json();
  toast(j.message || '已请求启动');
});
$('btn-use-map').addEventListener('click', async () => {
  const yaml = $('server-map').value;
  if (!yaml) return toast('请先选择地图');
  toast('正在生成三维场景…');
  const r = await fetch('/api/maps/use', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ yaml }),
  });
  const j = await r.json();
  toast(j.message || (j.ok ? '完成' : '失败'));
  if (j.ok) {
    await refreshScenes();
    const sel = $('scene');
    const custom = sel.querySelector('optgroup[label="自定义地图"] option:last-child');
    if (custom) {
      sel.value = custom.value;
      reloadMap();
    }
  }
});
$('btn-stop').addEventListener('click', async () => {
  const r = await fetch('/api/sim/stop', { method: 'POST' });
  toast((await r.json()).message || '已停止');
});
$('btn-cancel').addEventListener('click', async () => {
  const r = await fetch('/api/nav/cancel', { method: 'POST' });
  toast((await r.json()).message || '取消中');
  goalMarker.visible = false;
});
$('btn-upload').addEventListener('click', async () => {
  const yamlFile = $('map-yaml').files[0];
  const imgFile = $('map-image').files[0];
  if (!yamlFile || !imgFile) return toast('请选择 YAML 和地图图像');
  const [yamlText, imgB64] = await Promise.all([
    yamlFile.text(),
    new Promise((res) => {
      const fr = new FileReader();
      fr.onload = () => res(fr.result.split(',')[1]);
      fr.readAsDataURL(imgFile);
    }),
  ]);
  toast('正在生成三维场景…');
  const r = await fetch('/api/maps/upload', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name: $('map-name').value || yamlFile.name.replace(/\.(yaml|yml)$/i, ''),
      yaml: yamlText,
      image_b64: imgB64,
    }),
  });
  const j = await r.json();
  toast(j.message || (j.ok ? '完成' : '失败'));
  if (j.ok) {
    await refreshScenes();
    $('scene').value = $('scene').querySelector('optgroup[label="自定义地图"] option:last-child')?.value || $('scene').value;
    reloadMap();
  }
});

for (const id of ['ly-scan', 'ly-plan', 'ly-trail', 'ly-walls']) {
  $(id).addEventListener('change', () => {
    scanPts.visible = $('ly-scan').checked;
    planLine.visible = $('ly-plan').checked;
    trail.visible = $('ly-trail').checked;
    mapGroup.visible = $('ly-walls').checked;
  });
}

// ---------------------------------------------------------------- status poll
async function pollStatus() {
  try {
    const s = await (await fetch('/api/sim/status')).json();
    const b = $('b-sim');
    if (s.sim_running) {
      b.textContent = '仿真运行中';
      b.className = 'badge ok';
    } else {
      b.textContent = '仿真未运行';
      b.className = 'badge warn';
    }
    $('b-clock').textContent = `仿真时钟 ${s.clock.sec}s`;
    const nav = s.nav || {};
    const el = $('navstate');
    const label = {
      idle: '空闲', pending: '发送中', active: '导航中', canceling: '取消中',
      succeeded: '✅ 已到达', failed: '❌ 失败', aborted: '❌ 中止',
      canceled: '已取消', rejected: '目标被拒绝',
    }[nav.state] || nav.state;
    el.textContent = `${label}${nav.detail ? ' · ' + nav.detail : ''}`;
    el.className = nav.state === 'succeeded' ? 'ok' : (nav.state || '').match(/fail|abort|reject/) ? 'err' : '';
    $('sb-nav').textContent = label;
    $('sb-dist').textContent = nav.distance != null ? `${nav.distance} m` : '--';
    window.ROSBridgePort = s.rosbridge_port;
  } catch (_) { /* server restarting */ }
}

// ---------------------------------------------------------------- boot
async function boot() {
  await refreshScenes();
  await refreshServerMaps();
  await reloadMap();
  try {
    await loadRobot();
  } catch (e) {
    toast(`车模加载失败: ${e.message}`);
  }
  connectRos();
  pollStatus();
  setInterval(pollStatus, 1500);
  (function loop() {
    requestAnimationFrame(loop);
    renderer.render(scene, camera);
    drawMap2d();
    drawPano();
  })();
  toast('就绪 — 选择场景后点击「启动仿真」');
}

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  sizeMap2d();
  sizePano();
}
addEventListener('resize', resize);
resize();
boot();
