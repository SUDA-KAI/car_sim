'use strict';
const $ = id => document.getElementById(id);
const state = {maps: [], map: null, image: null, imagePixels: null, filter: '全部', points: [], mode: 'pan',
  runtime: null, online: false, events: [], eventId: 0, zoom: 1, offset: [0, 0], pointer: null,
  follow: false, busy: false, firstPoll: true, lastMission: 'idle', importBusy: false, previewPending: null, previewSubmitting: false};
const canvas = $('map-canvas'), ctx = canvas.getContext('2d');
const number = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const pad = value => String(value).padStart(2, '0');
const duration = seconds => `${pad(Math.floor((seconds || 0) / 60))}:${pad(Math.floor((seconds || 0) % 60))}`;
const busyMission = () => ['sending', 'running', 'canceling'].includes(state.runtime?.telemetry?.mission);
const busyPlanning = () => state.previewSubmitting || state.runtime?.telemetry?.preview?.state === 'planning';
const taskBusy = () => busyMission() || busyPlanning();
const pointsKey = points => JSON.stringify((points || []).map(p => [p.x, p.y, p.yaw]));
const simulationActive = () => ['starting', 'running', 'stopping'].includes(state.runtime?.state);

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function toast(message, error = false) {
  const node = element('div', 'toast' + (error ? ' error' : ''), message);
  $('toasts').append(node);
  setTimeout(() => node.remove(), error ? 6500 : 3500);
}

async function api(path, payload) {
  const options = {signal: AbortSignal.timeout(payload ? 120000 : 6000)};
  if (payload !== undefined) Object.assign(options, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload)});
  const response = await fetch(path, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `服务响应异常 (${response.status})`);
  return result;
}

async function action(path, payload = {}) {
  try { const result = await api(path, payload); await poll(); return result; }
  catch (error) { toast(error.message, true); return null; }
}

function renderMaps() {
  const query = $('map-search').value.trim().toLowerCase();
  const maps = state.maps.filter(map => (state.filter === '全部' || map.category === state.filter) &&
    `${map.title} ${map.id} ${map.description}`.toLowerCase().includes(query));
  $('map-count').textContent = pad(state.maps.length);
  $('map-list').replaceChildren();
  maps.forEach(map => {
    const card = element('button', 'map-card' + (map.id === state.map?.id ? ' selected' : ''));
    card.dataset.map = map.id;
    card.setAttribute('aria-pressed', String(map.id === state.map?.id));
    const thumb = element('div', 'map-thumb');
    const img = element('img'); img.src = `/api/maps/${encodeURIComponent(map.id)}/image.png`; img.alt = map.title; img.loading = 'lazy';
    thumb.append(img);
    if (map.id === state.map?.id) thumb.append(element('span', 'check', '✓'));
    const title = element('div', 'map-card-title'); title.append(element('span', '', map.title), element('span', '', '↗'));
    const meta = element('div', 'map-card-meta');
    meta.append(element('span', '', `${map.size[0]} × ${map.size[1]} m`), element('span', 'map-type', map.category));
    card.append(thumb, title, meta); card.addEventListener('click', () => selectMap(map));
    $('map-list').append(card);
  });
  if (!maps.length) $('map-list').append(element('div', 'loading-card', '没有找到匹配的地图'));
}

async function loadMaps() {
  const response = await api('/api/maps'); state.maps = response.maps;
  if (!state.map) {
    const cached = localStorage.getItem('nav-studio-map');
    await selectMap(state.maps.find(m => m.id === state.runtime?.map_id && simulationActive()) || state.maps.find(m => m.id === cached) || state.maps[0], true);
  } else renderMaps();
}

async function selectMap(map, force = false) {
  if (!map || state.map?.id === map.id) return;
  if (!force && simulationActive()) { toast('请先停止当前仿真，再切换地图', true); return; }
  state.map = map; state.image = null; state.imagePixels = null; state.points = []; state.follow = false;
  state.previewPending = null;
  if (simulationActive() && state.runtime?.map_id === map.id && state.runtime.telemetry.targets?.length) state.points = state.runtime.telemetry.targets.map(p => ({...p}));
  localStorage.setItem('nav-studio-map', map.id);
  $('scene-title').textContent = map.title;
  $('scene-description').textContent = map.description;
  $('scene-category').textContent = map.category;
  $('breadcrumb').textContent = map.category === '导入' ? 'CUSTOM WORLD' : 'WORKSPACE';
  $('map-dimensions').textContent = `${map.size[0]} × ${map.size[1]} m`;
  $('map-resolution').textContent = `${map.resolution} m / px`;
  $('map-label').textContent = map.localization === 'ground_truth' ? 'TEST ENVIRONMENT' : 'OCCUPANCY MAP';
  const truthOption = $('localization').querySelector('[value=ground_truth]');
  truthOption.disabled = map.localization !== 'ground_truth';
  if (truthOption.disabled && $('localization').value === 'ground_truth') $('localization').value = 'auto';
  $('route-mode').value = state.points.length ? state.runtime.telemetry.mode : (map.navigation_mode === 'through_poses' ? 'through_poses' : 'waypoints');
  $('load-route').disabled = !map.waypoints?.length;
  renderMaps(); renderWaypoints(); setMode('pan');
  const image = new Image();
  image.onload = () => {
    if (state.map.id !== map.id) return;
    state.image = image;
    const offscreen = document.createElement('canvas'); offscreen.width = image.width; offscreen.height = image.height;
    const offctx = offscreen.getContext('2d'); offctx.drawImage(image, 0, 0); state.imagePixels = offctx.getImageData(0, 0, image.width, image.height).data;
    fitMap();
  };
  image.onerror = () => toast('地图图像加载失败，请刷新地图列表', true);
  image.src = `/api/maps/${encodeURIComponent(map.id)}/image.png`;
  updateUI();
}

function pixelFromWorld(x, y) {
  const [ox, oy, yaw] = state.map.origin, dx = x - ox, dy = y - oy;
  return [(Math.cos(yaw) * dx + Math.sin(yaw) * dy) / state.map.resolution,
    state.map.height - (-Math.sin(yaw) * dx + Math.cos(yaw) * dy) / state.map.resolution];
}

function worldFromScreen(x, y) {
  const [ox, oy, yaw] = state.map.origin;
  const lx = (x - state.offset[0]) / state.zoom * state.map.resolution;
  const ly = (state.map.height - (y - state.offset[1]) / state.zoom) * state.map.resolution;
  return {x: ox + Math.cos(yaw) * lx - Math.sin(yaw) * ly, y: oy + Math.sin(yaw) * lx + Math.cos(yaw) * ly};
}

function screenFromWorld(x, y) {
  const [px, py] = pixelFromWorld(x, y);
  return [state.offset[0] + px * state.zoom, state.offset[1] + py * state.zoom];
}

function isFree(point) {
  if (!state.imagePixels) return false;
  const [px, py] = pixelFromWorld(point.x, point.y);
  const col = Math.floor(px), row = Math.ceil(py) - 1;
  return col >= 0 && row >= 0 && col < state.map.width && row < state.map.height && state.imagePixels[(row * state.map.width + col) * 4] === 39;
}

function fitMap() {
  if (!state.map) return;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  state.zoom = Math.min((w - 108) / state.map.width, (h - 120) / state.map.height);
  state.offset = [(w - state.map.width * state.zoom) / 2, (h - 22 - state.map.height * state.zoom) / 2 + 6];
}

function zoom(factor, x = canvas.clientWidth / 2, y = canvas.clientHeight / 2) {
  if (!state.map) return;
  const next = Math.max(.03, Math.min(50, state.zoom * factor));
  const actual = next / state.zoom;
  state.offset = [x - (x - state.offset[0]) * actual, y - (y - state.offset[1]) * actual]; state.zoom = next;
}

function setMode(mode) {
  state.mode = mode;
  canvas.className = mode === 'pan' ? '' : `${mode}-mode`;
  ['pan', 'goal', 'pose'].forEach(key => $('tool-' + key).classList.toggle('active', key === mode));
  $('canvas-hint').textContent = {pan: '滚轮缩放 · 拖动平移 · 选择「◎」添加目标', goal: '点击添加目标 · 按住拖动设置朝向 · Esc 退出', pose: '点击设置 AMCL 初始位置 · 拖动设置朝向'}[mode];
}

function renderWaypoints() {
  $('waypoint-count').textContent = pad(state.points.length);
  $('waypoints').replaceChildren();
  state.points.forEach((point, index) => {
    const row = element('li');
    const coords = element('span', 'waypoint-coordinates', `X ${number(point.x)}  Y ${number(point.y)}`);
    coords.append(element('small', '', '到达朝向'));
    const yawControl = element('label', 'yaw-control');
    const yawInput = element('input', 'waypoint-yaw'); yawInput.type = 'number'; yawInput.step = '1'; yawInput.value = number(point.yaw * 180 / Math.PI, 1);
    yawInput.setAttribute('aria-label', `目标点 ${index + 1} 到达朝向（度）`); yawInput.disabled = taskBusy();
    yawInput.addEventListener('change', () => { const value = Number(yawInput.value); if (yawInput.value.trim() && Number.isFinite(value)) { point.yaw = Math.atan2(Math.sin(value * Math.PI / 180), Math.cos(value * Math.PI / 180)); updateUI(); } else yawInput.value = number(point.yaw * 180 / Math.PI, 1); });
    yawControl.append(yawInput, document.createTextNode('°'));
    const remove = element('button', 'waypoint-delete', '×'); remove.title = '删除目标点'; remove.setAttribute('aria-label', `删除目标点 ${index + 1}`); remove.disabled = taskBusy();
    remove.addEventListener('click', () => { state.points.splice(index, 1); renderWaypoints(); updateUI(); });
    row.append(element('span', 'waypoint-number', pad(index + 1)), coords, yawControl, remove); $('waypoints').append(row);
  });
  if (!state.points.length) $('waypoints').append(element('li', 'empty-queue', '还没有目标点，去地图上选一个吧'));
}

function drawPolyline(points, color, width, dash = []) {
  if (!points?.length) return;
  ctx.save(); ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.lineJoin = 'round';
  points.forEach((point, index) => { const p = screenFromWorld(point[0], point[1]); index ? ctx.lineTo(...p) : ctx.moveTo(...p); });
  ctx.stroke(); ctx.restore();
}

function drawGoal(point, index, preview = false) {
  const [x, y] = screenFromWorld(point.x, point.y), yaw = point.yaw - state.map.origin[2];
  const completed = pointsKey(state.runtime?.telemetry?.targets) === pointsKey(state.points) ? (state.runtime?.telemetry?.completed || 0) : 0;
  const color = !preview && index < completed && state.runtime?.telemetry?.mission !== 'idle' ? '#a0b9b3' : '#e4b970';
  ctx.save(); ctx.translate(x, y); ctx.strokeStyle = color; ctx.fillStyle = color;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(Math.cos(yaw) * 27, -Math.sin(yaw) * 27); ctx.lineWidth = 1.3; ctx.stroke();
  ctx.save(); ctx.translate(Math.cos(yaw) * 27, -Math.sin(yaw) * 27); ctx.rotate(-yaw); ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-5, -3); ctx.lineTo(-5, 3); ctx.closePath(); ctx.fill(); ctx.restore();
  ctx.beginPath(); ctx.arc(0, 0, preview ? 7 : 10, 0, Math.PI * 2); ctx.fillStyle = '#213938'; ctx.fill(); ctx.lineWidth = 1.2; ctx.stroke();
  ctx.fillStyle = color; ctx.font = '9px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(preview ? '+' : String(index + 1), 0, .5); ctx.restore();
}

function drawCar(pose, live) {
  const [x, y] = screenFromWorld(pose.x, pose.y), yaw = pose.yaw - state.map.origin[2];
  const pixelsPerMetre = state.zoom / state.map.resolution;
  const length = Math.max(16, 1.2 * pixelsPerMetre), width = Math.max(11, .9 * pixelsPerMetre);
  ctx.save(); ctx.translate(x, y);
  if (live) { ctx.beginPath(); ctx.fillStyle = '#62d3ac0d'; ctx.arc(0, 0, Math.max(28, length * .95), 0, Math.PI * 2); ctx.fill(); ctx.strokeStyle = '#66d5b02e'; ctx.lineWidth = 1; ctx.stroke(); }
  ctx.rotate(-yaw); ctx.shadowColor = live ? '#6eddb0' : 'transparent'; ctx.shadowBlur = 10;
  ctx.fillStyle = live ? '#9de6bd' : '#84b9a5'; ctx.strokeStyle = live ? '#d0ffe6' : '#b6d6c3'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.roundRect(-length / 2, -width / 2, length, width, Math.min(3, width / 4)); ctx.fill(); ctx.stroke(); ctx.shadowBlur = 0;
  ctx.fillStyle = '#203e38'; ctx.fillRect(-length * .11, -width * .32, length * .32, width * .64);
  ctx.fillStyle = '#dcffdf'; ctx.beginPath(); ctx.moveTo(length / 2 + 7, 0); ctx.lineTo(length / 2 + 2, -3); ctx.lineTo(length / 2 + 2, 3); ctx.fill(); ctx.restore();
  ctx.fillStyle = live ? '#a4ddc6' : '#7ea593'; ctx.font = '8px monospace'; ctx.textAlign = 'center'; ctx.fillText(live ? 'PATROL / 01' : 'SPAWN', x, y + Math.max(20, width / 2 + 12));
}

let lastFrame = 0;
function draw(time) {
  requestAnimationFrame(draw);
  if (time - lastFrame < 45 || document.hidden) return;
  lastFrame = time;
  const width = canvas.clientWidth, height = canvas.clientHeight, dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) { canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#10232c'; ctx.fillRect(0, 0, width, height);
  const telemetry = state.online && state.runtime?.map_id === state.map?.id ? state.runtime?.telemetry : null;
  if (state.follow && telemetry?.pose_fresh) {
    const [px, py] = pixelFromWorld(telemetry.pose.x, telemetry.pose.y);
    state.offset = [width / 2 - px * state.zoom, height / 2 - py * state.zoom];
  }
  const grid = 23;
  ctx.fillStyle = '#35505a60';
  for (let x = state.offset[0] % grid; x < width; x += grid) for (let y = state.offset[1] % grid; y < height; y += grid) ctx.fillRect(x, y, 1, 1);
  if (!state.map || !state.image) return;
  ctx.save(); ctx.imageSmoothingEnabled = false; ctx.drawImage(state.image, state.offset[0], state.offset[1], state.map.width * state.zoom, state.map.height * state.zoom);
  ctx.strokeStyle = '#668b8433'; ctx.lineWidth = 1; ctx.strokeRect(state.offset[0], state.offset[1], state.map.width * state.zoom, state.map.height * state.zoom); ctx.restore();
  if ($('layer-trail').checked) drawPolyline(telemetry?.trail, '#d2b07080', 1.5, [4, 4]);
  if ($('layer-path').checked && telemetry?.connected) {
    const preview = telemetry.preview;
    const path = busyMission() ? telemetry.path : preview?.state === 'ready' && pointsKey(preview.points) === pointsKey(state.points) ? preview.path : [];
    drawPolyline(path, '#66e0ba17', 8); drawPolyline(path, '#68dfb5', 2.2);
  }
  if ($('layer-scan').checked && telemetry?.connected) {
    ctx.fillStyle = '#91b6da99'; for (const point of telemetry.scan || []) { const [x, y] = screenFromWorld(...point); ctx.fillRect(x - .8, y - .8, 1.6, 1.6); }
  }
  if (state.points.length > 1) drawPolyline(state.points.map(p => [p.x, p.y]), '#d4b77b55', 1, [3, 5]);
  state.points.forEach((point, index) => drawGoal(point, index));
  if (state.pointer && state.mode !== 'pan') drawGoal(state.pointer.point, -1, true);
  const pose = telemetry?.pose_fresh ? telemetry.pose : {x: state.map.spawn[0], y: state.map.spawn[1], yaw: state.map.spawn[2]};
  drawCar(pose, Boolean(telemetry?.pose_fresh));
  const ppm = state.zoom / state.map.resolution;
  const targetMetres = 60 / ppm;
  const power = 10 ** Math.floor(Math.log10(targetMetres));
  const scale = [1, 2, 5, 10].map(v => v * power).find(v => v >= targetMetres) || power;
  $('scale-label').textContent = `${number(scale, scale < 1 ? 1 : 0)} m`;
  ctx.strokeStyle = '#7c9c9e'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(width - 17 - scale * ppm, height - 43); ctx.lineTo(width - 17, height - 43); ctx.stroke();
}

canvas.addEventListener('wheel', event => { event.preventDefault(); const r = canvas.getBoundingClientRect(); state.follow = false; zoom(Math.exp(-event.deltaY * .0012), event.clientX - r.left, event.clientY - r.top); }, {passive: false});
canvas.addEventListener('pointerdown', event => {
  if (!state.map || event.button !== 0) return;
  if (taskBusy() && state.mode !== 'pan') { toast('请先取消当前任务，再修改目标', true); return; }
  const r = canvas.getBoundingClientRect(), x = event.clientX - r.left, y = event.clientY - r.top;
  const point = worldFromScreen(x, y);
  point.yaw = state.points.at(-1)?.yaw ?? (state.runtime?.telemetry?.pose_fresh ? state.runtime.telemetry.pose.yaw : state.map.spawn[2]);
  state.pointer = {start: [x, y], offset: [...state.offset], point}; state.follow = false; $('follow-car').classList.remove('active'); canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener('pointermove', event => {
  if (!state.map) return;
  const r = canvas.getBoundingClientRect(), x = event.clientX - r.left, y = event.clientY - r.top;
  const point = worldFromScreen(x, y);
  $('cursor-position').textContent = `X ${number(point.x)}  Y ${number(point.y)}`;
  if (!state.pointer) return;
  if (state.mode === 'pan') state.offset = [state.pointer.offset[0] + x - state.pointer.start[0], state.pointer.offset[1] + y - state.pointer.start[1]];
  else if (Math.hypot(x - state.pointer.start[0], y - state.pointer.start[1]) > 4) state.pointer.point.yaw = Math.atan2(point.y - state.pointer.point.y, point.x - state.pointer.point.x);
});
canvas.addEventListener('pointerup', async () => {
  const pointer = state.pointer; state.pointer = null;
  if (!pointer || state.mode === 'pan') return;
  if (!isFree(pointer.point)) { toast('请选择地图中已知的空闲区域', true); return; }
  if (state.mode === 'goal') {
    if (state.points.length >= 64) { toast('目标队列最多支持 64 个点', true); return; }
    state.points.push({...pointer.point}); renderWaypoints(); updateUI();
  } else {
    const result = await action('/api/initial-pose', {map_id: state.map.id, ...pointer.point});
    if (result) { toast('已发送 AMCL 初始位姿'); setMode('pan'); }
  }
});
canvas.addEventListener('pointercancel', () => { state.pointer = null; });
window.addEventListener('keydown', event => { if (event.key === 'Escape') { state.pointer = null; setMode('pan'); } });
new ResizeObserver(() => { if (!state.pointer) fitMap(); }).observe($('viewport'));
['pan', 'goal', 'pose'].forEach(mode => $('tool-' + mode).addEventListener('click', () => setMode(mode)));
$('zoom-in').addEventListener('click', () => zoom(1.3)); $('zoom-out').addEventListener('click', () => zoom(1 / 1.3));
$('fit-map').addEventListener('click', () => { state.follow = false; $('follow-car').classList.remove('active'); fitMap(); });
$('follow-car').addEventListener('click', () => { state.follow = !state.follow; $('follow-car').classList.toggle('active', state.follow); if (state.follow && !state.runtime?.telemetry?.pose_fresh) toast('定位数据到达后，将自动跟随车辆'); });

async function requestPreview(runAfter) {
  if (taskBusy() || !state.map || !state.points.length) return;
  const payload = {map_id: state.map.id, points: state.points.map(p => ({...p})), mode: $('route-mode').value};
  state.previewSubmitting = true; updateUI();
  try {
    const result = await api('/api/path/preview', payload);
    state.previewPending = {id: result.preview_id, payload, runAfter};
    state.previewSubmitting = false;
    await poll();
  } catch (error) { state.previewSubmitting = false; state.previewPending = null; toast(error.message, true); }
  updateUI();
}

async function handlePreview() {
  const pending = state.previewPending, preview = state.runtime?.telemetry?.preview;
  if (!pending || !preview || preview.id !== pending.id || preview.state === 'planning') return;
  state.previewPending = null;
  if (preview.state !== 'ready') { if (preview.state !== 'canceled') toast(preview.message || '路径检查未通过', true); return; }
  if (!pending.runAfter) { toast(preview.message); return; }
  if (state.map?.id !== pending.payload.map_id || pointsKey(state.points) !== pointsKey(pending.payload.points) || !simulationActive()) return;
  state.busy = true; updateUI();
  try {
    await api('/api/navigate', pending.payload);
    state.runtime.telemetry.mission = 'sending';
    toast('路径检查通过，导航目标已提交'); setMode('pan');
  } catch (error) { toast(error.message, true); }
  finally { state.busy = false; }
}

function updateUI() {
  const r = state.runtime, t = r?.telemetry || {}, active = simulationActive(), busy = taskBusy();
  const preview = t.preview;
  const showPreview = preview && pointsKey(preview.points) === pointsKey(state.points) && preview.state !== 'idle';
  const previewFailed = showPreview && preview.state === 'failed';
  const sameMission = pointsKey(t.targets) === pointsKey(state.points);
  const ready = state.online && t.ready && r?.map_id === state.map?.id && r?.settings?.nav && active && r?.state !== 'stopping';
  $('connection').classList.toggle('online', state.online);
  $('connection').lastChild.textContent = state.online ? '本机已连接' : '连接已断开';
  $('domain').textContent = r?.domain || '73';
  $('settings').disabled = active || !state.online || state.busy;
  if (active && r?.settings) {
    $('controller').value = r.settings.controller; $('localization').value = r.settings.localization;
    $('max-speed').value = r.settings.max_speed; $('speed-value').textContent = Number(r.settings.max_speed).toFixed(2);
    $('planning-timeout').value = r.settings.planning_timeout || 5;
    $('gui').checked = r.settings.gui; $('nav-enabled').checked = r.settings.nav;
    $('use-l2').checked = r.settings.use_l2; $('use-ultrasonic').checked = r.settings.use_ultrasonic;
  }
  $('start-sim').classList.toggle('hidden', active); $('stop-sim').classList.toggle('hidden', !active);
  $('start-sim').disabled = !state.online || !t.available || !state.map || state.busy;
  $('stop-sim').disabled = r?.state === 'stopping' || state.busy;
  $('stop-sim').textContent = r?.state === 'stopping' ? '正在停止…' : '■  停止仿真';
  $('control-caption').textContent = active ? (r?.state === 'stopping' ? '正在释放当前仿真资源' : '切换地图或配置前，请先停止当前仿真') : '加载选中地图、小车模型与导航服务';
  $('gazebo-state').textContent = r?.state === 'error' ? '启动失败' : r?.state === 'stopping' ? '停止中' : t.connected ? '运行中' : active ? '加载中' : '未启动';
  $('gazebo-dot').className = 'status-dot ' + (t.connected ? 'ok' : r?.state === 'error' ? 'error' : active ? 'wait' : '');
  $('nav-state').textContent = !active ? '等待启动' : !r.settings.nav ? '未启用' : ready ? '已就绪' : '初始化中';
  $('nav-dot').className = 'status-dot ' + (ready ? 'ok' : active && r.settings.nav ? 'wait' : '');
  $('runtime').textContent = `SESSION ${duration(r?.uptime)}`;
  $('view-state').textContent = busyPlanning() ? '路径检查中' : t.pose_fresh ? '● 实时数据' : active ? '等待定位' : '场景预览';
  $('starting-overlay').classList.toggle('hidden', !active || r?.state === 'stopping' || (t.connected && (!r?.settings?.nav || ready)));
  $('metric-speed').replaceChildren(document.createTextNode(number(state.online && t.connected ? t.speed : null)), element('em', '', 'm/s'));
  const pose = state.online && t.pose_fresh ? t.pose : null;
  $('metric-position').replaceChildren(document.createTextNode(pose ? `${number(pose.x)} / ${number(pose.y)}` : '— / —'), element('em', '', 'm'));
  $('metric-yaw').replaceChildren(document.createTextNode(number(pose ? pose.yaw * 180 / Math.PI : null, 1)), element('em', '', '°'));
  $('metric-angular').textContent = `角速度 ${number(state.online && t.connected ? t.angular_speed : null)} rad/s`;
  $('metric-progress').replaceChildren(document.createTextNode(t.total && sameMission ? `${t.completed} / ${t.total}` : '—'), element('em', '', '目标点'));
  $('metric-remaining').textContent = previewFailed ? '当前目标的路径检查未通过' : busyMission() && t.remaining != null ? `剩余 ${number(t.remaining, 1)} m · ${duration(t.elapsed)}` : sameMission && t.mission === 'succeeded' ? '全部目标已到达' : sameMission && t.mission === 'failed' ? '任务失败，请查看日志' : '等待导航任务';
  $('speed-bar').style.width = `${Math.min(100, Math.abs(t.speed || 0) / .8 * 100)}%`;
  $('navigate').disabled = !ready || busy || !state.points.length || state.busy;
  $('preview-path').disabled = !ready || busy || !state.points.length || state.busy;
  $('navigate').textContent = busyPlanning() ? '路径检查中…' : '开始导航 →';
  $('cancel-nav').disabled = !state.online || !busy || t.mission === 'canceling';
  const missionNames = {idle: '待命', sending: '发送中', running: '导航中', canceling: '取消中', canceled: '已取消', succeeded: '已完成', failed: '失败'};
  $('mission-status').textContent = busyPlanning() ? '检查路径' : previewFailed ? '路径未通过' : sameMission || busyMission() ? missionNames[t.mission] || '待命' : '待命'; $('mission-status').classList.toggle('busy', busy);
  $('navigation-note').textContent = !state.online ? '服务连接中断，请检查控制台进程' : ready ? (busyPlanning() ? 'Nav2 正在检查路径，尚未启动车辆' : busyMission() ? 'Nav2 正在执行任务，路径与位置实时更新' : '发送前自动检查路径；可调整目标点到达朝向') : '启动仿真后，等待定位与 Nav2 就绪';
  $('preview-message').textContent = showPreview ? preview.message : '空闲像素不等于可达，先用 Nav2 检查实际路径';
  $('preview-message').classList.toggle('failed', Boolean(showPreview && preview.state === 'failed'));
  $('clear-waypoints').disabled = busy; $('load-route').disabled = busy || !state.map?.waypoints?.length;
  $('route-mode').disabled = busy;
  document.querySelectorAll('.waypoint-delete').forEach(b => { b.disabled = busy; });
  document.querySelectorAll('.waypoint-yaw').forEach(b => { b.disabled = busy; });
  const ranges = $('ranges').querySelectorAll('b');
  ranges.forEach((node, i) => { const distance = state.online && t.connected ? t.ranges?.[i] : null; node.textContent = distance == null ? '—' : `${number(distance, 2)} m`; node.classList.toggle('near', distance != null && distance < .1); });
  if (state.lastMission !== t.mission && ['succeeded', 'failed', 'canceled'].includes(t.mission)) toast({succeeded: '全部导航目标已完成', failed: '导航任务失败，请查看运行日志', canceled: '导航任务已取消'}[t.mission], t.mission === 'failed');
  state.lastMission = t.mission || 'idle';
}

let polling = false;
async function poll() {
  if (polling) return;
  polling = true;
  try {
    const r = await api(`/api/state?after=${state.eventId}`);
    state.runtime = r; state.online = true;
    if (r.events?.length) {
      state.events = state.events.concat(r.events).slice(-1800); state.eventId = r.events.at(-1).id;
      $('latest-event').textContent = r.events.at(-1).message;
      const errors = state.events.filter(e => e.level === 'error').length; $('error-count').textContent = errors; $('error-count').classList.toggle('hidden', errors === 0);
      if (!$('log-panel').classList.contains('hidden')) renderLogs();
    }
    if (state.maps.length && simulationActive() && r.map_id !== state.map?.id) await selectMap(state.maps.find(m => m.id === r.map_id), true);
    if (state.firstPoll) {
      state.firstPoll = false;
      if (r.telemetry.error) toast(r.telemetry.error, true);
    }
    await handlePreview();
  } catch (_) { state.online = false; }
  finally { polling = false; updateUI(); }
}

function renderLogs() {
  const container = $('log-lines'), nearBottom = container.scrollHeight - container.scrollTop - container.clientHeight < 60;
  container.replaceChildren();
  state.events.filter(e => !$('errors-only').checked || ['warning', 'error'].includes(e.level)).forEach(event => {
    const line = element('div', 'log-line ' + event.level);
    line.append(element('span', '', new Date(event.time * 1000).toLocaleTimeString('zh-CN', {hour12: false})), document.createTextNode(event.message)); container.append(line);
  });
  if (nearBottom) container.scrollTop = container.scrollHeight;
}

function showLogs() {
  $('log-panel').classList.remove('hidden'); renderLogs(); $('log-panel').scrollIntoView({behavior: 'smooth', block: 'nearest'});
  document.querySelectorAll('.nav-item').forEach(b => b.classList.toggle('active', b.dataset.tab === 'logs'));
}
$('show-logs').addEventListener('click', showLogs); $('errors-only').addEventListener('change', renderLogs);
$('close-logs').addEventListener('click', () => { $('log-panel').classList.add('hidden'); document.querySelectorAll('.nav-item').forEach(b => b.classList.toggle('active', b.dataset.tab === 'workspace')); });
$('download-logs').addEventListener('click', () => {
  const text = state.events.map(e => `${new Date(e.time * 1000).toISOString()} [${e.level}] ${e.message}`).join('\n');
  const url = URL.createObjectURL(new Blob([text], {type: 'text/plain;charset=utf-8'})), a = element('a'); a.href = url; a.download = `nav-studio-${Date.now()}.log`; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
});
document.querySelectorAll('.nav-item').forEach(button => button.addEventListener('click', () => {
  if (button.dataset.tab === 'logs') { showLogs(); return; }
  document.querySelectorAll('.nav-item').forEach(b => b.classList.toggle('active', b === button));
  document.body.classList.toggle('maps-focus', button.dataset.tab === 'maps');
  if (button.dataset.tab === 'maps') { $('map-search').focus(); $('map-list').scrollIntoView({behavior: 'smooth', block: 'nearest'}); }
  else { $('log-panel').classList.add('hidden'); $('scene-title').scrollIntoView({behavior: 'smooth', block: 'nearest'}); }
}));
$('map-search').addEventListener('input', renderMaps);
$('filters').addEventListener('click', event => { const button = event.target.closest('[data-filter]'); if (!button) return; state.filter = button.dataset.filter; document.querySelectorAll('.filter').forEach(b => b.classList.toggle('active', b === button)); renderMaps(); });
$('refresh-maps').addEventListener('click', () => loadMaps().catch(e => toast(e.message, true)));
$('max-speed').addEventListener('input', () => { $('speed-value').textContent = Number($('max-speed').value).toFixed(2); });
$('start-sim').addEventListener('click', async () => {
  if (state.busy || !state.map) return;
  state.busy = true; updateUI();
  const result = await action('/api/simulation/start', {map_id: state.map.id, controller: $('controller').value, localization: $('localization').value,
    max_speed: Number($('max-speed').value), planning_timeout: Number($('planning-timeout').value), gui: $('gui').checked, nav: $('nav-enabled').checked, use_l2: $('use-l2').checked, use_ultrasonic: $('use-ultrasonic').checked});
  state.busy = false; updateUI(); if (result) toast('仿真启动请求已发送，正在加载场景');
});
$('stop-sim').addEventListener('click', () => { state.previewPending = null; return action('/api/simulation/stop'); });
$('navigate').addEventListener('click', () => requestPreview(true));
$('preview-path').addEventListener('click', () => requestPreview(false));
$('cancel-nav').addEventListener('click', () => action('/api/navigation/cancel'));
$('clear-waypoints').addEventListener('click', () => { if (!taskBusy()) { state.points = []; renderWaypoints(); updateUI(); } });
$('load-route').addEventListener('click', () => {
  if (!state.map || taskBusy()) return;
  state.points = state.map.waypoints.map(p => ({x: Number(p.x), y: Number(p.y), yaw: Number(p.yaw || 0)}));
  $('route-mode').value = state.map.navigation_mode === 'through_poses' ? 'through_poses' : 'waypoints';
  renderWaypoints(); updateUI(); toast(`已载入 ${state.points.length} 个预设目标点`);
});
$('import-open').addEventListener('click', () => { $('import-error').textContent = ''; $('import-dialog').showModal(); });
document.querySelectorAll('.dialog-close').forEach(b => b.addEventListener('click', () => { if (!state.importBusy) $('import-dialog').close(); }));
$('import-dialog').addEventListener('cancel', event => { if (state.importBusy) event.preventDefault(); });
$('import-yaml').addEventListener('change', () => { $('yaml-name').textContent = $('import-yaml').files[0]?.name || '选择 .yaml / .yml'; });
$('import-image').addEventListener('change', () => { $('image-name').textContent = $('import-image').files[0]?.name || '选择 .pgm / .png'; });
$('import-form').addEventListener('submit', async event => {
  event.preventDefault();
  const yaml = $('import-yaml').files[0], image = $('import-image').files[0];
  if (!yaml || !image || state.importBusy) return;
  if (image.size > 12000000 || yaml.size > 64000) { $('import-error').textContent = '地图图片最多 12 MB，配置文件最多 64 KB'; return; }
  state.importBusy = true; $('import-submit').disabled = true; $('import-submit').textContent = '正在构建场景…'; $('import-error').textContent = '';
  try {
    const base64 = await new Promise((resolve, reject) => { const reader = new FileReader(); reader.onload = () => resolve(reader.result.split(',')[1]); reader.onerror = reject; reader.readAsDataURL(image); });
    const result = await api('/api/maps/import', {title: $('import-title').value, yaml: await yaml.text(), image: base64, image_name: image.name, wall_height: Number($('wall-height').value)});
    await loadMaps();
    if (!simulationActive()) await selectMap(state.maps.find(m => m.id === result.map.id));
    $('import-dialog').close(); toast('地图已导入，可在场景库中选择并启动');
  } catch (error) { $('import-error').textContent = error.message; }
  finally { state.importBusy = false; $('import-submit').disabled = false; $('import-submit').textContent = '生成仿真场景 ↗'; }
});

async function boot() {
  await poll();
  try { await loadMaps(); } catch (error) { toast('地图列表读取失败：' + error.message, true); }
  updateUI(); requestAnimationFrame(draw); setInterval(poll, 700);
}
boot();
