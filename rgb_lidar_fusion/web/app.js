// Live colored map viewer. Talks to web_ui_node.py (see its docstring for the API).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);

const state = {
  mode: 'live',            // 'live' | 'saved'
  savedName: null,
  status: null,
  etags: {},
  touched: false,          // stop auto-fitting once the user moves the view
  fullColor: 'height',
  fullData: null,          // last full-lidar cloud, kept to recolor on demand
  busy: false,
};

// ---- scene ----------------------------------------------------------------
const host = $('view');
// ?shot keeps the drawn frame readable, so page screenshots (headless or
// otherwise) capture the 3D view instead of a cleared canvas.
const renderer = new THREE.WebGLRenderer({
  antialias: true, preserveDrawingBuffer: new URLSearchParams(location.search).has('shot') });
renderer.localClippingEnabled = true;          // height clip (view only)
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
host.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 2000);
camera.up.set(0, 0, 1);                       // lidar frame: Z up
camera.position.set(-6, -6, 5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.addEventListener('start', () => { state.touched = true; });

const grid = new THREE.GridHelper(20, 20, 0x2b3a4d, 0x1a2330);
grid.rotation.x = Math.PI / 2;                // XY plane
scene.add(grid, new THREE.AxesHelper(0.5));

// height clip: z is height above the floor in the map frame. Clipping planes
// are applied on the GPU, so changing them costs nothing and data is untouched.
const clipTop = new THREE.Plane(new THREE.Vector3(0, 0, -1), 2.2);      // keep z <= 2.2
const clipBottom = new THREE.Plane(new THREE.Vector3(0, 0, 1), -0.05);  // keep z >= 0.05
const clipPlanes = [];
const material = (size) => new THREE.PointsMaterial({
  size, sizeAttenuation: false, vertexColors: true, clippingPlanes: clipPlanes });
const fullPts = new THREE.Points(new THREE.BufferGeometry(), material(2));
const coloredPts = new THREE.Points(new THREE.BufferGeometry(), material(3));
coloredPts.renderOrder = 1;
scene.add(fullPts, coloredPts);

// path walked (orange), start (green), the lidar now (red, with heading)
const pathLine = new THREE.Line(new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0xff9f1a }));
const marker = (color) => new THREE.Mesh(new THREE.SphereGeometry(0.08, 16, 12),
  new THREE.MeshBasicMaterial({ color }));
const startMark = marker(0x3fb950);
const nowMark = marker(0xff4d4f);
const heading = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.5, 0xff4d4f, 0.15, 0.1);
nowMark.add(heading);
startMark.visible = nowMark.visible = false;
const pathGroup = new THREE.Group();
pathGroup.add(pathLine, startMark, nowMark);
scene.add(pathGroup);

function setPath(path, pose) {
  const arr = new Float32Array(path.length * 3);
  path.forEach(([x, y], i) => { arr[i * 3] = x; arr[i * 3 + 1] = y; arr[i * 3 + 2] = 0.01; });
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(arr, 3));
  pathLine.geometry.dispose();
  pathLine.geometry = g;
  startMark.visible = path.length > 0;
  if (path.length) startMark.position.set(path[0][0], path[0][1], 0.01);
  nowMark.visible = !!pose;
  if (pose) {
    nowMark.position.set(pose.x, pose.y, pose.z);
    heading.setDirection(new THREE.Vector3(Math.cos(pose.yaw), Math.sin(pose.yaw), 0));
  }
}

function pathLength(path) {
  let d = 0;
  for (let i = 1; i < path.length; i++) d += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
  return d;
}

function resize() {
  const w = host.clientWidth || 1, h = host.clientHeight || 1;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(host);
resize();
(function loop() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
})();

// ---- clouds ---------------------------------------------------------------
function decode(buf, kind) {
  const n = new DataView(buf).getUint32(0, true);
  const xyz = new Float32Array(buf, 4, n * 3);
  if (kind === 'rgb') return { n, xyz, rgb: new Uint8Array(buf, 4 + 12 * n, n * 3) };
  return { n, xyz, inten: new Float32Array(buf, 4 + 12 * n, n) };
}

function setGeometry(points, xyz, colorAttr) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
  g.setAttribute('color', colorAttr);
  g.computeBoundingSphere();
  points.geometry.dispose();
  points.geometry = g;
}

function setColored(data) {
  setGeometry(coloredPts, data.xyz, new THREE.Uint8BufferAttribute(data.rgb, 3, true));
}

// value -> blue..white..red ramp, clipped at the 2nd/98th percentile
function ramp(values) {
  const n = values.length;
  const out = new Float32Array(n * 3);
  if (!n) return out;
  const sorted = Float32Array.from(values).sort();
  const lo = sorted[Math.floor(n * 0.02)], hi = sorted[Math.floor(n * 0.98)];
  const span = hi > lo ? hi - lo : 1;
  const c = new THREE.Color();
  for (let i = 0; i < n; i++) {
    const t = Math.min(1, Math.max(0, (values[i] - lo) / span));
    c.setHSL(0.66 * (1 - t), 0.75, 0.5 + 0.12 * Math.sin(Math.PI * t));
    out[i * 3] = c.r; out[i * 3 + 1] = c.g; out[i * 3 + 2] = c.b;
  }
  return out;
}

function setFull(data) {
  state.fullData = data;
  let colors;
  if (state.fullColor === 'gray') {
    colors = new Float32Array(data.n * 3).fill(0.55);
  } else if (state.fullColor === 'intensity') {
    colors = ramp(data.inten);
  } else {
    const z = new Float32Array(data.n);
    for (let i = 0; i < data.n; i++) z[i] = data.xyz[i * 3 + 2];
    colors = ramp(z);
  }
  setGeometry(fullPts, data.xyz, new THREE.BufferAttribute(colors, 3));
}

// Robust bounds: the 2nd..98th percentile on each axis of a point sample, so a
// few stray far returns do not shrink the room to a speck.
function robustBox() {
  const xs = [], ys = [], zs = [];
  for (const p of [fullPts, coloredPts]) {
    const a = p.visible && p.geometry.attributes.position;
    if (!a || !a.count) continue;
    const step = Math.max(1, Math.floor(a.count / 20000));
    for (let i = 0; i < a.count; i += step) { xs.push(a.getX(i)); ys.push(a.getY(i)); zs.push(a.getZ(i)); }
  }
  if (!xs.length) return null;
  const q = (v, f) => v[Math.min(v.length - 1, Math.floor(f * v.length))];
  const lim = (v) => { v.sort((a, b) => a - b); return [q(v, 0.02), q(v, 0.98)]; };
  const [x0, x1] = lim(xs), [y0, y1] = lim(ys), [z0, z1] = lim(zs);
  return new THREE.Box3(new THREE.Vector3(x0, y0, z0), new THREE.Vector3(x1, y1, z1));
}

// Link options for a fixed view (shareable, and for rendering images):
//   ?map=<name>&embed=1&az=<deg>&el=<deg>&zoom=<x>&clip=<m>
// embed hides the panels; az/el orbit the camera around the map's centre
// (camera direction from the centre: az in the floor plane from +X, el above
// the floor); zoom > 1 is closer; clip sets "hide above" (m), clip=off off.
const params = new URLSearchParams(location.search);
const viewAz = params.has('az') ? Number(params.get('az')) : null;
const viewEl = params.has('el') ? Number(params.get('el')) : null;
const viewZoom = params.has('zoom') ? Number(params.get('zoom')) : 1;
if (params.get('embed') === '1') document.body.classList.add('embed');

function fitView(force = false) {
  if (state.touched && !force) return;
  const box = robustBox();
  if (!box) return;
  const center = box.getCenter(new THREE.Vector3());
  const r = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 1);
  controls.target.copy(center);
  if (viewAz !== null || viewEl !== null) {
    const az = THREE.MathUtils.degToRad(viewAz ?? 225), el = THREE.MathUtils.degToRad(viewEl ?? 35);
    const d = (r * 1.75) / (viewZoom || 1);
    camera.position.set(center.x + d * Math.cos(el) * Math.cos(az),
                        center.y + d * Math.cos(el) * Math.sin(az), center.z + d * Math.sin(el));
  } else {
    camera.position.set(center.x - r * 1.1, center.y - r * 1.1, center.z + r * 0.9);
  }
  if (force) state.touched = false;
}

async function fetchBin(url, key) {
  const headers = state.etags[key] ? { 'If-None-Match': state.etags[key] } : {};
  const res = await fetch(url, { headers });
  if (res.status === 304) return null;
  if (!res.ok) throw new Error(`${url}: ${res.status}`);
  state.etags[key] = res.headers.get('ETag');
  return res.arrayBuffer();
}

function clearClouds() {
  for (const p of [fullPts, coloredPts]) { p.geometry.dispose(); p.geometry = new THREE.BufferGeometry(); }
  state.fullData = null;
  state.etags = {};
}

// ---- live / saved ---------------------------------------------------------
let liveTimer = null;

async function pollLive() {
  if (state.mode !== 'live') return;
  const rec = state.status?.recording;
  // recording -> the growing maps; idle -> the latest scans as a preview
  const [c, f] = rec ? ['colored', 'full'] : ['scan', 'lidar'];
  try {
    const [cb, fb] = await Promise.all([
      fetchBin(`/api/live/${c}.bin`, c), fetchBin(`/api/live/${f}.bin`, f)]);
    if (state.mode !== 'live') return;
    if (cb) setColored(decode(cb, 'rgb'));
    if (fb) setFull(decode(fb, 'i'));
    if (cb || fb) fitView();
  } catch { /* keep polling */ }
  updateOverlay();
  liveTimer = setTimeout(pollLive, rec ? 1000 : 400);
}

function goLive() {
  history.replaceState(null, '', location.pathname);
  state.mode = 'live';
  state.savedName = null;
  clearClouds();
  state.touched = false;
  $('btn-live').hidden = true;
  renderBanner();
  renderMaps(lastMaps);
  clearTimeout(liveTimer);
  pollLive();
}

async function openSaved(name) {
  history.replaceState(null, '', `?map=${encodeURIComponent(name)}`);
  state.mode = 'saved';
  state.savedName = name;
  clearTimeout(liveTimer);
  clearClouds();
  setPath([], null);
  state.touched = false;
  $('btn-live').hidden = false;
  renderBanner('Loading…');
  renderMaps(lastMaps);
  try {
    const [cb, fb] = await Promise.all([
      fetch(`/api/maps/${name}/colored.bin`).then((r) => (r.ok ? r.arrayBuffer() : null)),
      fetch(`/api/maps/${name}/full.bin`).then((r) => (r.ok ? r.arrayBuffer() : null))]);
    if (state.savedName !== name) return;
    if (cb) setColored(decode(cb, 'rgb'));
    if (fb) setFull(decode(fb, 'i'));
    fitView(true);
  } catch (e) {
    flash(`Could not load ${name}: ${e.message}`, 'error');
  }
  renderBanner();
  updateOverlay();
}

// ---- status, banner, overlay ---------------------------------------------
const fmtInt = (n) => (n == null ? '–' : Number(n).toLocaleString());
// metres -> "85.2" cm (one decimal only when needed)
const toCm = (m) => Math.round(m * 1000) / 10;
const fmtTime = (s) => {
  if (!s) return '–';
  const m = Math.floor(s / 60), r = Math.floor(s % 60);
  return `${m}:${String(r).padStart(2, '0')}`;
};

async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    state.status = await res.json();
  } catch {
    state.status = { connected: false };
  }
  renderStatus();
  setTimeout(pollStatus, 500);
}

function renderStatus() {
  const s = state.status || {};
  const pill = $('pill');
  if (!s.connected) {
    pill.dataset.state = 'offline';
    $('pill-text').textContent = 'Colorizer offline';
  } else if (s.recording) {
    pill.dataset.state = 'recording';
    $('pill-text').textContent = `Recording · ${s.map_name}`;
  } else {
    pill.dataset.state = 'idle';
    $('pill-text').textContent = s.camera_ok ? 'Ready' : 'Waiting for camera';
  }
  $('st-colored').textContent = s.recording ? fmtInt(s.colored_points) : '–';
  $('st-full').textContent = s.recording ? fmtInt(s.full_points) : '–';
  $('st-walk').textContent = s.recording ? `${pathLength(s.path || []).toFixed(1)} m` : '–';
  // live lidar height above the floor (map z), with the range seen this recording
  if (s.recording && s.pose) {
    const cm = Math.round(s.pose.z * 100);
    if (state.heightRec !== s.map_name) { state.heightRec = s.map_name; state.hMin = cm; state.hMax = cm; }
    state.hMin = Math.min(state.hMin, cm);
    state.hMax = Math.max(state.hMax, cm);
    $('st-height').textContent = state.hMin === state.hMax
      ? `${cm} cm` : `${cm} cm (${state.hMin}–${state.hMax})`;
  } else {
    $('st-height').textContent = '–';
  }
  $('st-time').textContent = s.recording ? fmtTime(s.elapsed_s) : '–';

  // default height = the colorizer's configured default, unless this browser
  // remembers the last one used or the user is typing
  if (s.default_lidar_height) {
    state.defaultCm = toCm(s.default_lidar_height);
    $('height-default').textContent = state.defaultCm;
    if (!heightTouched && !s.recording) $('lidar-height').value = state.defaultCm;
    markHeight();
  }

  // turning too fast: the colorizer skips colour for these scans
  const tooFast = s.recording && s.turn_rate_dps > s.max_turn_rate_dps;
  $('turn-warn').hidden = !tooFast;
  if (tooFast) {
    $('turn-warn').textContent = `Turning too fast: ${Math.round(s.turn_rate_dps)}°/s ` +
      `(limit ${Math.round(s.max_turn_rate_dps)}°/s). Slow down, these scans are not colored.`;
  }
  $('fast-note').textContent = s.recording && s.rec_too_fast > 0
    ? `${fmtInt(s.rec_too_fast)} scans not colored (turning too fast)` : '';
  $('rd-odom').classList.toggle('ok', !!s.odometry_ok);
  $('rd-cam').classList.toggle('ok', !!s.camera_ok);
  $('rd-still').classList.toggle('ok', !!s.still);
  syncRobots(s);
  const ex = s.exports || [];
  $('exports-note').textContent = ex.length
    ? 'On save, also writes ' + ex.map(robotLabel).join(', ') + '.'
    : '';
  if (state.mode === 'live') setPath(s.recording ? (s.path || []) : [], s.pose || null);

  $('rec-idle').hidden = !!s.recording;
  $('rec-active').hidden = !s.recording;
  $('rec-name').textContent = s.map_name || '';
  $('btn-start').disabled = !s.connected || state.busy;
  $('btn-stop').disabled = state.busy;
  $('btn-cancel').disabled = state.busy;
  if (state.mode === 'live') renderBanner();
}

function renderBanner(text) {
  const s = state.status || {};
  let t = text;
  if (!t) {
    if (state.mode === 'saved') t = `Saved map · ${state.savedName}`;
    else if (s.recording) t = `Live map · ${s.map_name}`;
    else t = 'Live preview · latest scan';
  }
  $('banner-text').textContent = t;
}

function updateOverlay() {
  const s = state.status || {};
  const empty = !(coloredPts.geometry.attributes.position?.count || fullPts.geometry.attributes.position?.count);
  let text = '';
  if (state.mode === 'live' && empty) {
    if (!s.connected) text = 'Waiting for the colorizer… is colorize.launch.py running?';
    else text = 'Waiting for sensor data…';
  }
  $('overlay').hidden = !text;
  $('overlay-text').textContent = text;
}

let flashTimer = null;
function flash(text, kind = '') {
  const el = $('rec-msg');
  el.textContent = text;
  el.className = `msg ${kind}`;
  clearTimeout(flashTimer);
  flashTimer = setTimeout(() => { el.textContent = ''; }, 6000);
}

// ---- recording controls --------------------------------------------------
async function post(path, body) {
  state.busy = true;
  renderStatus();
  try {
    const res = await fetch(path, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}) });
    return await res.json();
  } catch (e) {
    return { ok: false, message: e.message };
  } finally {
    state.busy = false;
    renderStatus();
  }
}

$('btn-start').addEventListener('click', async () => {
  const input = $('map-name');
  const name = input.value.trim();
  if (!name) {
    flash('Name the map first.', 'error');
    input.focus();
    return;
  }
  if (!/^[A-Za-z0-9_-]{1,64}$/.test(name)) {
    flash('Use 1–64 letters, digits, _ or -', 'error');
    input.focus();
    return;
  }
  const cm = Math.round(Number($('lidar-height').value) * 10) / 10;
  if (!(cm >= 5 && cm <= 500)) {
    flash('Lidar height must be 5–500 cm', 'error');
    $('lidar-height').focus();
    return;
  }

  // the name must not collide with a saved map, unless the user types OVERWRITE
  await loadMaps();
  let overwrite = false;
  const existing = lastMaps.find((m) => m.name === name);
  if (existing) {                     // the OVERWRITE dialog is the confirmation
    overwrite = await askOverwrite(existing, cm);
    if (!overwrite) { flash('Not started: choose another name.', ''); return; }
  } else if (!await confirmStart(name, cm)) {
    return;
  }
  let r = await post('/api/start', { name, lidar_height: cm / 100, overwrite });
  if (!r.ok && !overwrite && /already exists/.test(r.message || '')) {   // saved meanwhile
    await loadMaps();
    overwrite = await askOverwrite(lastMaps.find((m) => m.name === name) || { name }, cm);
    if (!overwrite) { flash('Not started: choose another name.', ''); return; }
    r = await post('/api/start', { name, lidar_height: cm / 100, overwrite });
  }
  flash(r.message, r.ok ? 'ok' : 'error');
  if (r.ok) {
    input.value = '';
    if (state.mode !== 'live') goLive(); else { clearClouds(); state.touched = false; }
  }
});

$('btn-stop').addEventListener('click', async () => {
  const r = await post('/api/stop');
  flash(r.ok ? 'Map saved.' : r.message, r.ok ? 'ok' : 'error');
  if (r.ok) { await loadMaps(); clearClouds(); }
});

$('btn-cancel').addEventListener('click', async () => {
  if (!confirm('Discard this recording? Nothing will be saved.')) return;
  const r = await post('/api/cancel');
  flash(r.ok ? 'Recording discarded.' : r.message, r.ok ? '' : 'error');
  if (r.ok) clearClouds();
});

// ---- layers ---------------------------------------------------------------
$('show-colored').addEventListener('change', (e) => { coloredPts.visible = e.target.checked; });
$('show-full').addEventListener('change', (e) => { fullPts.visible = e.target.checked; });
$('show-path').addEventListener('change', (e) => { pathGroup.visible = e.target.checked; });
$('full-color').addEventListener('change', (e) => {
  state.fullColor = e.target.value;
  if (state.fullData) setFull(state.fullData);
});
$('point-size').addEventListener('input', (e) => {
  const v = Number(e.target.value);
  $('size-out').textContent = v;
  fullPts.material.size = v;
  coloredPts.material.size = v + 1;
});
$('btn-fit').addEventListener('click', () => fitView(true));

function updateClip() {
  const top = Number($('clip-top').value), bottom = Number($('clip-bottom').value);
  clipTop.constant = top;
  clipBottom.constant = -bottom;
  $('clip-top-out').textContent = top.toFixed(2).replace(/0$/, '');
  $('clip-bottom-out').textContent = bottom.toFixed(2).replace(/0$/, '');
  $('clip-top').disabled = !$('clip-top-on').checked;
  $('clip-bottom').disabled = !$('clip-bottom-on').checked;
  clipPlanes.length = 0;                       // the materials share this array
  if ($('clip-top-on').checked) clipPlanes.push(clipTop);
  if ($('clip-bottom-on').checked) clipPlanes.push(clipBottom);
  for (const p of [fullPts, coloredPts]) p.material.needsUpdate = true;
}
for (const id of ['clip-top', 'clip-bottom', 'clip-top-on', 'clip-bottom-on']) {
  $(id).addEventListener('input', updateClip);
}
if (params.has('clip')) {
  if (params.get('clip') === 'off') $('clip-top-on').checked = false;
  else if (Number(params.get('clip')) > 0) $('clip-top').value = Number(params.get('clip'));
}
updateClip();
$('btn-live').addEventListener('click', goLive);

// ---- confirmations --------------------------------------------------------
function showDialog(dlg, isOk) {
  return new Promise((resolve) => {
    dlg.addEventListener('close', () => resolve(dlg.returnValue === 'ok' && isOk()), { once: true });
    dlg.returnValue = '';
    dlg.showModal();
  });
}

function confirmStart(name, cm) {
  $('sd-name').textContent = name;
  const off = cm !== state.defaultCm;
  $('sd-height').textContent = `${cm} cm` + (off ? '' : ' (default)');
  $('sd-height').classList.toggle('warn-text', off);
  $('sd-height-warn').hidden = !off;
  $('sd-height-warn').textContent = off ? `Not the default (${state.defaultCm} cm). A wrong height ` +
    'offsets the whole map and the robot exports by the difference.' : '';
  const ex = (state.status && state.status.exports) || [];
  $('sd-exports').textContent = ex.length ? ex.map(robotLabel).join(', ') : 'none';
  const p = showDialog($('start-dlg'), () => true);
  $('sd-ok').focus();
  return p;
}

function askOverwrite(map, cm) {
  const dlg = $('overwrite-dlg');
  const input = $('ow-input');
  $('ow-name').textContent = map.name;
  $('ow-height').textContent = `${cm} cm` + (cm === state.defaultCm ? ' (default)'
    : ` (not the default ${state.defaultCm} cm)`);
  const when = map.started ? new Date(map.started).toLocaleString([], {
    year: 'numeric', month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' }) : '';
  $('ow-info').textContent = when ? ` (saved ${when}${map.full_points != null
    ? `, ${fmtInt(map.full_points)} points` : ''})` : '';
  input.value = '';
  $('ow-ok').disabled = true;
  input.oninput = () => { $('ow-ok').disabled = input.value.trim() !== 'OVERWRITE'; };
  const p = showDialog(dlg, () => input.value.trim() === 'OVERWRITE');
  input.focus();
  return p;
}

// ---- robot exports -------------------------------------------------------
state.robots = [];          // the list being edited (heights in cm)
state.robotsDirty = false;
state.robotsServer = '';

function syncRobots(s) {
  const locked = !!s.recording || !s.connected;
  $('robots-lock').textContent = s.recording ? 'locked while recording' : '';
  for (const el of $('robots-card').querySelectorAll('input, button')) {
    el.disabled = locked || (el.id === 'robots-save' || el.id === 'robots-discard'
      ? !state.robotsDirty : false);
  }
  const server = JSON.stringify(s.exports || []);
  if (!state.robotsDirty && server !== state.robotsServer) {
    state.robotsServer = server;
    state.robots = (s.exports || []).map((r) => ({
      name: r.name, cm: Math.round(r.height * 1000) / 10, roll: r.roll || 0, pitch: r.pitch || 0 }));
    renderRobots();
  }
}

function setRobotsDirty(d) {
  state.robotsDirty = d;
  $('robots-save').disabled = !d;
  $('robots-discard').disabled = !d;
}

// one robot = two lines: name + remove; then height, roll, pitch (labelled)
function numField(label, value, min, max, step, onInput, title) {
  const wrap = document.createElement('label');
  wrap.className = 'num';
  if (title) wrap.title = title;
  const span = document.createElement('span');
  span.textContent = label;
  const inp = document.createElement('input');
  inp.type = 'number';
  inp.min = String(min); inp.max = String(max); inp.step = String(step);
  inp.value = value;
  inp.addEventListener('input', () => onInput(Number(inp.value)));
  wrap.append(span, inp);
  return wrap;
}

function renderRobots() {
  const box = $('robots');
  box.replaceChildren();
  if (!state.robots.length) {
    const p = document.createElement('p');
    p.className = 'hint small';
    p.textContent = 'No robots: maps are saved without robot copies.';
    box.append(p);
  }
  state.robots.forEach((r, i) => {
    const card = document.createElement('div');
    card.className = 'robot';
    const top = document.createElement('div');
    top.className = 'robot-top';
    const name = document.createElement('input');
    name.type = 'text';
    name.value = r.name;
    name.maxLength = 32;
    name.placeholder = 'robot name';
    name.setAttribute('aria-label', 'Robot name');
    name.addEventListener('input', () => { r.name = name.value.trim(); setRobotsDirty(true); });
    const x = document.createElement('button');
    x.type = 'button';
    x.className = 'btn ghost small x';
    x.textContent = '✕';
    x.title = `Remove ${r.name || 'this robot'}`;
    x.addEventListener('click', () => { state.robots.splice(i, 1); setRobotsDirty(true); renderRobots(); });
    top.append(name, x);
    const nums = document.createElement('div');
    nums.className = 'robot-nums';
    nums.append(
      numField('Lidar cm', r.cm, 5, 500, 0.1, (v) => { r.cm = v; setRobotsDirty(true); },
        'Lidar height above the floor (centre of the lidar)'),
      numField('Roll °', r.roll, 0, 180, 1, (v) => { r.roll = v; setRobotsDirty(true); },
        'Rotation about the forward axis as the robot\'s driver reports it: 0 upright, ' +
        '180 upside down. A Livox driver set to roll 180 (like the G1\'s) already reports upright: 0.'),
      numField('Pitch °', r.pitch, -90, 90, 1, (v) => { r.pitch = v; setRobotsDirty(true); },
        'Forward tilt of the lidar, nose down positive (-90 to 90)'));
    card.append(top, nums);
    box.append(card);
  });
}

function robotLabel(r) {
  const angles = [r.roll ? `roll ${r.roll}°` : '', r.pitch ? `pitch ${r.pitch}°` : ''].filter(Boolean);
  return `${r.name} (${Math.round(r.height * 1000) / 10} cm${angles.length ? ', ' + angles.join(', ') : ''})`;
}

function robotsMsg(text, kind = '') {
  const el = $('robots-msg');
  el.textContent = text;
  el.className = `msg ${kind}`;
}

$('robot-add').addEventListener('click', () => {
  state.robots.push({ name: '', cm: 50, roll: 0, pitch: 0 });
  setRobotsDirty(true);
  renderRobots();
  const inputs = $('robots').querySelectorAll('input[type="text"]');
  inputs[inputs.length - 1].focus();
});
$('robots-discard').addEventListener('click', () => {
  setRobotsDirty(false);
  state.robotsServer = '';           // reload from the next status
  robotsMsg('');
  if (state.status) syncRobots(state.status);
});
$('robots-save').addEventListener('click', async () => {
  const robots = state.robots.map((r) => ({ name: r.name, height: Math.round(r.cm * 10) / 1000,
                                            roll: r.roll, pitch: r.pitch }));
  const r = await post('/api/robots', { robots });
  robotsMsg(r.message, r.ok ? 'ok' : 'error');
  if (r.ok) {
    setRobotsDirty(false);
    state.robotsServer = '';
  }
});

function confirmAction(title, body, okLabel) {
  $('cf-title').textContent = title;
  $('cf-body').textContent = body;
  $('cf-ok').textContent = okLabel;
  const p = showDialog($('confirm-dlg'), () => true);
  $('cf-ok').focus();
  return p;
}

async function reexportMap(map) {
  const robots = (state.status && state.status.exports) || [];
  const now = robots.map(robotLabel).join(', ');
  const gone = (map.exports || []).map((e) => e.name).filter((n) => !robots.some((r) => r.name === n));
  const body = robots.length
    ? `Rewrite the robot copies of ${map.name} for: ${now}.` : `${map.name} will have no robot copies.`;
  const ok = await confirmAction('Re-export robots?',
    body + (gone.length ? ` Copies for ${gone.join(', ')} are removed.` : '') +
    ' The map itself is not changed.', 'Re-export');
  if (!ok) return;
  flash(`Re-exporting ${map.name}…`, '');
  let r;
  try {
    const res = await fetch(`/api/maps/${encodeURIComponent(map.name)}/reexport`, { method: 'POST' });
    r = await res.json();
  } catch (e) {
    r = { ok: false, message: e.message };
  }
  flash(r.message, r.ok ? 'ok' : 'error');
  if (r.ok) await loadMaps();
}

async function deleteMap(map) {
  const input = $('dl-input');
  $('dl-name').textContent = map.name;
  input.value = '';
  $('dl-ok').disabled = true;
  input.oninput = () => { $('dl-ok').disabled = input.value !== map.name; };
  const p = showDialog($('delete-dlg'), () => input.value === map.name);
  input.focus();
  if (!await p) return;
  let r;
  try {
    const res = await fetch(`/api/maps/${encodeURIComponent(map.name)}/delete`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ confirm: map.name }) });
    r = await res.json();
  } catch (e) {
    r = { ok: false, message: e.message };
  }
  flash(r.message, r.ok ? 'ok' : 'error');
  if (r.ok) {
    openMaps.delete(map.name);
    if (state.savedName === map.name) goLive();
    await loadMaps();
  }
}

// ---- saved maps -----------------------------------------------------------
let lastMaps = [];

async function loadMaps() {
  try {
    lastMaps = await (await fetch('/api/maps')).json();
  } catch { /* keep the last list */ }
  renderMaps(lastMaps);
}

const openMaps = new Set();          // names whose details are expanded

function renderMaps(maps) {
  const ul = $('maps');
  const q = $('map-search').value.trim().toLowerCase();
  const shown = q ? maps.filter((m) => m.name.toLowerCase().includes(q)) : maps;
  $('maps-count').textContent = maps.length
    ? (q ? `(${shown.length} of ${maps.length})` : `(${maps.length})`) : '';
  ul.replaceChildren();
  if (!shown.length) {
    const li = document.createElement('li');
    li.className = 'empty';
    li.textContent = maps.length ? `No map matches "${q}".` : 'No saved maps yet.';
    ul.append(li);
    return;
  }
  for (const m of shown) {
    const viewing = state.savedName === m.name;
    const li = document.createElement('li');
    li.className = 'map-item' + (openMaps.has(m.name) ? ' open' : '') + (viewing ? ' viewing' : '');

    // collapsed row: name and date; click toggles the details
    const head = document.createElement('button');
    head.className = 'map-head';
    head.setAttribute('aria-expanded', String(openMaps.has(m.name)));
    const chev = document.createElement('span');
    chev.className = 'chev';
    chev.textContent = '▸';
    const name = document.createElement('span');
    name.className = 'map-name';
    name.textContent = m.name;
    const when = document.createElement('span');
    when.className = 'when';
    when.textContent = m.started ? new Date(m.started).toLocaleString([], {
      month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' }) : '';
    head.append(chev, name, when);
    head.addEventListener('click', () => {
      if (openMaps.has(m.name)) openMaps.delete(m.name); else openMaps.add(m.name);
      renderMaps(lastMaps);
    });

    // details
    const body = document.createElement('div');
    body.className = 'map-body';
    const meta = document.createElement('div');
    meta.className = 'map-meta';
    const walked = m.path_length_m != null ? ` · ${m.path_length_m} m walked` : '';
    const h = m.lidar_height_m != null ? ` · lidar ${Math.round(m.lidar_height_m * 1000) / 10} cm` : '';
    meta.textContent = `${fmtTime(m.duration_s)}${walked}${h} · ${fmtInt(m.colored_points)} colored · ${fmtInt(m.full_points)} full`;
    const view = document.createElement('button');
    view.className = 'btn small';
    view.textContent = viewing ? 'Viewing' : 'View in 3D';
    view.disabled = viewing;
    view.addEventListener('click', () => openSaved(m.name));
    const del = document.createElement('button');
    del.className = 'btn small ghost danger';
    del.textContent = 'Delete…';
    del.addEventListener('click', () => deleteMap(m));
    const rex = document.createElement('button');
    rex.className = 'btn small ghost';
    rex.textContent = 'Re-export robots';
    rex.title = 'Rewrite this map\'s robot copies from the current robot list';
    rex.addEventListener('click', () => reexportMap(m));
    const actions = document.createElement('div');
    actions.className = 'row';
    actions.append(view, rex, del);

    const files = document.createElement('div');
    files.className = 'map-files';
    // group: the map itself, then one group per robot export ("go2/full.pcd")
    const groups = new Map();
    for (const f of m.files || []) {
      if (f === 'info.json') continue;
      const [grp, file] = f.includes('/') ? f.split('/') : ['map', f];
      if (!groups.has(grp)) groups.set(grp, []);
      groups.get(grp).push([f, file]);
    }
    for (const [grp, list] of groups) {
      const label = document.createElement('span');
      label.className = 'group';
      const ex = (m.exports || []).find((e) => e.name === grp);
      label.textContent = grp === 'map' ? 'Map (floor at z = 0)' : (ex ? robotLabel(ex) : grp);
      files.append(label);
      for (const [path, file] of list) {
        const a = document.createElement('a');
        a.href = `/api/maps/${m.name}/files/${path}`;
        a.textContent = file;
        a.download = `${m.name}_${path.replace('/', '_')}`;
        files.append(a);
      }
    }
    body.append(meta, actions, files);
    li.append(head, body);
    ul.append(li);
  }
}

$('map-search').addEventListener('input', () => renderMaps(lastMaps));
$('btn-refresh').addEventListener('click', loadMaps);

// ---- start ----------------------------------------------------------------

// The height box always starts at the configured default. Editing is free; a
// value that differs from the default is outlined and confirmed once, in the
// Start dialog (a per-edit confirmation fired on every arrow click).
let heightTouched = false;
state.defaultCm = Number($('lidar-height').value);
function markHeight() {
  const v = Number($('lidar-height').value);
  const off = v !== state.defaultCm;
  $('lidar-height').classList.toggle('nondefault', off);
  $('height-reset').hidden = !off;
}
$('lidar-height').addEventListener('input', () => { heightTouched = true; markHeight(); });
$('height-reset').addEventListener('click', () => {
  $('lidar-height').value = state.defaultCm;
  heightTouched = false;
  markHeight();
});
pollStatus();
loadMaps();
setInterval(loadMaps, 15000);
// ?map=<name> opens a saved map directly (a link you can share)
const linked = new URLSearchParams(location.search).get('map');
if (linked && /^[A-Za-z0-9_-]{1,64}$/.test(linked)) openSaved(linked); else pollLive();
