/* rs_follow web console — jie_deamon-style frontend (adapted).
 * Real-time over WebSocket (ws://host:8890), REST fallback /api/status.
 */
'use strict';

const CFG = { wsPort: 8890, forwardRange: 12, backRange: 4, sideRange: 8 };
let ws = null, connected = false;
let state = { scan: [], target: null, target_valid: false,
              cmd: { vx: 0, vy: 0, wz: 0 }, mode: 1, moving: false,
              status: 'INIT', follow_dist: 1.0, max_linear: 0.9, max_angular: 1.0 };

// ---------- WebSocket ----------
function initWS() {
  const url = `ws://${location.hostname}:${CFG.wsPort}`;
  try { ws = new WebSocket(url); } catch (e) { setTimeout(initWS, 1500); return; }
  ws.onopen = () => { connected = true; setConn(true); };
  ws.onmessage = ev => { try { state = JSON.parse(ev.data); } catch (e) {} };
  ws.onclose = () => { connected = false; setConn(false); setTimeout(initWS, 1500); };
  ws.onerror = () => { try { ws.close(); } catch (e) {} };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }
function setConn(on) {
  const el = document.getElementById('connectionStatus');
  if (!el) return;
  el.querySelector('.status-dot').style.background = on ? '#2ecc71' : '#e74c3c';
  el.querySelector('.status-text').textContent = on ? '已连接' : '连接中...';
}

// ---------- tabs ----------
function initTabs() {
  document.querySelectorAll('.nav-item').forEach(item => {
    item.addEventListener('click', () => {
      document.querySelectorAll('.nav-item').forEach(i => i.classList.remove('active'));
      item.classList.add('active');
      const tab = item.dataset.tab;
      document.querySelectorAll('.tab-content').forEach(p => p.style.display = 'none');
      const panel = document.getElementById('panel-' + tab);
      if (panel) { panel.style.display = (tab === 'follow') ? '' : 'flex'; panel.classList.add('active'); }
      if (tab === 'follow') { send({ type: 'switch_mode', mode: 1 }); resize(); }
      if (tab === 'direct') send({ type: 'switch_mode', mode: 0 });
    });
  });
}

// ---------- follow canvas ----------
let cv, ctx, cx, cy, scale;
function resize() {
  cv = document.getElementById('lidarCanvas');
  if (!cv) return;
  const rect = cv.parentElement.getBoundingClientRect();
  const w = Math.max(300, rect.width), h = Math.max(300, rect.height);
  const dpr = window.devicePixelRatio || 1;
  cv.width = w * dpr; cv.height = h * dpr;
  cv.style.width = w + 'px'; cv.style.height = h + 'px';
  ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const spanX = CFG.forwardRange + CFG.backRange, spanY = CFG.sideRange * 2;
  scale = Math.min(w / spanY, h / spanX);
  cx = w / 2; cy = CFG.forwardRange * scale;   // robot at (cx,cy), +x up, +y left
  draw();
}
function W2S(x, y) { return [cx - y * scale, cy - x * scale]; }
function S2W(px, py) { return [(cy - py) / scale, (cx - px) / scale]; }
function draw() {
  if (!ctx) return;
  const w = cv.width, h = cv.height;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = '#1e3a2a'; ctx.lineWidth = 1;
  for (let m = -CFG.backRange; m <= CFG.forwardRange; m++) {
    const p = W2S(m, 0); ctx.beginPath(); ctx.moveTo(0, p[1]); ctx.lineTo(w, p[1]); ctx.stroke();
  }
  for (let m = -CFG.sideRange; m <= CFG.sideRange; m++) {
    const p = W2S(0, m); ctx.beginPath(); ctx.moveTo(p[0], 0); ctx.lineTo(p[0], h); ctx.stroke();
  }
  // follow_dist circle
  ctx.strokeStyle = '#3a6'; ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.arc(cx, cy, state.follow_dist * scale, 0, 2 * Math.PI); ctx.stroke();
  ctx.setLineDash([]);
  // scan
  ctx.fillStyle = '#8a8';
  for (const p of state.scan) { const s = W2S(p[0], p[1]); ctx.fillRect(s[0], s[1], 2, 2); }
  // robot
  ctx.fillStyle = '#39f'; ctx.beginPath();
  ctx.moveTo(cx, cy - 12); ctx.lineTo(cx - 8, cy + 8); ctx.lineTo(cx + 8, cy + 8);
  ctx.closePath(); ctx.fill();
  // target
  if (state.target && state.target_valid) {
    const s = W2S(state.target[0], state.target[1]);
    ctx.fillStyle = state.mode === 1 ? '#0d0' : '#ff0';
    ctx.beginPath(); ctx.arc(s[0], s[1], 7, 0, 2 * Math.PI); ctx.fill();
  }
  // labels
  ctx.fillStyle = '#ccc'; ctx.font = '13px monospace';
  ctx.fillText('status: ' + state.status + '   mode: ' + (state.mode ? 'FOLLOW' : 'DIRECT')
    + '   ' + (state.moving ? 'MOVING' : 'STOPPED'), 10, 18);
  const tEl = document.getElementById('targetX'), tYEl = document.getElementById('targetY');
  if (tEl) tEl.textContent = state.target ? state.target[0].toFixed(2) + ' m' : '--';
  if (tYEl) tYEl.textContent = state.target ? state.target[1].toFixed(2) + ' m' : '--';
  drawSpeed();
}
function drawSpeed() {
  const sc = document.getElementById('speedCanvas');
  if (!sc) return;
  const c = sc.getContext('2d');
  const w = sc.width, h = sc.height;
  c.clearRect(0, 0, w, h);
  const zero = h / 2;
  c.strokeStyle = '#555'; c.beginPath(); c.moveTo(0, zero); c.lineTo(w, zero); c.stroke();
  const bar = (v, mx, color, up) => {
    const len = Math.max(-1, Math.min(1, v / mx)) * (w / 2 - 4);
    c.strokeStyle = color; c.lineWidth = 8;
    c.beginPath(); c.moveTo(w / 2, zero); c.lineTo(w / 2 + len, zero); c.stroke();
  };
  bar(state.cmd.vx, state.max_linear, '#3498db');
  c.lineWidth = 4; c.strokeStyle = '#e67e22';
  const wz = Math.max(-1, Math.min(1, state.cmd.wz / state.max_angular));
  c.beginPath(); c.arc(w / 2, zero, 22, -Math.PI / 2, -Math.PI / 2 + wz * Math.PI); c.stroke();
  c.fillStyle = '#ccc'; c.font = '11px monospace';
  c.fillText('vx ' + state.cmd.vx.toFixed(2) + '  wz ' + state.cmd.wz.toFixed(2), 6, 12);
}
function initFollow() {
  cv = document.getElementById('lidarCanvas');
  if (!cv) return;
  cv.addEventListener('dblclick', e => {
    const r = cv.getBoundingClientRect();
    const [x, y] = S2W(e.clientX - r.left, e.clientY - r.top);
    send({ type: 'set_target', x: x, y: y });
  });
  const mt = document.getElementById('movingToggle');
  if (mt) mt.addEventListener('click', () => send({ type: 'set_moving', on: !state.moving }));
}

// ---------- direct control ----------
let joy = { x: 0, y: 0 }, yaw = 0, speedHi = false;
function initDirect() {
  const base = document.getElementById('joystickBase');
  const stick = document.getElementById('joystickStick');
  if (base && stick) {
    const R = 55;
    let active = false;
    const pos = e => {
      const r = base.getBoundingClientRect();
      const t = e.touches ? e.touches[0] : e;
      let dx = (t.clientX - (r.left + r.width / 2)) / (r.width / 2);
      let dy = (t.clientY - (r.top + r.height / 2)) / (r.height / 2);
      const n = Math.hypot(dx, dy); if (n > 1) { dx /= n; dy /= n; }
      joy.x = dx; joy.y = dy;
      stick.style.transform = `translate(${dx * R}px, ${dy * R}px)`;
    };
    const start = e => { active = true; pos(e); e.preventDefault(); };
    const move = e => { if (active) { pos(e); e.preventDefault(); } };
    const end = () => { active = false; joy.x = joy.y = 0; stick.style.transform = 'translate(0,0)'; };
    base.addEventListener('mousedown', start); document.addEventListener('mousemove', move);
    document.addEventListener('mouseup', end);
    base.addEventListener('touchstart', start, { passive: false });
    document.addEventListener('touchmove', move, { passive: false });
    document.addEventListener('touchend', end);
  }
  // rotation slider
  const knob = document.getElementById('rotationKnob');
  const track = document.querySelector('.slider-container');
  if (knob && track) {
    let active = false;
    const set = e => {
      const r = track.getBoundingClientRect();
      const t = e.touches ? e.touches[0] : e;
      let n = ((t.clientX - r.left) / r.width) * 2 - 1;
      n = Math.max(-1, Math.min(1, n)); yaw = n;
      knob.style.left = ((n + 1) / 2 * 100) + '%';
    };
    knob.addEventListener('mousedown', e => { active = true; e.preventDefault(); });
    track.addEventListener('mousedown', e => { active = true; set(e); });
    document.addEventListener('mousemove', e => { if (active) set(e); });
    document.addEventListener('mouseup', () => { if (active) { active = false; yaw = 0; knob.style.left = '50%'; } });
  }
  const st = document.getElementById('speedModeToggle');
  if (st) st.addEventListener('change', () => { speedHi = st.checked; });
  document.getElementById('btnLieDown') && document.getElementById('btnLieDown')
    .addEventListener('click', () => send({ type: 'action_cmd', action: 'liedown' }));
  document.getElementById('btnStandUp') && document.getElementById('btnStandUp')
    .addEventListener('click', () => send({ type: 'action_cmd', action: 'standup' }));
  setInterval(() => {
    const k = speedHi ? 1.0 : 0.35;
    const forward = -joy.y * state.max_linear * k;   // stick up -> +x
    const lateral = -joy.x * state.max_linear * k;   // stick right -> -y (robot +y is left)
    const z = yaw * state.max_angular * k;
    send({ type: 'direct_cmd', x: forward, y: lateral, z: z });
  }, 50);
}

document.addEventListener('DOMContentLoaded', () => {
  initTabs(); initFollow(); initDirect(); initWS();
  window.addEventListener('resize', resize);
  setInterval(() => { if (connected) draw(); }, 100);
  setTimeout(resize, 300);
});
