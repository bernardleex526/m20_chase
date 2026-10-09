'use strict';

const CFG = { forwardRange: 12, backRange: 4, sideRange: 8 };
const $ = id => document.getElementById(id);
let ws = null, token = '', authenticated = false, controller = false, reconnectTimer;
let connectGeneration = 0;
let state = {}, stateReceived = -Infinity, cloud = null, candidate = null;
let boundConfirmed = false, binding = false, tab = 'follow', dataError = '';
let pendingStart = null, boundObserved = false;
let cv, ctx, cx = 0, cy = 0, scale = 1, width = 0, height = 0;
let joy = { x: 0, y: 0 }, yaw = 0;
const held = new Map();

function send(message) {
  if (!authenticated || !controller || !ws || ws.readyState !== WebSocket.OPEN) return false;
  ws.send(JSON.stringify(message));
  return true;
}
function stateFresh() { return performance.now() - stateReceived <= 500; }
function controlsAllowed() { return authenticated && controller && stateFresh(); }
function cloudFresh() {
  if (!cloud || !cloud.points.length || performance.now() - cloud.received > 500) return false;
  if (!stateFresh() || !Number.isFinite(state.cloud_age_ms) || state.cloud_age_ms < 0) return false;
  return state.cloud_age_ms + performance.now() - stateReceived <= 500;
}
function motionAllowed() { return controlsAllowed() && state.estop === false && cloudFresh(); }
function directAllowed() { return motionAllowed() && tab === 'direct' && state.mode === 0 && state.active === true; }
function note(text) { $('actionStatus').textContent = text; }
function clearCloud() { cloud = null; candidate = null; }
function resetControls(emit = true) {
  held.clear(); joy.x = joy.y = yaw = 0;
  $('joystickStick').style.transform = 'translate(0,0)';
  $('rotationKnob').style.left = '50%';
  if (emit) send({ type: 'direct', vx: 0, vy: 0, wz: 0 });
}
function pause() { pendingStart = null; resetControls(); send({ type: 'enable', value: false }); }
function requestStart(mode) {
  pause();
  pendingStart = { mode, ack: false, requested: performance.now() };
  send({ type: 'mode', value: mode });
  note('已请求模式切换；节点确认暂停及模式后才启用');
}
function disconnect() {
  resetControls(false); authenticated = controller = false;
  pendingStart = null; boundObserved = false;
  state = {}; stateReceived = -Infinity; clearCloud();
  boundConfirmed = binding = false;
  $('connectionStatus').textContent = '连接断开 · 保持暂停，重连不会恢复运动';
  render();
}
async function connect() {
  const generation = ++connectGeneration;
  clearTimeout(reconnectTimer);
  if (ws) { ws.onclose = null; ws.close(); }
  disconnect();
  $('connectionStatus').textContent = '连接中 · 等待认证';
  try {
    const response = await fetch('/api/config', { cache: 'no-store' });
    if (!response.ok) throw new Error('配置读取失败');
    const config = await response.json();
    if (generation !== connectGeneration) return;
    if (!Number.isInteger(config.ws_port) || config.ws_port < 1 || config.ws_port > 65535) throw new Error('无效 WS 端口');
    const url = new URL(location.href);
    url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    url.port = String(config.ws_port); url.pathname = '/'; url.search = ''; url.hash = '';
    if (typeof config.ws_url === 'string' && config.ws_url) {
      const configured = new URL(config.ws_url, location.href);
      if (!['ws:', 'wss:'].includes(configured.protocol) || (location.protocol === 'https:' && configured.protocol !== 'wss:') || configured.username || configured.password || configured.search) throw new Error('非法 WS 地址');
      url.href = configured.href;
    }
    const socket = new WebSocket(url.href); ws = socket; socket.binaryType = 'arraybuffer';
    socket.onopen = () => { if (generation !== connectGeneration || socket !== ws) { socket.close(); return; } socket.send(JSON.stringify({ type: 'auth', token })); };
    socket.onmessage = event => {
      if (socket !== ws) return;
      if (event.data instanceof ArrayBuffer) {
        if (!authenticated) return;
        try { cloud = parseCloud(event.data); dataError = ''; }
        catch (error) { clearCloud(); dataError = '点云数据错误：' + error.message; }
      } else {
        try { receive(JSON.parse(event.data)); }
        catch (error) { note('状态数据错误：' + error.message); }
      }
      render();
    };
    socket.onerror = () => socket.close();
    socket.onclose = () => { if (socket !== ws) return; disconnect(); reconnectTimer = setTimeout(connect, 1500); };
  } catch (error) {
    if (generation !== connectGeneration) return;
    note(error.message); reconnectTimer = setTimeout(connect, 1500);
  }
}
function receive(message) {
  if (!message || typeof message !== 'object') throw new Error('非对象消息');
  if (message.type === 'auth') {
    authenticated = message.success === true; controller = authenticated && message.controller === true;
    if (!authenticated) { note('认证失败'); return; }
    if (controller) pause();
    $('connectionStatus').textContent = controller ? '已认证 · 操作者租约 · 重连保持暂停' : '已认证 · 只读（其他操作者持有租约）';
  } else if (message.type === 'state' && authenticated) {
    state = message; stateReceived = performance.now(); controller = message.controller === true;
    if (boundConfirmed && state.target_valid === true) boundObserved = true;
    if (boundObserved && state.target_valid !== true) { boundConfirmed = false; boundObserved = false; }
    if (pendingStart) {
      const request = pendingStart;
      if (!motionAllowed() || performance.now() - request.requested > 1000 || (request.mode === 1 && (!boundConfirmed || state.target_valid !== true))) {
        pendingStart = null; note('启动取消：数据失效或模式确认超时');
      } else if (request.ack && state.mode === request.mode && state.active === false) {
        if (request.mode === 0) send({ type: 'direct', vx: 0, vy: 0, wz: 0 });
        pendingStart = null; send({ type: 'enable', value: true }); note('已请求启用；等待节点确认');
      }
    }
    if (!directAllowed()) resetControls(held.size > 0);
    $('connectionStatus').textContent = controller ? '已认证 · 操作者租约' : '已认证 · 只读（无控制租约）';
  } else if (message.type === 'result' && authenticated) {
    note(message.action + ': ' + String(message.reason || (message.success ? 'OK' : 'FAILED')));
    if (message.action === 'mode' && pendingStart) {
      if (message.success === true) pendingStart.ack = true;
      else pendingStart = null;
    }
    if (message.action === 'set_target' && message.reason !== 'QUEUED') {
      binding = false;
      boundConfirmed = message.success === true && message.reason === 'OK';
      boundObserved = false;
    }
  }
}
function parseCloud(buffer) {
  if (buffer.byteLength < 22) throw new Error('头部截断');
  const view = new DataView(buffer);
  if (view.getUint32(0, false) !== 0x50433031) throw new Error('版本不是 PC01');
  const seq = view.getUint32(4, true), sec = view.getInt32(8, true);
  const nanosec = view.getUint32(12, true), count = view.getUint32(16, true), frameLength = view.getUint16(20, true);
  const offset = 22 + frameLength;
  if (nanosec >= 1000000000 || !frameLength || buffer.byteLength !== offset + count * 12) throw new Error('非法时间或长度');
  const frame = new TextDecoder('utf-8', { fatal: true }).decode(new Uint8Array(buffer, 22, frameLength));
  if (!frame.trim() || frame.includes('\0')) throw new Error('非法 frame');
  const points = new Float32Array(count * 3);
  for (let i = 0; i < points.length; i++) {
    const value = view.getFloat32(offset + i * 4, true);
    if (!Number.isFinite(value)) throw new Error('非有限坐标');
    points[i] = value;
  }
  if (!count) candidate = null;
  return { seq, sec, nanosec, frame, points, received: performance.now() };
}
function resize() {
  const rect = cv.parentElement.getBoundingClientRect();
  width = rect.width; height = rect.height;
  const dpr = window.devicePixelRatio || 1;
  cv.width = Math.round(width * dpr); cv.height = Math.round(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  scale = Math.min(width / (CFG.sideRange * 2), height / (CFG.forwardRange + CFG.backRange));
  cx = width / 2; cy = CFG.forwardRange * scale; draw();
}
function W2S(x, y) { return [cx - y * scale, cy - x * scale]; }
function S2W(px, py) { return [(cy - py) / scale, (cx - px) / scale]; }
function marker(x, y, color, radius) {
  const [sx, sy] = W2S(x, y); ctx.strokeStyle = color; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(sx, sy, radius, 0, Math.PI * 2); ctx.stroke();
}
function draw() {
  if (!ctx) return;
  ctx.clearRect(0, 0, width, height); ctx.strokeStyle = '#203847'; ctx.lineWidth = 1;
  for (let x = -CFG.backRange; x <= CFG.forwardRange; x++) {
    const [, sy] = W2S(x, 0); ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(width, sy); ctx.stroke();
  }
  for (let y = -CFG.sideRange; y <= CFG.sideRange; y++) {
    const [sx] = W2S(0, y); ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, height); ctx.stroke();
  }
  if (cloudFresh()) {
    const points = cloud.points;
    for (let i = 0; i < points.length; i += 3) {
      const [sx, sy] = W2S(points[i], points[i + 1]);
      const n = Math.max(0, Math.min(1, (points[i + 2] + .5) / 2.5));
      ctx.fillStyle = `hsl(${240 * (1 - n)} 90% 65%)`; ctx.fillRect(sx - 1, sy - 1, 3, 3);
    }
    if (candidate) marker(candidate.x, candidate.y, '#ffde32', 8);
  }
  ctx.fillStyle = '#62c4ff'; ctx.beginPath(); ctx.moveTo(cx, cy - 12); ctx.lineTo(cx - 8, cy + 8); ctx.lineTo(cx + 8, cy + 8); ctx.closePath(); ctx.fill();
  if (state.target_valid === true && Array.isArray(state.target) && state.target.slice(0, 2).every(Number.isFinite)) marker(state.target[0], state.target[1], '#54ed8a', 10);
  ctx.fillStyle = '#fff'; ctx.font = '13px sans-serif'; ctx.fillText('+x 前 ↑   +y 左 ← · 每格 1m', 10, 20);
}
function selectPoint(event) {
  if (!controlsAllowed() || !cloudFresh() || binding || state.bind_pending === true) return;
  const rect = cv.getBoundingClientRect(); const [x, y] = S2W(event.clientX - rect.left, event.clientY - rect.top);
  let nearest = -1, distance = .8 * .8;
  for (let i = 0; i < cloud.points.length; i += 3) {
    const d = (cloud.points[i] - x) ** 2 + (cloud.points[i + 1] - y) ** 2;
    if (d <= distance) { nearest = i; distance = d; }
  }
  boundConfirmed = false;
  if (nearest < 0) { candidate = null; note('NO_RETURN：点击附近 0.8m 内无显示回波'); }
  else candidate = { x: cloud.points[nearest], y: cloud.points[nearest + 1], z: cloud.points[nearest + 2], frame: cloud.frame, stamp_sec: cloud.sec, stamp_nanosec: cloud.nanosec, received: cloud.received };
  render();
}
function pointerControl(element, update, reset) {
  element.addEventListener('pointerdown', event => {
    if (!directAllowed() || held.has(element) || (event.pointerType === 'mouse' && event.button !== 0)) return;
    event.preventDefault(); held.set(element, event.pointerId); element.setPointerCapture(event.pointerId); update(event);
  });
  element.addEventListener('pointermove', event => { if (held.get(element) === event.pointerId) { if (directAllowed()) update(event); else resetControls(); } });
  const end = event => {
    if (held.get(element) !== event.pointerId) return;
    if (event.type !== 'pointerup') { resetControls(); return; }
    held.delete(element); reset();
    send({ type: 'direct', vx: 0, vy: 0, wz: 0 });
    if (element.hasPointerCapture(event.pointerId)) element.releasePointerCapture(event.pointerId);
  };
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type => element.addEventListener(type, end));
}
function render() {
  const fresh = stateFresh(), validCloud = cloudFresh();
  if (cloud && performance.now() - cloud.received > 500) clearCloud();
  if (!validCloud && held.size) resetControls();
  if (pendingStart && performance.now() - pendingStart.requested > 1000) { pendingStart = null; note('模式确认超时 · 保持暂停'); }
  $('nodeState').textContent = fresh ? `节点确认：${state.mode === 0 ? 'DIRECT' : state.mode === 1 ? 'FOLLOW' : '未知模式'} · ${state.active === true ? '已启用' : '暂停'} · 软件停止 ${state.estop === true ? '锁存' : state.estop === false ? '解除' : '未知'} · 目标 ${state.target_valid === true ? '有效' : '无效'}` : '节点状态未知 / 过期 · 禁止运动';
  $('cloudStatus').textContent = dataError || (validCloud ? `frame: ${cloud.frame} · ${Math.round(state.cloud_age_ms + performance.now() - stateReceived)}ms · ${cloud.points.length / 3} 点 · ${cloud.sec}.${String(cloud.nanosec).padStart(9, '0')}` : '无新鲜点云 · 已清空 · 禁止选择');
  const cmd = state.cmd;
  $('commandStatus').textContent = fresh && cmd && [cmd.vx, cmd.vy, cmd.wz].every(Number.isFinite) ? `节点输出 vx ${cmd.vx.toFixed(2)} · vy ${cmd.vy.toFixed(2)} m/s · wz ${cmd.wz.toFixed(2)} rad/s` : '输出指令未知';
  $('candidateStatus').textContent = candidate ? `黄色候选 xyz: ${candidate.x.toFixed(2)}, ${candidate.y.toFixed(2)}, ${candidate.z.toFixed(2)}m · ${candidate.frame}${performance.now() - candidate.received > 500 ? ' · 候选已过期，禁止绑定；请重新点选' : ''}` : '未选择候选';
  $('softwareStop').disabled = !authenticated || !controller;
  $('releaseStop').disabled = !controlsAllowed();
  $('bindTarget').disabled = !controlsAllowed() || !validCloud || !candidate || performance.now() - candidate.received > 500 || binding || state.bind_pending === true;
  $('startFollow').disabled = !motionAllowed() || !boundConfirmed || state.target_valid !== true || binding || state.bind_pending === true || pendingStart !== null;
  $('startDirect').disabled = !motionAllowed() || pendingStart !== null;
  ['pauseFollow', 'pauseDirect', 'clearTarget'].forEach(id => $(id).disabled = !controlsAllowed());
  ['btnStandUp', 'btnLieDown'].forEach(id => $(id).disabled = !controlsAllowed() || state.estop !== false);
  if (state.bind_pending === true) note('绑定处理中…');
  draw();
}

document.addEventListener('DOMContentLoaded', () => {
  cv = $('lidarCanvas'); ctx = cv.getContext('2d');
  $('connectForm').addEventListener('submit', event => { event.preventDefault(); token = $('authToken').value; $('authToken').value = ''; pause(); connect(); });
  $('softwareStop').addEventListener('click', () => { send({ type: 'estop', value: true }); pause(); boundConfirmed = false; binding = false; note('已请求软件停止，等待节点确认'); });
  $('releaseStop').addEventListener('click', () => { resetControls(); send({ type: 'estop', value: false }); note('已请求解除停止；不会自动启动'); });
  $('bindTarget').addEventListener('click', () => {
    if ($('bindTarget').disabled) return;
    pause(); boundConfirmed = false; binding = true;
    const { received, ...point } = candidate;
    send({ type: 'set_target', ...point }); note('等待绑定服务确认…'); render();
  });
  $('startFollow').addEventListener('click', () => { if (!$('startFollow').disabled) requestStart(1); });
  $('startDirect').addEventListener('click', () => { if (!$('startDirect').disabled) requestStart(0); });
  ['pauseFollow', 'pauseDirect'].forEach(id => $(id).addEventListener('click', pause));
  $('clearTarget').addEventListener('click', () => { pause(); send({ type: 'clear_target' }); candidate = null; boundConfirmed = false; render(); });
  ['StandUp', 'LieDown'].forEach((suffix, index) => $('btn' + suffix).addEventListener('click', () => { pause(); send({ type: 'action', value: index ? 'liedown' : 'standup' }); }));
  document.querySelectorAll('.nav-item').forEach(button => button.addEventListener('click', () => {
    pause(); tab = button.dataset.tab;
    document.querySelectorAll('.nav-item').forEach(item => { item.classList.toggle('active', item === button); item.setAttribute('aria-pressed', String(item === button)); });
    $('panel-follow').hidden = tab !== 'follow'; $('panel-direct').hidden = tab !== 'direct'; resize();
  }));
  let click = null;
  cv.addEventListener('pointerdown', event => { if (event.isPrimary && (event.pointerType !== 'mouse' || event.button === 0)) click = { id: event.pointerId, x: event.clientX, y: event.clientY }; });
  cv.addEventListener('pointermove', event => { if (click && click.id === event.pointerId && Math.hypot(event.clientX - click.x, event.clientY - click.y) > 8) click = null; });
  cv.addEventListener('pointerup', event => { if (click && click.id === event.pointerId && Math.hypot(event.clientX - click.x, event.clientY - click.y) <= 8) selectPoint(event); click = null; });
  ['pointercancel', 'lostpointercapture', 'pointerleave'].forEach(type => cv.addEventListener(type, () => { click = null; }));
  pointerControl($('joystickBase'), event => {
    const rect = $('joystickBase').getBoundingClientRect(); let x = (event.clientX - rect.left - rect.width / 2) / (rect.width / 2), y = (event.clientY - rect.top - rect.height / 2) / (rect.height / 2);
    const norm = Math.max(1, Math.hypot(x, y)); joy.x = x / norm; joy.y = y / norm;
    $('joystickStick').style.transform = `translate(${joy.x * 65}px,${joy.y * 65}px)`;
  }, () => { joy.x = joy.y = 0; $('joystickStick').style.transform = 'translate(0,0)'; });
  pointerControl($('rotationTrack'), event => {
    const rect = $('rotationTrack').getBoundingClientRect(); const n = Math.max(-1, Math.min(1, (event.clientX - rect.left) / rect.width * 2 - 1));
    yaw = -n; $('rotationKnob').style.left = `${(n + 1) * 50}%`;
  }, () => { yaw = 0; $('rotationKnob').style.left = '50%'; });
  window.addEventListener('blur', () => { click = null; resetControls(); });
  document.addEventListener('visibilitychange', () => { if (document.hidden) { click = null; pause(); } });
  window.addEventListener('pagehide', pause);
  window.addEventListener('resize', resize);
  setInterval(() => {
    if (authenticated && ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: 'heartbeat' }));
    render();
  }, 100);
  setInterval(() => {
    if (directAllowed() && held.size && !document.hidden) send({ type: 'direct', vx: -joy.y * .3, vy: -joy.x * .15, wz: yaw * .5 });
    else if (held.size) resetControls();
  }, 50);
  resize(); render();
});
