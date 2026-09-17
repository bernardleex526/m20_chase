#!/usr/bin/env python3
"""
web_target_ui.py — browser-based target selection for rs_follow (SSH friendly).

Serves a small web page (default http://<host>:8080) that draws the projected
2D scan and lets you **double-click** to bind the follow target — the same idea
as jie_deamon's web UI. No extra dependencies (Python stdlib HTTP server).

  * GET /            -> the page
  * GET /state       -> JSON {scan, target, target_raw, status, enabled, ...}
  * GET /bind?x=&y=  -> publish /rs_follow/bind_target
  * GET /clear       -> publish /rs_follow/clear_target
  * GET /enable?on=  -> publish /rs_follow/enable

Params:
  port (8080), bind_topic (/rs_follow/bind_target),
  scan_topic (/rs_follow/scan), enable_topic, clear_topic,
  follow_dist (1.0, drawn as a circle)
"""

import json
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>rs_follow target</title>
<style>
 body{margin:0;background:#111;color:#eee;font-family:monospace}
 #bar{padding:8px 12px;background:#1b1b1b}
 button{margin-right:8px;padding:6px 12px;background:#333;color:#eee;border:1px solid #555;cursor:pointer}
 button:hover{background:#444}
 #msg{margin-left:10px;color:#7fd}
 canvas{display:block;margin:0 auto;background:#0a0a0a}
</style></head>
<body>
<div id="bar">
  <b>rs_follow</b>
  <button onclick="bind(0,1.5)">绑定正前方1.5m</button>
  <button onclick="fetch('/clear')">清除目标</button>
  <button id="en" onclick="toggle()">使能</button>
  <span id="msg">双击画布选目标（+x 右, +y 上）</span>
</div>
<canvas id="cv" width="900" height="900"></canvas>
<script>
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const CX=cv.width/2, CY=cv.height/2, SCALE=32;   // px per meter
let enabled=false, followDist=1.0, frame='rslidar', lastBind=null;
function W2S(x,y){return [CX+x*SCALE, CY-y*SCALE];}
function S2W(px,py){return [(px-CX)/SCALE, (CY-py)/SCALE];}
function drawGrid(){
  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.strokeStyle='#1e3a2a'; ctx.lineWidth=1;
  for(let m=-14;m<=14;m++){
    let p=W2S(m,0)[0]; ctx.beginPath();ctx.moveTo(p,0);ctx.lineTo(p,cv.height);ctx.stroke();
    p=W2S(0,m)[1]; ctx.beginPath();ctx.moveTo(0,p);ctx.lineTo(cv.width,p);ctx.stroke();
  }
  ctx.strokeStyle='#2e6'; ctx.beginPath();
  ctx.moveTo(0,CY);ctx.lineTo(cv.width,CY);ctx.moveTo(CX,0);ctx.lineTo(CX,cv.height);ctx.stroke();
  // follow_dist circle
  ctx.strokeStyle='#4a4'; ctx.setLineDash([5,5]); ctx.beginPath();
  ctx.arc(CX,CY,followDist*SCALE,0,2*Math.PI); ctx.stroke(); ctx.setLineDash([]);
}
function draw(st){
  drawGrid();
  // scan points
  if(st.scan){ ctx.fillStyle='#888';
    for(const p of st.scan){ const s=W2S(p[0],p[1]); ctx.fillRect(s[0],s[1],2,2); } }
  // robot
  ctx.fillStyle='#39f'; ctx.beginPath();
  ctx.moveTo(CX+12,CY); ctx.lineTo(CX-8,CY-8); ctx.lineTo(CX-8,CY+8); ctx.closePath(); ctx.fill();
  // target
  if(st.target){ ctx.fillStyle='#0d0'; const s=W2S(st.target[0],st.target[1]);
    ctx.beginPath(); ctx.arc(s[0],s[1],7,0,2*Math.PI); ctx.fill(); }
  if(st.target_raw){ ctx.strokeStyle='#a0f'; const s=W2S(st.target_raw[0],st.target_raw[1]);
    ctx.beginPath(); ctx.moveTo(s[0]-5,s[1]-5);ctx.lineTo(s[0]+5,s[1]+5);
    ctx.moveTo(s[0]+5,s[1]-5);ctx.lineTo(s[0]-5,s[1]+5); ctx.stroke(); }
  if(lastBind){ ctx.fillStyle='#ff0'; const s=W2S(lastBind[0],lastBind[1]);
    ctx.beginPath(); ctx.arc(s[0],s[1],4,0,2*Math.PI); ctx.fill(); }
  // status
  ctx.fillStyle='#ccc'; ctx.font='16px monospace';
  ctx.fillText('status: '+st.status+'   enabled: '+st.enabled+
               '   follow_dist: '+st.follow_dist.toFixed(2)+'m', 12, 22);
  ctx.fillText('range/1m grid;  frame='+st.frame, 12, 44);
  document.getElementById('msg').textContent =
     '双击选目标  |  目标('+(st.target?st.target.map(v=>v.toFixed(2)).join(', '):'-')+')';
}
function tick(){ fetch('/state').then(r=>r.json()).then(st=>{
    enabled=st.enabled; followDist=st.follow_dist; frame=st.frame; draw(st);
    document.getElementById('en').textContent = enabled?'暂停':'使能';
  }).catch(e=>{}); }
function bind(x,y){ lastBind=[x,y];
  fetch('/bind?x='+x.toFixed(3)+'&y='+y.toFixed(3)).catch(e=>{}); }
function toggle(){ fetch('/enable?on='+(enabled?'0':'1')).catch(e=>{}); }
cv.addEventListener('dblclick', e=>{
  const r=cv.getBoundingClientRect();
  const [x,y]=S2W(e.clientX-r.left, e.clientY-r.top);
  bind(x,y);
});
setInterval(tick, 120); tick();
</script></body></html>
"""


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.scan = None
        self.target = None
        self.target_raw = None
        self.status = 'INIT'
        self.enabled = False
        self.follow_dist = 1.0
        self.frame = 'rslidar'


class UiNode(Node):
    def __init__(self):
        super().__init__('web_target_ui')
        self.port = self.declare_parameter('port', 8080).value
        self.bind_topic = self.declare_parameter('bind_topic', '/rs_follow/bind_target').value
        self.clear_topic = self.declare_parameter('clear_topic', '/rs_follow/clear_target').value
        self.enable_topic = self.declare_parameter('enable_topic', '/rs_follow/enable').value
        self.scan_topic = self.declare_parameter('scan_topic', '/rs_follow/scan').value
        self.follow_dist = self.declare_parameter('follow_dist', 1.0).value

        self.shared = Shared()
        self.shared.follow_dist = self.follow_dist
        self.actions = queue.Queue()

        self.bind_pub = self.create_publisher(PointStamped, self.bind_topic, 10)
        self.clear_pub = self.create_publisher(Bool, self.clear_topic, 10)
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/rs_follow/target', self._on_target, 10)
        self.create_subscription(PointStamped, '/rs_follow/target_raw', self._on_raw, 10)
        self.create_subscription(String, '/rs_follow/status', self._on_status, 10)
        self.create_timer(0.02, self._drain)

        self.httpd = ThreadingHTTPServer(('0.0.0.0', self.port), self._handler_cls())
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'web_target_ui: http://0.0.0.0:{self.port}  '
                               f'(scan="{self.scan_topic}", bind -> "{self.bind_topic}")')

    # -- ros callbacks ------------------------------------------------------- #
    def _on_scan(self, msg):
        pts = []
        sf = qos_profile_sensor_data  # noqa
        step = max(1, len(msg.ranges) // 720)
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append([round(r * math.cos(a), 2), round(r * math.sin(a), 2)])
        with self.shared.lock:
            self.shared.scan = pts
            self.shared.frame = msg.header.frame_id

    def _on_target(self, msg):
        with self.shared.lock:
            self.shared.target = [round(msg.point.x, 3), round(msg.point.y, 3)]

    def _on_raw(self, msg):
        with self.shared.lock:
            self.shared.target_raw = [round(msg.point.x, 3), round(msg.point.y, 3)]

    def _on_status(self, msg):
        with self.shared.lock:
            self.shared.status = msg.data
            self.shared.enabled = msg.data not in ('NO_TARGET',)  # informational only

    def _drain(self):
        while not self.actions.empty():
            act = self.actions.get_nowait()
            if act[0] == 'bind':
                p = PointStamped()
                p.header.frame_id = self.shared.frame
                p.header.stamp = self.get_clock().now().to_msg()
                p.point.x, p.point.y, p.point.z = act[1], act[2], 0.0
                self.bind_pub.publish(p)
                self.get_logger().info(f'bind target ({act[1]:.2f}, {act[2]:.2f})')
            elif act[0] == 'clear':
                self.clear_pub.publish(Bool(data=True))
            elif act[0] == 'enable':
                self.enable_pub.publish(Bool(data=bool(act[1])))

    # -- HTTP ---------------------------------------------------------------- #
    def _handler_cls(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype='application/json'):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body if isinstance(body, bytes) else body.encode())

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                if u.path == '/':
                    return self._send(200, HTML, 'text/html; charset=utf-8')
                if u.path == '/state':
                    with node.shared.lock:
                        st = {'scan': node.shared.scan, 'target': node.shared.target,
                              'target_raw': node.shared.target_raw,
                              'status': node.shared.status, 'enabled': node.shared.enabled,
                              'follow_dist': node.shared.follow_dist, 'frame': node.shared.frame}
                    return self._send(200, json.dumps(st))
                if u.path == '/bind':
                    try:
                        x = float(q.get('x', ['0'])[0]); y = float(q.get('y', ['0'])[0])
                    except ValueError:
                        return self._send(400, json.dumps({'ok': False}))
                    node.actions.put(('bind', x, y))
                    return self._send(200, json.dumps({'ok': True}))
                if u.path == '/clear':
                    node.actions.put(('clear',))
                    return self._send(200, json.dumps({'ok': True}))
                if u.path == '/enable':
                    on = q.get('on', ['1'])[0] in ('1', 'true', 'True')
                    node.actions.put(('enable', on))
                    return self._send(200, json.dumps({'ok': True, 'enabled': on}))
                return self._send(404, json.dumps({'error': 'not found'}))

        return H


def main():
    rclpy.init()
    node = UiNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
