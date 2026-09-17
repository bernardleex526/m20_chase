#!/usr/bin/env python3
"""
web_ui.py — jie_deamon-style web console for rs_follow (adapted).

Serves the static frontend (HTML/CSS/icons) + REST on HTTP, and pushes real-time
state over a self-contained WebSocket (no external deps), mirroring jie_deamon's
HTTP 8080 / WS 8890 design.

Endpoints
  HTTP  :  / , /app.js , /style.css , /icon/* , /api/status
           POST /api/set_target {x,y}, /api/set_moving {on}, /api/set_active {on}
  WS    :  server -> {scan,target,cmd,mode,moving,status,...}  @ ~15Hz
           client -> {type:set_target|switch_mode|direct_cmd|action_cmd|set_moving}

It drives:
  /rs_follow/bind_target (PointStamped), /rs_follow/enable (Bool),
  /rs_follow/control_mode (Int32; 0=DIRECT,1=FOLLOW),
  /rs_follow/direct_cmd (Twist, m/s & rad/s), /rs_follow/action_cmd (String)
"""

import json
import math
import mimetypes
import os
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String

from ws_server import WSServer

HERE = os.path.dirname(os.path.abspath(__file__))


def find_web_root():
    # 1) installed share/<pkg>/web, 2) source ../web
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('rs_follow'), 'web')
        if os.path.isdir(p):
            return p
    except Exception:
        pass
    p = os.path.normpath(os.path.join(HERE, '..', 'web'))
    return p if os.path.isdir(p) else HERE


class WebUi(Node):
    def __init__(self):
        super().__init__('web_ui')
        g = self.declare_parameter
        self.http_port = g('http_port', 8080).value
        self.ws_port = g('ws_port', 8890).value
        self.web_root = g('web_root', '').value or find_web_root()
        self.scan_topic = g('scan_topic', '/rs_follow/scan').value
        self.cmd_topic = g('cmd_vel_topic', '/cmd_vel').value
        self.bind_topic = g('bind_topic', '/rs_follow/bind_target').value
        self.enable_topic = g('enable_topic', '/rs_follow/enable').value
        self.mode_topic = g('control_mode_topic', '/rs_follow/control_mode').value
        self.direct_topic = g('direct_cmd_topic', '/rs_follow/direct_cmd').value
        self.action_topic = g('action_topic', '/rs_follow/action_cmd').value
        self.follow_dist = g('follow_dist', 1.0).value
        self.max_linear = g('max_linear', 0.9).value
        self.max_angular = g('max_angular', 1.0).value

        self.lock = threading.Lock()
        self.scan = []
        self.target = None
        self.target_valid = False
        self.cmd = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self.status = 'INIT'
        self.mode = 1          # 0 DIRECT, 1 FOLLOW
        self.moving = False    # == /rs_follow/enable
        self.actions = queue.Queue()

        self.bind_pub = self.create_publisher(PointStamped, self.bind_topic, 10)
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)
        self.mode_pub = self.create_publisher(Int32, self.mode_topic, 10)
        self.direct_pub = self.create_publisher(Twist, self.direct_topic, 10)
        self.action_pub = self.create_publisher(String, self.action_topic, 10)

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/rs_follow/target', self._on_target, 10)
        self.create_subscription(String, '/rs_follow/status', self._on_status, 10)
        self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 10)
        self.create_timer(1.0 / 15.0, self._broadcast)
        self.create_timer(0.02, self._drain)

        self.ws = WSServer(self.ws_port, self._on_ws)
        self.httpd = ThreadingHTTPServer(('0.0.0.0', self.http_port), self._handler_cls())
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'web_ui: http://0.0.0.0:{self.http_port}  ws://0.0.0.0:{self.ws_port}  '
            f'root={self.web_root}')

    # -- ROS callbacks ------------------------------------------------------- #
    def _on_scan(self, msg):
        pts = []
        step = max(1, len(msg.ranges) // 720)
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append([round(r * math.cos(a), 2), round(r * math.sin(a), 2)])
        with self.lock:
            self.scan = pts

    def _on_target(self, msg):
        with self.lock:
            self.target = [round(msg.point.x, 3), round(msg.point.y, 3)]
            self.target_valid = True

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data

    def _on_cmd(self, msg):
        with self.lock:
            self.cmd = {'vx': round(msg.linear.x, 3), 'vy': round(msg.linear.y, 3),
                        'wz': round(msg.angular.z, 3)}

    def _state(self):
        with self.lock:
            return {'scan': self.scan, 'target': self.target,
                    'target_valid': self.target_valid, 'cmd': self.cmd,
                    'mode': self.mode, 'moving': self.moving, 'status': self.status,
                    'follow_dist': self.follow_dist, 'max_linear': self.max_linear,
                    'max_angular': self.max_angular}

    def _broadcast(self):
        try:
            self.ws.broadcast(json.dumps(self._state()))
        except Exception:
            pass

    def _drain(self):
        while not self.actions.empty():
            a = self.actions.get_nowait()
            k = a[0]
            if k == 'target':
                p = PointStamped()
                p.header.frame_id = 'rslidar'
                p.header.stamp = self.get_clock().now().to_msg()
                p.point.x, p.point.y, p.point.z = a[1], a[2], 0.0
                self.bind_pub.publish(p)
            elif k == 'moving':
                self.moving = bool(a[1])
                self.enable_pub.publish(Bool(data=self.moving))
            elif k == 'mode':
                self.mode = int(a[1])
                self.mode_pub.publish(Int32(data=self.mode))
            elif k == 'direct':
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = a[1], a[2], a[3]
                self.direct_pub.publish(t)
            elif k == 'action':
                self.action_pub.publish(String(data=str(a[1])))

    # -- WebSocket message --------------------------------------------------- #
    def _on_ws(self, text):
        try:
            m = json.loads(text)
        except Exception:
            return
        t = m.get('type')
        if t == 'set_target':
            self.actions.put(('target', float(m.get('x', 0)), float(m.get('y', 0))))
        elif t == 'switch_mode':
            self.actions.put(('mode', int(m.get('mode', 1))))
        elif t == 'direct_cmd':
            self.actions.put(('direct', float(m.get('x', 0)), float(m.get('y', 0)),
                              float(m.get('z', 0))))
        elif t == 'action_cmd':
            self.actions.put(('action', m.get('action', '')))
        elif t == 'set_moving':
            self.actions.put(('moving', bool(m.get('on', False))))

    # -- HTTP ---------------------------------------------------------------- #
    def _handler_cls(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _json(self, obj, code=200):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def _file(self, path):
                rel = path.lstrip('/') or 'index.html'
                full = os.path.normpath(os.path.join(node.web_root, rel))
                if not full.startswith(node.web_root) or not os.path.isfile(full):
                    return self._json({'error': 'not found'}, 404)
                ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
                with open(full, 'rb') as f:
                    body = f.read()
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                p = urlparse(self.path).path
                if p == '/api/status':
                    return self._json(node._state())
                return self._file(p)

            def do_POST(self):
                p = urlparse(self.path).path
                n = int(self.headers.get('Content-Length', 0) or 0)
                try:
                    body = json.loads(self.rfile.read(n) or b'{}')
                except Exception:
                    body = {}
                if p == '/api/set_target':
                    node.actions.put(('target', float(body.get('x', 0)), float(body.get('y', 0))))
                    return self._json({'ok': True})
                if p == '/api/set_moving':
                    node.actions.put(('moving', bool(body.get('on', False))))
                    return self._json({'ok': True})
                if p == '/api/set_active':
                    node.actions.put(('moving', bool(body.get('on', False))))
                    return self._json({'ok': True})
                return self._json({'ok': False}, 404)

        return H


def main():
    rclpy.init()
    node = WebUi()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
