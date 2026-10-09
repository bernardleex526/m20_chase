#!/usr/bin/env python3
"""ROS web facade: packed display clouds and authenticated operator control."""
import ipaddress
import json
import math
import mimetypes
import os
import struct
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int32, String
from rs_follow_interfaces.srv import BindTarget
from ws_server import WSServer

HERE = os.path.dirname(os.path.abspath(__file__))


def find_web_root():
    try:
        from ament_index_python.packages import get_package_share_directory
        root = os.path.join(get_package_share_directory('rs_follow'), 'web')
        if os.path.isdir(root):
            # ament --symlink-install links individual assets rather than the web directory.
            # Resolve the trusted installed entry point once, then contain all requests
            # within its real asset directory (including their own symlink targets).
            entry = os.path.join(root, 'index.html')
            if os.path.isfile(entry):
                return os.path.dirname(os.path.realpath(entry))
            return root
    except ImportError:
        pass
    return os.path.normpath(os.path.join(HERE, '..', 'web'))


def validate_action(message):
    if not isinstance(message, dict):
        raise ValueError('BAD_MESSAGE')
    kind = message.get('type')
    if kind in ('enable', 'estop'):
        if type(message.get('value')) is not bool:
            raise ValueError('BAD_BOOL')
    elif kind == 'mode':
        if type(message.get('value')) is not int or message['value'] not in (0, 1):
            raise ValueError('BAD_MODE')
    elif kind in ('direct', 'set_target'):
        keys = ('vx', 'vy', 'wz') if kind == 'direct' else ('x', 'y', 'z')
        for key in keys:
            value = message.get(key)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError('BAD_POINT' if kind == 'set_target' else 'BAD_VELOCITY')
        if kind == 'set_target':
            frame = message.get('frame')
            if not isinstance(frame, str) or not frame or len(frame.encode('utf-8')) > 65535:
                raise ValueError('BAD_POINT')
            for key, low, high in (('stamp_sec', -2147483648, 2147483647),
                                   ('stamp_nanosec', 0, 999999999)):
                if type(message.get(key)) is not int or not low <= message[key] <= high:
                    raise ValueError('BAD_POINT')
    elif kind == 'action':
        if message.get('value') not in ('standup', 'liedown'):
            raise ValueError('BAD_ACTION')
    elif kind != 'clear_target':
        raise ValueError('BAD_ACTION')
    return dict(message)


def priority_action(message):
    return (message['type'] == 'clear_target' or
            message['type'] in ('enable', 'estop') and
            (message['type'] == 'enable' and not message['value'] or
             message['type'] == 'estop' and message['value']) or
            message['type'] == 'direct' and not any(message[k] for k in ('vx', 'vy', 'wz')))


def pack_cloud(msg, seq):
    frame = msg.header.frame_id.encode('utf-8')
    count = msg.width * msg.height
    if (not frame or len(frame) > 65535 or msg.height != 1 or count > 6000 or
            msg.is_bigendian or msg.point_step != 12 or msg.row_step != count * 12 or
            len(msg.data) != count * 12 or not 0 <= msg.header.stamp.nanosec < 1000000000):
        raise ValueError('BAD_CLOUD')
    fields = {f.name: (f.offset, f.datatype, f.count) for f in msg.fields}
    if any(fields.get(key) != (offset, 7, 1) for key, offset in (('x', 0), ('y', 4), ('z', 8))):
        raise ValueError('BAD_CLOUD')
    payload = bytes(msg.data)
    if any(not math.isfinite(v) for xyz in struct.iter_unpack('<fff', payload) for v in xyz):
        raise ValueError('BAD_CLOUD')
    return struct.pack('<4sIiIIH', b'PC01', seq, msg.header.stamp.sec,
                       msg.header.stamp.nanosec, count, len(frame)) + frame + payload


class WebUi(Node):
    def __init__(self):
        super().__init__('web_ui')
        g = self.declare_parameter
        self.http_port = g('http_port', 8080).value
        self.ws_port = g('ws_port', 8890).value
        self.ws_url = g('ws_url', '').value
        host = g('bind_host', '127.0.0.1').value
        token = g('auth_token', '').value
        origins = list(g('allowed_origins', ['']).value)
        origins = [o for o in origins if o]
        try:
            loopback = ipaddress.ip_address(host).is_loopback
        except ValueError:
            loopback = host == 'localhost'
        if not loopback and (len(token.encode('utf-8')) < 32 or not origins):
            raise ValueError('remote web exposure requires a >=32-byte random auth_token and explicit allowed_origins')
        if any(o == '*' or urlparse(o).scheme not in ('http', 'https') or not urlparse(o).netloc
               or urlparse(o).path or urlparse(o).query or urlparse(o).fragment for o in origins):
            raise ValueError('allowed_origins must contain explicit HTTP(S) origins')
        if not origins:
            origins = [f'http://127.0.0.1:{self.http_port}', f'http://localhost:{self.http_port}',
                       f'http://[::1]:{self.http_port}']
        self.origins = origins
        self.web_root = os.path.realpath(g('web_root', '').value or find_web_root())
        self.follow_dist = g('follow_dist', 1.0).value
        self.max_linear = g('max_linear', 0.9).value
        self.max_angular = g('max_angular', 1.0).value
        self.lock = threading.RLock()
        self.actions = deque(maxlen=64)
        self.generation = 0
        self.pending = None
        self.cancelled_bind = None
        self.bind_reason = ''
        self.binding_blocked = False
        self.cloud = None
        self.cloud_header = None
        self.cloud_received = None
        self.cloud_seq = 0
        self.control_received = None
        self.control = dict(active=False, mode=1, estop=False, target_valid=False)
        self.target = None
        self.cmd = dict(vx=0.0, vy=0.0, wz=0.0)
        self.status = 'INIT'
        self.enable_pub = self.create_publisher(Bool, g('enable_topic', '/rs_follow/enable').value, 10)
        self.mode_pub = self.create_publisher(Int32, g('control_mode_topic', '/rs_follow/control_mode').value, 10)
        self.direct_pub = self.create_publisher(Twist, g('direct_cmd_topic', '/rs_follow/direct_cmd').value, 10)
        self.action_pub = self.create_publisher(String, g('action_topic', '/rs_follow/action_cmd').value, 10)
        self.estop_pub = self.create_publisher(Bool, '/rs_follow/estop', 1)
        self.clear_pub = self.create_publisher(Bool, '/rs_follow/clear_target', 10)
        self.bind_client = self.create_client(BindTarget, '/rs_follow/bind')
        self.create_subscription(PointCloud2, g('cloud_topic', '/rs_follow/cloud_viz').value,
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(String, '/rs_follow/control_state', self._on_control, 10)
        self.create_subscription(PointStamped, '/rs_follow/target', self._on_target, 10)
        self.create_subscription(String, '/rs_follow/status', self._on_status, 10)
        self.create_subscription(Twist, g('cmd_vel_topic', '/cmd_vel').value, self._on_cmd, 10)
        self.create_timer(1.0 / 15.0, self._broadcast)
        self.create_timer(0.02, self._drain)
        self.ws = WSServer(self.ws_port, self._on_action, host=host, token=token,
                           origins=origins, on_release=self._release)
        self.httpd = ThreadingHTTPServer((host, self.http_port), self._handler_cls())
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def _on_cloud(self, msg):
        try:
            packed = pack_cloud(msg, self.cloud_seq)
        except (ValueError, struct.error, UnicodeError):
            with self.lock:
                self.cloud = None
                self.cloud_received = None
                self.status = 'BAD_CLOUD'
            return
        with self.lock:
            self.cloud_seq = (self.cloud_seq + 1) & 0xffffffff
            self.cloud = packed
            self.cloud_header = msg.header
            self.cloud_received = time.monotonic() if msg.width else None

    def _on_control(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict) or any(type(state.get(k)) is not bool for k in ('active', 'estop', 'target_valid')):
                return
            if type(state.get('mode')) is not int or state['mode'] not in (0, 1):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.control = {k: state[k] for k in ('active', 'mode', 'estop', 'target_valid')}
            self.control_received = time.monotonic()
            if not state['target_valid']:
                self.target = None

    def _on_target(self, msg):
        if all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)):
            with self.lock:
                self.target = [msg.point.x, msg.point.y, msg.point.z]

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data

    def _on_cmd(self, msg):
        with self.lock:
            self.cmd = dict(vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z)
            if not all(math.isfinite(v) for v in self.cmd.values()):
                self.cmd = dict(vx=0.0, vy=0.0, wz=0.0)

    def _state(self):
        with self.lock:
            now = time.monotonic()
            age = (now - self.cloud_received) * 1000 if self.cloud_received is not None else None
            fresh = age is not None and age <= 500
            confirmed = self.control_received is not None and now - self.control_received <= 0.5
            header = self.cloud_header
            valid = confirmed and fresh and self.control['target_valid'] and not self.binding_blocked
            return dict(type='state', scan=[], target=self.target if valid else None,
                        target_valid=valid, cmd=self.cmd, mode=self.control['mode'],
                        active=confirmed and self.control['active'], moving=confirmed and self.control['active'],
                        estop=self.control['estop'], status=self.status,
                        follow_dist=self.follow_dist, max_linear=self.max_linear, max_angular=self.max_angular,
                        cloud_frame=header.frame_id if header else '',
                        cloud_stamp_sec=header.stamp.sec if header else 0,
                        cloud_stamp_nanosec=header.stamp.nanosec if header else 0,
                        cloud_age_ms=age, bind_pending=self.pending is not None or self.cancelled_bind is not None,
                        bind_reason=self.bind_reason)

    def _broadcast(self):
        state = self._state()
        with self.lock:
            cloud = self.cloud if state['cloud_age_ms'] is not None and state['cloud_age_ms'] <= 500 else None
        self.ws.broadcast(state, cloud)

    def _pause(self):
        self.enable_pub.publish(Bool(data=False))
        self.direct_pub.publish(Twist())

    def _invalidate(self):
        self.actions.clear()
        self.generation += 1
        if self.pending is not None:
            self.binding_blocked = True
            client = self.pending[2]
            # Keep one outstanding request until the server completes: service cancellation
            # is local only, so clearing before its response cannot prevent a late commit.
            self.clear_pub.publish(Bool(data=True))
            self.cancelled_bind = self.pending[0]
            self.pending = None
            self.bind_reason = 'CANCELLED'
            if client is not None and hasattr(self, 'ws'):
                self.ws.reply(client, dict(type='result', action='set_target', success=False, reason='CANCELLED'))

    def _release(self):
        with self.lock:
            self._invalidate()
            self._pause()

    def _on_action(self, message, client=None):
        action = validate_action(message)
        with self.lock:
            if priority_action(action):
                self._invalidate()
                # Stops publish immediately, ahead of the ROS action timer.
                self._execute(action, client)
            else:
                if len(self.actions) >= 64:
                    return dict(success=False, reason='QUEUE_FULL')
                self.actions.append((action, client, self.generation))
        return dict(success=True, reason='QUEUED')

    def _drain(self):
        with self.lock:
            if self.cancelled_bind is not None:
                self._pause()
                if self.cancelled_bind.done():
                    self.clear_pub.publish(Bool(data=True))
                    self.cancelled_bind = None
            if self.pending is not None:
                future, deadline, client, generation = self.pending
                expired = time.monotonic() >= deadline
                if future.done() or expired:
                    self.pending = None
                    success = False
                    reason = 'TIMEOUT'
                    if not expired and future.done() and not future.cancelled():
                        try:
                            response = future.result()
                            success, reason = response.success, response.reason
                            if success:
                                self.target = [response.target.point.x, response.target.point.y, response.target.point.z]
                        except Exception:
                            reason = 'SERVICE_ERROR'
                    else:
                        if not future.done():
                            self.cancelled_bind = future
                        self._pause()
                        self.binding_blocked = True
                        self.clear_pub.publish(Bool(data=True))
                    if generation == self.generation:
                        self.bind_reason = reason
                        if success:
                            self.binding_blocked = False
                        if client is not None:
                            self.ws.reply(client, dict(type='result', action='set_target', success=success, reason=reason))
            # Bounded work per executor tick; no service waits in the ROS thread.
            for _ in range(min(64, len(self.actions))):
                action, client, generation = self.actions.popleft()
                if generation == self.generation:
                    result = self._execute(action, client)
                    if result is not None and client is not None:
                        self.ws.reply(client, dict(result, type='result', action=action['type']))

    def _execute(self, action, client):
        kind = action['type']
        if kind == 'enable':
            if action['value']:
                state = self._state()
                if (self.control_received is None or time.monotonic() - self.control_received > 0.5 or
                        state['estop'] or state['cloud_age_ms'] is None or state['cloud_age_ms'] > 500 or
                        (state['mode'] == 1 and (not state['target_valid'] or self.pending is not None or self.cancelled_bind is not None))):
                    return dict(success=False, reason='NOT_READY')
            self.enable_pub.publish(Bool(data=action['value']))
            if not action['value']:
                self.direct_pub.publish(Twist())
        elif kind == 'mode':
            self._pause()
            self.mode_pub.publish(Int32(data=action['value']))
        elif kind == 'estop':
            self._pause()
            self.estop_pub.publish(Bool(data=action['value']))
        elif kind == 'clear_target':
            self._pause()
            self.clear_pub.publish(Bool(data=True))
        elif kind == 'direct':
            if any(action[k] for k in ('vx', 'vy', 'wz')):
                state = self._state()
                if state['mode'] != 0 or not state['active'] or state['estop'] or state['cloud_age_ms'] is None or state['cloud_age_ms'] > 500:
                    return dict(success=False, reason='NOT_READY')
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = float(action['vx']), float(action['vy']), float(action['wz'])
            self.direct_pub.publish(msg)
        elif kind == 'action':
            self.action_pub.publish(String(data=action['value']))
        elif kind == 'set_target':
            if self.pending is not None or self.cancelled_bind is not None:
                return dict(success=False, reason='BIND_PENDING')
            self._pause()
            self.binding_blocked = True
            self.target = None
            if not self.bind_client.service_is_ready():
                self.bind_reason = 'SERVICE_UNAVAILABLE'
                return dict(success=False, reason=self.bind_reason)
            request = BindTarget.Request()
            request.point.header.frame_id = action['frame']
            request.point.header.stamp.sec = action['stamp_sec']
            request.point.header.stamp.nanosec = action['stamp_nanosec']
            request.point.point.x, request.point.point.y, request.point.point.z = float(action['x']), float(action['y']), float(action['z'])
            self.pending = (self.bind_client.call_async(request), time.monotonic() + 1.0, client, self.generation)
            self.bind_reason = ''
            return None
        return dict(success=True, reason='SENT')

    def _handler_cls(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _json(self, obj, code=200):
                body = json.dumps(obj, allow_nan=False).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _authorized(self):
                origin = self.headers.get('Origin')
                if origin is not None and origin not in node.origins:
                    self._json(dict(success=False, reason='BAD_ORIGIN'), 403)
                    return False
                header = self.headers.get('Authorization', '')
                if not header.startswith('Bearer ') or not node.ws.authenticate(header[7:]):
                    self._json(dict(success=False, reason='UNAUTHORIZED'), 401)
                    return False
                return True

            def do_GET(self):
                path = unquote(urlparse(self.path).path)
                if path == '/api/config':
                    return self._json(dict(ws_port=node.ws_port, ws_url=node.ws_url))
                if path == '/api/status':
                    if self._authorized():
                        return self._json(node._state())
                    return
                if path.startswith('/api/'):
                    return self._json(dict(success=False, reason='NOT_FOUND'), 404)
                full = os.path.realpath(os.path.join(node.web_root, path.lstrip('/') or 'index.html'))
                if os.path.commonpath((node.web_root, full)) != node.web_root or not os.path.isfile(full):
                    return self._json(dict(success=False, reason='NOT_FOUND'), 404)
                with open(full, 'rb') as stream:
                    body = stream.read()
                self.send_response(200)
                self.send_header('Content-Type', mimetypes.guess_type(full)[0] or 'application/octet-stream')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                if not self._authorized():
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 4096:
                        raise ValueError('BAD_LENGTH')
                    message = json.loads(self.rfile.read(length))
                    if not isinstance(message, dict):
                        raise ValueError('BAD_MESSAGE')
                    kind = urlparse(self.path).path.removeprefix('/api/')
                    if urlparse(self.path).path != '/api/' + kind:
                        raise ValueError('BAD_ACTION')
                    if 'type' in message and message['type'] != kind:
                        raise ValueError('BAD_ACTION')
                    message['type'] = kind
                    action = validate_action(message)
                    # HTTP has no heartbeat identity: permit safe stops, never acquire motion authority.
                    if not priority_action(action):
                        return self._json(dict(success=False, reason='WS_LEASE_REQUIRED'), 403)
                    return self._json(node._on_action(action))
                except (ValueError, TypeError, UnicodeError) as exc:
                    return self._json(dict(success=False, reason=str(exc)), 400)

        return Handler

    def destroy_node(self):
        self.ws.close()
        self.httpd.shutdown()
        self.httpd.server_close()
        self._release()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = WebUi()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
