#!/usr/bin/env python3
"""M20 basic_server adapter with fail-closed status and command gating."""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from dog_adapters.command_gate import CLEAR_ACTIONS, STOP_ACTIONS, CommandGate
from m20_bridge import protocol as P


class M20Bridge(Node):
    def __init__(self):
        super().__init__('m20_bridge')
        self.transport = self.declare_parameter('transport', 'udp').value
        self.ip = self.declare_parameter('ip', '10.21.31.103').value
        self.port = self.declare_parameter('port', 30000).value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.odom_topic = self.declare_parameter('odom_topic', '/m20/odom').value
        self.rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.heartbeat_hz = self.declare_parameter('heartbeat_hz', 2.0).value
        self.watchdog_timeout = self.declare_parameter('watchdog_timeout', 0.3).value
        self.status_timeout = self.declare_parameter('status_timeout', 1.5).value
        self.full_x = self.declare_parameter('full_scale_x', 2.0).value
        self.full_y = self.declare_parameter('full_scale_y', 1.0).value
        self.full_yaw = self.declare_parameter('full_scale_yaw', 1.5).value
        self.auto_setup = self.declare_parameter('auto_setup', False).value
        self.mode = self.declare_parameter('mode', 0).value
        self.motion_state = self.declare_parameter('motion_state', -1).value
        self.gait = self.declare_parameter('gait', -1).value
        self.enums_confirmed = self.declare_parameter('setup_enums_confirmed', False).value
        self.stand_state = self.declare_parameter('stand_motion_state', -1).value
        self.lie_state = self.declare_parameter('lie_motion_state', -1).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        for name, value in (('rate_hz', self.rate_hz), ('heartbeat_hz', self.heartbeat_hz),
                            ('watchdog_timeout', self.watchdog_timeout),
                            ('status_timeout', self.status_timeout),
                            ('full_scale_x', self.full_x), ('full_scale_y', self.full_y),
                            ('full_scale_yaw', self.full_yaw)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.transport not in ('udp', 'tcp'):
            raise ValueError('transport must be udp or tcp')
        if self.auto_setup and (not self.enums_confirmed or self.mode != 0 or
                                not self._enum_valid(self.motion_state) or not self._enum_valid(self.gait)):
            raise ValueError('auto_setup requires confirmed mode=0, motion_state and gait enums')
        self.gate = CommandGate(self.watchdog_timeout, 0.3, 0.15, 0.5, True)
        self.ok = False
        self.hes = self.mode_now = self.motion_now = -1
        self.status_t = None
        self.ox = self.oy = self.oyaw = 0.0
        self.last_odom_t = None
        self._rx_stop = threading.Event()
        self._rx_failed = threading.Event()
        self._rx_lock = threading.Lock()
        self._rx_newest = {}  # At most one BasicStatus and one MotionStatus.
        self._closed = False
        self._timers = []
        self._setup_index = 0
        self._setup_timer = None
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd, 10)
        self.act_sub = self.create_subscription(String, '/rs_follow/action_cmd', self._on_action, 10)
        self.estop_sub = self.create_subscription(Bool, '/rs_follow/estop', self._on_estop, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.stat_pub = self.create_publisher(String, '~/robot_status', 10)
        self._connect()
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()
        self._timers.append(self.create_timer(1.0 / self.heartbeat_hz, self._heartbeat))
        self._timers.append(self.create_timer(1.0 / self.rate_hz, self._tick))
        if self.auto_setup:
            self._setup_timer = self.create_timer(0.05, self._setup_once)
            self._timers.append(self._setup_timer)

    @staticmethod
    def _enum_valid(value):
        return type(value) is int and value >= 0

    def _connect(self):
        self.peer = (socket.gethostbyname(self.ip), self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if self.transport == 'udp' else socket.SOCK_STREAM)
        self.sock.settimeout(0.2)
        try:
            if self.transport == 'udp':
                self.sock.bind(('0.0.0.0', 0))
            else:
                self.sock.connect(self.peer)
        except OSError:
            self.sock.close()
            raise
        self.ok = True

    def _lose_readiness(self):
        self.ok = False
        self.status_t = None
        self.gate.output(time.monotonic(), False)

    def _send(self, payload):
        try:
            if self.transport == 'udp':
                self.sock.sendto(payload, self.peer)
            else:
                self.sock.sendall(payload)
        except OSError as exc:
            self._rx_failed.set()
            self._lose_readiness()
            self.get_logger().warning(f'send failed: {exc}', throttle_duration_sec=2.0)

    def _ready(self, now):
        return (self.ok and not self._rx_failed.is_set() and self.status_t is not None
                and 0 <= now - self.status_t <= self.status_timeout
                and self.hes == 0 and self.mode_now == 0)

    def _on_cmd(self, msg):
        now = time.monotonic()
        if not self._ready(now):
            self.gate.output(now, False)
            return
        self.gate.accept(msg.linear.x, msg.linear.y, msg.angular.z, now)

    def _on_estop(self, msg):
        self.gate.set_estop(msg.data)

    def _on_action(self, msg):
        action = msg.data.strip().lower()
        if action in STOP_ACTIONS:
            self.gate.set_estop(True)
        elif action in CLEAR_ACTIONS:
            self.gate.set_estop(False)
        elif action in ('standup', 'stand_up', 'stand', 'up', 'liedown', 'lie_down', 'down'):
            state = self.stand_state if action in ('standup', 'stand_up', 'stand', 'up') else self.lie_state
            if self.enums_confirmed and self._enum_valid(state) and self._ready(time.monotonic()):
                self.gate.output(time.monotonic(), False)
                self._send(P.set_motion_state(state))
            else:
                self.get_logger().warning('action rejected: requires fresh safe status and confirmed motion enum')

    def _heartbeat(self):
        if self.ok and not self._rx_failed.is_set():
            self._send(P.heartbeat())

    def _setup_once(self):
        if not self.ok or self._rx_failed.is_set():
            self._setup_timer.cancel()
            return
        packets = (P.set_mode(self.mode), P.set_motion_state(self.motion_state), P.set_gait(self.gait))
        self._send(packets[self._setup_index])
        self._setup_index += 1
        if self._setup_index == len(packets):
            self._setup_timer.cancel()

    def _tick(self):
        if self._rx_failed.is_set():
            self._lose_readiness()
        with self._rx_lock:
            newest = self._rx_newest
            self._rx_newest = {}
        if not self._rx_failed.is_set():
            for command, (msg, received) in newest.items():
                if command == 6:
                    self._on_status(msg['items'].get('BasicStatus'), received)
                elif command == 4:
                    self._on_motion(msg['items'].get('MotionStatus'), received)
        self._send_axis()

    def _send_axis(self):
        now = time.monotonic()
        vx, vy, wz = self.gate.output(now, self._ready(now))
        if self.ok and not self._rx_failed.is_set():
            self._send(P.axis_cmd(max(-1.0, min(1.0, vx / self.full_x)),
                                  max(-1.0, min(1.0, vy / self.full_y)),
                                  max(-1.0, min(1.0, wz / self.full_yaw))))

    def _recv_loop(self):
        decoder = P.StreamDecoder()
        try:
            while not self._rx_stop.is_set() and not self._rx_failed.is_set():
                try:
                    if self.transport == 'udp':
                        data, peer = self.sock.recvfrom(65535)
                        if peer != self.peer:
                            continue
                        msg = P.decode(data)
                        messages = [] if msg is None else [msg]
                    else:
                        data = self.sock.recv(65535)
                        if not data:
                            decoder.eof()
                            break
                        messages = decoder.feed(data)
                except socket.timeout:
                    continue
                received = time.monotonic()
                with self._rx_lock:
                    for msg in messages:
                        if msg['type'] == 1002 and msg['command'] in (4, 6):
                            self._rx_newest[msg['command']] = (msg, received)
        except (OSError, ValueError):
            pass
        finally:
            self._rx_failed.set()
            with self._rx_lock:
                self._rx_newest.clear()

    def _on_status(self, bs, received):
        if not isinstance(bs, dict) or type(bs.get('HES')) is not int or type(bs.get('ControlUsageMode')) is not int:
            self.status_t = None
            self.gate.output(time.monotonic(), False)
            return
        self.hes, self.mode_now = bs['HES'], bs['ControlUsageMode']
        self.motion_now = bs.get('MotionState', -1)
        self.status_t = received
        if not self._ready(time.monotonic()):
            self.gate.output(time.monotonic(), False)
        status = String()
        status.data = f'HES={self.hes} mode={self.mode_now} motion={self.motion_now} gait={bs.get("Gait", -1)}'
        self.stat_pub.publish(status)

    def _on_motion(self, ms, received):
        if not isinstance(ms, dict) or not 0 <= time.monotonic() - received <= self.status_timeout:
            return
        values = [ms.get(key) for key in ('Yaw', 'LinearX', 'LinearY', 'OmegaZ')]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            self.last_odom_t = None
            return
        yaw, vx, vy, wz = values
        dt = None if self.last_odom_t is None else received - self.last_odom_t
        self.last_odom_t = received
        if dt is not None and 0 < dt <= 1.0:
            self.ox += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            self.oy += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.oyaw = yaw
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.ox, self.oy
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vx, vy, wz
        # Feedback integration is not a measured localization estimate. No TF.
        for index in (0, 7, 14, 21, 28, 35):
            od.pose.covariance[index] = 1e6
            od.twist.covariance[index] = 1e6
        self.odom_pub.publish(od)

    def destroy_node(self):
        if not self._closed:
            self._closed = True
            for timer in self._timers:
                timer.cancel()
            self.gate.set_estop(True)
            for index in range(3):
                self._send(P.axis_cmd(0.0, 0.0, 0.0))
                if index < 2:
                    time.sleep(0.05)
            self._rx_stop.set()
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()
            self.recv_thread.join(timeout=1.0)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = M20Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
