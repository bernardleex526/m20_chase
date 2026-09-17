#!/usr/bin/env python3
"""
m20_bridge — connect rs_follow (/cmd_vel) to a DEEP Robotics Lynx M20 / M20 Pro.

The M20 external motion control uses the ``basic_server`` TCP/UDP protocol
(Type 2 / Command 21 axis commands, values are a ratio of max speed). This node

  * maintains a heartbeat (Type 100/100) so the robot reports status
  * switches to Regular mode + a motion state + gait at startup
  * converts /cmd_vel (m/s, rad/s) to axis ratios X/Y/Yaw at 20 Hz
  * has a watchdog: no /cmd_vel (or hard e-stop / wrong mode) -> send zeros
  * integrates the robot's reported velocities into nav_msgs/Odometry and
    publishes it, so rs_follow can use it for slip compensation and
    inertial-frame target filtering

Parameters:
  transport (udp|tcp), ip, port
  cmd_vel_topic, odom_topic
  rate_hz (20), heartbeat_hz (2), watchdog_timeout (0.5)
  full_scale_x (2.0 m/s per axis=1), full_scale_y (1.0), full_scale_yaw (1.5 rad/s)
  auto_setup (true), mode (0), motion_state (1 stand / 17 RL), gait (0x1001)
  odom_frame, base_frame
"""

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

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
        self.watchdog_timeout = self.declare_parameter('watchdog_timeout', 0.5).value
        self.full_x = self.declare_parameter('full_scale_x', 2.0).value
        self.full_y = self.declare_parameter('full_scale_y', 1.0).value
        self.full_yaw = self.declare_parameter('full_scale_yaw', 1.5).value
        self.auto_setup = self.declare_parameter('auto_setup', True).value
        self.mode = self.declare_parameter('mode', 0).value
        self.motion_state = self.declare_parameter('motion_state', 1).value
        self.gait = self.declare_parameter('gait', 0x1001).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_t = 0.0
        self.seq = 0
        self.ok = False
        self.hes = 0
        self.mode_now = -1
        self.motion_now = -1
        self.sock = None

        # odom integration
        self.ox = self.oy = self.oyaw = 0.0
        self.last_odom_t = time.time()

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd, 10)
        self.act_sub = self.create_subscription(String, '/rs_follow/action_cmd', self._on_action, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.stat_pub = self.create_publisher(String, '~/robot_status', 10)

        self._connect()
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        self.create_timer(1.0 / self.heartbeat_hz, self._heartbeat)
        self.create_timer(1.0 / self.rate_hz, self._send_axis)
        self._setup_timer = None
        if self.auto_setup:
            self._setup_timer = self.create_timer(1.0, self._setup_once)

        self.get_logger().info(
            f'm20_bridge: {self.transport} {self.ip}:{self.port} | cmd="{self.cmd_vel_topic}" '
            f'-> axis @{self.rate_hz:.0f}Hz | odom -> "{self.odom_topic}"')

    # -- transport ----------------------------------------------------------- #
    def _connect(self):
        if self.transport == 'udp':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('0.0.0.0', 0))
            self.sock.settimeout(0.2)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            self.sock.settimeout(0.2)
        self.ok = True

    def _send(self, payload):
        try:
            if self.transport == 'udp':
                self.sock.sendto(payload, (self.ip, self.port))
            else:
                self.sock.sendall(payload)
        except OSError as exc:
            self.ok = False
            self.get_logger().warn(f'send failed: {exc}', throttle_duration_sec=2.0)

    # -- subscriptions ------------------------------------------------------- #
    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_t = time.time()

    def _on_action(self, msg):
        a = msg.data.strip().lower()
        if a in ('liedown', 'lie_down', 'down'):
            self._send(P.set_motion_state(4))
        elif a in ('standup', 'stand_up', 'stand', 'up'):
            self._send(P.set_motion_state(1))
        elif a in ('softstop', 'estop', 'stop'):
            self._send(P.set_motion_state(2))
        self.get_logger().info(f'action -> {a}')

    # -- periodic ------------------------------------------------------------ #
    def _heartbeat(self):
        self._send(P.heartbeat())

    def _setup_once(self):
        self._send(P.set_mode(self.mode))
        time.sleep(0.05)
        self._send(P.set_motion_state(self.motion_state))
        time.sleep(0.05)
        self._send(P.set_gait(self.gait))
        self.get_logger().info(
            f'setup sent: mode={self.mode} motion_state={self.motion_state} gait=0x{self.gait:x}')
        if self._setup_timer is not None:
            self._setup_timer.cancel()
            self._setup_timer = None

    def _send_axis(self):
        stale = (time.time() - self.last_cmd_t) > self.watchdog_timeout
        safe = stale or self.hes or (self.mode_now not in (-1, 0))
        vx, vy, wz = (0.0, 0.0, 0.0) if safe else self.cmd
        x = max(-1.0, min(1.0, vx / self.full_x))
        y = max(-1.0, min(1.0, vy / self.full_y))
        yaw = max(-1.0, min(1.0, wz / self.full_yaw))
        self._send(P.axis_cmd(x, y, yaw))

    # -- receive / status ---------------------------------------------------- #
    def _recv_loop(self):
        while rclpy.ok():
            try:
                data = self.sock.recv(65535) if self.transport == 'udp' \
                    else self.sock.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.2)
                continue
            msg = P.decode(data)
            if msg is None:
                continue
            if msg['type'] == 1002 and msg['command'] == 6:
                bs = msg['items'].get('BasicStatus', {})
                self.hes = int(bs.get('HES', 0))
                self.mode_now = int(bs.get('ControlUsageMode', -1))
                self.motion_now = int(bs.get('MotionState', -1))
                s = String()
                s.data = (f'HES={self.hes} mode={self.mode_now} motion={self.motion_now} '
                          f'gait=0x{int(bs.get("Gait", 0)):x}')
                self.stat_pub.publish(s)
            elif msg['type'] == 1002 and msg['command'] == 4:
                self._on_motion(msg['items'].get('MotionStatus', {}))

    def _on_motion(self, ms):
        now = time.time()
        dt = now - self.last_odom_t
        self.last_odom_t = now
        if dt <= 0 or dt > 1.0:
            dt = 0.1
        yaw = float(ms.get('Yaw', self.oyaw))
        vx = float(ms.get('LinearX', 0.0))
        vy = float(ms.get('LinearY', 0.0))
        wz = float(ms.get('OmegaZ', 0.0))
        self.ox += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.oy += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.oyaw = yaw

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.ox
        od.pose.pose.position.y = self.oy
        od.pose.pose.orientation.z = math.sin(yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(yaw / 2.0)
        od.twist.twist.linear.x = vx
        od.twist.twist.linear.y = vy
        od.twist.twist.angular.z = wz
        self.odom_pub.publish(od)


def main():
    rclpy.init()
    node = M20Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
