#!/usr/bin/env python3
"""Obstacle-placement matrix against the live rs_follow_node.

Question: is the APF emergency stop *directional* (forward cone / path-based) or
*omnidirectional* (nearest return anywhere outside the self-occlusion frame)?

Gazebo cannot place obstacles this precisely or this fast, so this bypasses
Gazebo entirely: it publishes synthetic XYZ PointCloud2 frames containing
(a) a person-sized cluster at (3.0, 0.0) which is bound as the target, and
(b) one small obstacle cluster at a chosen (ox, oy).

For every placement it records /cmd_vel and /rs_follow/status.

Usage: python3 obs_matrix.py
"""
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, String

HZ = 20.0
CASE_SECS = 2.0   # per placement
WARMUP = 1.5      # target bind + kalman settle before cases

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]

TARGET_XY = (3.0, 0.0)

# (label, ox, oy)   ox>0 forward, oy>0 left.  None => no obstacle at all.
CASES = [
    ("none",            None,   None),
    ("ahead_0.30",      0.30,   0.00),
    ("ahead_0.34",      0.34,   0.00),
    ("ahead_0.40",      0.40,   0.00),
    ("ahead_0.80",      0.80,   0.00),
    ("behind_-0.30",   -0.30,   0.00),
    ("behind_-0.60",   -0.60,   0.00),
    ("left_0.30",       0.00,   0.30),
    ("left_0.40",       0.00,   0.40),
    ("left_0.60",       0.00,   0.60),
    ("left_1.50",       0.00,   1.50),
    ("right_-0.30",     0.00,  -0.30),
    ("diag_0.21_0.21",  0.21,   0.21),
    ("rearleft_-0.21", -0.21,   0.21),
]


def cluster(cx, cy, r, z0, z1, n, rng):
    """Small vertical cylindrical blob of returns at (cx, cy)."""
    th = rng.uniform(-np.pi, np.pi, n)
    rr = rng.uniform(0.0, r, n)
    x = cx + rr * np.cos(th)
    y = cy + rr * np.sin(th)
    z = rng.uniform(z0, z1, n)
    return x.astype(np.float32), y.astype(np.float32), z.astype(np.float32)


def make_cloud(obstacle, rng, n_target=240, n_obs=90):
    xs, ys, zs = [], [], []
    # the bound target: person-sized cylinder, spanning the height band
    x, y, z = cluster(TARGET_XY[0], TARGET_XY[1], 0.15, 0.10, 1.00, n_target, rng)
    xs.append(x); ys.append(y); zs.append(z)
    if obstacle is not None:
        ox, oy = obstacle
        x, y, z = cluster(ox, oy, 0.05, 0.10, 0.80, n_obs, rng)
        xs.append(x); ys.append(y); zs.append(z)
    x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
    n = len(x)
    buf = np.empty(n * 3, dtype=np.float32)
    buf[0::3], buf[1::3], buf[2::3] = x, y, z
    return n, buf.tobytes()


class Matrix(Node):
    def __init__(self):
        super().__init__("obs_matrix")
        self.pub = self.create_publisher(PointCloud2, "/rslidar_points", 10)
        self.bind_pub = self.create_publisher(PointStamped, "/rs_follow/bind_target", 10)
        self.en_pub = self.create_publisher(Bool, "/rs_follow/enable", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "/rs_follow/status", self.on_status, 10)
        self.cmd = (0.0, 0.0, 0.0)
        self.status = "?"
        self.n_cmd = 0

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.n_cmd += 1

    def on_status(self, m):
        self.status = m.data

    def publish_cloud(self, obstacle, rng):
        n, data = make_cloud(obstacle, rng)
        m = PointCloud2()
        m.header = Header()
        m.header.frame_id = "rslidar"
        m.header.stamp = self.get_clock().now().to_msg()
        m.height = 1
        m.width = n
        m.fields = FIELDS
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * n
        m.is_dense = True
        m.data = data
        self.pub.publish(m)


def main():
    rclpy.init()
    node = Matrix()
    rng = np.random.default_rng(11)

    def pump(seconds, obstacle):
        t_end = time.time() + seconds
        next_pub = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= next_pub:
                node.publish_cloud(obstacle, rng)
                next_pub = now + 1.0 / HZ
            rclpy.spin_once(node, timeout_sec=0.01)

    # --- bind the target and enable ---
    pump(0.5, None)
    p = PointStamped()
    p.header.frame_id = "rslidar"
    p.point.x, p.point.y = TARGET_XY
    for _ in range(5):
        p.header.stamp = node.get_clock().now().to_msg()
        node.bind_pub.publish(p)
        time.sleep(0.08)
    node.en_pub.publish(Bool(data=True))
    time.sleep(0.3)
    print(f"bound target at {TARGET_XY}, warming up {WARMUP}s ...")
    pump(WARMUP, None)
    print(f"warmup status={node.status} cmd={node.cmd}\n")

    print(f"{'case':<18} {'ox':>6} {'oy':>6} {'range':>7} "
          f"{'status':<16} {'vx':>7} {'vy':>7} {'wz':>7}  nz/n")
    print("-" * 96)
    for label, ox, oy in CASES:
        obs = None if ox is None else (ox, oy)
        rng2 = np.random.default_rng(11)
        node.n_cmd = 0
        # let the field settle for this placement, then measure the tail
        pump(CASE_SECS * 0.5, obs)
        node.n_cmd = 0
        samples = []
        t_end = time.time() + CASE_SECS
        next_pub = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= next_pub:
                node.publish_cloud(obs, rng2)
                next_pub = now + 1.0 / HZ
            rclpy.spin_once(node, timeout_sec=0.01)
            samples.append((node.cmd, node.status))
        nz = sum(1 for c, s in samples
                 if abs(c[0]) > 1e-6 or abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)
        vx = float(np.mean([c[0] for c, _ in samples]))
        vy = float(np.mean([c[1] for c, _ in samples]))
        wz = float(np.mean([c[2] for c, _ in samples]))
        st = max(set(s for _, s in samples), key=[s for _, s in samples].count)
        rngv = float(np.hypot(ox, oy)) if ox is not None else float("nan")
        rngs = f"{rngv:7.3f}" if ox is not None else "      -"
        print(f"{label:<18} {('-' if ox is None else f'{ox:6.2f}'):>6} "
              f"{('-' if oy is None else f'{oy:6.2f}'):>6} {rngs} "
              f"{st:<16} {vx:7.3f} {vy:7.3f} {wz:7.3f}  {nz}/{len(samples)}")

    node.en_pub.publish(Bool(data=False))
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
