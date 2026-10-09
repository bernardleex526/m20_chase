#!/usr/bin/env python3
"""Follow-up probes for the obstacle-avoidance question.

GROUP 3 - occlusion: an obstacle is placed ON the target's line of sight but
OUTSIDE the 0.35 m emergency bubble (1.0-2.5 m away). The 2D projection keeps
only the NEAREST return per azimuth bin, so the obstacle should steal the
target's bins. Does the target stay locked, and how far does the estimate drift?

GROUP 4 - symmetric pair / gap: obstacles on both sides at a forward distance,
forming a gap the dog would have to thread.

GROUP 5 - stop margin: bind the target and place an obstacle just outside the
bubble, then creep it inward, to bracket the exact trip radius.

Usage: python3 obs_matrix3.py
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, String

HZ = 20.0
SETTLE = 0.8
MEASURE = 0.9

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]

G3 = [
    ("no_occl",          (3.0, 0.0), None),
    ("occl_at_1.0",      (3.0, 0.0), (1.0, 0.0)),
    ("occl_at_1.5",      (3.0, 0.0), (1.5, 0.0)),
    ("occl_at_2.0",      (3.0, 0.0), (2.0, 0.0)),
    ("occl_at_2.5",      (3.0, 0.0), (2.5, 0.0)),
    ("occl_at_2.8",      (3.0, 0.0), (2.8, 0.0)),
]

G4 = [
    ("gap_0.60_fwd_0.30", (3.0, 0.0), [(0.30, 0.30), (0.30, -0.30)]),
    ("gap_0.60_fwd_0.50", (3.0, 0.0), [(0.50, 0.30), (0.50, -0.30)]),
    ("gap_1.00_fwd_1.00", (3.0, 0.0), [(1.00, 0.50), (1.00, -0.50)]),
]

G5 = [
    ("obs_0.50", (3.0, 0.0), (0.50, 0.0)),
    ("obs_0.45", (3.0, 0.0), (0.45, 0.0)),
    ("obs_0.40", (3.0, 0.0), (0.40, 0.0)),
    ("obs_0.37", (3.0, 0.0), (0.37, 0.0)),
    ("obs_0.35", (3.0, 0.0), (0.35, 0.0)),
    ("obs_0.33", (3.0, 0.0), (0.33, 0.0)),
]


def cluster(cx, cy, r, z0, z1, n, rng):
    th = rng.uniform(-np.pi, np.pi, n)
    rr = rng.uniform(0.0, r, n)
    return (cx + rr * np.cos(th)).astype(np.float32), \
           (cy + rr * np.sin(th)).astype(np.float32), \
           rng.uniform(z0, z1, n).astype(np.float32)


def make_cloud(target_xy, obstacles, n_target=240, n_obs=90):
    rng = np.random.default_rng(11)
    xs, ys, zs = [], [], []
    x, y, z = cluster(target_xy[0], target_xy[1], 0.15, 0.10, 1.00, n_target, rng)
    xs.append(x); ys.append(y); zs.append(z)
    for ob in (obstacles or []):
        x, y, z = cluster(ob[0], ob[1], 0.05, 0.10, 0.80, n_obs, rng)
        xs.append(x); ys.append(y); zs.append(z)
    x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
    n = len(x)
    buf = np.empty(n * 3, dtype=np.float32)
    buf[0::3], buf[1::3], buf[2::3] = x, y, z
    return n, buf.tobytes()


class Matrix(Node):
    def __init__(self):
        super().__init__("obs_matrix3")
        self.pub = self.create_publisher(PointCloud2, "/rslidar_points", 10)
        self.bind = self.create_publisher(PointStamped, "/rs_follow/bind_target", 10)
        self.clear = self.create_publisher(Bool, "/rs_follow/clear_target", 10)
        self.en = self.create_publisher(Bool, "/rs_follow/enable", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "/rs_follow/status", self.on_status, 10)
        self.create_subscription(PointStamped, "/rs_follow/target", self.on_target, 10)
        self.cmd = (0.0, 0.0, 0.0)
        self.status = "?"
        self.est = (float("nan"), float("nan"))

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def on_status(self, m):
        self.status = m.data

    def on_target(self, m):
        self.est = (m.point.x, m.point.y)

    def send_cloud(self, target_xy, obstacles):
        n, data = make_cloud(target_xy, obstacles)
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

    def pump(self, seconds, target_xy, obstacles):
        t_end = time.time() + seconds
        nxt = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= nxt:
                self.send_cloud(target_xy, obstacles)
                nxt = now + 1.0 / HZ
            rclpy.spin_once(self, timeout_sec=0.005)

    def prepare(self, target_xy, obstacles):
        self.pump(0.30, target_xy, obstacles)
        self.clear.publish(Bool(data=True))
        self.pump(0.20, target_xy, obstacles)
        p = PointStamped()
        p.header.frame_id = "rslidar"
        p.point.x, p.point.y = target_xy
        for _ in range(4):
            p.header.stamp = self.get_clock().now().to_msg()
            self.bind.publish(p)
            self.pump(0.06, target_xy, obstacles)
        self.en.publish(Bool(data=True))
        self.pump(SETTLE, target_xy, obstacles)

    def measure(self, target_xy, obstacles):
        ests, cmds, sts = [], [], []
        t_end = time.time() + MEASURE
        nxt = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= nxt:
                self.send_cloud(target_xy, obstacles)
                nxt = now + 1.0 / HZ
            rclpy.spin_once(self, timeout_sec=0.005)
            ests.append(self.est)
            cmds.append(self.cmd)
            sts.append(self.status)
        nz = sum(1 for c in cmds
                 if abs(c[0]) > 1e-6 or abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)
        em = sum(1 for s in sts if s == "EMERGENCY_STOP")
        nt = sum(1 for s in sts if s == "NO_TARGET")
        vx = float(np.mean([c[0] for c in cmds]))
        vy = float(np.mean([c[1] for c in cmds]))
        wz = float(np.mean([c[2] for c in cmds]))
        er = np.nanmean([np.hypot(e[0], e[1]) for e in ests])
        st = max(set(sts), key=sts.count)
        return vx, vy, wz, st, nz, em, nt, er, len(sts)


def main():
    rclpy.init()
    node = Matrix()
    node.en.publish(Bool(data=True))

    groups = [
        ("GROUP 3 - occlusion (obstacle on the sight line, outside the bubble)", G3),
        ("GROUP 4 - symmetric pairs / gap", G4),
        ("GROUP 5 - exact trip radius bracket", G5),
    ]
    for title, cases in groups:
        print(f"\n=== {title} ===")
        print(f"{'case':<22} {'status':<16} {'vx':>7} {'vy':>7} {'wz':>7} "
              f"{'est_rng':>8} {'nz/n':>8} {'EM/n':>7} {'NO/n':>7}")
        print("-" * 110)
        for label, txy, obs in cases:
            obstacles = None if obs is None else (
                obs if isinstance(obs, list) else [obs])
            node.prepare(txy, obstacles)
            vx, vy, wz, st, nz, em, nt, er, n = node.measure(txy, obstacles)
            print(f"{label:<22} {st:<16} {vx:7.3f} {vy:7.3f} {wz:7.3f} "
                  f"{er:8.3f} {nz:>4}/{n:<3} {em:>3}/{n:<3} {nt:>3}/{n:<3}")
        node.clear.publish(Bool(data=True))
        node.pump(0.2, (3.0, 0.0), None)

    node.en.publish(Bool(data=False))
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
