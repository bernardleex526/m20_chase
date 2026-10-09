#!/usr/bin/env python3
"""Is rs_follow's obstacle avoidance directional, and what is its real envelope?

Two independent groups, both against the live node with synthetic clouds so
placement is exact:

GROUP 1 - directional matrix. The bound target sits at (3.0, 0.0). Every
obstacle is placed OFF the target's azimuth line (|azimuth| >= ~40 deg) so it
cannot occlude the target in the 2D projection. The target is cleared and
re-bound before every case, so target loss cannot contaminate a later case.

GROUP 2 - target-as-obstacle coupling. Bind a target very close (0.45 m) and
check whether the followed person's own returns trip the emergency bubble.

Usage: python3 obs_matrix2.py
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header, String

HZ = 20.0
SETTLE = 0.8     # let the field/scan settle for this placement
MEASURE = 0.9    # measurement tail (< lost_frames_timeout at 10 Hz scan rate)

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]

# GROUP 1: (label, target_xy, obstacle_xy or None)
# Obstacles are off the target's azimuth so the target stays visible.
G1 = [
    ("baseline_no_obs",   (3.0, 0.0), None),
    ("right_0.30",        (3.0, 0.0), (0.00, -0.30)),
    ("right_0.34",        (3.0, 0.0), (0.00, -0.34)),
    ("right_0.40",        (3.0, 0.0), (0.00, -0.40)),
    ("right_0.60",        (3.0, 0.0), (0.00, -0.60)),
    ("right_1.50",        (3.0, 0.0), (0.00, -1.50)),
    ("left_0.30",         (3.0, 0.0), (0.00,  0.30)),
    ("left_0.40",         (3.0, 0.0), (0.00,  0.40)),
    ("left_0.60",         (3.0, 0.0), (0.00,  0.60)),
    ("left_1.50",         (3.0, 0.0), (0.00,  1.50)),
    ("rear_0.30",         (3.0, 0.0), (-0.30, 0.00)),
    ("rear_0.40",         (3.0, 0.0), (-0.40, 0.00)),
    ("rear_0.60",         (3.0, 0.0), (-0.60, 0.00)),
    ("rear_1.50",         (3.0, 0.0), (-1.50, 0.00)),
    ("rearleft_0.297",    (3.0, 0.0), (-0.21, 0.21)),
    ("rearright_0.297",   (3.0, 0.0), (-0.21, -0.21)),
    ("frontleft_0.30",    (3.0, 0.0), (0.21,  0.21)),
    ("frontright_0.30",   (3.0, 0.0), (0.21, -0.21)),
    ("beside_body_0.28",  (3.0, 0.0), (0.10,  0.28)),
]

# GROUP 2: (label, target_xy, obstacle_xy)
G2 = [
    ("target_at_1.0",  (1.0, 0.0), None),
    ("target_at_0.70", (0.7, 0.0), None),
    ("target_at_0.55", (0.55, 0.0), None),
    ("target_at_0.45", (0.45, 0.0), None),
    ("target_at_0.40", (0.4, 0.0), None),
]


def cluster(cx, cy, r, z0, z1, n, rng):
    th = rng.uniform(-np.pi, np.pi, n)
    rr = rng.uniform(0.0, r, n)
    return (cx + rr * np.cos(th)).astype(np.float32), \
           (cy + rr * np.sin(th)).astype(np.float32), \
           rng.uniform(z0, z1, n).astype(np.float32)


def make_cloud(target_xy, obstacle, n_target=240, n_obs=90):
    rng = np.random.default_rng(11)   # deterministic every frame
    xs, ys, zs = [], [], []
    x, y, z = cluster(target_xy[0], target_xy[1], 0.15, 0.10, 1.00, n_target, rng)
    xs.append(x); ys.append(y); zs.append(z)
    if obstacle is not None:
        x, y, z = cluster(obstacle[0], obstacle[1], 0.05, 0.10, 0.80, n_obs, rng)
        xs.append(x); ys.append(y); zs.append(z)
    x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
    n = len(x)
    buf = np.empty(n * 3, dtype=np.float32)
    buf[0::3], buf[1::3], buf[2::3] = x, y, z
    return n, buf.tobytes()


class Matrix(Node):
    def __init__(self):
        super().__init__("obs_matrix2")
        self.pub = self.create_publisher(PointCloud2, "/rslidar_points", 10)
        self.bind = self.create_publisher(PointStamped, "/rs_follow/bind_target", 10)
        self.clear = self.create_publisher(Bool, "/rs_follow/clear_target", 10)
        self.en = self.create_publisher(Bool, "/rs_follow/enable", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "/rs_follow/status", self.on_status, 10)
        self.cmd = (0.0, 0.0, 0.0)
        self.status = "?"
        self.tgt = (float("nan"), float("nan"))

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def on_status(self, m):
        self.status = m.data

    def send_cloud(self, target_xy, obstacle):
        n, data = make_cloud(target_xy, obstacle)
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

    def pump(self, seconds, target_xy, obstacle):
        t_end = time.time() + seconds
        nxt = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= nxt:
                self.send_cloud(target_xy, obstacle)
                nxt = now + 1.0 / HZ
            rclpy.spin_once(self, timeout_sec=0.005)

    def prepare(self, target_xy, obstacle):
        """Clear, re-bind, enable, then settle for this case."""
        self.pump(0.30, target_xy, obstacle)
        self.clear.publish(Bool(data=True))
        self.pump(0.20, target_xy, obstacle)
        p = PointStamped()
        p.header.frame_id = "rslidar"
        p.point.x, p.point.y = target_xy
        for _ in range(4):
            p.header.stamp = self.get_clock().now().to_msg()
            self.bind.publish(p)
            self.pump(0.06, target_xy, obstacle)
        self.en.publish(Bool(data=True))
        self.pump(SETTLE, target_xy, obstacle)

    def measure(self, target_xy, obstacle):
        samples = []
        t_end = time.time() + MEASURE
        nxt = 0.0
        while time.time() < t_end:
            now = time.time()
            if now >= nxt:
                self.send_cloud(target_xy, obstacle)
                nxt = now + 1.0 / HZ
            rclpy.spin_once(self, timeout_sec=0.005)
            samples.append((self.cmd, self.status))
        nz = sum(1 for c, _ in samples
                 if abs(c[0]) > 1e-6 or abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)
        em = sum(1 for _, s in samples if s == "EMERGENCY_STOP")
        vx = float(np.mean([c[0] for c, _ in samples]))
        vy = float(np.mean([c[1] for c, _ in samples]))
        wz = float(np.mean([c[2] for c, _ in samples]))
        st = max(set(s for _, s in samples), key=[s for _, s in samples].count)
        return vx, vy, wz, st, nz, em, len(samples)


def main():
    rclpy.init()
    node = Matrix()
    node.en.publish(Bool(data=True))

    for title, cases in (("GROUP 1 - directional matrix "
                          "(target always at given xy, obstacle off-axis)", G1),
                         ("GROUP 2 - target-as-obstacle coupling "
                          "(no obstacle at all)", G2)):
        print(f"\n=== {title} ===")
        print(f"{'case':<20} {'ox':>6} {'oy':>6} {'rng':>6} "
              f"{'status':<16} {'vx':>7} {'vy':>7} {'wz':>7} {'nz/n':>8} {'EM/n':>8}")
        print("-" * 108)
        for label, txy, obs in cases:
            node.prepare(txy, obs)
            vx, vy, wz, st, nz, em, n = node.measure(txy, obs)
            ox = "-" if obs is None else f"{obs[0]:6.2f}"
            oy = "-" if obs is None else f"{obs[1]:6.2f}"
            rngv = "-" if obs is None else f"{np.hypot(*obs):6.3f}"
            print(f"{label:<20} {ox:>6} {oy:>6} {rngv:>6} "
                  f"{st:<16} {vx:7.3f} {vy:7.3f} {wz:7.3f} "
                  f"{nz:>4}/{n:<3} {em:>4}/{n:<3}")
            # keep the target for the next baseline-ish case only
        node.clear.publish(Bool(data=True))
        node.pump(0.2, (3.0, 0.0), None)

    node.en.publish(Bool(data=False))
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
