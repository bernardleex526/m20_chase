#!/usr/bin/env python3
"""Throughput benchmark for rs_follow's point-cloud projection.

Gazebo's ray sensor emits only ~6 k points, but a real RoboSense RS-Helios /
Ruby emits 32-128 k points at 10 Hz. This publishes synthetic XYZ PointCloud2
frames of a configurable size straight at the node so the measured CPU reflects
the real sensor load rather than the simulator's.

Usage: python3 bench_cloud.py <n_points> <seconds> [hz]
"""
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

N = int(sys.argv[1]) if len(sys.argv) > 1 else 65536
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


class Bench(Node):
    def __init__(self):
        super().__init__("bench_cloud")
        self.pub = self.create_publisher(PointCloud2, "/rslidar_points", 10)
        rng = np.random.default_rng(7)

        # A realistic-ish scene: a ring of returns on a ground plane plus a
        # person-sized cylinder 3 m ahead. Keeps the height-band filter and the
        # clustering path exercised (not just the fast reject).
        a = rng.uniform(-np.pi, np.pi, N).astype(np.float32)
        r = rng.uniform(0.5, 30.0, N).astype(np.float32)
        x = r * np.cos(a)
        y = r * np.sin(a)
        z = rng.uniform(-1.7, 0.2, N).astype(np.float32)     # ground ring
        # inject a dense person cluster
        m = N // 50
        th = rng.uniform(-np.pi, np.pi, m).astype(np.float32)
        rr = rng.uniform(0.10, 0.30, m).astype(np.float32)
        x[:m] = 3.0 + rr * np.cos(th)
        y[:m] = 0.0 + rr * np.sin(th)
        z[:m] = rng.uniform(-1.3, 0.6, m).astype(np.float32)

        buf = np.empty(N * 3, dtype=np.float32)
        buf[0::3], buf[1::3], buf[2::3] = x, y, z
        self.data = buf.tobytes()
        self.msg = PointCloud2()
        self.msg.header = Header()
        self.msg.header.frame_id = "rslidar"
        self.msg.height = 1
        self.msg.width = N
        self.msg.fields = FIELDS
        self.msg.is_bigendian = False
        self.msg.point_step = 12
        self.msg.row_step = 12 * N
        self.msg.is_dense = True
        self.n = 0
        self.create_timer(1.0 / HZ, self.tick)
        self.t_end = time.time() + SECS
        self.get_logger().info(f"publishing {N} pts @ {HZ} Hz for {SECS}s")

    def tick(self):
        if time.time() > self.t_end:
            raise KeyboardInterrupt
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.data = self.data
        self.pub.publish(self.msg)
        self.n += 1


def main():
    rclpy.init()
    b = Bench()
    try:
        rclpy.spin(b)
    except KeyboardInterrupt:
        pass
    print(f"published {b.n} frames of {N} points")
    b.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
