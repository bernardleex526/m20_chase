#!/usr/bin/env python3
"""Diagnose the dual-band projection: where do returns land in each band?

Publishes nothing; only subscribes. Prints, for the live Gazebo world, the
z-histogram of raw returns and the nearest return in the TARGET band vs the
LOW band, so a "phantom obstacle" can be traced to a physical surface.
"""
import math
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

LIDAR_Z = 0.95   # world height of the lidar in all_scenarios.sdf
DTYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
         5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        dt = DTYPE.get(f.datatype)
        if dt is None:
            continue
        formats.append((dt, f.count) if f.count > 1 else dt)
        names.append(f.name)
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats,
                     'offsets': offsets, 'itemsize': msg.point_step})


class Diag(Node):
    def __init__(self):
        super().__init__("band_diag")
        q = QoSProfile(depth=5)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(PointCloud2, "/rslidar_points", self.cb, q)
        self.n = 0

    def cb(self, msg):
        if self.n > 0:
            return
        self.n += 1
        arr = np.frombuffer(msg.data, dtype=cloud_dtype(msg),
                            count=msg.width * msg.height)
        x = arr['x'].astype(np.float64)
        y = arr['y'].astype(np.float64)
        z = arr['z'].astype(np.float64)
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[fin], y[fin], z[fin]
        r = np.hypot(x, y)
        print(f"total returns {len(x)}   frame_id={msg.header.frame_id}")
        print(f"z range {z.min():+.3f} .. {z.max():+.3f}  (sensor frame)")

        bins = [-2.0, -1.2, -0.9, -0.75, -0.6, -0.45, -0.3, 0.0, 0.5, 1.5, 3.0]
        h, _ = np.histogram(z, bins=bins)
        print("z histogram (sensor frame, world = z + %.2f):" % LIDAR_Z)
        for i, c in enumerate(h):
            lo, hi = bins[i], bins[i + 1]
            print(f"  [{lo:+.2f},{hi:+.2f})  world[{lo+LIDAR_Z:+.2f},{hi+LIDAR_Z:+.2f})"
                  f"  {c:7d}  {'#' * min(50, c // 200)}")

        for name, zmin, zmax in (("TARGET band", -0.40, 1.50),
                                 ("LOW band   ", -0.75, -0.45)):
            m = (z >= zmin) & (z <= zmax)
            if not m.any():
                print(f"{name}: no returns")
                continue
            rr = r[m]
            # nearest per 1-degree azimuth bin
            az = np.degrees(np.arctan2(y[m], x[m]))
            prof = np.full(360, np.inf)
            bi = ((az + 180.0) % 360.0).astype(int) % 360
            np.minimum.at(prof, bi, rr)
            k = int(np.argmin(prof))
            a = k - 180.0 + 0.5
            print(f"{name}: {int(m.sum()):6d} pts  occupied {int(np.isfinite(prof).sum()):3d}/360 bins"
                  f"  nearest {prof[k]:.3f} m at azimuth {a:+.1f} deg"
                  f"  -> world z of that return: "
                  f"{np.median(z[m][rr < prof[k] + 0.05]) + LIDAR_Z:+.2f}")
            # list the 5 nearest bins
            order = np.argsort(prof)[:5]
            s = ", ".join(f"{o-180+0.5:+.0f}deg:{prof[o]:.2f}m"
                          for o in order if np.isfinite(prof[o]))
            print(f"            nearest bins: {s}")


def main():
    rclpy.init()
    n = Diag()
    t0 = time.time()
    while n.n == 0 and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
