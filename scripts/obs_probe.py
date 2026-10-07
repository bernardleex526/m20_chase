#!/usr/bin/env python3
"""Replicate the node's obstacle computation on the LIVE Gazebo cloud.

Prints which physical surface produces the clearance that trips the hard stop,
by re-implementing the same band split, self-occlusion box and support function
and reporting the nearest offending return in world terms.

Usage: obs_probe.py [seconds]
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

DTYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
         5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}

# must match run_scenarios.sh
SENSOR_H = 0.75          # MEASURED: the body settles, so the lidar ends at ~0.75
HEIGHT_MIN, HEIGHT_MAX = -0.40, 1.50
GROUND_CLEARANCE = 0.10
FRAME_FRONT = 0.31 + 0.08
FRAME_BACK = 0.31 + 0.08
FRAME_LEFT = 0.18 + 0.08
FRAME_RIGHT = 0.18 + 0.08
ROBOT_L, ROBOT_W = 0.62, 0.36
TARGET_RADIUS, TARGET_EXCL_SLACK = 0.60, 0.25
SECTOR = math.radians(60.0)
RANGE_MIN = 0.25


def dtype_of(msg):
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


class Probe(Node):
    def __init__(self):
        super().__init__("obs_probe")
        q = QoSProfile(depth=5)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(PointCloud2, "/rslidar_points", self.cb, q)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        a = np.frombuffer(msg.data, dtype=dtype_of(msg),
                          count=msg.width * msg.height)
        x = a['x'].astype(np.float64)
        y = a['y'].astype(np.float64)
        z = a['z'].astype(np.float64)
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[fin], y[fin], z[fin]
        r = np.hypot(x, y)
        az = np.arctan2(y, x)

        low_lo = -SENSOR_H + GROUND_CLEARANCE
        low_hi = HEIGHT_MIN - 0.05
        print(f"sensor_height={SENSOR_H}  low band=[{low_lo:+.2f},{low_hi:+.2f}]  "
              f"target band=[{HEIGHT_MIN:+.2f},{HEIGHT_MAX:+.2f}]")
        print(f"total {len(x)} returns; z {z.min():+.3f}..{z.max():+.3f} (sensor frame)")

        in_t = (z >= HEIGHT_MIN) & (z <= HEIGHT_MAX)
        in_l = (z >= low_lo) & (z <= low_hi)
        in_r = (r >= RANGE_MIN)
        print(f"  target band: {int(in_t.sum()):6d} pts   "
              f"low band: {int(in_l.sum()):6d} pts   r>={RANGE_MIN}: {int(in_r.sum())}")

        # Per-azimuth nearest, exactly like projectPointCloud
        def scan_of(mask):
            prof = np.full(1440, np.inf)
            if not mask.any():
                return prof
            idx = ((az[mask] + math.pi) / (2 * math.pi) * 1440).astype(int) % 1440
            np.minimum.at(prof, idx, r[mask])
            return prof

        tgt = scan_of(in_t & in_r)
        low = scan_of(in_l & in_r)

        # what is in the low band, in world terms?
        if (in_l & in_r).any():
            zz = z[in_l & in_r] + SENSOR_H
            rr = r[in_l & in_r]
            aa = np.degrees(az[in_l & in_r])
            print("\nLOW band returns (world z, range, azimuth):")
            order = np.argsort(rr)
            for i in order[:12]:
                print(f"   r={rr[i]:6.3f} m  az={aa[i]:+7.1f} deg  "
                      f"world_z={zz[i]:+.3f}  xy=({x[in_l&in_r][i]:+.2f},"
                      f"{y[in_l&in_r][i]:+.2f})")
            print(f"   ... {int((in_l&in_r).sum())} total, "
                  f"min r={rr.min():.3f}")

        # replicate the obstacle loop
        worst = []
        for i in range(1440):
            for prof, tag in ((tgt, "T"), (low, "L")):
                rf = prof[i]
                if not np.isfinite(rf):
                    continue
                a_ = -math.pi + (2 * math.pi) * (i + 0.5) / 1440
                px, py = rf * math.cos(a_), rf * math.sin(a_)
                in_frame = (-FRAME_BACK < px < FRAME_FRONT and
                            -FRAME_RIGHT < py < FRAME_LEFT)
                if in_frame:
                    continue
                sup = 0.5 * (ROBOT_L * abs(math.cos(a_)) + ROBOT_W * abs(math.sin(a_)))
                clr = max(0.0, rf - sup)
                worst.append((clr, rf, math.degrees(a_), tag, px, py))
        worst.sort()
        print("\nnearest obstacle clearances (what the governor sees):")
        for clr, rf, adeg, tag, px, py in worst[:10]:
            print(f"   clr={clr:6.3f} m  range={rf:6.3f}  az={adeg:+7.1f} deg  "
                  f"band={tag}  sensor_xy=({px:+.2f},{py:+.2f})")
        if worst:
            print(f"\n  --> min clearance {worst[0][0]:.3f} m "
                  f"(hard stop if < 0.25)  from band {worst[0][3]} "
                  f"at {worst[0][2]:+.1f} deg")
        # front sector only
        fs = [w for w in worst if abs(math.radians(w[2])) <= SECTOR]
        if fs:
            print(f"  --> front-sector clearance {fs[0][0]:.3f} m "
                  f"at {fs[0][2]:+.1f} deg (band {fs[0][3]})")


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    p = Probe()
    t0 = time.time()
    while not p.done and time.time() - t0 < secs:
        rclpy.spin_once(p, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
