#!/usr/bin/env python3
"""
pointcloud_status.py — RoboSense point-cloud health report (CLI / SSH friendly).

Subscribes to a PointCloud2 topic for a few seconds and prints:
  - topic / frame / stamp / field layout
  - publish rate (Hz) and points per frame
  - valid-return ratio, NaN ratio
  - x/y/z extent, horizontal range statistics
  - z histogram (ground / body / ceiling), forward-cone density

Usage:
  ros2 run rs_follow pointcloud_status.py --topic /rslidar_points --duration 5
  python3 pointcloud_status.py --topic /rslidar_points
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

DTYPE = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def cloud_dtype(msg: PointCloud2) -> np.dtype:
    names, formats, offsets = [], [], []
    for f in msg.fields:
        dt = DTYPE.get(f.datatype)
        if dt is None:
            continue
        formats.append((dt, f.count) if f.count > 1 else dt)
        names.append(f.name)
        offsets.append(f.offset)
    return np.dtype(
        {'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})


class Status(Node):
    def __init__(self, topic, duration, sample_stride):
        super().__init__('pointcloud_status')
        self.duration = duration
        self.sample_stride = sample_stride
        self.n = 0
        self.t0 = None
        self.t_last = None
        self.header = None
        self.pts_per_frame = []
        self.valid_ratio = []
        self.xyz_min = np.array([np.inf, np.inf, np.inf])
        self.xyz_max = np.array([-np.inf, -np.inf, -np.inf])
        self.range_samples = []
        self.z_hist = np.zeros(8)  # bins: <-0.5 .. >2.5 step 0.5
        self.fwd_samples = []
        self.create_subscription(PointCloud2, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg: PointCloud2):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            self.header = (
                msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec,
                [f.name for f in msg.fields], msg.point_step, msg.row_step,
                msg.is_bigendian, msg.is_dense)
        self.t_last = now
        self.n += 1

        npts = msg.width * msg.height
        self.pts_per_frame.append(npts)

        try:
            arr = np.frombuffer(msg.data, dtype=cloud_dtype(msg), count=npts)
            x = arr['x'].astype(np.float64)
            y = arr['y'].astype(np.float64)
            z = arr['z'].astype(np.float64)
        except (KeyError, ValueError) as exc:
            self.get_logger().error(f'cannot parse cloud: {exc}')
            return

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        nonzero = finite & ((x != 0) | (y != 0) | (z != 0))
        self.valid_ratio.append(float(nonzero.sum()) / max(1, npts))

        if nonzero.any():
            xs, ys, zs = x[nonzero], y[nonzero], z[nonzero]
            self.xyz_min = np.minimum(self.xyz_min, [xs.min(), ys.min(), zs.min()])
            self.xyz_max = np.maximum(self.xyz_max, [xs.max(), ys.max(), zs.max()])
            rng = np.hypot(xs, ys)
            self.range_samples.append(rng[::self.sample_stride])
            self.z_hist += np.histogram(zs, bins=[-1e9, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 1e9])[0]
            bearing = np.degrees(np.arctan2(ys, xs))
            self.fwd_samples.append(int(np.count_nonzero((np.abs(bearing) < 30) & (rng < 5.0))))

    def report(self):
        # drain remaining time
        while self.t0 is not None and (time.time() - self.t0) < self.duration:
            time.sleep(0.05)
        while self.t0 is None and rclpy.ok():
            time.sleep(0.05)
            if time.time() - (self.t_last or time.time()) > 5 and self.n == 0:
                pass

        print('=' * 64)
        print(' POINT CLOUD STATUS')
        print('=' * 64)
        if self.header is None or self.n == 0:
            print(' no messages received (check topic / driver / QoS / network)')
            return 1

        frame, sec, nsec, fields, pstep, rstep, big, dense = self.header
        span = max(1e-6, self.t_last - self.t0)
        print(f' frames received : {self.n}')
        print(f' rate            : {self.n / span:.2f} Hz   (measured over {span:.2f}s)')
        print(f' frame_id        : {frame}')
        print(f' stamp           : {sec}.{nsec:09d}')
        print(f' fields          : {fields}')
        print(f' point_step/row  : {pstep} / {rstep} bytes   big_endian={big} dense={dense}')
        p = np.array(self.pts_per_frame, dtype=float)
        print(f' points/frame    : min {int(p.min())}  mean {p.mean():.0f}  max {int(p.max())}')
        v = np.array(self.valid_ratio)
        print(f' valid ratio     : mean {v.mean() * 100:.1f}%   min {v.min() * 100:.1f}%')
        print(f' x range (m)     : {self.xyz_min[0]:+.2f} .. {self.xyz_max[0]:+.2f}')
        print(f' y range (m)     : {self.xyz_min[1]:+.2f} .. {self.xyz_max[1]:+.2f}')
        print(f' z range (m)     : {self.xyz_min[2]:+.2f} .. {self.xyz_max[2]:+.2f}')
        if self.range_samples:
            r = np.concatenate(self.range_samples)
            print(f' horiz range (m) : p5 {np.percentile(r, 5):.2f}  '
                  f'p50 {np.percentile(r, 50):.2f}  p95 {np.percentile(r, 95):.2f}  max {r.max():.2f}')
        edges = ['<-0.5', '-0.5~0', '0~0.5', '0.5~1.0', '1.0~1.5', '1.5~2.0', '2.0~2.5', '>2.5']
        total = max(1, self.z_hist.sum())
        hist = '  '.join(f'{e}:{int(c) * 100 // total}%' for e, c in zip(edges, self.z_hist))
        print(f' z histogram     : {hist}')
        if self.fwd_samples:
            print(f' front cone pts  : mean {np.mean(self.fwd_samples):.0f} '
                  f'(|bearing|<30deg, range<5m)')
        print('=' * 64)
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/rslidar_points')
    ap.add_argument('--duration', type=float, default=5.0)
    ap.add_argument('--sample-stride', type=int, default=37,
                    help='keep every Nth range sample to bound memory')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)
    node = Status(args.topic, args.duration, args.sample_stride)
    try:
        # spin until we have at least one frame and duration elapsed
        start = time.time()
        while rclpy.ok() and (time.time() - start) < args.duration + 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.n > 0 and (time.time() - node.t0) >= args.duration:
                break
        rc = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
