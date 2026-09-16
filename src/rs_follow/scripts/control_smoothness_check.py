#!/usr/bin/env python3
"""
control_smoothness_check.py — quantify cmd_vel continuity (acceleration & jitter).

Runs rs_follow_node against synthetic clouds and measures, from the /cmd_vel
stream:
  - max |dvx/dt|, |dvy/dt| (linear acceleration, m/s^2)
  - max |dwz/dt|        (angular acceleration, rad/s^2)
for three transitions: standstill->approach, bearing step, emergency stop.
Also measures cmd jitter (std) while holding on a noisy target, comparing
Kalman on/off.

Usage:
  ros2 run rs_follow control_smoothness_check.py            # kalman off + on
  python3 control_smoothness_check.py --kalman false
"""

import argparse
import math
import os
import random
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import PointCloud2, PointField

TEST_TOPIC = '/test_points'


def make_cloud(points, stamp):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = 'rslidar'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    return msg


def cluster(cx, cy, n=80, spread=0.12, zmin=-0.2, zmax=1.4, seed=1):
    rnd = random.Random(seed)
    return [(cx + rnd.uniform(-spread, spread),
             cy + rnd.uniform(-spread, spread),
             rnd.uniform(zmin, zmax)) for _ in range(n)]


class Probe(Node):
    def __init__(self):
        super().__init__('smoothness_probe')
        self.pub = self.create_publisher(PointCloud2, TEST_TOPIC, qos_profile_sensor_data)
        self.bind_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.gen = None
        self.rec = []
        self.lock = threading.Lock()
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.create_timer(0.1, self._tick)
        self.times = {}

    def _cb(self, m):
        with self.lock:
            self.rec.append((time.time(), m.linear.x, m.linear.y, m.angular.z))

    def _tick(self):
        with self.lock:
            gen = self.gen
        if gen is not None:
            self.pub.publish(make_cloud(gen(), self.get_clock().now().to_msg()))

    def bind(self, x, y):
        p = PointStamped()
        p.header.frame_id = 'rslidar'
        p.point.x, p.point.y, p.point.z = float(x), float(y), 0.0
        self.bind_pub.publish(p)

    def snapshot(self):
        with self.lock:
            return list(self.rec)


def stats(rec):
    accmax = [0.0, 0.0, 0.0]
    jerkmax = [0.0, 0.0, 0.0]
    prev_a = [None, None, None]
    npairs = 0
    dts = []
    for (t0, x0, y0, z0), (t1, x1, y1, z1) in zip(rec, rec[1:]):
        dt = t1 - t0
        dts.append(dt)
        if dt < 0.005 or dt > 0.5:  # ignore DDS bursts / gaps
            continue
        npairs += 1
        vals = [x1 - x0, y1 - y0, z1 - z0]
        for k in range(3):
            a = vals[k] / dt
            accmax[k] = max(accmax[k], abs(a))
            if prev_a[k] is not None:
                jerkmax[k] = max(jerkmax[k], abs(a - prev_a[k]) / dt)
            prev_a[k] = a
    dt_med = sorted(dts)[len(dts) // 2] if dts else 0.0
    return {'accel': tuple(accmax), 'jerk': tuple(jerkmax), 'samples': len(rec),
            'pairs': npairs, 'dt_median': dt_med}


def std(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def run_pass(kalman):
    node_cmd = ['ros2', 'run', 'rs_follow', 'rs_follow_node']
    params = ['-p', f'input_topic:={TEST_TOPIC}', '-p', 'active:=true',
              '-p', f'enable_kalman:={str(kalman).lower()}',
              '-p', 'lost_frames_timeout:=5',
              '-p', 'max_linear_accel:=0.8',
              '-p', 'max_angular_accel:=1.5',
              '-p', 'cmd_filter_alpha:=0.6']
    proc = subprocess.Popen(node_cmd + ['--ros-args'] + params,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                            start_new_session=True)
    rclpy.init()
    p = Probe()
    th = threading.Thread(target=rclpy.spin, args=(p,), daemon=True)
    th.start()
    time.sleep(3.0)
    out = {}

    # A) standstill -> approach 2 m
    p.gen = None
    with p.lock:
        p.rec.clear()
    time.sleep(0.5)
    p.gen = lambda: cluster(2.0, 0.0)
    time.sleep(0.3)
    p.bind(2.0, 0.0)
    time.sleep(2.0)
    out['start_to_approach'] = stats(p.snapshot())

    # B) bearing step: rebind to 30 deg
    with p.lock:
        p.rec.clear()
    p.gen = lambda: cluster(2.0, 1.155)
    time.sleep(0.3)
    p.bind(2.0, 1.155)
    time.sleep(2.0)
    out['bearing_step_30deg'] = stats(p.snapshot())

    # C) emergency stop (obstacle appears at 0.3 m)
    with p.lock:
        p.rec.clear()
    p.gen = lambda: cluster(2.0, 0.0) + cluster(0.30, 0.0, n=20, spread=0.03)
    time.sleep(0.3)
    p.bind(2.0, 0.0)
    time.sleep(1.5)
    out['emergency_stop'] = stats(p.snapshot())

    # D) jitter while holding on a noisy target at 1 m
    def noisy():
        return cluster(1.0 + random.uniform(-0.05, 0.05),
                       0.0 + random.uniform(-0.05, 0.05), seed=random.randint(0, 1 << 30))
    p.gen = noisy
    time.sleep(0.3)
    p.bind(1.0, 0.0)
    with p.lock:
        p.rec.clear()
    time.sleep(3.0)
    rec = p.snapshot()
    out['hold_jitter_std'] = (std([r[1] for r in rec]), std([r[3] for r in rec]))

    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=5)
    rclpy.shutdown()
    th.join(timeout=2)
    p.destroy_node()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kalman', choices=['false', 'true', 'both'], default='both')
    args = ap.parse_args()
    if not shutil.which('ros2'):
        print('ERROR: source your ROS workspace first')
        return 2

    modes = [False, True] if args.kalman == 'both' else [args.kalman == 'true']
    results = {k: run_pass(k) for k in modes}

    print()
    print('=' * 100)
    print(f'{"transition":<22}{"kalman":<8}{"max|dvx/dt|":>12}{"max|dwz/dt|":>12}'
          f'{"max|jerk_vx|":>14}{"max|jerk_wz|":>14}')
    print('-' * 100)
    for k in modes:
        for name in ['start_to_approach', 'bearing_step_30deg', 'emergency_stop']:
            st = results[k][name]
            dvx, dvy, dwz = st['accel']
            jvx, jvy, jwz = st['jerk']
            print(f'{name:<22}{str(k):<8}{dvx:>12.2f}{dwz:>12.2f}{jvx:>14.1f}{jwz:>14.1f}'
                  f'   (n={st["samples"]}, dt~{st["dt_median"]*1000:.0f}ms)')
    print('=' * 100)
    print('hold-jitter (std of cmd while target is noisy +/-5cm):')
    for k in modes:
        vx_s, wz_s = results[k]['hold_jitter_std']
        print(f'  kalman={str(k):<5}  std(vx)={vx_s:.4f} m/s   std(wz)={wz_s:.4f} rad/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
