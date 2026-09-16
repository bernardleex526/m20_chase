#!/usr/bin/env python3
"""
follow_scenario_test.py — deterministic cmd_vel behaviour tests for rs_follow.

Because there is no real moving target, this script synthesises PointCloud2
frames (a "person" cluster, walls, obstacles) and feeds them to a running
rs_follow_node on a test topic, binds a target via /clicked_point, then records
/cmd_vel and /rs_follow/status for each scenario.

Prerequisite (source the workspaces first):
  source /opt/ros/humble/setup.bash
  source ~/dog_follower/install/setup.bash
  python3 src/rs_follow/scripts/follow_scenario_test.py

Options:
  --node-cmd "ros2 run rs_follow rs_follow_node"   override the node command
  --duration 1.5                                   seconds sampled per scenario
  --keep                                           keep the node running afterwards
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
from std_msgs.msg import Bool, String

TEST_TOPIC = '/test_points'


# --------------------------------------------------------------------------- #
# synthetic point clouds
# --------------------------------------------------------------------------- #
def make_cloud(points, stamp, frame_id='rslidar'):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
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


def wall(x0, x1, y, n=60, z=0.0):
    return [(x0 + (x1 - x0) * i / (n - 1), y, z) for i in range(n)]


# --------------------------------------------------------------------------- #
# test harness
# --------------------------------------------------------------------------- #
class Harness(Node):
    def __init__(self):
        super().__init__('follow_scenario_test')
        self.pub = self.create_publisher(PointCloud2, TEST_TOPIC, qos_profile_sensor_data)
        self.bind_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.clear_pub = self.create_publisher(Bool, '/rs_follow/clear_target', 10)
        self.cmds = []
        self.status = 'INIT'
        self.lock = threading.Lock()
        self.current = None
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, '/rs_follow/status', self._on_status, 10)
        self.create_timer(0.1, self._tick)

    def _on_cmd(self, msg):
        with self.lock:
            self.cmds.append((msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data

    def _tick(self):
        with self.lock:
            pts = self.current
        if pts is not None:
            self.pub.publish(make_cloud(pts, self.get_clock().now().to_msg()))

    def set_points(self, pts):
        with self.lock:
            self.current = pts

    def clear(self):
        m = Bool()
        m.data = True
        self.clear_pub.publish(m)

    def bind(self, x, y):
        p = PointStamped()
        p.header.frame_id = 'rslidar'
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x, p.point.y, p.point.z = float(x), float(y), 0.0
        self.bind_pub.publish(p)

    def sample(self):
        with self.lock:
            return list(self.cmds), self.status


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def stop_proc(proc):
    """Terminate the whole process group (ros2 run spawns a child node)."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


def run_scenario(h, name, points, bind=None, duration=1.5, settle=0.5, expect=None,
                 wait_status=None, ramp=0.8):
    h.clear()
    time.sleep(0.3)
    h.set_points(points)
    time.sleep(settle)
    if bind is not None:
        h.bind(*bind)
        time.sleep(0.4)

    if wait_status is not None:
        deadline = time.time() + 4.0
        while time.time() < deadline:
            with h.lock:
                if h.status == wait_status:
                    break
            time.sleep(0.05)

    # discard the acceleration-limited ramp, then measure steady state
    with h.lock:
        h.cmds.clear()
    time.sleep(ramp)
    with h.lock:
        h.cmds.clear()
    time.sleep(duration)
    cmds, status = h.sample()

    vx = mean([c[0] for c in cmds])
    vy = mean([c[1] for c in cmds])
    wz = mean([c[2] for c in cmds])
    ok, why = (True, '')
    if expect is not None:
        ok, why = expect(status, vx, vy, wz)
    return {
        'name': name, 'status': status, 'n': len(cmds),
        'vx': vx, 'vy': vy, 'wz': wz, 'pass': ok, 'why': why,
    }


def build_scenarios():
    S = []
    # 1) far target straight ahead -> drive forward
    S.append(('approach_2m', cluster(2.0, 0.0), (2.0, 0.0), None,
              lambda s, vx, vy, wz: (vx > 0.05 and abs(wz) < 0.15, 'vx>0, wz~0')))
    # 2) target at the standoff distance -> hold still
    S.append(('hold_1m', cluster(1.0, 0.0), (1.0, 0.0), None,
              lambda s, vx, vy, wz: (abs(vx) < 0.06 and abs(wz) < 0.12, 'vx~0, wz~0')))
    # 3) target too close -> back off
    S.append(('backoff_0p5m', cluster(0.5, 0.0), (0.5, 0.0), None,
              lambda s, vx, vy, wz: (vx < -0.02, 'vx<0')))
    # 4) target to the left -> turn left (wz>0)
    S.append(('turn_left', cluster(2.0, 1.155), (2.0, 1.155), None,
              lambda s, vx, vy, wz: (wz > 0.05, 'wz>0')))
    # 5) target to the right -> turn right (wz<0)
    S.append(('turn_right', cluster(2.0, -1.155), (2.0, -1.155), None,
              lambda s, vx, vy, wz: (wz < -0.05, 'wz<0')))
    # 6) target behind -> rotate in place, no forward motion
    S.append(('behind', cluster(-1.5, 0.0), (-1.5, 0.0), None,
              lambda s, vx, vy, wz: (abs(vx) < 0.06 and abs(wz) > 0.1, 'vx~0, |wz|>0')))
    # 7) close obstacle in front -> emergency stop
    obs = cluster(2.0, 0.0, n=40) + cluster(0.30, 0.0, n=20, spread=0.03)
    S.append(('emergency_obstacle', obs, (2.0, 0.0), None,
              lambda s, vx, vy, wz: (s == 'EMERGENCY_STOP' and
                                     abs(vx) < 1e-6 and abs(wz) < 1e-6, 'EMERGENCY_STOP, cmd=0')))
    # 8) left wall between robot and target -> lateral shift away from wall
    corr = cluster(2.0, 0.0) + wall(0.9, 1.4, 0.2)
    S.append(('corridor_left_wall', corr, (2.0, 0.0), None,
              lambda s, vx, vy, wz: (vy < -0.05, 'vy<0')))
    # 9) nothing bound -> auto select front target
    S.append(('auto_select_front', cluster(2.0, 0.0), None, None,
              lambda s, vx, vy, wz: (s == 'TRACKING_AUTO' and vx > 0.05,
                                     'TRACKING_AUTO, vx>0')))
    # 10) target lost (empty cloud) -> no target, zero cmd
    S.append(('target_lost', [], (2.0, 0.0), 'NO_TARGET',
              lambda s, vx, vy, wz: (s == 'NO_TARGET' and
                                     abs(vx) < 1e-3 and abs(wz) < 1e-3, 'NO_TARGET, cmd~0')))
    # 11) target at 45 deg -> proportional turn (wz ~= 0.785 * k_angular)
    S.append(('turn_45deg', cluster(2.0, 2.0, seed=5), (2.0, 2.0), None,
              lambda s, vx, vy, wz: (wz > 0.5 and vx > 0.05, 'wz>0.5, vx>0')))
    # 12) far target 10 m -> clamped forward speed
    S.append(('far_10m', cluster(10.0, 0.0, seed=6), (10.0, 0.0), None,
              lambda s, vx, vy, wz: (vx > 0.5 and abs(wz) < 0.15, 'vx~max, wz~0')))
    # 13) right wall between robot and target -> lateral shift left
    corr_r = cluster(2.0, 0.0) + wall(0.9, 1.4, -0.2)
    S.append(('corridor_right_wall', corr_r, (2.0, 0.0), None,
              lambda s, vx, vy, wz: (vy > 0.05, 'vy>0')))
    return S


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--node-cmd', default='ros2 run rs_follow rs_follow_node')
    ap.add_argument('--duration', type=float, default=1.5)
    ap.add_argument('--keep', action='store_true')
    args = ap.parse_args()

    node_cmd = args.node_cmd.split()
    if not shutil.which(node_cmd[0]):
        print(f'ERROR: {node_cmd[0]} not found; source your ROS workspace first')
        return 2

    params = [
        '-p', f'input_topic:={TEST_TOPIC}',
        '-p', 'active:=true',
        '-p', 'enable_kalman:=false',
        '-p', 'lost_frames_timeout:=5',
        '-p', 'auto_select_front:=true',
        '-p', 'max_linear_accel:=0.8',
        '-p', 'max_angular_accel:=1.5',
        '-p', 'cmd_filter_alpha:=0.6',
    ]
    print('starting node:', ' '.join(node_cmd), '--ros-args', ' '.join(params))
    proc = subprocess.Popen(node_cmd + ['--ros-args'] + params,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                            start_new_session=True)

    rclpy.init()
    h = Harness()
    spin_thread = threading.Thread(target=rclpy.spin, args=(h,), daemon=True)
    spin_thread.start()
    time.sleep(3.0)  # let the node come up

    scenarios = build_scenarios()
    results = []
    try:
        for name, pts, bind, wait_status, expect in scenarios:
            print(f'  running {name} ...')
            results.append(run_scenario(h, name, pts, bind, args.duration,
                                        expect=expect, wait_status=wait_status))
    finally:
        if not args.keep:
            stop_proc(proc)
        rclpy.shutdown()
        spin_thread.join(timeout=2)
        h.destroy_node()

    print()
    print('=' * 78)
    print(f'{"scenario":<22}{"status":<18}{"vx":>8}{"vy":>8}{"wz":>8}   result')
    print('-' * 78)
    npass = 0
    for r in results:
        verdict = 'PASS' if r['pass'] else f"FAIL ({r['why']})"
        npass += int(r['pass'])
        print(f"{r['name']:<22}{r['status']:<18}{r['vx']:>8.3f}{r['vy']:>8.3f}"
              f"{r['wz']:>8.3f}   {verdict}")
    print('=' * 78)
    print(f'{npass}/{len(results)} scenarios passed')
    return 0 if npass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
