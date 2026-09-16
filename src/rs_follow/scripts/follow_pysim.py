#!/usr/bin/env python3
"""
follow_pysim.py — self-contained physics + ray-casting LiDAR simulator (headless).

No Gazebo rendering needed. It models:
  - differential-drive dynamics with first-order response, slip and latency
  - a 3D ray-casting LiDAR over a world of ground plane, vertical walls and a
    cylindrical (person) target, with range noise, dropout and real occlusion
  - optional quadruped gait (body pitch/roll oscillation) on the sensor pose

The REAL rs_follow_node is driven in closed loop (it consumes the synthesised
PointCloud2 and /odom, and publishes /cmd_vel). Frames are rendered to GIF/PNG
with the same style as follow_sim_visualize.py.

Usage:
  source /opt/ros/humble/setup.bash && source ~/dog_follower/install/setup.bash
  ros2 run rs_follow follow_pysim.py --scenario open
  ros2 run rs_follow follow_pysim.py --all
"""

import argparse
import math
import os
import shutil
import signal
import struct
import subprocess
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

# reuse the renderers from the other simulator
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follow_sim_visualize as fsv  # noqa: E402

SIM_TOPIC = '/sim_points'
DT = 0.1
FOLLOW_DIST = 1.0
TARGET_R = 0.25
SCENARIOS = ['open', 'corridor', 'gait']


# --------------------------------------------------------------------------- #
# world
# --------------------------------------------------------------------------- #
class World:
    half = 15.0

    def __init__(self, walls=None):
        self.walls = list(walls or [])      # (x0,y0,x1,y1,height)
        self.cylinders = []                 # (cx,cy,radius,height) dynamic target

    def box(self, cx, cy, sx, sy, height):
        hx, hy = sx / 2, sy / 2
        c = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        for i in range(4):
            a, b = c[i], c[(i + 1) % 4]
            self.walls.append((a[0], a[1], b[0], b[1], height))


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def raycast(origin, R_ws, V_s, world, cfg, rng):
    """Return Nx3 points in the SENSOR frame for all rays that hit something."""
    Vw = V_s @ R_ws.T
    t = np.full(V_s.shape[0], np.inf)

    # ground plane z = 0
    vz = Vw[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        tg = np.where(vz < -1e-6, -origin[2] / np.where(vz < -1e-6, vz, -1.0), np.inf)
    tg_safe = np.where(np.isfinite(tg), tg, 0.0)
    hx = origin[0] + tg_safe * Vw[:, 0]
    hy = origin[1] + tg_safe * Vw[:, 1]
    ok = np.isfinite(tg) & (tg > cfg['min_range']) & (tg < cfg['max_range']) & \
        (np.abs(hx) < world.half) & (np.abs(hy) < world.half)
    t = np.where(ok, np.minimum(t, tg), t)

    O2 = origin[:2]
    d2 = Vw[:, :2]
    for (ax, ay, bx, by, H) in world.walls:
        ex, ey = bx - ax, by - ay
        aox, aoy = ax - O2[0], ay - O2[1]
        denom = d2[:, 0] * ey - d2[:, 1] * ex
        numt = aox * ey - aoy * ex
        numu = aox * d2[:, 1] - aoy * d2[:, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            tw = numt / denom
            uw = -numu / denom
        z = origin[2] + tw * Vw[:, 2]
        good = (np.abs(denom) > 1e-9) & (tw > cfg['min_range']) & (tw < cfg['max_range']) & \
               (uw >= 0) & (uw <= 1) & (z >= 0) & (z <= H) & (tw < t)
        t = np.where(good, tw, t)

    for (cx, cy, R, H) in world.cylinders:
        ocx, ocy = O2[0] - cx, O2[1] - cy
        a = d2[:, 0] ** 2 + d2[:, 1] ** 2
        b = 2 * (ocx * d2[:, 0] + ocy * d2[:, 1])
        c = ocx ** 2 + ocy ** 2 - R ** 2
        disc = b * b - 4 * a * c
        sq = np.sqrt(np.maximum(disc, 0))
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (-b - sq) / (2 * a)
            t2 = (-b + sq) / (2 * a)
        tt = np.where(t1 > cfg['min_range'], t1, t2)
        z = origin[2] + tt * Vw[:, 2]
        good = (disc > 0) & (tt > cfg['min_range']) & (tt < cfg['max_range']) & \
               (z >= 0) & (z <= H) & (tt < t)
        t = np.where(good, tt, t)

    valid = np.isfinite(t) & (t < cfg['max_range'])
    if not np.any(valid):
        return np.zeros((0, 3))
    r = t[valid]
    pts = (r[:, None]) * V_s[valid]
    # range noise (grows with distance) + dropout
    noise = rng.normal(0.0, cfg['noise_abs'] + cfg['noise_rel'] * r)
    pts *= ((r + noise) / r)[:, None]
    keep = rng.random(r.shape) > cfg['dropout']
    return pts[keep]


# --------------------------------------------------------------------------- #
# simulator node
# --------------------------------------------------------------------------- #
class PySim(Node):
    def __init__(self, scenario, duration, slip=1.0, gait_deg=0.0,
                 sensor_h=0.8, seed=0):
        super().__init__('follow_pysim')
        self.scenario = scenario
        self.duration = duration
        self.slip = slip
        self.sensor_h = sensor_h
        self.gait_deg = gait_deg
        self.rng = np.random.default_rng(seed)

        self.cfg = dict(min_range=0.15, max_range=25.0, noise_abs=0.01,
                        noise_rel=0.01, dropout=0.02)
        naz, nel = 360, 16
        az = np.linspace(-math.pi, math.pi, naz, endpoint=False)
        el = np.linspace(math.radians(-15), math.radians(15), nel)
        AZ, EL = np.meshgrid(az, el, indexing='ij')
        self.V_s = np.stack([np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ),
                             np.sin(EL)], axis=-1).reshape(-1, 3)

        self.world = World()
        self._build_scenario(scenario)

        # robot state
        self.rx = self.ry = self.ryaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.status = 'INIT'
        self.filt = self.raw = None
        self.t0 = None
        self.bound = False
        self.frames = []

        self.cloud_pub = self.create_publisher(PointCloud2, SIM_TOPIC, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.bind_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, '/rs_follow/status', self._on_status, 10)
        self.create_subscription(PointStamped, '/rs_follow/target', self._on_target, 10)
        self.create_subscription(PointStamped, '/rs_follow/target_raw', self._on_raw, 10)
        self.create_timer(DT, self._tick)

    # -- scenario geometry --------------------------------------------------- #
    def _build_scenario(self, name):
        if name == 'corridor':
            self.world.walls += [(-2, 0.7, 10, 0.7, 2.0), (-2, -0.7, 10, -0.7, 2.0)]
        if name == 'gait':
            self.gait_deg = 4.0            # quadruped body pitch/roll

    def target_world(self, t):
        if self.scenario in ('open', 'gait'):
            a = math.pi + 0.12 * t
            return (4.0 + 2.0 * math.cos(a), 2.0 * math.sin(a))
        if self.scenario == 'corridor':
            return (1.5 + 0.40 * min(t, 12.0), 0.0)
        return (2.0, 0.0)

    # -- callbacks ----------------------------------------------------------- #
    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def _on_status(self, m):
        self.status = m.data

    def _on_target(self, m):
        self.filt = (m.point.x, m.point.y)

    def _on_raw(self, m):
        self.raw = (m.point.x, m.point.y)

    # -- main tick ----------------------------------------------------------- #
    def _tick(self):
        if self.t0 is None:
            self.t0 = time.time()
            return
        t = time.time() - self.t0
        dt = DT

        # first-order actuator response + slip
        cx, cy, cw = self.cmd
        tau = 0.12
        self.vx += (cx - self.vx) / tau * dt
        self.vy += (cy - self.vy) / tau * dt
        self.wz += (cw - self.wz) / tau * dt
        ax, ay = self.vx * self.slip, self.vy * self.slip
        self.rx += (ax * math.cos(self.ryaw) - ay * math.sin(self.ryaw)) * dt
        self.ry += (ax * math.sin(self.ryaw) + ay * math.cos(self.ryaw)) * dt
        self.ryaw += self.wz * dt

        # quadruped gait on the sensor orientation
        gp = math.radians(self.gait_deg) * math.sin(2 * math.pi * 2.0 * t)
        gr = math.radians(self.gait_deg * 0.6) * math.sin(2 * math.pi * 2.7 * t + 1.0)
        R_ws = rot_z(self.ryaw) @ np.array(
            [[1, 0, 0], [0, math.cos(gr), -math.sin(gr)], [0, math.sin(gr), math.cos(gr)]]) @ \
            np.array([[math.cos(gp), 0, math.sin(gp)], [0, 1, 0],
                      [-math.sin(gp), 0, math.cos(gp)]])
        origin = R_ws @ np.array([0.0, 0.0, self.sensor_h]) + np.array([self.rx, self.ry, 0.0])

        tw = self.target_world(t)
        self.world.cylinders = [(tw[0], tw[1], TARGET_R, 1.7)]
        pts_sensor = raycast(origin, R_ws, self.V_s, self.world, self.cfg, self.rng)
        self.cloud_pub.publish(self._make_cloud(pts_sensor))

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.rx + self.rng.normal(0, 0.005)
        od.pose.pose.position.y = self.ry + self.rng.normal(0, 0.005)
        yaw_n = self.ryaw + self.rng.normal(0, 0.002)
        od.pose.pose.orientation.z = math.sin(yaw_n / 2)
        od.pose.pose.orientation.w = math.cos(yaw_n / 2)
        od.twist.twist.linear.x = ax
        od.twist.twist.linear.y = ay
        od.twist.twist.angular.z = self.wz
        self.odom_pub.publish(od)

        if 0.2 < t < 0.4 and not self.bound:
            rtx, rty = self._to_sensor(tw[0], tw[1], R_ws, origin)
            p = PointStamped()
            p.header.frame_id = 'rslidar'
            p.point.x, p.point.y, p.point.z = rtx, rty, 0.0
            self.bind_pub.publish(p)
            self.bound = True

        rtx, rty = self._to_sensor(tw[0], tw[1], R_ws, origin)
        surf = max(0.05, math.hypot(tw[0] - self.rx, tw[1] - self.ry) - TARGET_R)
        self.frames.append({
            't': t, 'rx': self.rx, 'ry': self.ry, 'yaw': self.ryaw, 'tw': tw,
            'cmd': self.cmd, 'status': self.status, 'pts_flat': pts_sensor,
            'filt': self.filt, 'raw': self.raw,
            'dist': surf, 'bear': math.degrees(math.atan2(rty, rtx)),
        })

    # -- helpers ------------------------------------------------------------- #
    def _to_sensor(self, wx, wy, R_ws, origin):
        d = np.array([wx - origin[0], wy - origin[1], -origin[2]])
        s = R_ws.T @ d
        return s[0], s[1]

    def _make_cloud(self, pts):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rslidar'
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.astype('<f4').tobytes()
        return msg

    def finished(self):
        return self.t0 is not None and (time.time() - self.t0) >= self.duration


def stop_proc(proc):
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


def start_node():
    params = ['-p', f'input_topic:={SIM_TOPIC}', '-p', 'active:=true',
              '-p', 'odom_topic:=/odom', '-p', 'enable_kalman:=true',
              '-p', 'filter_in_world:=true', '-p', 'auto_select_front:=true',
              '-p', 'follow_dist:=1.0', '-p', 'max_linear:=0.9', '-p', 'max_angular:=1.0',
              '-p', 'k_linear:=2.5', '-p', 'k_angular:=1.5', '-p', 'max_linear_cmd:=1.5']
    return subprocess.Popen(['ros2', 'run', 'rs_follow', 'rs_follow_node', '--ros-args'] + params,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                            start_new_session=True)


def run_one(scenario, duration, outdir, slip, gait, sensor_h):
    print(f'--- {scenario} (slip={slip} gait={gait}deg sensor_h={sensor_h}) ---')
    proc = start_node()
    rclpy.init()
    sim = PySim(scenario, duration, slip, gait, sensor_h)
    time.sleep(3.0)
    try:
        while rclpy.ok() and not sim.finished():
            rclpy.spin_once(sim, timeout_sec=0.02)
    finally:
        stop_proc(proc)

    # renderers expect frames with 'pts' (robot frame, list of [x,y,z])
    for f in sim.frames:
        f['pts'] = [(p[0], p[1], p[2]) for p in f['pts_flat']]
    sim.experiment = scenario

    fsv.render_gif(sim, os.path.join(outdir, f'{scenario}.gif'))
    fsv.render_panel(sim, os.path.join(outdir, f'{scenario}.png'))
    fsv.render_filmstrip(sim, os.path.join(outdir, f'{scenario}_frames.png'))

    cmd = [f['cmd'] for f in sim.frames]
    dists = [f['dist'] for f in sim.frames if f['dist'] is not None]
    errs = [abs(d - FOLLOW_DIST) for d in dists[len(dists) // 3:]]
    pts = [len(f['pts']) for f in sim.frames]
    print(f'  frames={len(sim.frames)}  pts/frame~{int(np.mean(pts))}  '
          f'max|vx|={max(abs(c[0]) for c in cmd):.2f}  '
          f'standoff mean={sum(errs)/len(errs)*100:.1f}cm max={max(errs)*100:.1f}cm')
    sim.destroy_node()
    rclpy.shutdown()
    return sim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', choices=SCENARIOS, default='open')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--duration', type=float, default=14.0)
    ap.add_argument('--slip', type=float, default=1.0)
    ap.add_argument('--gait', type=float, default=0.0, help='quadruped pitch/roll deg')
    ap.add_argument('--sensor-h', type=float, default=0.8)
    ap.add_argument('--outdir', default=os.path.expanduser('~/dog_follower/rs_follow_pysim'))
    args = ap.parse_args()

    if not shutil.which('ros2'):
        print('ERROR: source /opt/ros/humble/setup.bash first')
        return 2
    os.makedirs(args.outdir, exist_ok=True)

    if args.all:
        sims = [run_one(s, args.duration, args.outdir, args.slip, args.gait, args.sensor_h)
                for s in SCENARIOS]
        fsv.render_summary(sims, os.path.join(args.outdir, 'summary.png'))
        print(f'\nwrote {args.outdir}/*.gif|png and summary.png')
    else:
        run_one(args.scenario, args.duration, args.outdir, args.slip, args.gait, args.sensor_h)
    return 0


if __name__ == '__main__':
    sys.exit(main())
