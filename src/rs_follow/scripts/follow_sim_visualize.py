#!/usr/bin/env python3
"""
follow_sim_visualize.py — closed-loop follow simulation + figures for eyeballing.

A virtual robot is driven by the *real* rs_follow_node: the simulator integrates
the robot pose from the published /cmd_vel, looks at the moving target from the
robot viewpoint, synthesises a PointCloud2, and feeds it back to the node.

Experiments (each -> a GIF + a multi-panel PNG):
  approach  static target ahead  : approach then hold at follow_dist
  chase     target walks a circle: continuous tracking
  stopgo    target walks/stops   : hold, then resume
  lost      target disappears    : NO_TARGET + stop, then auto re-acquire
  obstacle  bar blocks the path  : EMERGENCY_STOP then resume
  turn      target sweeps around : robot turns to keep the target ahead

Usage (source workspaces first):
  source /opt/ros/humble/setup.bash && source ~/dog_follower/install/setup.bash
  ros2 run rs_follow follow_sim_visualize.py --experiment chase
  ros2 run rs_follow follow_sim_visualize.py --all
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
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyArrowPatch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

SIM_TOPIC = '/sim_points'
DT = 0.1
FOLLOW_DIST = 1.0
EXPERIMENTS = ['approach', 'chase', 'stopgo', 'lost', 'obstacle', 'turn']
EXTRA_PARAMS = []


# --------------------------------------------------------------------------- #
# point-cloud helpers
# --------------------------------------------------------------------------- #
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


def world_to_robot(px, py, rx, ry, ryaw):
    dx, dy = px - rx, py - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    return c * dx - s * dy, s * dx + c * dy


def person_cloud(cx, cy, n=90, spread=0.15, zmin=-0.2, zmax=1.4, seed=0):
    rnd = random.Random(seed)
    return [(cx + rnd.uniform(-spread, spread),
             cy + rnd.uniform(-spread, spread),
             rnd.uniform(zmin, zmax)) for _ in range(n)]


def bar_cloud(cx, cy, half=0.5, n=40, seed=1):
    rnd = random.Random(seed)
    return [(cx + rnd.uniform(-0.05, 0.05),
             cy + rnd.uniform(-half, half),
             rnd.uniform(-0.2, 1.2)) for _ in range(n)]


def wall_cloud(x0, y0, x1, y1, n=60, seed=2):
    rnd = random.Random(seed)
    return [(x0 + (x1 - x0) * i / (n - 1) + rnd.uniform(-0.03, 0.03),
             y0 + (y1 - y0) * i / (n - 1) + rnd.uniform(-0.03, 0.03),
             rnd.uniform(-0.2, 1.6)) for i in range(n)]


# --------------------------------------------------------------------------- #
# experiment definitions: return (target_world or None, extra world objects)
# --------------------------------------------------------------------------- #
def exp_approach(t):
    return (2.5, 0.0), []


def exp_chase(t):
    a = math.pi + 0.12 * t
    return (4.0 + 2.0 * math.cos(a), 2.0 * math.sin(a)), []


def exp_stopgo(t):
    if t < 5.0:
        x = 2.0 + 0.40 * t
    elif t <= 9.0:
        x = 2.0 + 0.40 * 5.0
    else:
        x = 2.0 + 0.40 * 5.0 + 0.40 * (t - 9.0)
    return (x, 0.0), []


def exp_lost(t):
    if 5.0 <= t <= 8.0:
        return None, []          # target not observable
    return (2.0 + 0.40 * t, 0.0), []


def exp_obstacle(t):
    if t < 4.0:
        x = 2.0 + 0.35 * t
    elif t <= 7.5:
        x = 2.0 + 0.35 * 4.0
    else:
        x = 2.0 + 0.35 * 4.0 + 0.35 * (t - 7.5)
    return (x, 0.0), []


def exp_turn(t):
    a = 0.20 * t
    return (3.0 * math.cos(a), 3.0 * math.sin(a)), []


EXPFUN = {
    'approach': exp_approach, 'chase': exp_chase, 'stopgo': exp_stopgo,
    'lost': exp_lost, 'obstacle': exp_obstacle, 'turn': exp_turn,
}


# --------------------------------------------------------------------------- #
# simulator
# --------------------------------------------------------------------------- #
class Sim(Node):
    def __init__(self, experiment, duration, slip=1.0):
        super().__init__('follow_sim')
        self.experiment = experiment
        self.duration = duration
        self.slip = slip
        self.pub = self.create_publisher(PointCloud2, SIM_TOPIC, qos_profile_sensor_data)
        self.bind_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd = (0.0, 0.0, 0.0)
        self.status = 'INIT'
        self.target_filt = None
        self.target_raw = None
        self.rx = self.ry = self.ryaw = 0.0
        self.t0 = None
        self.bound = False
        self.frames = []
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, '/rs_follow/status', self._on_status, 10)
        self.create_subscription(PointStamped, '/rs_follow/target', self._on_target, 10)
        self.create_subscription(PointStamped, '/rs_follow/target_raw', self._on_raw, 10)
        self.create_timer(DT, self._tick)

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def _on_status(self, m):
        self.status = m.data

    def _on_target(self, m):
        self.target_filt = (m.point.x, m.point.y)

    def _on_raw(self, m):
        self.target_raw = (m.point.x, m.point.y)

    def _tick(self):
        if self.t0 is None:
            self.t0 = time.time()
            return
        t = time.time() - self.t0

        vx, vy, wz = self.cmd
        ax, ay = vx * self.slip, vy * self.slip   # actual motion (slip != command)
        self.rx += (ax * math.cos(self.ryaw) - ay * math.sin(self.ryaw)) * DT
        self.ry += (ax * math.sin(self.ryaw) + ay * math.cos(self.ryaw)) * DT
        self.ryaw += wz * DT

        # publish odometry of the ACTUAL motion (what a real robot's odom gives)
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.rx
        od.pose.pose.position.y = self.ry
        od.pose.pose.orientation.z = math.sin(self.ryaw / 2.0)
        od.pose.pose.orientation.w = math.cos(self.ryaw / 2.0)
        od.twist.twist.linear.x = ax
        od.twist.twist.linear.y = ay
        od.twist.twist.angular.z = wz
        self.odom_pub.publish(od)

        tw, extra = EXPFUN[self.experiment](t)
        rnd = random.Random(int(t * 1000))
        pts = []
        rtx = rty = None
        if tw is not None:
            nz = (rnd.gauss(0, 0.03), rnd.gauss(0, 0.03))
            rtx, rty = world_to_robot(tw[0] + nz[0], tw[1] + nz[1],
                                      self.rx, self.ry, self.ryaw)
            pts += person_cloud(rtx, rty, seed=int(t * 1000))

        if self.experiment == 'obstacle' and 4.0 <= t <= 7.5:
            rox, roy = world_to_robot(self.rx + 0.30, self.ry, self.rx, self.ry, self.ryaw)
            pts += bar_cloud(rox, roy)

        self.pub.publish(make_cloud(pts, self.get_clock().now().to_msg()))

        if 0.2 < t < 0.4 and not self.bound and rtx is not None:
            p = PointStamped()
            p.header.frame_id = 'rslidar'
            p.point.x, p.point.y, p.point.z = rtx, rty, 0.0
            self.bind_pub.publish(p)
            self.bound = True

        dist = math.hypot(rtx, rty) if rtx is not None else None
        bear = math.degrees(math.atan2(rty, rtx)) if rtx is not None else None
        self.frames.append({
            't': t, 'rx': self.rx, 'ry': self.ry, 'yaw': self.ryaw,
            'tw': tw, 'cmd': self.cmd, 'status': self.status,
            'pts': pts, 'filt': self.target_filt, 'raw': self.target_raw,
            'dist': dist, 'bear': bear,
        })

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
              '-p', 'enable_kalman:=true', '-p', 'auto_select_front:=true',
              '-p', 'follow_dist:=1.0', '-p', 'max_linear:=0.9', '-p', 'max_angular:=1.0',
              '-p', 'k_linear:=2.5', '-p', 'k_angular:=1.5', '-p', 'max_linear_cmd:=1.5',
              '-p', 'max_linear_accel:=0.8', '-p', 'max_angular_accel:=1.5',
              '-p', 'cmd_filter_alpha:=0.6'] + EXTRA_PARAMS
    return subprocess.Popen(['ros2', 'run', 'rs_follow', 'rs_follow_node', '--ros-args'] + params,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                            start_new_session=True)


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #
def _to_world(px, py, rx, ry, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return rx + c * px - s * py, ry + s * px + c * py


def _world_points(frames):
    """Pre-transform every frame's LiDAR points into the world frame."""
    for f in frames:
        w = [_to_world(p[0], p[1], f['rx'], f['ry'], f['yaw']) for p in f['pts']]
        f['wpts'] = w
        if f['filt']:
            f['filt_w'] = _to_world(f['filt'][0], f['filt'][1], f['rx'], f['ry'], f['yaw'])
        if f['raw']:
            f['raw_w'] = _to_world(f['raw'][0], f['raw'][1], f['rx'], f['ry'], f['yaw'])


def render_gif(sim, out_gif):
    """World-frame animation: the robot really moves, arrows show velocity/turn."""
    frames = sim.frames
    _world_points(frames)

    xs = [p[0] for f in frames for p in f['wpts']] + [f['rx'] for f in frames]
    ys = [p[1] for f in frames for p in f['wpts']] + [f['ry'] for f in frames]
    tw = [(f['tw'][0], f['tw'][1]) for f in frames if f['tw'] is not None]
    xs += [p[0] for p in tw]
    ys += [p[1] for p in tw]
    pad = 1.0
    xlim = (min(xs) - pad, max(xs) + pad)
    ylim = (min(ys) - pad, max(ys) + pad)

    fig, ax = plt.subplots(figsize=(7, 7))

    def draw(i):
        ax.clear()
        f = frames[i]
        # trails
        ax.plot([g['rx'] for g in frames[:i + 1]], [g['ry'] for g in frames[:i + 1]],
                '-', color='tab:blue', lw=1, alpha=0.6)
        tg = [(g['tw'][0], g['tw'][1]) for g in frames[:i + 1] if g['tw'] is not None]
        if tg:
            ax.plot([p[0] for p in tg], [p[1] for p in tg], '--', color='tab:red',
                    lw=1, alpha=0.5)
        # LiDAR returns (world frame) + current target
        ax.scatter([p[0] for p in f['wpts']], [p[1] for p in f['wpts']],
                   s=3, c='0.65', label='LiDAR')
        if f['tw'] is not None:
            ax.plot(*f['tw'], '*', color='tab:red', ms=15, label='target (truth)')
        if f.get('filt_w'):
            ax.plot(*f['filt_w'], 'o', color='tab:green', ms=8, label='filtered')
        if f.get('raw_w'):
            ax.plot(*f['raw_w'], 'x', color='tab:purple', ms=7, label='raw')

        # robot (triangle oriented along heading)
        ax.plot([f['rx']], [f['ry']], marker=(3, 0, math.degrees(f['yaw']) - 90),
                markersize=18, color='tab:blue', label='robot')

        # linear velocity arrow (world direction = robot heading)
        vx = f['cmd'][0]
        ax.add_patch(FancyArrowPatch((f['rx'], f['ry']),
                                     (f['rx'] + vx * 2.0 * math.cos(f['yaw']),
                                      f['ry'] + vx * 2.0 * math.sin(f['yaw'])),
                                     color='tab:red', mutation_scale=14, lw=2.5,
                                     label='vx (forward speed)'))
        # yaw-rate arc arrow around the robot
        wz = f['cmd'][2]
        if abs(wz) > 0.02:
            r = 0.6
            d = 1.0 if wz > 0 else -1.0
            a0 = f['yaw'] - d * 0.7
            a1 = f['yaw'] + d * 0.7
            ax.add_patch(FancyArrowPatch(
                (f['rx'] + r * math.cos(a0), f['ry'] + r * math.sin(a0)),
                (f['rx'] + r * math.cos(a1), f['ry'] + r * math.sin(a1)),
                connectionstyle=f'arc3,rad={0.6 * d}', color='tab:orange',
                mutation_scale=14, lw=2.5, label='wz (turn)'))

        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{sim.experiment} (WORLD frame)  t={f['t']:.1f}s  {f['status']}\n"
                     f"vx={vx:+.2f} m/s   wz={wz:+.2f} rad/s")
        ax.legend(loc='upper right', fontsize=7)

    FuncAnimation(fig, draw, frames=len(frames), interval=100).save(out_gif, writer=PillowWriter(fps=10))
    plt.close(fig)


def render_filmstrip(sim, out_png, nshots=4):
    """Static world-frame snapshots to eyeball the process without playing the GIF."""
    frames = sim.frames
    _world_points(frames)
    idx = [int(k * (len(frames) - 1) / (nshots - 1)) for k in range(nshots)]
    fig, axes = plt.subplots(1, nshots, figsize=(4.2 * nshots, 4.6))
    for ax, i in zip(axes, idx):
        f = frames[i]
        ax.scatter([p[0] for p in f['wpts']], [p[1] for p in f['wpts']], s=2, c='0.7')
        ax.plot([g['rx'] for g in frames[:i + 1]], [g['ry'] for g in frames[:i + 1]],
                '-', color='tab:blue', lw=1, alpha=0.5)
        if f['tw'] is not None:
            ax.plot(*f['tw'], '*', color='tab:red', ms=13, label='target truth')
        if f.get('filt_w'):
            ax.plot(*f['filt_w'], 'o', color='tab:green', ms=8, label='filtered')
        if f.get('raw_w'):
            ax.plot(*f['raw_w'], 'x', color='tab:purple', ms=7, label='raw meas')
        ax.plot([f['rx']], [f['ry']], marker=(3, 0, math.degrees(f['yaw']) - 90),
                markersize=15, color='tab:blue', label='robot')
        vx = f['cmd'][0]
        ax.add_patch(FancyArrowPatch((f['rx'], f['ry']),
                                     (f['rx'] + vx * 2.0 * math.cos(f['yaw']),
                                      f['ry'] + vx * 2.0 * math.sin(f['yaw'])),
                                     color='tab:red', mutation_scale=12, lw=2))
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"t={f['t']:.1f}s  {f['status']}\nvx={vx:+.2f}  wz={f['cmd'][2]:+.2f}",
                     fontsize=9)
    fig.suptitle(f'{sim.experiment}: world-frame snapshots (robot really moves)')
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def render_panel(sim, out_png):
    frames = sim.frames
    t = [f['t'] for f in frames]
    vx = [f['cmd'][0] for f in frames]
    wz = [f['cmd'][2] for f in frames]
    dist = [f['dist'] if f['dist'] is not None else float('nan') for f in frames]
    bear = [f['bear'] if f['bear'] is not None else float('nan') for f in frames]
    status = [f['status'] for f in frames]

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    ax[0][0].plot([f['rx'] for f in frames], [f['ry'] for f in frames], '-o', ms=2,
                  color='tab:blue', label='robot')
    tw = [(f['tw'][0], f['tw'][1]) for f in frames if f['tw'] is not None]
    if tw:
        ax[0][0].plot([p[0] for p in tw], [p[1] for p in tw], '--', color='tab:red',
                      label='target (truth)')
    ax[0][0].set_title('world trajectory (top-down)')
    ax[0][0].set_aspect('equal')
    ax[0][0].grid(True, alpha=0.3)
    ax[0][0].legend()

    ax[0][1].plot(t, vx, color='tab:blue', label='vx (m/s)')
    ax[0][1].plot(t, wz, color='tab:orange', label='wz (rad/s)')
    ax[0][1].axhline(0, color='0.7', lw=0.8)
    ax[0][1].set_title('cmd_vel vs time')
    ax[0][1].set_xlabel('t (s)')
    ax[0][1].grid(True, alpha=0.3)
    em = [i for i, s in enumerate(status) if s == 'EMERGENCY_STOP']
    if em:
        ax[0][1].axvspan(t[em[0]], t[em[-1]], color='red', alpha=0.15, label='EMERGENCY')
    nt = [i for i, s in enumerate(status) if s == 'NO_TARGET']
    if nt:
        ax[0][1].axvspan(t[nt[0]], t[nt[-1]], color='grey', alpha=0.15, label='NO_TARGET')
    ax[0][1].legend()

    ax[1][0].plot(t, dist, color='tab:green', label='range (m)')
    ax[1][0].axhline(FOLLOW_DIST, color='0.5', ls='--', label='follow_dist')
    ax[1][0].set_title('target range')
    ax[1][0].set_xlabel('t (s)')
    ax[1][0].grid(True, alpha=0.3)
    ax[1][0].legend()

    ax[1][1].plot(t, bear, color='tab:purple', label='bearing (deg)')
    ax[1][1].axhline(0, color='0.7', lw=0.8)
    ax[1][1].set_title('target bearing')
    ax[1][1].set_xlabel('t (s)')
    ax[1][1].grid(True, alpha=0.3)
    ax[1][1].legend()

    fig.suptitle(f'rs_follow simulation: {sim.experiment}')
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def render_summary(sims, out_png):
    n = len(sims)
    fig, ax = plt.subplots(n, 2, figsize=(13, 2.6 * n))
    for r, sim in enumerate(sims):
        frames = sim.frames
        t = [f['t'] for f in frames]
        ax[r][0].plot([f['rx'] for f in frames], [f['ry'] for f in frames], '-',
                      color='tab:blue', lw=1.5, label='robot')
        tw = [(f['tw'][0], f['tw'][1]) for f in frames if f['tw'] is not None]
        if tw:
            ax[r][0].plot([p[0] for p in tw], [p[1] for p in tw], '--',
                          color='tab:red', lw=1, label='target')
        ax[r][0].set_aspect('equal')
        ax[r][0].set_title(f"{sim.experiment}: path", fontsize=9)
        ax[r][0].grid(True, alpha=0.3)
        ax[r][0].tick_params(labelsize=7)

        ax[r][1].plot(t, [f['cmd'][0] for f in frames], color='tab:blue', lw=1.2, label='vx')
        ax[r][1].plot(t, [f['cmd'][2] for f in frames], color='tab:orange', lw=1.2, label='wz')
        d = [f['dist'] if f['dist'] is not None else float('nan') for f in frames]
        ax[r][1].plot(t, d, color='tab:green', lw=1, label='range')
        ax[r][1].axhline(0, color='0.8', lw=0.6)
        ax[r][1].set_title(f"{sim.experiment}: cmd_vel & range", fontsize=9)
        ax[r][1].grid(True, alpha=0.3)
        ax[r][1].tick_params(labelsize=7)
        ax[r][1].legend(fontsize=7, loc='upper right')
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def run_one(experiment, duration, outdir, slip=1.0):
    print(f'--- {experiment} (slip={slip}) ---')
    proc = start_node()
    rclpy.init()
    sim = Sim(experiment, duration, slip)
    time.sleep(3.0)
    try:
        while rclpy.ok() and not sim.finished():
            rclpy.spin_once(sim, timeout_sec=0.05)
    finally:
        stop_proc(proc)
    render_gif(sim, os.path.join(outdir, f'{experiment}.gif'))
    render_panel(sim, os.path.join(outdir, f'{experiment}.png'))
    render_filmstrip(sim, os.path.join(outdir, f'{experiment}_frames.png'))
    cmd = [f['cmd'] for f in sim.frames]
    dists = [f['dist'] for f in sim.frames if f['dist'] is not None]
    settled = dists[len(dists) // 3:]
    errs = [abs(d - FOLLOW_DIST) for d in settled] if settled else []
    mx_vx = max([abs(c[0]) for c in cmd], default=0.0)
    mx_wz = max([abs(c[2]) for c in cmd], default=0.0)
    lag = [math.hypot(f['filt'][0] - f['raw'][0], f['filt'][1] - f['raw'][1])
           for f in sim.frames if f['filt'] and f['raw']]
    lag_s = f'filter-vs-raw lag mean={sum(lag)/len(lag)*100:.1f}cm' if lag else 'lag n/a'
    if errs:
        print(f'  frames={len(sim.frames)}  max|vx|={mx_vx:.3f}  max|wz|={mx_wz:.3f}  '
              f'standoff mean={sum(errs)/len(errs)*100:.1f}cm  max={max(errs)*100:.1f}cm  {lag_s}')
    else:
        print(f'  frames={len(sim.frames)}  max|vx|={mx_vx:.3f}  max|wz|={mx_wz:.3f}  '
              f'(no target seen)  {lag_s}')
    tr = [f['dist'] for f in sim.frames if f['dist'] is not None]
    if tr:
        st = max(1, len(tr) // 14)
        print('  range trace(cm): ' + ' '.join(f'{d * 100:.0f}' for d in tr[::st]))
    sim.destroy_node()
    rclpy.shutdown()
    return sim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--experiment', choices=EXPERIMENTS, default='chase')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--duration', type=float, default=13.0)
    ap.add_argument('--slip', type=float, default=1.0,
                    help='translation slip factor: actual motion = cmd * slip')
    ap.add_argument('--no-odom', action='store_true',
                    help='do not let the node use /odom (test cmd-integration fallback)')
    ap.add_argument('--extra', action='append', default=[],
                    help='extra ros param "key:=value" (repeatable)')
    ap.add_argument('--outdir', default=os.path.expanduser('~/dog_follower/rs_follow_sim'))
    args = ap.parse_args()

    global EXTRA_PARAMS
    EXTRA_PARAMS = [x for v in args.extra for x in ('-p', v)]
    if args.no_odom:
        EXTRA_PARAMS += ['-p', 'odom_topic:=/no_odom']

    if not shutil.which('ros2'):
        print('ERROR: source /opt/ros/humble/setup.bash first')
        return 2
    os.makedirs(args.outdir, exist_ok=True)

    if args.all:
        sims = [run_one(e, args.duration, args.outdir, args.slip) for e in EXPERIMENTS]
        render_summary(sims, os.path.join(args.outdir, 'summary.png'))
        print(f'\nwrote {args.outdir}/summary.png and per-experiment gif/png')
    else:
        run_one(args.experiment, args.duration, args.outdir, args.slip)
    return 0


if __name__ == '__main__':
    sys.exit(main())
