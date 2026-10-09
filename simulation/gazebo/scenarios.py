#!/usr/bin/env python3
"""Physical Gazebo Classic CHAMP scenarios; run after simulation, before follow stack.

Only fixture entities are spawned/moved. The first 16 simulated seconds drive
CHAMP directly; thereafter dog_adapters' actual twist adapter owns /cmd_vel.
The relay copies real PointCloud2 messages unchanged (no synthetic scans/TF).
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Int32, String
from gazebo_msgs.srv import SpawnEntity, SetEntityState, DeleteEntity
from rs_follow_interfaces.srv import BindTarget


class GateError(RuntimeError):
    pass


def attitude(q):
    return (math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


class Scenarios(Node):
    def __init__(self, args):
        super().__init__('champ_physical_scenarios', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.out = Path(args.output_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.stream = (self.out/'samples.csv').open('w', newline='')
        self.csv = csv.DictWriter(self.stream, fieldnames=[
            'wall_s', 'sim_s', 'case', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
            'roll', 'pitch', 'yaw', 'target_x', 'target_y', 'target_distance',
            'obstacle_x', 'obstacle_y', 'obstacle_distance', 'vx', 'vy', 'wz',
            'follow_vx', 'follow_vy', 'follow_wz', 'odom_age', 'cloud_age',
            'joint_age', 'command_age', 'joints', 'control_state', 'state', 'status'])
        self.csv.writeheader()
        self.started = time.monotonic()
        self.case = 'startup'
        self.pose = None
        self.joints = None
        self.odom_at = self.cloud_at = self.joint_at = self.command_at = 0.
        self.control_at = self.follow_at = 0.
        self.control = {}
        self.state = self.status = ''
        self.cmd = self.follow_cmd = (0., 0., 0.)
        self.target = self.obstacle = None
        self.relay_enabled = True
        self.samples = []
        self.results = []
        self.stack = None
        self.stack_log = None
        self.spawned = []
        self.relay = self.create_publisher(PointCloud2, '/fixtures/points', qos_profile_sensor_data)
        self.raw_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.direct = self.create_publisher(Twist, '/rs_follow/direct_cmd', 10)
        self.enable = self.create_publisher(Bool, '/rs_follow/enable', 10)
        self.estop = self.create_publisher(Bool, '/rs_follow/estop', 10)
        self.mode = self.create_publisher(Int32, '/rs_follow/control_mode', 10)
        self.bind = self.create_client(BindTarget, '/rs_follow/bind')
        self.spawn = self.create_client(SpawnEntity, args.spawn_service)
        self.move = self.create_client(SetEntityState, args.state_service)
        self.delete = self.create_client(DeleteEntity, args.delete_service)
        self.create_subscription(Odometry, '/odom/ground_truth', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Twist, '/rs_follow/cmd_vel', self.on_follow, 10)
        self.create_subscription(String, '/rs_follow/control_state', self.on_control, 10)
        self.create_subscription(String, '/rs_follow/state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(String, '/rs_follow/status', lambda m: setattr(self, 'status', m.data), 10)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        self.odom_at = time.monotonic()

    def on_joints(self, msg):
        self.joints = msg
        self.joint_at = time.monotonic()

    def on_cloud(self, msg):
        self.cloud_at = time.monotonic()
        if self.relay_enabled:
            self.relay.publish(msg)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.command_at = time.monotonic()

    def on_follow(self, msg):
        self.follow_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.follow_at = time.monotonic()

    def on_control(self, msg):
        self.control = json.loads(msg.data)
        self.control_at = time.monotonic()

    def check(self, condition, message):
        if not condition:
            raise GateError(message)

    def upright(self):
        if self.pose is None:
            return
        p = self.pose.position
        r, pitch, _ = attitude(self.pose.orientation)
        self.check(all(math.isfinite(v) for v in (p.x, p.y, p.z, r, pitch)), 'nonfinite robot ground truth')
        self.check(p.z > self.args.min_height and abs(r) < .65 and abs(pitch) < .65,
                   'fall gate: low base height or roll/pitch > .65 rad')

    def sample(self):
        if self.pose is None:
            return
        p, q = self.pose.position, self.pose.orientation
        r, pitch, yaw = attitude(q)
        wall = time.monotonic()
        row = dict(wall_s=wall-self.started, sim_s=self.now(), case=self.case,
                   x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                   roll=r, pitch=pitch, yaw=yaw,
                   target_x=self.target[0] if self.target else '',
                   target_y=self.target[1] if self.target else '',
                   target_distance=math.hypot(p.x-self.target[0], p.y-self.target[1]) if self.target else '',
                   obstacle_x=self.obstacle[0] if self.obstacle else '',
                   obstacle_y=self.obstacle[1] if self.obstacle else '',
                   obstacle_distance=math.hypot(p.x-self.obstacle[0], p.y-self.obstacle[1]) if self.obstacle else '',
                   vx=self.cmd[0], vy=self.cmd[1], wz=self.cmd[2],
                   follow_vx=self.follow_cmd[0], follow_vy=self.follow_cmd[1], follow_wz=self.follow_cmd[2],
                   odom_age=wall-self.odom_at, cloud_age=wall-self.cloud_at,
                   joint_age=wall-self.joint_at, command_age=wall-self.command_at,
                   joints=json.dumps(dict(zip(self.joints.name, self.joints.position))) if self.joints else '{}',
                   control_state=json.dumps(self.control), state=self.state, status=self.status)
        self.samples.append(row)
        self.csv.writerow(row)

    def pump(self, seconds, action=None, guard=True):
        start, wall = self.now(), time.monotonic()
        next_action = start
        next_sample = start
        while self.now()-start < seconds:
            self.check(time.monotonic()-wall < self.args.wall_timeout, 'sim time stalled or case wall timeout')
            if self.stack is not None:
                self.check(self.stack.poll() is None, 'follow_stack exited; inspect follow_stack.log')
            rclpy.spin_once(self, timeout_sec=.01)
            if self.now() >= next_action:
                if action:
                    action(self.now()-start)
                next_action = self.now()+.05
            if self.now() >= next_sample:
                self.sample()
                next_sample = self.now()+.05
                if guard:
                    self.upright()
                    self.check(time.monotonic()-self.odom_at < 2., 'ground truth odometry stale')
                    if self.case not in ('startup', 'stand_sanity'):
                        self.check(time.monotonic()-self.joint_at < 2., 'physical joint states stale')
                    self.check(all(math.isfinite(v) for v in self.cmd+self.follow_cmd), 'nonfinite actual/controller command')
                    if self.stack is not None:
                        self.check(abs(self.cmd[0]) <= .201,
                                   'configured actual .20m/s command ceiling exceeded')

    def call(self, client, request):
        self.check(client.wait_for_service(timeout_sec=10.), 'missing service '+client.srv_name)
        future = client.call_async(request)
        deadline = time.monotonic()+10.
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
        self.check(future.done(), 'service timeout '+client.srv_name)
        result = future.result()
        self.check(result is not None and result.success,
                   'service rejected '+client.srv_name+': '+str(result))
        return result

    def fixture(self, name, shape, xy):
        self.check(name in ('scenario_target', 'scenario_obstacle'), 'fixture-only entity allowlist')
        geom = '<cylinder><radius>0.18</radius><length>1.4</length></cylinder>' if shape == 'target' else '<box><size>0.20 0.06 0.60</size></box>'
        z = .7 if shape == 'target' else .3
        yaw = attitude(self.pose.orientation)[2] if shape == 'obstacle' else 0.
        if name not in self.spawned:
            req = SpawnEntity.Request()
            req.name = name
            req.xml = f'<sdf version="1.6"><model name="{name}"><static>true</static><link name="body"><collision name="collision"><geometry>{geom}</geometry></collision><visual name="visual"><geometry>{geom}</geometry></visual></link></model></sdf>'
            req.initial_pose.position.x, req.initial_pose.position.y = xy
            req.initial_pose.position.z = z
            req.initial_pose.orientation.z = math.sin(yaw/2.)
            req.initial_pose.orientation.w = math.cos(yaw/2.)
            req.reference_frame = 'world'
            self.call(self.spawn, req)
            self.spawned.append(name)
        else:
            req = SetEntityState.Request()
            req.state.name = name
            req.state.reference_frame = 'world'
            req.state.pose.position.x, req.state.pose.position.y = xy
            req.state.pose.position.z = z
            req.state.pose.orientation.z = math.sin(yaw/2.)
            req.state.pose.orientation.w = math.cos(yaw/2.)
            self.call(self.move, req)
        if shape == 'target':
            self.target = tuple(xy)
        else:
            self.obstacle = tuple(xy)

    def ahead(self, distance, lateral=0.):
        p = self.pose.position
        yaw = attitude(self.pose.orientation)[2]
        return (p.x+distance*math.cos(yaw)-lateral*math.sin(yaw),
                p.y+distance*math.sin(yaw)+lateral*math.cos(yaw))

    def command(self, publisher, vx=.15):
        msg = Twist()
        msg.linear.x = vx
        publisher.publish(msg)

    def set_mode(self, mode):
        self.enable.publish(Bool(data=False))
        self.pump(.15)
        self.mode.publish(Int32(data=mode))
        self.pump(.3)
        self.check(self.control.get('mode') == mode, 'mode handshake failed')
        self.enable.publish(Bool(data=True))
        self.pump(.3)
        self.check(self.control.get('active') and self.control.get('mode') == mode, 'enable/mode handshake failed')

    def bind_follow(self, distance=2.):
        self.enable.publish(Bool(data=False))
        self.fixture('scenario_target', 'target', self.ahead(distance))
        self.pump(1.)
        self.upright()
        yaw = attitude(self.pose.orientation)[2]
        dx, dy = self.target[0]-self.pose.position.x, self.target[1]-self.pose.position.y
        req = BindTarget.Request()
        req.point = PointStamped()
        req.point.header.frame_id = 'base_link'
        req.point.header.stamp = self.get_clock().now().to_msg()
        req.point.point.x = math.cos(yaw)*dx+math.sin(yaw)*dy
        req.point.point.y = -math.sin(yaw)*dx+math.cos(yaw)*dy
        req.point.point.z = .5
        response = self.call(self.bind, req)
        self.check(response.reason == 'OK', 'binding reason '+response.reason)
        self.set_mode(1)
        self.check(self.control.get('target_valid'), 'no observed real-cloud target after binding')

    def rows(self, start):
        return self.samples[start:]

    def moving(self, start):
        rows = self.rows(start)
        self.check(any(abs(r['vx']) > .03 for r in rows), 'no actual /cmd_vel motion command prerequisite')
        self.check(any(abs(r['follow_vx']) > .03 for r in rows), 'no controller command prerequisite')
        self.check(rows and math.hypot(rows[-1]['x']-rows[0]['x'], rows[-1]['y']-rows[0]['y']) > .03,
                   'commanded but no physical displacement prerequisite')

    def stopped(self, start, wall_trigger, bound):
        rows = [r for r in self.rows(start) if self.started+r['wall_s'] >= wall_trigger+bound]
        self.check(len(rows) >= 3, 'missing post-deadline physical command samples')
        self.check(all(max(abs(r[k]) for k in ('vx', 'vy', 'wz')) < .001 for r in rows), 'actual command did not remain zero after deadline')
        self.check(all(r['command_age'] < 1. for r in rows), 'zero inferred from stale command')
        drift = math.hypot(rows[-1]['x']-rows[0]['x'], rows[-1]['y']-rows[0]['y'])
        self.check(drift < .15, f'physical stop drift {drift:.3f}m exceeds .15m')
        return {'post_deadline_drift_m': drift, 'stop_deadline_wall_s': bound}

    def independent_reset(self):
        # Control assertion failures do not erase independent safety evidence.
        # Physical prerequisites remain fatal, including a fall during a case.
        self.upright()
        wall = time.monotonic()
        self.check(wall-self.odom_at < 2., 'ground truth odometry stale prerequisite')
        self.check(wall-self.joint_at < 2., 'physical joint states stale prerequisite')
        self.check(wall-self.cloud_at < 2., 'real cloud stale prerequisite')
        self.check(self.joints and len(self.joints.position) >= 12, 'physical joints prerequisite missing')
        self.relay_enabled = True
        self.enable.publish(Bool(data=False))
        self.estop.publish(Bool(data=True))
        self.pump(.5)
        self.estop.publish(Bool(data=False))
        self.pump(.5)
        self.check(not self.control.get('active') and not self.control.get('estop'),
                   'safe reset handshake prerequisite failed')
        if 'scenario_obstacle' in self.spawned:
            self.fixture('scenario_obstacle', 'obstacle', (30., 30.))

    def run_case(self, name, function):
        self.case = name
        start = len(self.samples)
        began = self.now()
        result = {'case': name, 'status': 'fail'}
        try:
            result.update(function(start) or {})
            self.upright()
            result['status'] = 'pass'
        except GateError as exc:
            result['reason'] = str(exc)
            raise
        finally:
            rows = self.rows(start)
            result.update(sim_duration_s=self.now()-began, samples=len(rows))
            if rows:
                result.update(min_base_z_m=min(r['z'] for r in rows),
                              max_abs_roll_rad=max(abs(r['roll']) for r in rows),
                              max_abs_pitch_rad=max(abs(r['pitch']) for r in rows),
                              displacement_m=math.hypot(rows[-1]['x']-rows[0]['x'], rows[-1]['y']-rows[0]['y']))
                result.update(max_abs_actual_vx_mps=max(abs(r['vx']) for r in rows),
                              max_abs_raw_follow_vx_mps=max(abs(r['follow_vx']) for r in rows),
                              raw_follow_ceiling_overshoot_observed=any(abs(r['follow_vx']) > .201 for r in rows))
            self.results.append(result)
            self.stream.flush()

    def stand(self, start):
        self.pump(8., lambda _: self.command(self.raw_cmd, 0.))
        rows = self.rows(start)
        self.check(self.joints and len(self.joints.position) >= 12, 'missing 12 physical joint states')
        self.check(time.monotonic()-self.joint_at < 2., 'joint states stale')
        self.check(time.monotonic()-self.cloud_at < 2., 'real velodyne cloud stale')
        self.check(len(rows) >= 20, 'insufficient standing ground truth samples')
        self.check(abs(rows[-1]['z']-rows[-20]['z']) < .04, 'standing height not settled')

    def walk(self, start):
        initial = (self.pose.position.x, self.pose.position.y)
        yaw = attitude(self.pose.orientation)[2]
        self.pump(8., lambda _: self.command(self.raw_cmd))
        dx, dy = self.pose.position.x-initial[0], self.pose.position.y-initial[1]
        progress = dx*math.cos(yaw)+dy*math.sin(yaw)
        self.check(progress > .35, f'CHAMP physical walk prerequisite failed: forward progress {progress:.3f}m')
        rows = self.rows(start)
        values = [json.loads(r['joints']) for r in rows]
        varying = sum(max(v.get(name, 0.) for v in values)-min(v.get(name, 0.) for v in values) > .05 for name in values[0])
        self.check(varying >= 4, 'joint motion prerequisite failed')
        self.pump(2., lambda _: self.command(self.raw_cmd, 0.))
        return {'forward_progress_m': progress, 'varying_joints': varying}

    def start_stack(self):
        self.check(self.raw_cmd.get_subscription_count() > 0, 'missing CHAMP command consumer')
        self.check(self.count_publishers('/rs_follow/cmd_vel') == 0, 'follow stack already running: start this driver before stack')
        self.stack_log = (self.out/'follow_stack.log').open('w')
        self.stack = subprocess.Popen(['ros2', 'launch', 'dog_adapters', 'follow_stack.launch.py',
            'adapter:=twist', 'with_web:=false', 'robot_config:='+str(Path(self.args.follow_config).resolve())],
            stdout=self.stack_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+30.
        while time.monotonic() < deadline:
            self.pump(.1)
            if self.bind.service_is_ready() and self.control_at and self.follow_at and self.relay.get_subscription_count():
                self.check(not self.control.get('active'), 'stack did not start safe-disabled')
                return
        raise GateError('follow stack startup prerequisite failed; inspect follow_stack.log')

    def stationary(self, start):
        self.bind_follow(3.)
        begin = len(self.samples)
        self.pump(self.args.approach_seconds)
        self.moving(begin)
        rows = self.rows(begin)
        self.check(rows[-1]['target_distance'] < 1.5, 'did not approach 3m stationary target to hold band')
        self.check(rows[0]['target_distance']-rows[-1]['target_distance'] > 1., 'insufficient physical approach')
        hold = len(self.samples)
        self.pump(5.)
        tail = self.rows(hold)
        self.check(all(.65 < r['target_distance'] < 1.5 for r in tail), 'hold standoff outside .65..1.5m')
        self.check(sum(abs(r['vx']) for r in tail)/len(tail) < .06, 'hold command not settled')
        return {'final_center_distance_m': tail[-1]['target_distance']}

    def departure(self, start):
        xy = self.target
        yaw = attitude(self.pose.orientation)[2]
        self.pump(12., lambda t: self.fixture('scenario_target', 'target',
            (xy[0]+.06*t*math.cos(yaw), xy[1]+.06*t*math.sin(yaw))))
        self.moving(start)
        self.check(self.control.get('target_valid'), 'lost target during slow departure')
        self.check(self.samples[-1]['target_distance'] < 1.8, 'departure gap exceeded 1.8m')

    def turn(self, start):
        xy = self.target
        yaw = attitude(self.pose.orientation)[2]
        initial_yaw = yaw
        self.pump(15., lambda t: self.fixture('scenario_target', 'target',
            (xy[0]+.03*t*math.cos(yaw)-.04*t*math.sin(yaw),
             xy[1]+.03*t*math.sin(yaw)+.04*t*math.cos(yaw))))
        rows = self.rows(start)
        self.check(any(abs(r['wz']) > .03 for r in rows), 'no actual turning command')
        final_yaw = attitude(self.pose.orientation)[2]
        delta = math.atan2(math.sin(final_yaw-initial_yaw), math.cos(final_yaw-initial_yaw))
        self.check(delta > .1, f'physical turn insufficient or wrong direction {delta:.3f}rad')
        self.check(self.control.get('target_valid'), 'lost target during turn')
        return {'yaw_change_rad': delta}

    def obstacle_case(self, start):
        self.bind_follow(2.)
        moving = len(self.samples)
        self.pump(2.)
        self.moving(moving)
        # Tight corridor = robot half-width .145 + stop_margin .05 = .195.
        # Box spans y=.11..17, inside it but clear of the target bearing;
        # front surface x=.40 is outside selfmask .31 and clearance .15.
        # Physical lidar must publish its real .30m near range, not drop <.9m.
        self.fixture('scenario_obstacle', 'obstacle', self.ahead(.50, .14))
        trigger, stop_start = time.monotonic(), len(self.samples)
        try:
            self.pump(3.)
            metrics = self.stopped(stop_start, trigger, .8)
            evidence = []
            for row in self.rows(stop_start):
                state = dict(item.split('=', 1) for item in row['state'].split() if '=' in item)
                control = json.loads(row['control_state'])
                clearance = float(state.get('d_stop', '-1'))
                if (control.get('active') and control.get('target_valid')
                        and 0. <= clearance <= .25
                        and float(state.get('vlim', '-1')) == 0.
                        and abs(row['vx']) < .001):
                    evidence.append(clearance)
            self.check(evidence, 'no active target-valid governor hardstop with d_stop<=.25 and actual zero command')
            metrics.update(hardstop_evidence_samples=len(evidence), min_hardstop_clearance_m=min(evidence))
            return metrics
        finally:
            self.fixture('scenario_obstacle', 'obstacle', (30., 30.))

    def cloud_loss(self, start):
        self.bind_follow(2.)
        moving = len(self.samples)
        self.pump(2.)
        self.moving(moving)
        trigger, stopped = time.monotonic(), len(self.samples)
        self.relay_enabled = False
        try:
            self.pump(3.)
            metrics = self.stopped(stopped, trigger, .8)
            self.check(time.monotonic()-self.cloud_at < 2., 'raw sensor died: relay dropout case not isolated')
            self.check(not self.control.get('target_valid'), 'cloud loss left target valid')
            return metrics
        finally:
            self.relay_enabled = True

    def direct_loss(self, start):
        self.pump(1.)
        self.set_mode(0)
        moving = len(self.samples)
        self.pump(3., lambda _: self.command(self.direct))
        self.moving(moving)
        trigger, stopped = time.monotonic(), len(self.samples)
        self.pump(3.)
        return self.stopped(stopped, trigger, .5)

    def estop_case(self, start):
        self.set_mode(0)
        moving = len(self.samples)
        self.pump(2., lambda _: self.command(self.direct))
        self.moving(moving)
        trigger, stopped = time.monotonic(), len(self.samples)
        self.estop.publish(Bool(data=True))
        self.pump(2., lambda _: self.command(self.direct))
        metrics = self.stopped(stopped, trigger, .25)
        self.check(self.control.get('estop') and not self.control.get('active'), 'estop not latched safe')
        self.estop.publish(Bool(data=False))
        trigger, stopped = time.monotonic(), len(self.samples)
        self.pump(2., lambda _: self.command(self.direct))
        self.stopped(stopped, trigger, .25)
        self.check(not self.control.get('active'), 'release automatically reenabled motion')
        self.set_mode(0)
        moving = len(self.samples)
        self.pump(2., lambda _: self.command(self.direct))
        self.moving(moving)
        metrics['explicit_reenable_resumed_physical_motion'] = True
        return metrics

    def shutdown_safe(self):
        self.enable.publish(Bool(data=False))
        self.estop.publish(Bool(data=True))
        for _ in range(10):
            self.command(self.raw_cmd, 0.)
            rclpy.spin_once(self, timeout_sec=.02)
        if self.stack:
            try:
                os.killpg(self.stack.pid, signal.SIGTERM)
                self.stack.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(self.stack.pid, signal.SIGKILL)
                self.stack.wait(timeout=5.)
            except ProcessLookupError:
                pass
        for name in self.spawned:
            if self.delete.service_is_ready():
                req = DeleteEntity.Request()
                req.name = name
                try:
                    self.call(self.delete, req)
                except Exception:
                    pass
        self.stream.close()
        if self.stack_log:
            self.stack_log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--follow-config', default=str(Path(__file__).with_name('follow.yaml')))
    parser.add_argument('--cloud-topic', default='/velodyne/velodyne_points')
    parser.add_argument('--approach-seconds', type=float, default=30.)
    parser.add_argument('--wall-timeout', type=float, default=300., help='per phase wall watchdog; durations otherwise use simulation time')
    parser.add_argument('--min-height', type=float, default=.10, help='base_link fall height in world meters')
    parser.add_argument('--spawn-service', default='/spawn_entity')
    parser.add_argument('--state-service', default='/gazebo/set_entity_state')
    parser.add_argument('--delete-service', default='/delete_entity')
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '73')
    rclpy.init()
    driver = Scenarios(args)
    cases = [('stand_sanity', driver.stand), ('walk_sanity', driver.walk),
             ('stationary_approach_hold', driver.stationary), ('slow_departure', driver.departure),
             ('turn', driver.turn), ('obstacle_hardstop', driver.obstacle_case),
             ('cloud_loss', driver.cloud_loss), ('direct_loss', driver.direct_loss),
             ('estop_release', driver.estop_case)]
    failure = None
    try:
        deadline = time.monotonic()+30.
        while (driver.pose is None or driver.now() <= 0.) and time.monotonic() < deadline:
            rclpy.spin_once(driver, timeout_sec=.05)
        driver.check(driver.pose is not None and driver.now() > 0., 'missing ground truth /clock prerequisite')
        driver.check(driver.count_publishers('/rs_follow/cmd_vel') == 0, 'start driver before follow stack: sanity requires sole command ownership')
        previous_failed = False
        control_failures = []
        for index, (name, action) in enumerate(cases):
            if index == 2:
                driver.start_stack()
            prepare_follow = previous_failed and name in ('slow_departure', 'turn')
            def exercise(start, action=action, prepare_follow=prepare_follow):
                if prepare_follow:
                    driver.bind_follow(2.)
                return action(start)
            try:
                driver.run_case(name, exercise)
                previous_failed = False
            except GateError as exc:
                if index < 2 or any(gate in str(exc) for gate in (
                        'fall gate', 'nonfinite robot ground truth', 'ground truth odometry stale',
                        'physical joint states stale', 'sim time stalled', 'follow_stack exited')):
                    raise
                control_failures.append(name+': '+str(exc))
                previous_failed = True
                driver.independent_reset()
        if control_failures:
            failure = '; '.join(control_failures)
    except (Exception, KeyboardInterrupt) as exc:
        failure = f'{type(exc).__name__}: {exc}'
    finally:
        driver.shutdown_safe()
        attempted = {r['case'] for r in driver.results}
        for name, _ in cases:
            if name not in attempted:
                driver.results.append({'case': name, 'status': 'blocked', 'reason': 'prerequisite gate: '+str(failure)})
        metrics = {'status': 'fail' if failure else 'pass', 'failure': failure,
                   'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
                   'physics_contract': 'CHAMP joints and Gazebo contact dynamics; never sets robot state',
                   'cloud_contract': 'unchanged real '+args.cloud_topic+' relay, forwarding intentionally dropped only for cloud_loss',
                   'command_contract': 'sanity /cmd_vel; follow rs_follow -> dog_adapters twist_adapter -> /cmd_vel',
                   'distance_definition': 'world XY robot base to cylinder center (radius .18m)',
                   'cases': driver.results}
        (driver.out/'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False)+'\n')
        driver.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'status': metrics['status'], 'metrics': str(driver.out/'metrics.json'), 'failure': failure}))
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
