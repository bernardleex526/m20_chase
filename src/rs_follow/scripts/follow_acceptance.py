#!/usr/bin/env python3
"""Real-node acceptance checks; run from a sourced ROS 2 workspace.

Each case gets a fresh rs_follow_node, namespace and ROS domain. Clock-pause
checks publish a real, namespaced /clock and measure safety with monotonic time.
"""

import argparse
import json
import math
import os
import secrets
import signal
import struct
import subprocess
import sys
import tempfile
import time

IMPLEMENTED = ('direct_timeout', 'binding', 'estop', 'cloud_loss', 'crossing', 'clock_pause')
DIRECT, FOLLOW = 0, 1  # /rs_follow/control_mode is std_msgs/Int32.


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def stop_process(proc):
    # ros2 run may exit before its child; always address the owned process group.
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=3.0)


def make_harness(namespace, simulated=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped, Twist
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Bool, Int32, String
    from rs_follow_interfaces.srv import BindTarget
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock

    class Harness(Node):
        def __init__(self):
            super().__init__('acceptance_driver', namespace=namespace)
            self.simulated = simulated
            self.sim_seconds = 100.0
            self.clock_running = True
            self.clock_wall = time.monotonic()
            self.clock_pub = self.create_publisher(Clock, 'clock', 10) if simulated else None
            self.cloud_times = []
            self.cloud = self.create_publisher(PointCloud2, 'rslidar_points', qos_profile_sensor_data)
            self.mode = self.create_publisher(Int32, 'rs_follow/control_mode', 10)
            self.enable = self.create_publisher(Bool, 'rs_follow/enable', 10)
            self.estop = self.create_publisher(Bool, 'rs_follow/estop', 10)
            self.direct = self.create_publisher(Twist, 'rs_follow/direct_cmd', 10)
            self.clear = self.create_publisher(Bool, 'rs_follow/clear_target', 10)
            self.bind_client = self.create_client(BindTarget, 'rs_follow/bind')
            self.commands = []
            self.targets = []
            self.control_states = []
            self.statuses = []
            self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
            self.create_subscription(PointStamped, 'rs_follow/target', self.on_target, 10)
            self.create_subscription(String, 'rs_follow/control_state', self.on_control_state, 10)
            self.create_subscription(String, 'rs_follow/status', self.on_status, 10)
            self.cloud_enabled = True
            self.last_cloud = None
            self.last_direct = None
            # Dense deterministic person-sized cluster, no obstacles.
            self.set_clusters((3.0, 0.0))
            self.create_timer(0.1, self.publish_cloud)

        def on_command(self, msg):
            components = (msg.linear.x, msg.linear.y, msg.linear.z,
                          msg.angular.x, msg.angular.y, msg.angular.z)
            self.commands.append((time.monotonic(), components))

        def on_target(self, msg):
            self.targets.append((time.monotonic(), msg.point.x, msg.point.y))

        def on_control_state(self, msg):
            self.control_states.append((time.monotonic(), json.loads(msg.data)))

        def on_status(self, msg):
            self.statuses.append((time.monotonic(), msg.data))

        def set_clusters(self, *centres):
            points = [(cx + ix * 0.025, cy + iy * 0.025, z)
                      for cx, cy in centres for ix in range(-4, 5)
                      for iy in range(-4, 5) for z in (0.2, 0.5, 0.9)]
            self.cloud_data = b''.join(struct.pack('<fff', *p) for p in points)
            self.point_count = len(points)

        def stamp(self):
            if not self.simulated:
                return self.get_clock().now().to_msg()
            ns = round(self.sim_seconds * 1_000_000_000)
            return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

        def publish_clock(self):
            now = time.monotonic()
            if self.clock_running:
                self.sim_seconds += now - self.clock_wall
            self.clock_wall = now
            self.clock_pub.publish(Clock(clock=self.stamp()))

        def publish_cloud(self):
            if not self.cloud_enabled:
                return
            msg = PointCloud2()
            msg.header.stamp = self.stamp()
            msg.header.frame_id = 'rslidar'
            msg.height, msg.width = 1, self.point_count
            msg.fields = [PointField(name=name, offset=offset,
                                     datatype=PointField.FLOAT32, count=1)
                          for name, offset in (('x', 0), ('y', 4), ('z', 8))]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = 12 * self.point_count
            msg.is_dense = True
            msg.data = self.cloud_data
            self.cloud.publish(msg)
            self.last_cloud = time.monotonic()
            self.cloud_times.append((self.last_cloud, msg.header.stamp.sec,
                                     msg.header.stamp.nanosec))

        def pump(self, duration, action=None):
            deadline = time.monotonic() + duration
            next_action = 0.0
            while time.monotonic() < deadline:
                if self.clock_pub is not None:
                    self.publish_clock()
                if action and time.monotonic() >= next_action:
                    action()
                    next_action = time.monotonic() + 0.05
                rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))

        def ready(self, proc):
            deadline = time.monotonic() + 10.0
            publishers = (self.cloud, self.mode, self.enable, self.estop, self.direct)
            if self.clock_pub is not None:
                publishers += (self.clock_pub,)
            while time.monotonic() < deadline:
                require(proc.poll() is None, 'real rs_follow_node exited during startup')
                if (all(p.get_subscription_count() for p in publishers)
                        and self.bind_client.service_is_ready() and self.commands):
                    self.pump(0.6)
                    return
                self.pump(0.05)
            raise AssertionError('node topics/service not ready within 10s')

        def activate(self, mode):
            self.mode.publish(Int32(data=mode))
            self.enable.publish(Bool(data=True))
            self.pump(0.15)

        def command(self):
            msg = Twist()
            msg.linear.x = 0.4
            self.direct.publish(msg)
            self.last_direct = time.monotonic()

        def binding(self, x, y, frame='rslidar', stamp=None):
            request = BindTarget.Request()
            request.point = PointStamped()
            request.point.header.frame_id = frame
            request.point.header.stamp = stamp if stamp is not None else self.stamp()
            request.point.point.x, request.point.point.y = float(x), float(y)
            future = self.bind_client.call_async(request)
            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                self.pump(0.01)
            require(future.done(), 'BindTarget response timed out')
            response = future.result()
            require(response is not None, 'BindTarget returned no response')
            return response

        def moving(self, since):
            require(any(nonzero(values) for stamp, values in self.commands if stamp >= since),
                    'no observed nonzero Twist (case precondition failed)')

        def stopped(self, since, duration, label):
            self.pump(duration)
            observed = [(t, v) for t, v in self.commands if t >= since]
            require(len(observed) >= 3, label + ': missing Twist output')
            require(all(not nonzero(v) for _, v in observed), label + ': nonzero Twist observed')

        def stop_timing(self, trigger, bound, label):
            self.pump(bound + 0.4)
            observed = [(t, v) for t, v in self.commands if t >= trigger]
            require(observed, label + ': no Twist after trigger')
            zeros = [t for t, v in observed if not nonzero(v)]
            require(zeros, label + ': no zero Twist observed')
            first_zero = zeros[0]
            max_nonzero = max((t - trigger for t, v in observed if nonzero(v)), default=0.0)
            print(f'{label}: observed_zero={first_zero - trigger:.4f}s '
                  f'max_nonzero_duration={max_nonzero:.4f}s limit={bound:.2f}s', flush=True)
            require(first_zero - trigger <= bound, label + ': zero deadline exceeded')
            require(all(not nonzero(v) for t, v in observed if t >= first_zero),
                    label + ': motion resumed after zero')

    return Harness()


def nonzero(values):
    require(all(math.isfinite(v) for v in values), 'non-finite Twist')
    return any(abs(v) > 1e-6 for v in values)


def exercise(h, case):
    if case == 'clock_pause':
        exercise_clock_pause(h)
        return
    if case in ('direct_timeout', 'estop'):
        h.activate(DIRECT)
        start = time.monotonic()
        h.pump(0.7, h.command)
        h.moving(start)
        if case == 'direct_timeout':
            h.stop_timing(h.last_direct, 0.35, case)
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Bool, Int32
            start = time.monotonic()
            h.pump(0.4, h.command)
            h.moving(start)
            h.mode.publish(Int32(data=17))
            start = time.monotonic()
            h.pump(0.2, h.command)
            states = [s for t, s in h.control_states if t >= start]
            require(states and all(s['mode'] == DIRECT for s in states),
                    'invalid mode changed the accepted DIRECT mode')
            h.moving(start)
            last_valid = h.last_direct
            invalid = Twist()
            invalid.linear.x = float('nan')
            h.direct.publish(invalid)
            h.stop_timing(last_valid, 0.35, 'NaN direct rejection')
            h.pump(0.3, h.command)
            h.enable.publish(Bool(data=False))
            h.pump(0.1)
            h.stopped(time.monotonic(), 0.15, 'disable stop')
            h.enable.publish(Bool(data=True))
            h.stopped(time.monotonic(), 0.4, 'enable cannot replay old direct command')
            return
        from std_msgs.msg import Bool
        trigger = time.monotonic()
        h.estop.publish(Bool(data=True))
        h.pump(0.1, h.command)
        # Continue fresh input under both asserted and released emergency stop.
        observed = [(t, v) for t, v in h.commands if t >= trigger]
        zeros = [t for t, v in observed if not nonzero(v)]
        require(zeros, 'estop: no immediate zero Twist')
        zero = zeros[0]
        require(zero - trigger <= 0.10, 'estop: stop exceeded 0.10s')
        require(all(not nonzero(v) for t, v in observed if t >= zero), 'estop: motion after zero')
        print(f'estop: observed_zero={zero-trigger:.4f}s '
              f'max_nonzero_duration={max((t-trigger for t,v in observed if nonzero(v)), default=0.0):.4f}s', flush=True)
        start = time.monotonic()
        h.pump(0.5, h.command)
        h.stopped(start, 0.1, 'estop asserted')
        h.estop.publish(Bool(data=False))
        start = time.monotonic()
        h.pump(0.6, h.command)
        h.stopped(start, 0.1, 'estop false without explicit enable')
        h.enable.publish(Bool(data=True))
        h.stopped(time.monotonic(), 0.15, 'explicit enable without new command')
        start = time.monotonic()
        h.pump(0.6, h.command)
        h.moving(start)
        return

    response = h.binding(3.0, 0.0)
    require(response.success and response.reason == 'OK',
            f'valid binding rejected: success={response.success} reason={response.reason!r}')
    require(response.target.header.frame_id == 'rslidar', 'bound response frame is not rslidar')
    require(math.hypot(response.target.point.x - 3.0, response.target.point.y) < 0.3,
            'bound response target is not the real cluster')
    h.activate(FOLLOW)
    start = time.monotonic()
    h.pump(0.8)
    h.moving(start)
    if case == 'cloud_loss':
        h.cloud_enabled = False
        require(h.last_cloud is not None, 'no cloud was sent')
        h.stop_timing(h.last_cloud, 0.55, case)
        h.cloud_enabled = True
        h.stopped(time.monotonic(), 0.6, 'cloud resume cannot replay follow command')
        response = h.binding(3.0, 0.0)
        require(response.success and response.reason == 'OK', 'cloud resume rebind failed')
        h.activate(FOLLOW)
        start = time.monotonic()
        h.pump(0.6)
        h.moving(start)
        from std_msgs.msg import Bool
        h.clear.publish(Bool(data=True))
        h.pump(0.1)
        start = time.monotonic()
        h.stopped(start, 0.4, 'clear_target stop')
        states = [s for t, s in h.control_states if t >= start]
        require(states and all(not s['active'] and not s['target_valid'] for s in states),
                'clear_target did not pause and clear target')
        return
    if case == 'crossing':
        exercise_crossing(h)
        return
    original_data, original_count = h.cloud_data, h.point_count
    if case == 'hard_stop':
        obstacle = [(0.45 + ix * 0.01, iy * 0.02, -0.25)
                    for ix in range(-2, 3) for iy in range(-5, 6)]
        trigger = time.monotonic()
        h.cloud_data += b''.join(struct.pack('<fff', *p) for p in obstacle)
        h.point_count += len(obstacle)
        h.pump(0.25)
        require(any(t >= trigger and status == 'EMERGENCY_STOP'
                    for t, status in h.statuses),
                'hard-stop fixture never triggered EMERGENCY_STOP')
        h.stopped(time.monotonic(), 0.8, 'hardStop cannot be overridden by recovery')
        h.cloud_data, h.point_count = original_data, original_count
        return
    at_distance = [(1.0 + ix * 0.025, iy * 0.025, z)
                   for ix in range(-4, 5) for iy in range(-4, 5)
                   for z in (0.2, 0.5, 0.9)]
    h.cloud_data = b''.join(struct.pack('<fff', *p) for p in at_distance)
    h.point_count = len(at_distance)
    h.pump(0.2)
    response = h.binding(1.0, 0.0)
    require(response.success and response.reason == 'OK', 'follow-distance rebind failed')
    h.pump(2.0)
    start = time.monotonic()
    h.pump(0.5)
    samples = [v for t, v in h.commands if t >= start]
    require(samples and all(abs(v[0]) <= 0.02 for v in samples),
            'linear.x did not settle near zero at follow_dist=1.0')
    print('binding: observed linear.x near zero at follow_dist=1.0', flush=True)
    h.cloud_data, h.point_count = original_data, original_count
    h.pump(0.2)
    response = h.binding(3.0, 0.0)
    require(response.success and response.reason == 'OK', 'restore original binding failed')
    h.pump(0.6)
    for x, y, frame, reason in ((0.0, 4.0, 'rslidar', 'NO_RETURN'),
                                (float('nan'), 0.0, 'rslidar', 'BAD_POINT'),
                                (3.0, 0.0, '', 'BAD_POINT'),
                                (3.0, 0.0, 'missing_frame', 'TF_UNAVAILABLE')):
        response = h.binding(x, y, frame)
        require(not response.success and response.reason == reason,
                f'binding expected {reason}, got {response.success}, {response.reason!r}')
        start = time.monotonic()
        h.pump(0.4)
        targets = [(x, y) for t, x, y in h.targets if t >= start]
        require(targets, f'{reason}: no tracked target after rejected binding')
        require(all(math.hypot(x - 3.0, y) < 0.3 for x, y in targets),
                f'{reason}: changed the previously bound target')
        h.moving(start)
        print(f'binding: {reason} preserved prior target and motion', flush=True)
    displayed_stamp = h.get_clock().now().to_msg()
    h.pump(0.75)  # Keep transport fresh while the selected display frame ages.
    response = h.binding(3.0, 0.0, stamp=displayed_stamp)
    require(not response.success and response.reason == 'STALE_SCAN',
            f'old display selection expected STALE_SCAN, got {response.reason!r}')
    start = time.monotonic()
    h.pump(0.4)
    targets = [(x, y) for t, x, y in h.targets if t >= start]
    require(targets and all(math.hypot(x - 3.0, y) < 0.3 for x, y in targets),
            'stale display selection changed previous target')
    h.moving(start)
    print('binding: stale display selection preserved prior target and motion', flush=True)
    h.cloud_enabled = False
    h.pump(0.65)
    response = h.binding(3.0, 0.0)
    require(not response.success and response.reason == 'STALE_SCAN',
            f'stale binding expected STALE_SCAN, got {response.reason!r}')
    h.stopped(time.monotonic(), 0.2, 'STALE_SCAN stop')
    # Rebind explicitly, then feed fresh unrelated scans: filter predictions must
    # not renew real-observation age while the transport itself stays healthy.
    h.cloud_enabled = True
    h.pump(0.3)
    response = h.binding(3.0, 0.0)
    require(response.success and response.reason == 'OK', 'rebind failed after stale scan')
    h.activate(FOLLOW)
    start = time.monotonic()
    h.pump(0.6)
    h.moving(start)
    trigger = h.last_cloud
    unrelated = [(3.0 + ix * 0.025, 4.0 + iy * 0.025, z)
                 for ix in range(-4, 5) for iy in range(-4, 5)
                 for z in (0.2, 0.5, 0.9)]
    h.cloud_data = b''.join(struct.pack('<fff', *p) for p in unrelated)
    h.point_count = len(unrelated)
    h.stop_timing(trigger, 0.55, 'target observation loss with fresh clouds')
    states = [s for t, s in h.control_states if t >= trigger + 0.55]
    require(states and all(not s['active'] and not s['target_valid'] for s in states),
            'target observation expiry did not latch inactive and clear target')


def exercise_crossing(h):
    # Distinct clusters approach but stay outside the geometry association gate.
    for other_y in (1.6, 1.3, 1.0):
        h.set_clusters((3.0, 0.0), (3.0, other_y))
        start = time.monotonic()
        h.pump(0.35)
        targets = [(x, y) for t, x, y in h.targets if t >= start]
        require(targets and all(math.hypot(x - 3.0, y) < 0.35 for x, y in targets),
                'crossing: nearby distinct cluster displaced bound target')
        h.moving(start)
    trigger = h.last_cloud
    h.set_clusters((3.0, 1.0))
    h.stop_timing(trigger, 0.55, 'crossing original observation timeout')
    start = time.monotonic()
    h.stopped(start, 1.0, 'crossing cannot auto-select remaining cluster')
    states = [s for t, s in h.control_states if t >= start]
    require(states and all(not s['active'] and not s['target_valid'] for s in states),
            'crossing: timeout did not latch paused with no target')
    require(not any(t >= start for t, _, _ in h.targets),
            'crossing: remaining cluster acquired without explicit binding')
    clouds = [t for t, _, _ in h.cloud_times if t >= start]
    require(len(clouds) >= 5, 'crossing: fresh remaining-cluster clouds missing')
    print(f'crossing: nearby separation=1.0m, fresh_clouds={len(clouds)}, '
          'paused=true, auto_selected=false; geometry-only identity limitation: '
          'overlapping/in-gate people can be indistinguishable; this does not '
          'claim persistent person identity through a merged crossing', flush=True)
    response = h.binding(3.0, 1.0)
    require(response.success and response.reason == 'OK', 'crossing: explicit rebind failed')
    h.activate(FOLLOW)
    start = time.monotonic()
    h.pump(0.6)
    h.moving(start)


def exercise_clock_pause(h):
    # Establish live estimator output to make frozen-output checks non-vacuous.
    response = h.binding(3.0, 0.0)
    require(response.success and response.reason == 'OK', 'clock_pause: initial bind failed')
    h.activate(FOLLOW)
    start = time.monotonic()
    h.pump(0.6)
    h.moving(start)
    require(any(t >= start for t, _, _ in h.targets), 'clock_pause: no live estimator output')
    h.activate(DIRECT)
    start = time.monotonic()
    h.pump(0.7, h.command)
    h.moving(start)
    h.clock_running = False
    frozen = (h.stamp().sec, h.stamp().nanosec)
    h.stop_timing(h.last_direct, 0.35, 'clock_pause frozen-clock DIRECT watchdog')
    # Duplicate-stamp safety invalidates tracking, rather than extrapolating or
    # filtering cached measurements repeatedly on elapsed wall time.
    start = time.monotonic()
    h.set_clusters((3.25, 0.0))
    h.stopped(start, 0.6, 'clock_pause no estimator/smoother wall-time advancement')
    states = [s for t, s in h.control_states if t >= start]
    require(states and all(not s['target_valid'] for s in states),
            'clock_pause: stale target remained valid under frozen ROS time')
    require(not any(t >= start for t, _, _ in h.targets),
            'clock_pause: estimator published target while ROS time was frozen')
    clouds = [(sec, ns) for t, sec, ns in h.cloud_times if t >= start]
    require(len(clouds) >= 4 and all(stamp == frozen for stamp in clouds),
            'clock_pause: missing transport-fresh clouds with frozen ROS stamps')
    # Fresh input must not make the smoother ramp on the wall timer.
    h.activate(DIRECT)
    h.pump(0.1)
    start = time.monotonic()
    h.pump(0.6, h.command)
    samples = [v for t, v in h.commands if t >= start]
    require(len(samples) >= 3 and all(not nonzero(v) for v in samples),
            'clock_pause: fresh DIRECT input advanced motion on frozen ROS time')
    require((h.stamp().sec, h.stamp().nanosec) == frozen,
            'clock_pause: driver inadvertently advanced /clock')
    print(f'clock_pause: frozen_stamp={frozen[0]}.{frozen[1]:09d}, '
          f'fresh_clouds={len(clouds)}, fresh_direct_zero_samples={len(samples)}, '
          'target_publications=0; estimator cache invalidated, smoother output '
          'held at zero (internal estimator state is not exposed)', flush=True)
    h.clock_running = True
    h.pump(0.4)
    response = h.binding(3.25, 0.0)
    require(response.success and response.reason == 'OK', 'clock_pause: resume rebind failed')
    h.activate(FOLLOW)
    start = time.monotonic()
    h.pump(0.7)
    h.moving(start)
    require(any(t >= start for t, _, _ in h.targets),
            'clock_pause: estimator did not resume after explicit bind and enable')
    print('clock_pause: advancing /clock plus fresh bind/enable restored '
          'estimator output and smoothed motion', flush=True)


def run_case(case, domain):
    import rclpy
    namespace = 'follow_acceptance_' + secrets.token_hex(6)
    absolute_paths = ('/rslidar_points', '/cmd_vel', '/clicked_point', '/odom', '/clock',
                      '/tf', '/tf_static', '/rs_follow/bind', '/rs_follow/bind_target',
                      '/rs_follow/clear_target', '/rs_follow/enable', '/rs_follow/estop',
                      '/rs_follow/control_mode', '/rs_follow/direct_cmd', '/rs_follow/scan',
                      '/rs_follow/target', '/rs_follow/target_raw', '/rs_follow/status',
                      '/rs_follow/target_marker', '/rs_follow/state', '/rosout', '/parameter_events')
    absolute_paths += ('/rs_follow/control_state', '/rs_follow/cloud_viz')
    args = ['ros2', 'run', 'rs_follow', 'rs_follow_node', '--ros-args',
            '-r', '__ns:=/' + namespace, '-r', '__node:=rs_follow_node']
    for path in absolute_paths:
        args += ['-r', f'{path}:=/{namespace}{path}']
    # Fix only timeout/start-state parameters; leave distance/VFH laws unchanged.
    # Synthetic coordinates are already body-aligned in rslidar; no installation TF.
    args += ['-p', 'control_frame:=rslidar', '-p', 'height_min:=-0.4',
             '-p', 'height_max:=1.8', '-p', 'enable_low_band:=false']
    for name, value in (('active', 'false'), ('direct_cmd_timeout', '0.3'),
                        ('cmd_timeout', '0.5'), ('target_observation_timeout', '0.5'),
                        ('use_sim_time', 'true' if case == 'clock_pause' else 'false')):
        args += ['-p', f'{name}:={value}']
    if case == 'hard_stop':
        # Fixture only: force a collision course instead of VFH avoidance so
        # this exercises actual hardStop arbitration against enabled recovery.
        args += ['-p', 'recovery_enable:=true', '-p', 'recovery_stuck_window:=0.2',
                 '-p', 'vfh_enable:=false']
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    proc, harness, initialized = None, None, False
    with tempfile.TemporaryFile(mode='w+t') as log:
        try:
            rclpy.init(args=[])
            initialized = True
            harness = make_harness(namespace, simulated=case == 'clock_pause')
            if case == 'binding':
                harness.cloud_enabled = False
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                    env=os.environ.copy(), start_new_session=True)
            print(f'{case}: domain={domain} namespace=/{namespace}', flush=True)
            harness.ready(proc)
            if case == 'binding':
                response = harness.binding(3.0, 0.0)
                require(not response.success and response.reason == 'NO_SCAN',
                        f'initial binding expected NO_SCAN, got {response.reason!r}')
                harness.cloud_enabled = True
                harness.pump(0.6)
            exercise(harness, case)
            require(proc.poll() is None, 'node exited during case')
            print(f'{case}: PASS', flush=True)
            return True
        except Exception as exc:
            print(f'{case}: FAIL: {exc}', file=sys.stderr, flush=True)
            log.seek(0)
            lines = log.readlines()
            if lines:
                print('Node log (last 15 lines):\n' + ''.join(lines[-15:]), file=sys.stderr)
            return False
        finally:
            if proc is not None:
                stop_process(proc)
            if harness is not None:
                harness.destroy_node()
            if initialized:
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, choices=IMPLEMENTED + ('all',))
    args = parser.parse_args()
    cases = IMPLEMENTED if args.case == 'all' else (args.case,)
    # Linux DDS participants have a safe range of domains below 101.
    domains = secrets.SystemRandom().sample(range(1, 101), len(cases))
    passed = True
    for case, domain in zip(cases, domains):
        try:
            passed = run_case(case, domain) and passed
            if case == 'binding':
                extra_domain = secrets.choice([d for d in range(1, 101) if d not in domains])
                passed = run_case('hard_stop', extra_domain) and passed
        except Exception as exc:
            print(f'{case}: FAIL: {exc}', file=sys.stderr)
            passed = False
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
