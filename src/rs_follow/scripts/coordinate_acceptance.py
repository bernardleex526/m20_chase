#!/usr/bin/env python3
"""Real rs_follow_node coordinate checks in isolated localhost DDS, with no robot.

Run from the sourced workspace: coordinate_acceptance.py --case all.
"""
import argparse
import math
import os
import secrets
import struct
import subprocess
import sys
import tempfile
import time

from follow_acceptance import require, nonzero, stop_process

CASES = ('stamped_transform', 'same_frame', 'missing_tf', 'malformed', 'nan',
         'empty', 'backwards', 'visualization')


def make_harness(namespace):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PointStamped, TransformStamped, Twist
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from std_msgs.msg import Bool, Int32
    from tf2_msgs.msg import TFMessage
    from rs_follow_interfaces.srv import BindTarget

    class Harness(Node):
        def __init__(self):
            super().__init__('coordinate_driver', namespace=namespace)
            self.cloud = self.create_publisher(PointCloud2, 'rslidar_points', qos_profile_sensor_data)
            self.tf = self.create_publisher(TFMessage, 'tf', 10)
            self.mode = self.create_publisher(Int32, 'rs_follow/control_mode', 10)
            self.enable = self.create_publisher(Bool, 'rs_follow/enable', 10)
            self.direct = self.create_publisher(Twist, 'rs_follow/direct_cmd', 10)
            self.bind_client = self.create_client(BindTarget, 'rs_follow/bind')
            self.commands, self.scans, self.targets, self.raw, self.markers, self.viz = [], [], [], [], [], []
            from visualization_msgs.msg import Marker
            for typ, topic, dest, qos in (
                    (Twist, 'cmd_vel', self.commands, 10),
                    (LaserScan, 'rs_follow/scan', self.scans, 10),
                    (PointStamped, 'rs_follow/target', self.targets, 10),
                    (PointStamped, 'rs_follow/target_raw', self.raw, 10),
                    (Marker, 'rs_follow/target_marker', self.markers, 10),
                    (PointCloud2, 'rs_follow/cloud_viz', self.viz, qos_profile_sensor_data)):
                self.create_subscription(typ, topic, lambda m, d=dest: d.append((time.monotonic(), m)), qos)

        def pump(self, duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)

        def stamp(self, offset=0.0):
            ns = self.get_clock().now().nanoseconds + int(offset * 1e9)
            return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

        def transform(self, stamp, yaw, translation):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'base_link', 'sensor'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.tf.publish(TFMessage(transforms=[t]))

        def send(self, points, frame='base_link', stamp=None, malformed=False):
            m = PointCloud2()
            m.header.frame_id, m.header.stamp = frame, stamp or self.stamp()
            m.height, m.width, m.point_step = 1, len(points), 12
            m.row_step = m.width * 12
            m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                        for n, o in (('x', 0), ('y', 4), ('z', 8))]
            m.data = b''.join(struct.pack('<fff', *p) for p in points)
            if malformed:
                m.data = m.data[:-1]
            m.is_dense = False
            self.cloud.publish(m)
            return m.header.stamp

        def bind(self, x, y, frame='base_link', stamp=None):
            request = BindTarget.Request()
            request.point.header.frame_id, request.point.header.stamp = frame, stamp or self.stamp()
            request.point.point.x, request.point.point.y = float(x), float(y)
            f = self.bind_client.call_async(request)
            deadline = time.monotonic() + 3
            while not f.done() and time.monotonic() < deadline:
                self.pump(0.01)
            require(f.done() and f.result() is not None, 'binding response timeout')
            return f.result()

        def ready(self, proc):
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                require(proc.poll() is None, 'node exited during startup')
                if (all(p.get_subscription_count() for p in (self.cloud, self.mode, self.enable, self.direct))
                        and self.bind_client.service_is_ready() and self.commands):
                    return
                self.pump(0.05)
            raise AssertionError('node discovery timeout')

        def moving(self):
            self.mode.publish(Int32(data=0))
            self.enable.publish(Bool(data=True))
            start = time.monotonic()
            deadline = start + 0.5
            while time.monotonic() < deadline:
                self.send(person())
                cmd = Twist()
                cmd.linear.x = 0.4
                self.direct.publish(cmd)
                self.pump(0.04)
            require(any(nonzero(components(m)) for t, m in self.commands if t >= start),
                    'motion precondition failed')
            require(self.commands and time.monotonic() - self.commands[-1][0] < 0.1 and
                    nonzero(components(self.commands[-1][1])),
                    'latest command is not fresh motion before invalid-cloud trigger')

        def stopped(self, trigger):
            self.pump(0.15)
            msgs = [(t, m) for t, m in self.commands if t >= trigger]
            require(msgs, 'no command after invalid cloud')
            zeros = [t for t, m in msgs if not nonzero(components(m))]
            require(zeros and zeros[0] - trigger < 0.1, 'invalid cloud did not stop immediately')
            print(f'immediate_stop: observed_zero={zeros[0] - trigger:.4f}s limit=0.10s', flush=True)
            require(all(not nonzero(components(m)) for t, m in msgs if t >= zeros[0]),
                    'motion resumed after invalid cloud')
    return Harness()


def components(m):
    return (m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z)


def person():
    return [(3 + i * 0.025, j * 0.025, z)
            for i in range(-4, 5) for j in range(-4, 5) for z in (0.3, 0.6, 0.9)]


def stamp_key(stamp):
    return stamp.sec, stamp.nanosec


def exercise(h, case):
    if case in ('stamped_transform', 'same_frame'):
        expected_stamp = h.stamp(-0.25)
        from builtin_interfaces.msg import Time
        ns = expected_stamp.sec * 1000000000 + expected_stamp.nanosec + 1000000
        next_stamp = Time(sec=ns // 1000000000, nanosec=ns % 1000000000)
        points, frame = person(), 'base_link'
        if case == 'stamped_transform':
            # Older transform maps (sensor x,y,z) -> (-y+1,x+2,z+0.2).
            # Latest transform is intentionally different: latest-TF shortcuts fail.
            h.transform(expected_stamp, math.pi / 2, (1.0, 2.0, 0.2))
            h.transform(next_stamp, math.pi / 2, (1.0, 2.0, 0.2))
            h.transform(h.stamp(), 0.0, (12.0, 0.0, 0.0))
            points = [(y - 2, 1 - x, z - 0.2) for x, y, z in person()]
            frame = 'sensor'
            h.pump(0.08)
        h.send(points, frame, expected_stamp)
        h.pump(0.08)
        response = h.bind(-2.0, -2.0, 'sensor', expected_stamp) if frame == 'sensor' else h.bind(3, 0, stamp=expected_stamp)
        require(response.success and response.reason == 'OK', f'valid bind rejected: {response.reason}')
        require(response.target.header.frame_id == 'base_link', 'bind response frame wrong')
        require(stamp_key(response.target.header.stamp) == stamp_key(expected_stamp), 'bind response stamp wrong')
        require(abs(response.target.point.x - 3) < 0.2 and abs(response.target.point.y) < 0.2,
                'rotation+translation binding geometry wrong')
        # A strictly newer cloud supplies the first actual target observation.
        h.send(points, frame, next_stamp)
        h.pump(0.12)
        for label, messages in (('scan', h.scans), ('target', h.targets), ('raw', h.raw), ('marker', h.markers)):
            require(messages, f'{label} not published')
            allowed = {stamp_key(expected_stamp), stamp_key(next_stamp)} if label == 'scan' else {stamp_key(next_stamp)}
            require(all(m.header.frame_id == 'base_link' and
                        stamp_key(m.header.stamp) in allowed for _, m in messages),
                    f'{label} header not in stamped control frame')
        return
    if case == 'visualization':
        # More than 6000 distinct 0.1m voxels plus a nearer body return on
        # the target ray. Self filtering must happen before polar minima.
        points = [(1 + i * 0.11, -5 + j * 0.11, 0.3 + k * 0.11)
                  for i in range(70) for j in range(90) for k in range(2)]
        points += person() + [(0.35, 0.0, 0.3)]
        start = time.monotonic()
        sent = set()
        while time.monotonic() - start < 1.2:
            sent.add(stamp_key(h.send(points)))
            h.pump(0.025)
        require(h.viz and len(h.viz) <= 8, 'visualization absent or exceeds 5Hz')
        require(all(b[0] - a[0] >= 0.18 for a, b in zip(h.viz, h.viz[1:])),
                'visualization publishes faster than 5Hz')
        for _, m in h.viz:
            require(m.header.frame_id == 'base_link' and stamp_key(m.header.stamp) in sent, 'visualization header wrong')
            require(0 < m.width * m.height <= 6000, 'visualization cap violated')
            offsets = {f.name: f.offset for f in m.fields}
            require(set(('x', 'y', 'z')) <= offsets.keys(), 'visualization is not XYZ')
            voxels = set()
            for row in range(m.height):
                for col in range(m.width):
                    base = row * m.row_step + col * m.point_step
                    xyz = tuple(struct.unpack_from('>f' if m.is_bigendian else '<f', m.data, base + offsets[n])[0]
                                for n in ('x', 'y', 'z'))
                    require(all(math.isfinite(v) for v in xyz), 'nonfinite display coordinate')
                    require(not (-0.4 <= xyz[0] <= 0.4 and -0.3 <= xyz[1] <= 0.3), 'self point in display')
                    voxel = tuple(math.floor(v / 0.1) for v in xyz)
                    require(voxel not in voxels, 'visualization has duplicate 0.1m voxel')
                    voxels.add(voxel)
        h.send(person() + [(0.35, 0, 0.3)])
        h.pump(0.08)
        require(h.scans, 'self-filter fixture produced no scan')
        scan = h.scans[-1][1]
        center = round((0.0 - scan.angle_min) / scan.angle_increment)
        require(2.0 < scan.ranges[center] < 4.0, 'self point won target polar minimum')
        response = h.bind(3, 0)
        require(response.success, 'body return hid valid target before polar minimum')
        return
    h.moving()
    trigger = time.monotonic()
    if case == 'missing_tf':
        rejected_stamp = h.send(person(), 'unconnected_sensor')
    elif case == 'malformed':
        rejected_stamp = h.send(person(), malformed=True)
    elif case == 'nan':
        rejected_stamp = h.send([(float('nan'), 0, 0.5)])
    elif case == 'empty':
        rejected_stamp = h.send([])
    elif case == 'backwards':
        rejected_stamp = h.send(person(), stamp=h.stamp(-2))
    h.stopped(trigger)
    h.pump(0.25)
    cleared = [m for t, m in h.viz if t >= trigger and
               stamp_key(m.header.stamp) == stamp_key(rejected_stamp)]
    require(cleared and all(m.width * m.height == 0 and m.header.frame_id == 'base_link' for m in cleared),
            'invalid input did not clear stamped control-frame visualization')


def run_case(case):
    import rclpy
    namespace = 'coordinate_acceptance_' + secrets.token_hex(6)
    os.environ['ROS_DOMAIN_ID'] = str(secrets.randbelow(100) + 1)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args = ['ros2', 'run', 'rs_follow', 'rs_follow_node', '--ros-args', '-r', '__ns:=/' + namespace]
    for topic in ('rslidar_points', 'cmd_vel', 'clicked_point', 'odom', 'clock', 'tf', 'tf_static',
                  'rosout', 'parameter_events', 'rs_follow/bind', 'rs_follow/bind_target',
                  'rs_follow/clear_target', 'rs_follow/enable', 'rs_follow/estop',
                  'rs_follow/control_mode', 'rs_follow/direct_cmd', 'rs_follow/scan',
                  'rs_follow/target', 'rs_follow/target_raw', 'rs_follow/target_marker',
                  'rs_follow/status', 'rs_follow/state', 'rs_follow/control_state', 'rs_follow/cloud_viz'):
        args += ['-r', f'/{topic}:=/{namespace}/{topic}']
    for name, value in (('control_frame', 'base_link'), ('active', 'false'), ('auto_select_front', 'false'),
                        ('cmd_timeout', '0.5'), ('publish_scan_debug', 'true'), ('auto_frame', 'false'),
                        ('frame_front', '0.4'), ('frame_back', '0.4'), ('frame_left', '0.3'),
                        ('frame_right', '0.3'), ('use_sim_time', 'false')):
        args += ['-p', f'{name}:={value}']
    proc, h = None, None
    with tempfile.TemporaryFile(mode='w+t') as log:
        rclpy.init(args=[])
        try:
            h = make_harness(namespace)
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            h.ready(proc)
            exercise(h, case)
            require(proc.poll() is None, 'node exited during test')
            print(f'{case}: PASS', flush=True)
            return True
        except Exception as exc:
            print(f'{case}: FAIL: {exc}', file=sys.stderr)
            log.seek(0)
            print(''.join(log.readlines()[-15:]), file=sys.stderr)
            return False
        finally:
            if proc is not None:
                stop_process(proc)
            if h is not None:
                h.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES + ('all',), default='all')
    args = parser.parse_args()
    results = [run_case(case) for case in (CASES if args.case == 'all' else (args.case,))]
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
