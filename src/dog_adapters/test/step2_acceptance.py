#!/usr/bin/env python3
"""Real ROS/DDS and loopback-wire Step 2 acceptance; requires sourced workspace.

Run: ROS_DOMAIN_ID=71 python3 src/dog_adapters/test/step2_acceptance.py
No SDK substitutes, in-process adapter instances, or launch files are used.
"""
import json
import math
import os
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '71'
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from unitree_api.msg import Request


TIMEOUT = 0.6


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def near(a, b):
    return math.isclose(a, b, rel_tol=0, abs_tol=1e-6)


def vector(msg):
    msg = msg.twist if isinstance(msg, TwistStamped) else msg
    return msg.linear.x, msg.linear.y, msg.angular.z


def zero(msg):
    return all(near(v, 0) for v in vector(msg))


def frame(command=6, hes=0, mode=0):
    body = json.dumps({'PatrolDevice': {'Type': 1002, 'Command': command,
                       'Time': '2026-01-01 00:00:00', 'Items':
                       {'BasicStatus': {'HES': hes, 'ControlUsageMode': mode,
                                        'MotionState': 0}} if command == 6 else
                       {'MotionStatus': {'Yaw': 0., 'LinearX': 0.,
                                         'LinearY': 0., 'OmegaZ': 0.}}}}).encode()
    return b'\xeb\x91\xeb\x90' + struct.pack('<HHB', len(body), 0, 1) + bytes(7) + body


class Wire:
    """Independent real socket peer, recording actual bridge wire frames."""
    def __init__(self, transport):
        self.transport = transport
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if transport == 'udp' else socket.SOCK_STREAM)
        self.socket.bind(('127.0.0.1', 0))
        self.port = self.socket.getsockname()[1]
        self.socket.settimeout(.1)
        if transport == 'tcp':
            self.socket.listen(1)
        self.connection = None
        self.peer = None
        self.records = []
        self.errors = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()

    def receive(self):
        pending = b''
        try:
            while not self.stop.is_set():
                try:
                    if self.transport == 'tcp':
                        if self.connection is None:
                            self.connection, self.peer = self.socket.accept()
                            self.connection.settimeout(.1)
                        data = self.connection.recv(65535)
                        if not data:
                            break
                        pending += data
                    else:
                        pending, self.peer = self.socket.recvfrom(65535)
                    while len(pending) >= 16:
                        require(pending[:4] == b'\xeb\x91\xeb\x90', 'wire sync invalid')
                        require(pending[8:16] == b'\x01' + bytes(7), 'wire header invalid')
                        size = 16 + struct.unpack_from('<H', pending, 4)[0]
                        if len(pending) < size:
                            break
                        record = json.loads(pending[16:size])['PatrolDevice']
                        with self.lock:
                            self.records.append((time.monotonic(), record))
                        pending = pending[size:]
                    if self.transport == 'udp':
                        require(not pending, 'partial UDP frame')
                except socket.timeout:
                    continue
        except Exception as exc:
            if not self.stop.is_set():
                self.errors.append(str(exc))

    def send(self, data, fragmented=False, wrong_peer=False):
        require(self.peer is not None, 'bridge has not contacted fixture')
        if self.transport == 'udp':
            if wrong_peer:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other:
                    other.sendto(data, self.peer)
            else:
                self.socket.sendto(data, self.peer)
        elif fragmented:
            for byte in data:
                self.connection.sendall(bytes([byte]))
                time.sleep(.0005)
        else:
            self.connection.sendall(data)

    def axes(self, since=0):
        with self.lock:
            return [r['Items'] for t, r in self.records if t >= since and r['Type'] == 2 and r['Command'] == 21]

    def close(self):
        self.stop.set()
        if self.connection:
            self.connection.close()
        self.socket.close()
        self.thread.join(1)


class Driver:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('step2_acceptance')
        self.cmd = self.node.create_publisher(Twist, '/rs_follow/cmd_vel', 10)
        self.estop = self.node.create_publisher(Bool, '/rs_follow/estop', 10)
        self.action = self.node.create_publisher(String, '/rs_follow/action_cmd', 10)
        self.processes = []
        self.logs = tempfile.TemporaryDirectory(prefix='step2-acceptance-')

    def spin(self, seconds, callback=None):
        end = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if callback and now >= next_publish:
                callback()
                next_publish = now + .05
            rclpy.spin_once(self.node, timeout_sec=.02)

    def until(self, predicate, label, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=.02)
        raise AssertionError('timed out: ' + label)

    def start(self, package, executable, parameters=None, remaps=None):
        args = ['ros2', 'run', package, executable, '--ros-args']
        for key, value in (parameters or {}).items():
            if isinstance(value, bool):
                value = str(value).lower()
            args += ['-p', f'{key}:={value}']
        for old, new in (remaps or {}).items():
            args += ['-r', f'{old}:={new}']
        path = Path(self.logs.name) / f'{len(self.processes)}-{executable}.log'
        log = open(path, 'w')
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   env=os.environ.copy(), start_new_session=True)
        self.processes.append((process, log, path))
        return process

    def stop(self, process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            self.until(lambda: process.poll() is not None, 'SIGINT process exit', 8)
        self.spin(.15)

    def live(self, process):
        require(process.poll() is None, 'real node exited unexpectedly')

    def velocity(self, x=.2, y=0., yaw=.3):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, yaw
        self.cmd.publish(msg)

    def bool(self, enabled):
        self.estop.publish(Bool(data=enabled))
        self.spin(.12)

    def act(self, text):
        self.action.publish(String(data=text))
        self.spin(.1)

    def subscription(self, cls, topic):
        records = []
        sub = self.node.create_subscription(cls, topic, lambda msg: records.append((time.monotonic(), msg)), 100)
        return sub, records

    def close(self):
        for process, log, path in self.processes:
            if process.poll() is None:
                try:
                    self.stop(process)
                except Exception:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.logs.cleanup()

    def twist(self, stamped):
        topic = '/step2/stamped' if stamped else '/step2/twist'
        sub, records = self.subscription(TwistStamped if stamped else Twist, topic)
        process = self.start('dog_adapters', 'twist_adapter',
                             {'output_topic': topic, 'stamped': stamped, 'base_frame': 'step2_base',
                              'lateral': False, 'watchdog_timeout': TIMEOUT})
        self.until(lambda: self.cmd.get_subscription_count() > 0 and records, 'Twist DDS discovery')
        self.live(process)
        mark = time.monotonic()
        self.spin(.2, lambda: self.velocity(2., 3., -4.))
        active = [msg for t, msg in records if t >= mark and not zero(msg)]
        require(active, 'Twist never forwarded real command')
        require(all(all(near(a, b) for a, b in zip(vector(msg), (.3, 0., -.5))) for msg in active),
                'Twist limits/lateral suppression violated')
        require(all(near((msg.twist if stamped else msg).linear.z, 0.) and
                    near((msg.twist if stamped else msg).angular.x, 0.) and
                    near((msg.twist if stamped else msg).angular.y, 0.) for msg in active), 'unused axes nonzero')
        if stamped:
            stamps = [msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec for msg in active]
            require(all(msg.header.frame_id == 'step2_base' for msg in active), 'frame_id mismatch')
            now = self.node.get_clock().now().nanoseconds
            require(all(0 < now - stamp < 2 * 10**9 for stamp in stamps), 'header stamp is not current')
            require(all(a <= b for a, b in zip(stamps, stamps[1:])), 'header stamps regressed')
        self.spin(TIMEOUT + .2)
        require(zero(records[-1][1]), 'watchdog did not zero Twist')
        self.spin(.15, self.velocity)
        require(not zero(records[-1][1]), 'fresh Twist did not resume')
        self.bool(True)
        self.spin(.15, self.velocity)
        require(all(zero(msg) for t, msg in records if t >= time.monotonic() - .1), 'estop forwarded Twist')
        self.bool(False)
        self.spin(.15)
        require(zero(records[-1][1]), 'estop release resurrected Twist')
        self.spin(.15, self.velocity)
        require(not zero(records[-1][1]), 'new post-release Twist rejected')
        for stop, clear in (('e_stop', 'resume'), ('soft_stop', 'reset_estop'),
                            ('softstop', 'clear'), ('estop', 'clear_estop'), ('stop', 'unlock')):
            require(not zero(records[-1][1]), f'{stop}: Twist was not moving before stop')
            self.act(stop)
            mark = time.monotonic()
            self.spin(.15, self.velocity)
            require(all(zero(msg) for t, msg in records if t >= mark), f'{stop}: Twist not latched')
            self.act(clear)
            mark = time.monotonic()
            self.spin(.1)
            require(all(zero(msg) for t, msg in records if t >= mark), f'{clear}: cached Twist resumed')
            self.spin(.15, self.velocity)
            require(not zero(records[-1][1]), f'{clear}: fresh Twist rejected')
        self.stop(process)
        require(zero(records[-1][1]), 'Twist exit did not zero')
        self.node.destroy_subscription(sub)
        print(f'PASS Twist{"Stamped" if stamped else ""}: DDS, limits, lateral, header, watchdog, estop, exit', flush=True)

    def equality(self):
        for parameters, remaps in [({'cmd_vel_topic': 'same', 'output_topic': '/same'}, {}),
                                   ({'output_topic': '/step2/out'}, {'/step2/out': '/rs_follow/cmd_vel'})]:
            process = self.start('dog_adapters', 'twist_adapter', parameters, remaps)
            self.until(lambda: process.poll() is not None, 'same-topic rejection')
            require(process.returncode != 0, 'same resolved/remapped topics accepted')
            path = next(path for p, _, path in self.processes if p is process)
            require('input and output topics must differ' in path.read_text(), 'wrong same-topic failure reason')
        print('PASS Twist input/output equality: resolved and remapped rejection', flush=True)

    def unitree(self):
        sub, records = self.subscription(Request, '/api/sport/request')
        client = self.node.create_client(SetBool, '/unitree_adapter/arm')
        process = self.start('dog_adapters', 'unitree_adapter', {'watchdog_timeout': TIMEOUT})
        self.until(lambda: client.service_is_ready() and records and self.cmd.get_subscription_count() > 0,
                   'official Unitree DDS/service discovery')
        self.live(process)
        def api(msg):
            return msg.header.identity.api_id
        def only_stop(since):
            seen = [msg for t, msg in records if t >= since]
            require(seen and all(api(msg) == 1003 and msg.parameter == '' for msg in seen), 'expected fresh Stop only')
        def arm():
            future = client.call_async(SetBool.Request(data=True))
            self.until(future.done, 'arm service response')
            require(future.result().success, 'arm service rejected')
        mark = time.monotonic()
        self.spin(.2, self.velocity)
        only_stop(mark)
        arm()
        mark = time.monotonic()
        self.spin(.1)
        only_stop(mark)
        self.spin(.2, lambda: self.velocity(2., 3., -4.))
        moves = [msg for t, msg in records if t >= mark and api(msg) == 1008]
        require(moves, 'arm + new command did not Move')
        for msg in moves:
            values = json.loads(msg.parameter, parse_constant=lambda v: (_ for _ in ()).throw(AssertionError('nonfinite JSON')))
            require(set(values) == {'x', 'y', 'z'} and all(math.isfinite(v) for v in values.values()), 'invalid Move JSON')
            require(all(near(values[k], v) for k, v in zip(('x', 'y', 'z'), (.3, .15, -.5))), 'Move limits/axis mapping')
            require(isinstance(msg, Request), 'not official Request identity')
        self.spin(TIMEOUT + .2)
        mark = time.monotonic()
        self.spin(.2, self.velocity)
        only_stop(mark)
        arm()
        self.spin(.15, self.velocity)
        self.act('standup')
        stand = [msg for _, msg in records if api(msg) == 1004]
        require(stand and all(msg.parameter == '' for msg in stand), 'stand action leaked Move parameter')
        self.spin(.1, self.velocity)
        self.act('liedown')
        lie = [msg for _, msg in records if api(msg) == 1005]
        require(lie and all(msg.parameter == '' for msg in lie), 'lie action leaked Move parameter')
        self.bool(True)
        mark = time.monotonic()
        self.spin(.15, self.velocity)
        only_stop(mark)
        future = client.call_async(SetBool.Request(data=True))
        self.until(future.done, 'estopped arm rejection')
        require(not future.result().success, 'armed while estopped')
        self.act('unlock')
        mark = time.monotonic()
        self.spin(.2, self.velocity)
        only_stop(mark)
        arm()
        self.spin(.15, self.velocity)
        require(api(records[-1][1]) == 1008, 'explicit rearm did not resume')
        for stop, clear in (('e_stop', 'resume'), ('soft_stop', 'reset_estop'),
                            ('softstop', 'clear'), ('estop', 'clear_estop'), ('stop', 'unlock')):
            require(api(records[-1][1]) == 1008, f'{stop}: Unitree was not moving before stop')
            self.act(stop)
            mark = time.monotonic()
            self.spin(.15, self.velocity)
            only_stop(mark)
            future = client.call_async(SetBool.Request(data=True))
            self.until(future.done, f'{stop}: arm rejection')
            require(not future.result().success, f'{stop}: armed while latched')
            self.act(clear)
            mark = time.monotonic()
            self.spin(.1)
            only_stop(mark)
            mark = time.monotonic()
            self.spin(.15, self.velocity)
            only_stop(mark)
            arm()
            mark = time.monotonic()
            self.spin(.1)
            only_stop(mark)
            self.spin(.15, self.velocity)
            require(api(records[-1][1]) == 1008, f'{clear}: rearm plus new command did not Move')
        mark = time.monotonic()
        self.stop(process)
        require(any(t >= mark and api(msg) == 1003 for t, msg in records), 'exit Stop absent')
        require(all(msg.parameter == '' for _, msg in records if api(msg) != 1008), 'non-Move parameter not fresh')
        require(all(api(msg) in (1003, 1004, 1005, 1008) for _, msg in records), 'unexpected Unitree action')
        for _, msg in records:
            require(msg.header.identity.id == 0 and msg.header.lease.id == 0 and
                    msg.header.policy.priority == 0 and not msg.header.policy.noreply,
                    'Request identity/lease/policy defaults leaked or mutated')
        self.node.destroy_client(client)
        self.node.destroy_subscription(sub)
        print('PASS Unitree: official DDS Request, finite Move, arm/new command, timeout disarm, estop/unlock/exit, fresh posture', flush=True)

    def m20(self, transport):
        wire = Wire(transport)
        status_timeout = 0.6
        status_sub, statuses = self.subscription(String, '/m20_bridge/robot_status')
        process = self.start('m20_bridge', 'bridge_node', {'transport': transport, 'ip': '127.0.0.1',
                             'port': wire.port, 'cmd_vel_topic': '/rs_follow/cmd_vel',
                             'watchdog_timeout': TIMEOUT, 'status_timeout': status_timeout})
        try:
            self.until(lambda: wire.peer is not None and self.cmd.get_subscription_count() > 0, 'M20 real socket/discovery')
            self.live(process)
            def axes_zero(since):
                axes = wire.axes(since)
                require(axes and all(all(near(a[key], 0) for key in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')) for a in axes), 'unsafe wire axes')
            def status(hes=0, mode=0, fragmented=False, coalesced=False):
                count = len(statuses)
                wire.send(frame(hes=hes, mode=mode) + (frame(command=4) if coalesced else b''), fragmented)
                self.until(lambda: len(statuses) > count, 'fresh M20 status callback')
            def moving():
                mark = time.monotonic()
                self.spin(.18, self.velocity)
                axes = wire.axes(mark)
                require(any(near(a['X'], .1) and near(a['Yaw'], .2) and near(a['Y'], 0) for a in axes), 'default M20 normalization not X=.1 Yaw=.2')
            mark = time.monotonic()
            self.spin(.2, self.velocity)
            axes_zero(mark)
            if transport == 'udp':
                wire.send(frame(), wrong_peer=True)
                mark = time.monotonic()
                self.spin(.2, self.velocity)
                axes_zero(mark)
                require(not statuses, 'wrong UDP peer status accepted')
            status(fragmented=transport == 'tcp')
            moving()
            if transport == 'tcp':
                status(coalesced=True)
                moving()
            for hes, mode in ((1, 0), (0, -1)):
                status(hes, mode)
                mark = time.monotonic()
                self.spin(.18, self.velocity)
                axes_zero(mark)
                status()
                mark = time.monotonic()
                self.spin(.1)
                axes_zero(mark)
                moving()
            status()
            self.spin(status_timeout + .2, self.velocity)
            mark = time.monotonic()
            self.spin(.15, self.velocity)
            axes_zero(mark)
            status()
            mark = time.monotonic()
            self.spin(.1)
            axes_zero(mark)
            moving()
            # Command expiry is independent of fresh robot status.
            self.spin(TIMEOUT + .15)
            mark = time.monotonic()
            self.spin(.1)
            axes_zero(mark)
            status()
            moving()
            for stop, clear in (('e_stop', 'resume'), ('soft_stop', 'reset_estop'),
                                ('softstop', 'clear'), ('estop', 'clear_estop'), ('stop', 'unlock')):
                status()
                moving()
                self.act(stop)
                mark = time.monotonic()
                self.spin(.15, self.velocity)
                axes_zero(mark)
                status()
                self.act(clear)
                mark = time.monotonic()
                self.spin(.1)
                axes_zero(mark)
                moving()
            with wire.lock:
                require(all((r['Type'], r['Command']) in ((100, 100), (2, 21)) for _, r in wire.records), 'default auto setup/action sent')
            if transport == 'tcp':
                # Half-close fixture send direction; receive stays open to detect illicit output.
                child = self.actual_node_pid(process.pid)
                before_threads = len(list(Path(f'/proc/{child}/task').iterdir()))
                wire.connection.shutdown(socket.SHUT_WR)
                self.spin(.35, self.velocity)
                self.live(process)
                after_threads = len(list(Path(f'/proc/{child}/task').iterdir()))
                require(after_threads < before_threads, 'TCP receive thread did not exit on EOF')
                tick_before = self.cpu_ticks(child)
                mark = time.monotonic()
                self.spin(.5, self.velocity)
                tick_after = self.cpu_ticks(child)
                require(tick_after - tick_before < os.sysconf('SC_CLK_TCK') * .15, 'TCP EOF CPU spin')
                require(not wire.axes(mark), 'TCP EOF retained/sent command')
            self.stop(process)
            axes = wire.axes()
            require(len(axes) >= 3 and all(all(near(a[key], 0) for key in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')) for a in axes[-3:]), 'SIGINT last three axes not zero')
            require(not wire.errors, 'wire fixture decoding errors: ' + '; '.join(wire.errors))
            print(f'PASS M20 {transport}: wire normalization, freshness/HES/mode/watchdog, default no setup, peer/framing/EOF, SIGINT triple zero', flush=True)
        finally:
            if process.poll() is None:
                self.stop(process)
            self.node.destroy_subscription(status_sub)
            wire.close()

    @staticmethod
    def actual_node_pid(root):
        pending = [root]
        descendants = []
        while pending:
            pid = pending.pop()
            descendants.append(pid)
            children = Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
            pending.extend(int(child) for child in children)
        # ros2 run invokes the installed executable as its descendant.
        return descendants[-1]

    @staticmethod
    def cpu_ticks(pid):
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return int(fields[11]) + int(fields[12])


def main():
    driver = Driver()
    try:
        driver.twist(False)
        driver.twist(True)
        driver.equality()
        driver.unitree()
        driver.m20('udp')
        driver.m20('tcp')
        print('PASS Step 2 real-node acceptance (ROS_DOMAIN_ID=71)', flush=True)
        return 0
    except Exception as exc:
        print(f'FAIL {type(exc).__name__}: {exc}', flush=True)
        for _, log, path in driver.processes:
            log.flush()
            text = path.read_text()
            if text:
                print(f'NODE {path.name}: {text[-2500:].strip()}', flush=True)
        return 1
    finally:
        driver.close()


if __name__ == '__main__':
    raise SystemExit(main())
