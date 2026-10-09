"""Opt-in real ROS/websocket backpressure acceptance, isolated DDS domain only.

WEB_UI_PRESSURE=1 WEB_UI_WS=ws://127.0.0.1:8890 WEB_UI_ORIGIN=http://127.0.0.1:8080
WEB_UI_TOKEN=... /usr/bin/python3 -m pytest -q test_web_pressure_live.py
Run actual rs_follow_node and a dedicated WebUi configured cloud_topic:=/web_pressure/cloud.
Set WEB_UI_PRESSURE_CLOUD=/web_pressure/cloud. Do not mix publishers on the display topic.
"""
import asyncio
import base64
import json
import os
import socket
import struct
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('WEB_UI_PRESSURE') != '1',
                                reason='requires isolated live ROS nodes and WEB_UI_PRESSURE=1')


def test_live_slow_reader_does_not_block_ros_or_controller():
    import rclpy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from websockets.asyncio.client import connect
    from websockets.exceptions import ConnectionClosed
    rclpy.init()
    node = rclpy.create_node('web_pressure_acceptance')
    observations = []
    node.create_subscription(String, '/rs_follow/control_state',
                             lambda msg: observations.append(time.monotonic()), 10)
    pub = node.create_publisher(PointCloud2, os.environ.get('WEB_UI_PRESSURE_CLOUD', '/rs_follow/cloud_viz'), 1)
    cloud = PointCloud2()
    cloud.header.frame_id = 'base_link'
    cloud.height, cloud.width = 1, 6000
    cloud.point_step, cloud.row_step = 12, 72000
    cloud.is_dense = True
    cloud.fields = [PointField(name=k, offset=i * 4, datatype=7, count=1) for i, k in enumerate('xyz')]
    cloud.data = struct.pack('<fff', 3.0, 0.0, 0.5) * 6000
    def publish():
        cloud.header.stamp = node.get_clock().now().to_msg()
        pub.publish(cloud)
    node.create_timer(0.05, publish)
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    thread = threading.Thread(target=spin)
    thread.start()
    url = os.environ.get('WEB_UI_WS', 'ws://127.0.0.1:8890')
    origin = os.environ.get('WEB_UI_ORIGIN', 'http://127.0.0.1:8080')
    auth = json.dumps(dict(type='auth', token=os.environ.get('WEB_UI_TOKEN', '')))
    async def exercise():
        async with connect(url, origin=origin) as controller:
            await controller.send(auth)
            assert json.loads(await controller.recv())['controller'] is True
            parsed = __import__('urllib.parse', fromlist=['urlparse']).urlparse(url)
            # Set the receive window before connect, so TCP negotiates the intended
            # tiny window. Use a raw peer: no websocket library read-ahead/high-water
            # state can hide an already-closed transport during final EOF detection.
            peer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            peer.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
            peer.setblocking(False)
            loop = asyncio.get_running_loop()
            await loop.sock_connect(peer, (parsed.hostname, parsed.port or 80))
            try:
                key = base64.b64encode(os.urandom(16)).decode()
                handshake = (f'GET {parsed.path or "/"} HTTP/1.1\r\nHost: {parsed.netloc}\r\n'
                             f'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                             f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
                             f'Origin: {origin}\r\n\r\n').encode()
                await loop.sock_sendall(peer, handshake)
                async def exact(size):
                    data = bytearray()
                    while len(data) < size:
                        chunk = await loop.sock_recv(peer, size - len(data))
                        assert chunk, 'unexpected EOF before authenticated pressure peer'
                        data.extend(chunk)
                    return bytes(data)
                headers = bytearray()
                while not headers.endswith(b'\r\n\r\n'):
                    headers.extend(await exact(1))
                    assert len(headers) < 8192
                assert headers.startswith(b'HTTP/1.1 101')
                payload = auth.encode()
                mask = os.urandom(4)
                length = bytes([len(payload)]) if len(payload) < 126 else b'\x7e' + struct.pack('>H', len(payload))
                frame = b'\x81' + bytes([length[0] | 128]) + length[1:] + mask
                await loop.sock_sendall(peer, frame + bytes(v ^ mask[i % 4] for i, v in enumerate(payload)))
                first = await exact(2)
                assert first[0] == 0x81
                size = first[1] & 127
                if size == 126:
                    size = struct.unpack('>H', await exact(2))[0]
                assert json.loads(await exact(size))['controller'] is False
                begin = time.monotonic()
                states = 0
                full_clouds = 0
                # Don't read the raw pressure peer at all until the server aborts.
                while time.monotonic() - begin < 20:
                    await controller.send(json.dumps(dict(type='heartbeat')))
                    deadline = time.monotonic() + 0.08
                    while time.monotonic() < deadline:
                        try:
                            raw = await asyncio.wait_for(controller.recv(), max(0.001, deadline - time.monotonic()))
                        except asyncio.TimeoutError:
                            break
                        if isinstance(raw, str):
                            message = json.loads(raw)
                            if message.get('type') == 'state':
                                assert message['controller'] is True
                                states += 1
                        elif raw[:4] == b'PC01':
                            assert struct.unpack_from('<I', raw, 16)[0] == 6000, 'pressure topic has competing publishers'
                            full_clouds += 1
                assert states >= 50
                assert full_clouds >= 50, 'pressure did not deliver sustained full-size clouds'
                peer.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
                async def drain_until_closed():
                    try:
                        while await loop.sock_recv(peer, 1048576):
                            pass
                    except ConnectionResetError:
                        pass  # saturated server abort is also an actual disconnect.
                # EOF/reset must arrive before the server's 40s keepalive expiry.
                await asyncio.wait_for(drain_until_closed(), 10)
                times = [t for t in observations if t >= begin]
                assert len(times) >= 50, 'actual ROS control_state did not continue'
                assert max(b - a for a, b in zip(times, times[1:])) <= 0.2
            finally:
                peer.close()
    try:
        asyncio.run(exercise())
    finally:
        stop.set()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
