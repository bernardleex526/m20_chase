"""Opt-in actual WebUi auth and heartbeat loss evidence via ROS publications."""
import asyncio
import json
import os
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(not os.environ.get('WEB_UI_WS'), reason='needs live isolated ROS WebUi')


def test_live_auth_lease_loss_publishes_pause_zero_and_reconnect_paused():
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from websockets.asyncio.client import connect
    from websockets.exceptions import ConnectionClosed, InvalidStatus
    rclpy.init()
    node = rclpy.create_node('web_lease_acceptance')
    pauses, zeros = [], []
    node.create_subscription(Bool, '/rs_follow/enable', lambda m: pauses.append((time.monotonic(), m.data)), 10)
    node.create_subscription(Twist, '/rs_follow/direct_cmd', lambda m: zeros.append((time.monotonic(), m)), 10)
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    thread = threading.Thread(target=spin)
    thread.start()
    url = os.environ['WEB_UI_WS']
    origin = os.environ.get('WEB_UI_ORIGIN', 'http://127.0.0.1:8081')
    token = os.environ.get('WEB_UI_TOKEN', '')
    async def auth(socket, value=token):
        await socket.send(json.dumps(dict(type='auth', token=value)))
        return json.loads(await socket.recv())
    async def exercise():
        await asyncio.sleep(0.5)  # DDS discovery before observing lease publications.
        with pytest.raises(InvalidStatus):
            async with connect(url, origin='http://invalid.example'):
                pass
        async with connect(url, origin=origin) as bad:
            await bad.send(json.dumps(dict(type='auth', token=token + 'wrong')))
            with pytest.raises(ConnectionClosed):
                await bad.recv()
        async with connect(url, origin=origin) as owner:
            assert (await auth(owner))['controller']
            async with connect(url, origin=origin) as observer:
                assert not (await auth(observer))['controller']
                await observer.send(json.dumps(dict(type='enable', value=True)))
                while True:
                    raw = await observer.recv()
                    if isinstance(raw, str) and json.loads(raw).get('type') == 'result':
                        assert json.loads(raw)['reason'] == 'READ_ONLY'
                        break
            begin = time.monotonic()
            await asyncio.sleep(0.75)  # deliberately withhold owner heartbeat.
            assert any(t >= begin and not value for t, value in pauses)
            assert any(t >= begin and all(v == 0 for v in (m.linear.x, m.linear.y, m.linear.z,
                       m.angular.x, m.angular.y, m.angular.z)) for t, m in zeros)
            with pytest.raises(ConnectionClosed):
                while True:
                    await owner.recv()
        async with connect(url, origin=origin) as reconnect:
            assert (await auth(reconnect))['controller']
            while True:
                raw = await reconnect.recv()
                if isinstance(raw, str) and json.loads(raw).get('type') == 'state':
                    assert json.loads(raw)['active'] is False
                    break
    try:
        asyncio.run(exercise())
    finally:
        stop.set()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
