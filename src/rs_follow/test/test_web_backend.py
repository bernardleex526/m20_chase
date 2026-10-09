"""Pure validation/packing and real loopback websocket security regression tests."""
import ast
import asyncio
import importlib.util
import json
import math
from pathlib import Path
import struct
import threading
from types import SimpleNamespace as NS

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
# These pure boundary functions don't require ROS to exercise malformed inputs.
source = ast.parse((SCRIPTS / 'web_ui.py').read_text())
namespace = dict(math=math, struct=struct)
functions = [node for node in source.body if isinstance(node, ast.FunctionDef)
             and node.name in ('validate_action', 'priority_action', 'pack_cloud')]
exec(compile(ast.Module(body=functions, type_ignores=[]), 'web_ui.py', 'exec'), namespace)
validate = namespace['validate_action']
pack = namespace['pack_cloud']


@pytest.mark.parametrize('message', [None, [], {}, {'type': 'enable', 'value': 1},
    {'type': 'estop', 'value': 'false'}, {'type': 'mode', 'value': True},
    {'type': 'mode', 'value': 2}, {'type': 'direct', 'vx': float('nan'), 'vy': 0, 'wz': 0},
    {'type': 'set_target', 'x': 1, 'y': 2, 'z': 3, 'frame': '', 'stamp_sec': 0, 'stamp_nanosec': 0},
    {'type': 'action', 'value': 'arbitrary'}])
def test_reject_malformed_actions(message):
    with pytest.raises(ValueError):
        validate(message)


def test_priority_stops_and_valid_binding():
    for message in ({'type': 'enable', 'value': False}, {'type': 'estop', 'value': True},
                    {'type': 'clear_target'}, {'type': 'direct', 'vx': 0, 'vy': 0, 'wz': 0}):
        assert namespace['priority_action'](validate(message))
    point = dict(type='set_target', x=2.5, y=-1, z=0.3, frame='base_link', stamp_sec=123, stamp_nanosec=456)
    assert validate(point) == point
    assert not namespace['priority_action'](point)


def cloud():
    return NS(header=NS(frame_id='基座', stamp=NS(sec=-1, nanosec=456)),
              width=1, height=1, point_step=12, row_step=12, is_bigendian=False,
              fields=[NS(name=k, offset=i * 4, datatype=7, count=1) for i, k in enumerate('xyz')],
              data=struct.pack('<fff', 1.25, -2.5, 0.75))


def test_cloud_self_describing_header():
    msg = cloud()
    data = pack(msg, 42)
    assert struct.unpack('<4sIiIIH', data[:22]) == (b'PC01', 42, -1, 456, 1, len('基座'.encode()))
    assert data[22:28].decode() == '基座'
    assert struct.unpack('<fff', data[28:]) == (1.25, -2.5, 0.75)


@pytest.mark.parametrize('field,value', [('data', b''), ('data', struct.pack('<fff', float('inf'), 0, 0)),
                                        ('is_bigendian', True), ('point_step', 16), ('row_step', 13), ('height', 2)])
def test_reject_bad_cloud(field, value):
    msg = cloud()
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        pack(msg, 0)


def test_auth_origin_lease_expiry_and_readonly():
    pytest.importorskip('websockets.asyncio.server')
    spec = importlib.util.spec_from_file_location('backend_ws', SCRIPTS / 'ws_server.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from websockets.asyncio.client import connect
    from websockets.exceptions import ConnectionClosed, InvalidStatus
    released = threading.Event()
    actions = []
    server = module.WSServer(0, lambda msg, client: actions.append(msg) or dict(success=True),
                             host='127.0.0.1', token='secret', origins=['http://localhost'],
                             on_release=released.set)
    # Port zero asks the real OS listener to allocate an unused ephemeral port.
    # WSServer publishes its bound port for network tests and port-zero deployments.
    async def exercise():
        url = f'ws://127.0.0.1:{server.bound_port}'
        with pytest.raises(InvalidStatus):
            async with connect(url, origin='http://attacker'):
                pass
        async with connect(url, origin='http://localhost') as unauth:
            server.broadcast(dict(type='state', moving=False))
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(unauth.recv(), 0.05)
            await unauth.send(json.dumps(dict(type='auth', token='wrong')))
            with pytest.raises(ConnectionClosed):
                await unauth.recv()
        async with connect(url, origin='http://localhost') as owner:
            await owner.send(json.dumps(dict(type='auth', token='secret')))
            assert json.loads(await owner.recv())['controller'] is True
            await owner.recv()  # cached state
            released.clear()
            async with connect(url, origin='http://localhost') as observer:
                await observer.send(json.dumps(dict(type='auth', token='secret')))
                assert json.loads(await observer.recv())['controller'] is False
                await observer.recv()
                await observer.send(json.dumps(dict(type='enable', value=True)))
                result = json.loads(await observer.recv())
                assert result['reason'] == 'READ_ONLY'
                assert not actions
                await asyncio.sleep(0.65)
                assert released.is_set()
                assert server.owner is None
    try:
        asyncio.run(exercise())
    finally:
        server.close()


def test_sender_deadline_closes_and_releases_only_slow_owner():
    spec = importlib.util.spec_from_file_location('deadline_ws', SCRIPTS / 'ws_server.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    async def exercise():
        closed = []
        released = []
        class BlockedTransport:
            async def send(self, message):
                await asyncio.Event().wait()
            async def close(self, **kwargs):
                closed.append(kwargs)
        server = module.WSServer.__new__(module.WSServer)
        server.lock = threading.RLock()
        server.on_release = lambda: released.append(True)
        client = module._Client(BlockedTransport())
        client.state = dict(type='state')
        client.event.set()
        server.clients = {client}
        server.owner = client
        server.deadline = float('inf')
        start = asyncio.get_running_loop().time()
        await asyncio.wait_for(server._send(client), 1)
        elapsed = asyncio.get_running_loop().time() - start
        assert 0.19 <= elapsed < 0.5
        assert closed == [dict(code=1008, reason='slow client')]
        assert released == [True]
        assert server.owner is None
    asyncio.run(exercise())
