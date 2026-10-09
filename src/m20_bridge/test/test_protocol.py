import json
import struct

import pytest

from m20_bridge import protocol as P
from m20_bridge.fake_m20_server import FakeState, fragments


def frame(body):
    return P.SYNC + struct.pack('<HHB', len(body), 42, P.FMT_JSON) + bytes(7) + body


def test_roundtrip_unicode_and_id():
    msg = P.decode(P.encode(1002, 6, {'name': '山猫'}, msg_id=65537, t='fixed'))
    assert msg['msg_id'] == 1
    assert msg['items'] == {'name': '山猫'}
    assert msg['raw']['Time'] == 'fixed'


@pytest.mark.parametrize('build, key, items', [
    (P.heartbeat, (100, 100), {}),
    (lambda: P.set_mode(0), (1101, 5), {'Mode': 0}),
    (lambda: P.set_motion_state(17), (2, 22), {'MotionParam': 17}),
    (lambda: P.set_gait(0x3002), (2, 23), {'GaitParam': 0x3002}),
    (lambda: P.axis_cmd(.1, -.2, .3), (2, 21),
     {'X': .1, 'Y': -.2, 'Yaw': .3, 'Z': 0., 'Roll': 0., 'Pitch': 0.}),
])
def test_command_wire_fields(build, key, items):
    msg = P.decode(build())
    assert (msg['type'], msg['command']) == key
    assert msg['items'] == items


def test_requires_exact_frame_length():
    good = P.heartbeat()
    for size in range(len(good)):
        assert P.decode(good[:size]) is None
    assert P.decode(good + b'\0') is None
    assert P.decode(good + good) is None


@pytest.mark.parametrize('offset, value', [(0, 0), (8, 2), (9, 1), (15, 1)])
def test_rejects_bad_header(offset, value):
    packet = bytearray(P.heartbeat())
    packet[offset] = value
    assert P.decode(packet) is None
    with pytest.raises(ValueError):
        P.StreamDecoder().feed(packet)


@pytest.mark.parametrize('body', [b'not json', b'\xff', b'[]', b'null', b'1', b'{}',
    b'{"PatrolDevice":[]}',
    b'{"PatrolDevice":{"Type":1,"Command":2,"Items":[]}}',
    b'{"PatrolDevice":{"Type":true,"Command":2,"Items":{}}}',
    b'{"PatrolDevice":{"Type":1,"Command":"2","Items":{}}}',
    b'{"PatrolDevice":{"Type":1,"Command":2}}',
    b'{"PatrolDevice":{"Type":1,"Command":2,"Items":{"v":NaN}}}',
])
def test_rejects_invalid_json_envelope(body):
    assert P.decode(frame(body)) is None
    with pytest.raises(ValueError):
        P.StreamDecoder().feed(frame(body))


@pytest.mark.parametrize('size', [1, 2, 7, 15, 16, 17, 10000])
def test_stream_fragmentation_and_coalescing(size):
    packets = [P.encode(1002, i, {'n': i}) for i in (4, 6, 4)]
    decoder = P.StreamDecoder()
    messages = []
    for chunk in fragments(b''.join(packets), size):
        messages.extend(decoder.feed(chunk))
    assert [m['command'] for m in messages] == [4, 6, 4]
    decoder.eof()


def test_stream_partial_tail_and_failure_is_terminal():
    packet = P.heartbeat()
    decoder = P.StreamDecoder()
    assert len(decoder.feed(packet + packet[:10])) == 1
    with pytest.raises(ValueError):
        decoder.eof()
    with pytest.raises(ValueError):
        decoder.feed(packet)


def test_stream_bad_magic_never_resynchronizes():
    decoder = P.StreamDecoder()
    with pytest.raises(ValueError):
        decoder.feed(b'x' * 16 + P.heartbeat())
    with pytest.raises(ValueError):
        decoder.feed(P.heartbeat())


def test_maximum_body_and_oversized_encode():
    body = json.dumps({'PatrolDevice': {'Type': 1, 'Command': 2, 'Items': {'x': ''}}}).encode()
    body = body.replace(b'"x": ""', b'"x": "' + b'a' * (65535 - len(body)) + b'"')
    assert len(body) == 65535
    packet = frame(body)
    assert P.decode(packet) is not None
    decoder = P.StreamDecoder()
    assert decoder.feed(packet[:65535]) == []
    assert len(decoder.feed(packet[65535:])) == 1
    with pytest.raises(ValueError):
        P.encode(1, 2, {'x': 'a' * 65535})


@pytest.mark.parametrize('args', [(True, 1, {}), (1, '2', {}), (1, 2, []), (1, 2, {'x': float('nan')})])
def test_encode_rejects_invalid_objects(args):
    with pytest.raises(ValueError):
        P.encode(*args)


def test_fake_state_configuration_and_feedback():
    state = FakeState(hes=1, mode=2)
    for packet in (P.set_mode(0), P.set_motion_state(17), P.set_gait(0x3002), P.axis_cmd(.1, .2, .3)):
        assert P.decode(state.handle(P.decode(packet)))['items']['ErrorCode'] == 0
    basic, motion = (P.decode(packet) for packet in state.feedback(.1))
    assert basic['items']['BasicStatus'] == {'HES': 1, 'ControlUsageMode': 0, 'MotionState': 17, 'Gait': 0x3002}
    assert motion['items']['MotionStatus']['LinearX'] == 0.
    state.hes = 0
    _, motion = (P.decode(packet) for packet in state.feedback(.1))
    assert motion['items']['MotionStatus']['LinearX'] == pytest.approx(.2)
