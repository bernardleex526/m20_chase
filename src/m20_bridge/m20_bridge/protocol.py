"""Strict M20 basic_server frames: 16-byte header plus object JSON ASDU.

Thin adapter over the shared ``m20_protocol`` package (src/m20_protocol
submodule). Keeps this package's dict-based API for bridge_node and the fake
server; framing and validation live in ``m20_protocol``.
"""

from m20_protocol import FMT_JSON, HEADER_SIZE, MAX_BODY_SIZE, SYNC, now_str  # noqa: F401
from m20_protocol import StreamDecoder as _SharedStreamDecoder
from m20_protocol import commands as _C
from m20_protocol import encode as _encode
from m20_protocol import try_decode as _try_decode


def _as_dict(frame):
    return {'msg_id': frame.msg_id, 'type': frame.type, 'command': frame.command,
            'items': frame.items, 'raw': frame.raw}


def encode(type_, command, items=None, msg_id=0, t=None):
    return _encode(type_, command, items, msg_id & 0xFFFF, t)


def decode(buf):
    """Decode exactly one frame; return None for any invalid frame."""
    frame = _try_decode(buf)
    return None if frame is None else _as_dict(frame)


class StreamDecoder(_SharedStreamDecoder):
    """Incremental TCP decoder. Invalid frames poison the connection, never resync."""

    def feed(self, data):
        return [_as_dict(f) for f in super().feed(data)]


def heartbeat():
    return _C.heartbeat()


def set_mode(mode):
    return _C.set_usage_mode(int(mode))


def set_motion_state(param):
    return _C.set_motion_state(int(param))


def set_gait(gait):
    return _C.set_gait(int(gait))


def axis_cmd(x, y, yaw, z=0.0, roll=0.0, pitch=0.0):
    """Cmd=21 normalized axes. Z/Roll/Pitch are always sent as 0 (planar control)."""
    if (z, roll, pitch) != (0.0, 0.0, 0.0):
        raise ValueError('non-planar axes are not supported')
    return _C.axis_normalized(x, y, yaw)
