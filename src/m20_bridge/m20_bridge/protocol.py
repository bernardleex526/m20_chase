"""Strict M20 basic_server frames: 16-byte header plus object JSON ASDU."""

import json
import struct
import time

SYNC = bytes([0xEB, 0x91, 0xEB, 0x90])
FMT_JSON = 0x01
HEADER_SIZE = 16
MAX_BODY_SIZE = 65535


def now_str():
    return time.strftime('%Y-%m-%d %H:%M:%S')


def encode(type_, command, items=None, msg_id=0, t=None):
    if type(type_) is not int or type(command) is not int:
        raise ValueError('Type and Command must be integers')
    if items is not None and not isinstance(items, dict):
        raise ValueError('Items must be an object')
    asdu = {'PatrolDevice': {'Type': type_, 'Command': command,
                            'Time': t or now_str(), 'Items': items if items is not None else {}}}
    body = json.dumps(asdu, ensure_ascii=False, allow_nan=False).encode('utf-8')
    if len(body) > MAX_BODY_SIZE:
        raise ValueError('ASDU exceeds 65535 bytes')
    return SYNC + struct.pack('<HHB', len(body), msg_id & 0xFFFF, FMT_JSON) + bytes(7) + body


def _body_length(buf):
    if buf[:4] != SYNC or buf[8] != FMT_JSON or buf[9:16] != bytes(7):
        raise ValueError('invalid frame header')
    return struct.unpack_from('<H', buf, 4)[0]


def _reject_constant(value):
    raise ValueError(f'invalid JSON constant: {value}')


def decode(buf):
    """Decode exactly one frame; return None for any invalid frame."""
    try:
        if len(buf) < HEADER_SIZE:
            return None
        length = _body_length(buf)
        if len(buf) != HEADER_SIZE + length:
            return None
        obj = json.loads(bytes(buf[HEADER_SIZE:]).decode('utf-8'),
                         parse_constant=_reject_constant)
        if not isinstance(obj, dict):
            return None
        pd = obj.get('PatrolDevice')
        if not isinstance(pd, dict) or not isinstance(pd.get('Items'), dict):
            return None
        if type(pd.get('Type')) is not int or type(pd.get('Command')) is not int:
            return None
        return {'msg_id': struct.unpack_from('<H', buf, 6)[0],
                'type': pd['Type'], 'command': pd['Command'],
                'items': pd['Items'], 'raw': pd}
    except (ValueError, TypeError, UnicodeError, struct.error, RecursionError):
        return None


class StreamDecoder:
    """Incremental TCP decoder. Invalid frames poison the connection, never resync."""

    def __init__(self):
        self._buffer = bytearray()
        self._failed = False

    def feed(self, data):
        if self._failed:
            raise ValueError('decoder has failed')
        self._buffer.extend(data)
        messages = []
        try:
            while len(self._buffer) >= HEADER_SIZE:
                size = HEADER_SIZE + _body_length(self._buffer)
                if len(self._buffer) < size:
                    break
                msg = decode(self._buffer[:size])
                if msg is None:
                    raise ValueError('invalid JSON frame')
                del self._buffer[:size]
                messages.append(msg)
            return messages
        except ValueError:
            self._failed = True
            self._buffer.clear()
            raise

    def eof(self):
        if self._failed or self._buffer:
            self._failed = True
            self._buffer.clear()
            raise ValueError('truncated TCP frame')


def heartbeat():
    return encode(100, 100)


def set_mode(mode):
    return encode(1101, 5, {'Mode': int(mode)})


def set_motion_state(param):
    return encode(2, 22, {'MotionParam': int(param)})


def set_gait(gait):
    return encode(2, 23, {'GaitParam': int(gait)})


def axis_cmd(x, y, yaw, z=0.0, roll=0.0, pitch=0.0):
    return encode(2, 21, {'X': float(x), 'Y': float(y), 'Z': float(z),
                          'Roll': float(roll), 'Pitch': float(pitch), 'Yaw': float(yaw)})
