"""M20 / M20 Pro basic_server protocol codec.

APDU = 16-byte header + ASDU(JSON)
header: EB 91 EB 90 | len(u16 LE) | msg_id(u16 LE) | fmt(0x01=JSON) | 7 x 0x00
ASDU  : {"PatrolDevice": {"Type": t, "Command": c, "Time": "...", "Items": {...}}}
"""

import json
import struct
import time

SYNC = bytes([0xEB, 0x91, 0xEB, 0x90])
FMT_JSON = 0x01


def now_str():
    return time.strftime('%Y-%m-%d %H:%M:%S')


def encode(type_, command, items=None, msg_id=0, t=None):
    asdu = {"PatrolDevice": {"Type": type_, "Command": command,
                             "Time": t or now_str(), "Items": items or {}}}
    body = json.dumps(asdu, ensure_ascii=False).encode('utf-8')
    header = SYNC + struct.pack('<H', len(body)) + struct.pack('<H', msg_id & 0xFFFF) \
        + bytes([FMT_JSON]) + bytes(7)
    return header + body


def decode(buf):
    """Return dict(msg_id, type, command, items) or None."""
    if len(buf) < 16 or buf[:4] != SYNC:
        return None
    length = struct.unpack('<H', buf[4:6])[0]
    msg_id = struct.unpack('<H', buf[6:8])[0]
    body = buf[16:16 + length]
    try:
        obj = json.loads(body.decode('utf-8'))
    except Exception:
        return None
    pd = obj.get('PatrolDevice', {})
    return {'msg_id': msg_id, 'type': pd.get('Type'), 'command': pd.get('Command'),
            'items': pd.get('Items', {}), 'raw': pd}


# convenience wrappers ------------------------------------------------------- #
def heartbeat():
    return encode(100, 100)


def set_mode(mode):                      # 0 regular, 1 nav, 2 assist
    return encode(1101, 5, {"Mode": int(mode)})


def set_motion_state(param):             # 1 stand, 2 damp/soft-stop, 4 lie, 17 RL
    return encode(2, 22, {"MotionParam": int(param)})


def set_gait(gait):                      # 0x1001 basic, 0x3002 flat
    return encode(2, 23, {"GaitParam": int(gait)})


def axis_cmd(x, y, yaw, z=0.0, roll=0.0, pitch=0.0):
    return encode(2, 21, {"X": float(x), "Y": float(y), "Z": float(z),
                          "Roll": float(roll), "Pitch": float(pitch), "Yaw": float(yaw)})
