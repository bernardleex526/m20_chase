#!/usr/bin/env python3
"""Fake M20 basic_server (UDP) for loopback testing of m20_bridge.

It acks every request (Type/Command echoed, ErrorCode 0), tracks the client
that sent the heartbeat, prints received axis commands, and streams a synthetic
1002/4 MotionStatus at 10 Hz so the bridge's odometry integration is exercised.

  python3 -m m20_bridge.fake_m20_server --ip 127.0.0.1 --port 30000
"""

import argparse
import math
import socket
import time

from m20_bridge import protocol as P

FULL_X, FULL_Y, FULL_YAW = 2.0, 1.0, 1.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=30000)
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((args.ip, args.port))
    s.settimeout(0.1)
    print(f'[fake-m20] listening UDP {args.ip}:{args.port}', flush=True)

    client = None
    x = y = yaw = 0.0
    last_x = last_y = last_yaw = 0.0
    last_print = 0.0
    last_status = 0.0
    last_t = time.time()

    while True:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            data = None
        if data:
            msg = P.decode(data)
            if msg:
                client = addr
                if msg['type'] == 100 and msg['command'] == 100:
                    pass  # heartbeat
                elif msg['type'] == 2 and msg['command'] == 21:
                    it = msg['items']
                    last_x, last_y, last_yaw = it.get('X', 0.0), it.get('Y', 0.0), it.get('Yaw', 0.0)
                    now = time.time()
                    if now - last_print > 1.0:
                        last_print = now
                        print(f'[fake-m20] axis X={last_x:+.3f} Y={last_y:+.3f} Yaw={last_yaw:+.3f}', flush=True)
                else:
                    print(f'[fake-m20] req Type={msg["type"]} Cmd={msg["command"]} Items={msg["items"]}', flush=True)
                # generic ack
                s.sendto(P.encode(msg['type'], msg['command'],
                                  {"ErrorCode": 0, "ErrorMessage": "Success"}), addr)

        now = time.time()
        if client and now - last_status > 0.1:
            last_status = now
            dt = now - last_t
            last_t = now
            yaw += last_yaw * FULL_YAW * dt
            vx = last_x * FULL_X
            vy = last_y * FULL_Y
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            ms = {"Yaw": yaw, "OmegaZ": last_yaw * FULL_YAW, "LinearX": vx, "LinearY": vy,
                  "Height": 0.35, "Roll": 0.0, "Pitch": 0.0}
            s.sendto(P.encode(1002, 4, {"MotionStatus": ms}), client)
            bs = {"MotionState": 1, "Gait": 0x1001, "HES": 0, "ControlUsageMode": 0,
                  "Direction": 0, "Version": "PRO"}
            s.sendto(P.encode(1002, 6, {"BasicStatus": bs}), client)


if __name__ == '__main__':
    main()
