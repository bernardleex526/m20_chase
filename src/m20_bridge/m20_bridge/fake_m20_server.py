#!/usr/bin/env python3
"""Controlled UDP/TCP basic_server fixture, not a robot SDK substitute."""

import argparse
import socket
import time

from m20_bridge import protocol as P


class FakeState:
    def __init__(self, hes=0, mode=0, motion=1, gait=0x1001):
        self.hes, self.mode, self.motion, self.gait = hes, mode, motion, gait
        self.axes = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.requests = []

    def handle(self, msg):
        self.requests.append((msg['type'], msg['command'], msg['items']))
        del self.requests[:-100]
        key, items = (msg['type'], msg['command']), msg['items']
        if key == (1101, 5):
            self.mode = items['Mode']
        elif key == (2, 22):
            self.motion = items['MotionParam']
        elif key == (2, 23):
            self.gait = items['GaitParam']
        elif key == (2, 21):
            self.axes = tuple(items.get(k, 0.0) for k in ('X', 'Y', 'Yaw'))
        return P.encode(msg['type'], msg['command'], {'ErrorCode': 0, 'ErrorMessage': 'Success'})

    def feedback(self, dt):
        vx, vy, wz = (self.axes[0] * 2.0, self.axes[1], self.axes[2] * 1.5)
        if self.hes != 0 or self.mode != 0:
            vx = vy = wz = 0.0
        if 0 < dt <= 1.0:
            self.yaw += wz * dt
        return (P.encode(1002, 6, {'BasicStatus': {'HES': self.hes,
                    'ControlUsageMode': self.mode, 'MotionState': self.motion, 'Gait': self.gait}}),
                P.encode(1002, 4, {'MotionStatus': {'Yaw': self.yaw, 'LinearX': vx,
                                                  'LinearY': vy, 'OmegaZ': wz}}))


def fragments(data, size):
    """Deterministic fragmentation fixture; size=0 sends one coalesced write."""
    if size <= 0:
        yield data
    else:
        for offset in range(0, len(data), size):
            yield data[offset:offset + size]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=30000)
    parser.add_argument('--transport', choices=('udp', 'tcp'), default='udp')
    parser.add_argument('--hes', type=int, default=0)
    parser.add_argument('--mode', type=int, default=0)
    parser.add_argument('--motion-state', type=int, default=1)
    parser.add_argument('--gait', type=lambda v: int(v, 0), default=0x1001)
    parser.add_argument('--fragment-size', type=int, default=0)
    parser.add_argument('--no-feedback', action='store_true')
    parser.add_argument('--feedback-seconds', type=float, default=0.0,
                        help='Stop feedback after this duration; zero means indefinitely')
    parser.add_argument('--disconnect-seconds', type=float, default=0.0)
    args = parser.parse_args()
    state = FakeState(args.hes, args.mode, args.motion_state, args.gait)
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if args.transport == 'udp' else socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.ip, args.port))
    connection = listener
    try:
        if args.transport == 'tcp':
            listener.listen(1)
            connection, _ = listener.accept()
        connection.settimeout(0.05)
        decoder = P.StreamDecoder()
        client = None
        start = last_feedback = time.monotonic()

        def send(data):
            if args.transport == 'udp':
                connection.sendto(data, client)
            else:
                for chunk in fragments(data, args.fragment_size):
                    connection.sendall(chunk)

        while True:
            if args.disconnect_seconds > 0 and time.monotonic() - start >= args.disconnect_seconds:
                break
            try:
                if args.transport == 'udp':
                    data, client = connection.recvfrom(65535)
                    msg = P.decode(data)
                    messages = [] if msg is None else [msg]
                else:
                    data = connection.recv(65535)
                    if not data:
                        decoder.eof()
                        break
                    messages = decoder.feed(data)
                for msg in messages:
                    send(state.handle(msg))
                    print(f'[fake-m20] Type={msg["type"]} Command={msg["command"]} Items={msg["items"]}', flush=True)
            except socket.timeout:
                pass
            now = time.monotonic()
            feedback_enabled = not args.no_feedback and (args.feedback_seconds <= 0 or now - start < args.feedback_seconds)
            if feedback_enabled and (client is not None or args.transport == 'tcp') and now - last_feedback >= 0.1:
                packets = state.feedback(now - last_feedback)
                last_feedback = now
                if args.transport == 'tcp':
                    send(b''.join(packets))
                else:
                    for packet in packets:
                        send(packet)
    except (KeyboardInterrupt, OSError, ValueError):
        pass
    finally:
        connection.close()
        if connection is not listener:
            listener.close()


if __name__ == '__main__':
    main()
