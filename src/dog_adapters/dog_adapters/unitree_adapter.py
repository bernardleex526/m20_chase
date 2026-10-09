"""Unitree sport requests using only the official ROS unitree_api messages."""
import json
import time

from std_srvs.srv import SetBool

from dog_adapters.adapter_node import AdapterNode, LIE_ACTIONS, STAND_ACTIONS
from dog_adapters.runtime import run

MOVE = 1008
STOP = 1003
STAND = 1004
LIE = 1005


def load_request_type():
    # Keep Twist and CommandGate usable without the optional Unitree dependency.
    try:
        from unitree_api.msg import Request
    except ImportError as exc:
        raise RuntimeError(
            'unitree_adapter requires the official unitree_api ROS 2 message package. '
            'Build unitree_api from unitree_ros2 and source its install/setup.bash; '
            'no substitute SDK or message definitions are supported.') from exc
    return Request


class UnitreeAdapter(AdapterNode):
    def __init__(self):
        self.request_type = load_request_type()
        super().__init__('unitree_adapter')
        self.request_topic = self.declare_parameter('request_topic', '/api/sport/request').value
        self.publisher = self.create_publisher(self.request_type, self.request_topic, 10)
        self.armed = False
        self._armed_at = None
        self.arm_service = self.create_service(SetBool, '~/arm', self._on_arm)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick, clock=self.safety_clock)
        self._send(STOP)

    def _send(self, api_id, velocity=None):
        # Request headers must never leak from a preceding request.
        request = self.request_type()
        request.header.identity.api_id = api_id
        request.parameter = (json.dumps(dict(zip(('x', 'y', 'z'), velocity)),
                                        allow_nan=False) if velocity is not None else '')
        self.publisher.publish(request)

    def _on_arm(self, request, response):
        if not request.data:
            self._halt()
            response.success = True
            response.message = 'Disarmed and StopMove sent'
        elif self.gate.estopped:
            response.success = False
            response.message = 'Cannot arm while emergency stop is latched; clear it first'
        elif self.armed:
            response.success = True
            response.message = 'Already armed'
        else:
            self.gate.accept(float('nan'), 0.0, 0.0, time.monotonic())
            self.armed = True
            self._armed_at = time.monotonic()
            response.success = True
            response.message = 'Armed; a new velocity command is required before movement'
        return response

    def _halt(self):
        self.armed = False
        self._armed_at = None
        self.gate.output(time.monotonic(), False)
        self._send(STOP)

    def _tick(self):
        now = time.monotonic()
        if not self.armed or self.gate.estopped:
            self._halt()
            return
        if not self.gate.fresh(now):
            # Arming permits one watchdog interval to receive the first command.
            if self._armed_at is None or now - self._armed_at >= self.gate.timeout_s:
                self._halt()
            else:
                self._send(STOP)
            return
        self._send(MOVE, self.gate.output(now, True))

    def _posture(self, action):
        if action not in STAND_ACTIONS + LIE_ACTIONS:
            super()._posture(action)
            return
        if not self.armed or self.gate.estopped:
            self.get_logger().error(f'Rejected {action!r}: arm first and clear emergency stop')
            return
        # Explicit posture only; never Damp or automatic standing.
        self.gate.output(time.monotonic(), False)
        self._armed_at = time.monotonic()
        self._send(STAND if action in STAND_ACTIONS else LIE)

    def close(self):
        self.gate.set_estop(True)
        self._halt()


def main(args=None):
    run(UnitreeAdapter, args)
