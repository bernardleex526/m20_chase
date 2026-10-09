"""Common ROS wiring; safety decisions remain in CommandGate."""
import math
import time

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool, String

from dog_adapters.command_gate import CLEAR_ACTIONS, STOP_ACTIONS, CommandGate

STAND_ACTIONS = ('standup', 'stand_up', 'stand', 'up')
LIE_ACTIONS = ('liedown', 'lie_down', 'down', 'lie')


class AdapterNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/rs_follow/cmd_vel').value
        self.estop_topic = self.declare_parameter('estop_topic', '/rs_follow/estop').value
        self.rate_hz = self.declare_parameter('rate_hz', 20.0).value
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0:
            raise ValueError('rate_hz must be finite and positive')
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.gate = CommandGate(
            self.declare_parameter('watchdog_timeout', 0.3).value,
            self.declare_parameter('max_vx', 0.3).value,
            self.declare_parameter('max_vy', 0.15).value,
            self.declare_parameter('max_wz', 0.5).value,
            self.declare_parameter('lateral', True).value)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd, 10)
        self.estop_sub = self.create_subscription(Bool, self.estop_topic, self._on_estop, 10)
        self.action_sub = self.create_subscription(
            String, '/rs_follow/action_cmd', self._on_action, 10)

    def _on_cmd(self, msg):
        if not self.gate.accept(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic()):
            self.get_logger().warning('Rejected invalid or estopped velocity command')
            self._halt()

    def _on_estop(self, msg):
        self.gate.set_estop(msg.data)
        if msg.data:
            self._halt()

    def _on_action(self, msg):
        action = msg.data.strip().lower()
        if action in STOP_ACTIONS:
            self.gate.set_estop(True)
            self._halt()
        elif action in CLEAR_ACTIONS:
            self.gate.set_estop(False)
        else:
            self._posture(action)

    def _posture(self, action):
        self.get_logger().error(f'Unsupported action: {action!r}')

    def _halt(self):
        raise NotImplementedError
