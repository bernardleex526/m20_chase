"""Watchdog-limited Twist forwarding to an explicitly configured controller."""
import time

from geometry_msgs.msg import Twist, TwistStamped

from dog_adapters.adapter_node import AdapterNode
from dog_adapters.runtime import run


class TwistAdapter(AdapterNode):
    def __init__(self):
        super().__init__('twist_adapter')
        output_topic = self.declare_parameter('output_topic', '').value
        if not output_topic or not output_topic.strip():
            raise ValueError('output_topic is required; configure the robot controller topic explicitly')
        if self.resolve_topic_name(self.cmd_vel_topic) == self.resolve_topic_name(output_topic):
            raise ValueError('input and output topics must differ: resolve/remap to the same topic')
        self.stamped = self.declare_parameter('stamped', False).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publisher = self.create_publisher(
            TwistStamped if self.stamped else Twist, output_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick, clock=self.safety_clock)

    def _publish(self, velocity):
        msg = TwistStamped() if self.stamped else Twist()
        if self.stamped:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
        twist = msg.twist if self.stamped else msg
        twist.linear.x, twist.linear.y, twist.angular.z = velocity
        self.publisher.publish(msg)

    def _tick(self):
        self._publish(self.gate.output(time.monotonic(), True))

    def _halt(self):
        self._publish((0.0, 0.0, 0.0))

    def close(self):
        self.gate.set_estop(True)
        self._halt()


def main(args=None):
    run(TwistAdapter, args)
