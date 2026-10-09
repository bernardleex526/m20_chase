"""A configured distinct topic must still be rejected after ROS remapping."""
import pytest
import rclpy

from dog_adapters.twist_adapter import TwistAdapter


def test_remapped_output_cannot_feed_adapter_input():
    rclpy.init(args=['--ros-args', '-p', 'output_topic:=/loopback/controller_cmd_vel',
                    '-r', '/loopback/controller_cmd_vel:=/rs_follow/cmd_vel'])
    try:
        with pytest.raises(ValueError, match='resolve/remap to the same topic'):
            TwistAdapter()
    finally:
        rclpy.shutdown()
