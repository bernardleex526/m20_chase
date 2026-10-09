"""Keep the ROS context alive long enough to publish an exit stop."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def run(node_type, args=None):
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        for sig in handlers:
            signal.signal(sig, _interrupt)
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
            rclpy.try_shutdown()
