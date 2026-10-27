"""run(MyNode): the usual init / spin / shutdown, without noise on Ctrl+C.

When a launch is stopped, a timer can fire while ROS is already shutting
down, and publishing then raises an error that means nothing. This hides
those, but real crashes (while ROS is still running) are still raised.
"""
import rclpy
from rclpy.executors import ExternalShutdownException


def run(node_factory):
    rclpy.init()
    node = node_factory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
