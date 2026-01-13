"""Minimal dante node stub.

This node is intentionally lightweight: if rclpy (ROS2) is available it will
start a real node; otherwise it will run as a simple heartbeat stub so tests
and local checks don't require a ROS2 install.
"""
import time


def main():
    try:
        import rclpy
        from rclpy.node import Node

        class DanteNode(Node):
            def __init__(self):
                super().__init__('dante_node')
                self.get_logger().info('Dante node started (ROS2)')
        rclpy.init()
        node = DanteNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        # Fallback: simple loop that prints heartbeat for local testing
        for i in range(3):
            print(f"dante_node heartbeat {i+1}/3")
            time.sleep(0.2)


if __name__ == '__main__':
    main()
"""Example rclpy node for Dante robot (minimal).

This node demonstrates a simple ROS2 Python node that logs a heartbeat.
"""

import rclpy
from rclpy.node import Node


class DanteNode(Node):
    def __init__(self):
        super().__init__('dante_node')
        self.get_logger().info('Dante node started')
        self.timer = self.create_timer(2.0, self._heartbeat)

    def _heartbeat(self):
        self.get_logger().info('Dante heartbeat')


def main(args=None):
    rclpy.init(args=args)
    node = DanteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
