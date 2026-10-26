#!/usr/bin/env python3
"""Bound the entire Nav2 startup, including replies from the relay itself."""
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('lifecycle_startup_watchdog')
    timeout = float(node.declare_parameter('startup_timeout', 180.0).value)
    if timeout <= 0:
        raise ValueError('startup_timeout must be positive')
    client = node.create_client(Trigger, 'lifecycle_manager_navigation/is_active')
    deadline = time.monotonic() + timeout
    success = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline-time.monotonic()
            if not client.wait_for_service(timeout_sec=min(.5, remaining)):
                continue
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=min(1.0, remaining))
            if future.done() and not future.exception() and future.result().success:
                success = True
                node.get_logger().info('Navigation startup verified: manager active and lifecycle bonds established')
                break
            if not future.done():
                client.remove_pending_request(future)
            # Use wall time: simulation time may be paused or unavailable.
            rclpy.spin_once(node, timeout_sec=min(.2, max(0.,deadline-time.monotonic())))
        if not success and rclpy.ok():
            node.get_logger().error('Navigation startup deadline exceeded; shutting down navigation instead of waiting indefinitely')
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
