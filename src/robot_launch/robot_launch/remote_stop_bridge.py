#!/usr/bin/env python3
"""Cancel navigation when manual velocity arrives; twist_mux owns arbitration."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RemoteStopBridge(Node):
    def __init__(self):
        super().__init__('remote_stop_bridge')
        self.last_stop = -math.inf
        self.stop = self.create_publisher(Bool, 'stop_planner', 10)
        self.create_subscription(Twist, 'cmd_vel_remote', self.receive, 10)

    def receive(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        now = time.monotonic()
        if all(math.isfinite(v) for v in values) and any(abs(v) > 1e-6 for v in values) and now-self.last_stop > .2:
            self.stop.publish(Bool(data=True))
            self.last_stop = now

def main():
    rclpy.init()
    node = RemoteStopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
