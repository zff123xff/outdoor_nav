#!/usr/bin/env python3
"""Relay mavros odom (BEST_EFFORT) to /{ns}/odom (RELIABLE).

mavros publishes /{ns}/mavros/odom (the local_position plugin's ~/odom)
with BEST_EFFORT QoS. Nav2 controller_server subscribes to its odom_topic
with RELIABLE QoS by default — the two are incompatible, so Nav2 silently
never receives odom and never publishes /cmd_vel. This node bridges the two
QoS regimes by re-publishing each incoming odom with RELIABLE.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("mavros_odom_relay")

        self.declare_parameter("input_topic", "mavros/odom")
        self.declare_parameter("output_topic", "odom")
        self.declare_parameter("frame_id_rewrite", "")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # Subscribe with BEST_EFFORT + sensor_data profile to match mavros.
        sub_qos = qos_profile_sensor_data
        sub_qos.depth = 30  # keep a small buffer against transient drops
        self._sub = self.create_subscription(
            Odometry, in_topic, self._on_odom, sub_qos
        )

        # Publish RELIABLE so Nav2's default RELIABLE subscriber accepts.
        pub_qos = QoSProfile(depth=30, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Odometry, out_topic, pub_qos)

        self.get_logger().info(
            f"mavros_odom_relay: {in_topic} (BEST_EFFORT) -> {out_topic} (RELIABLE)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        rewrite = self.get_parameter("frame_id_rewrite").value
        if rewrite:
            msg.header.frame_id = rewrite
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: the context was already shut down (e.g.
        # launch sent SIGINT during shutdown) — not an error.
        pass
    finally:
        node.destroy_node()
        # rcl_shutdown may already have been called by the SIGINT handler; a
        # second call raises RCLError and makes the process exit with code 1,
        # which launch reports as "process has died" — misleading, because the
        # node was shutting down normally.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()