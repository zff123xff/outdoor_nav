#!/usr/bin/env python3
"""Relay mavros odom (BEST_EFFORT) to /{ns}/odom (RELIABLE).

mavros publishes /{ns}/mavros/odom (the local_position plugin's ~/odom)
with BEST_EFFORT QoS. Nav2 controller_server subscribes to its odom_topic
with RELIABLE QoS by default — the two are incompatible, so Nav2 silently
never receives odom and never publishes /cmd_vel. This node bridges the two
QoS regimes by re-publishing each incoming odom with RELIABLE.

Simulation launch additionally enables publish_gps: odom positions must be ENU
relative to the navigation YAML reference. Each sample produces a NavSatFix on
mavros/global_position/global using that same projection and input timestamp.
This is disabled by default for hardware use.
"""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from multi_goal_to_gp_goal.coordinates import enu_to_lla


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("mavros_odom_relay")

        self.declare_parameter("input_topic", "mavros/odom")
        self.declare_parameter("output_topic", "odom")
        self.declare_parameter("frame_id_rewrite", "")
        self.declare_parameter("publish_gps", False)
        self.declare_parameter("gps_topic", "mavros/global_position/global")
        for name in ("reference_latitude", "reference_longitude", "reference_altitude"):
            self.declare_parameter(name, 0.0)
        self._origin = tuple(self.get_parameter(name).value for name in (
            "reference_latitude", "reference_longitude", "reference_altitude"))
        self.add_on_set_parameters_callback(self._on_origin_parameters)
        self._gps_pub = None
        if self.get_parameter("publish_gps").value:
            enu_to_lla(0., 0., 0., *self._origin)  # Validate before publishing.
            self._gps_pub = self.create_publisher(
                NavSatFix, self.get_parameter("gps_topic").value, QoSProfile(depth=10))
            self.get_logger().info(
                f"Simulation GPS: ENU odometry -> {self._gps_pub.topic_name}; reference={self._origin}")

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

    def _on_origin_parameters(self, parameters):
        keys = ('reference_latitude', 'reference_longitude', 'reference_altitude')
        updates = {p.name: p.value for p in parameters if p.name in keys}
        if not updates:
            return SetParametersResult(successful=True)
        candidate = tuple(updates.get(key, value) for key, value in zip(keys, self._origin))
        try:
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in candidate):
                raise ValueError('GPS reference parameters must be numeric')
            enu_to_lla(0., 0., 0., *candidate)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        self._origin = tuple(float(v) for v in candidate)
        return SetParametersResult(successful=True)

    def _on_odom(self, msg: Odometry) -> None:
        rewrite = self.get_parameter("frame_id_rewrite").value
        if rewrite:
            msg.header.frame_id = rewrite
        self._pub.publish(msg)
        if self._gps_pub is not None:
            position = msg.pose.pose.position
            try:
                latitude, longitude, altitude = enu_to_lla(
                    position.x, position.y, position.z, *self._origin)
            except ValueError as exc:
                self.get_logger().warning(f"Skipping invalid simulation GPS: {exc}", throttle_duration_sec=5.)
                return
            fix = NavSatFix()
            fix.header.stamp = msg.header.stamp
            fix.header.frame_id = msg.child_frame_id or (
                self.get_namespace().strip('/') + '/base_link').lstrip('/')
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude, fix.longitude, fix.altitude = latitude, longitude, altitude
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self._gps_pub.publish(fix)


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
