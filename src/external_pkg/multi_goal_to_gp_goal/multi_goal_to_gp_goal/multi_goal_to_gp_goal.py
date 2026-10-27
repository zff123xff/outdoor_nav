#!/usr/bin/env python3
"""Bridge msg_set_msgs/MultiGoal to sensor_msgs/NavSatFix on /gp_goal.

Stateful sequential dispatcher:
  - Subscribes to /waypoint_goals (default) and queues all GPS waypoints
  - Subscribes to /mavros/local_position/odom to detect "arrival" at the
    current waypoint
  - Publishes one NavSatFix at a time on /gp_goal; when the robot gets
    within `reach_threshold_m` of the current goal for `consecutive_required`
    consecutive samples, advances to the next waypoint in the queue
  - new MultiGoal arriving while the queue is non-empty is *appended* to
    the tail (currently-executing goal is not disturbed)

Local-frame waypoints (is_gps_aid=False) are skipped with a warning since
they cannot be expressed as NavSatFix. Origin lat/lon/alt are taken from
parameters so the bridge can compute ENU distances to detect arrival.
"""

import math
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


WGS84_R = 6378137.0


@dataclass
class Waypoint:
    latitude: float
    longitude: float
    altitude: float


def lla_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_m: float,
) -> tuple[float, float, float]:
    """Spherical tangent-plane projection (matches nav2_input_bridge)."""
    lat_r = math.radians(lat_deg)
    lon_r = math.radians(lon_deg)
    lat0_r = math.radians(lat0_deg)
    lon0_deg_r = math.radians(lon0_deg)
    sin_lat = math.sin(lat_r)
    cos_lat = math.cos(lat_r)
    sin_lat0 = math.sin(lat0_r)
    cos_lat0 = math.cos(lat0_r)
    dlon = lon_r - lon0_deg_r
    arg = sin_lat0 * sin_lat + cos_lat0 * cos_lat * math.cos(dlon)
    arg = max(-1.0, min(1.0, arg))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 1e-6 else 1.0
    north = k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * math.cos(dlon)) * WGS84_R
    east = k * cos_lat * math.sin(dlon) * WGS84_R
    up = alt_m - alt0_m
    return east, north, up


class MultiGoalToGpGoal(Node):
    def __init__(self) -> None:
        super().__init__("multi_goal_to_gp_goal")

        self.declare_parameter("input_topic", "waypoint_goals")
        self.declare_parameter("output_topic", "gp_goal")
        # odom_topic defaults to RELATIVE "odom" so the per-node namespace
        # (set via launch `namespace=robot_namespace`) prefixes the topic to
        # /{robot_namespace}/odom — matching the RELIABLE odom produced by
        # mavros_odom_relay in launch_all_go2w.sh. Pass an absolute path
        # explicitly when running un-namespaced.
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("navsat_status", 1)   # NavSatStatus.STATUS_FIX
        self.declare_parameter("navsat_service", 1)  # NavSatStatus.SERVICE_GPS

        # Origin for ENU distance computation
        self.declare_parameter("reference_latitude", 0.0)
        self.declare_parameter("reference_longitude", 0.0)
        self.declare_parameter("reference_altitude", 0.0)

        # Arrival detection
        self.declare_parameter("reach_threshold_m", 0.5)
        self.declare_parameter("consecutive_required", 5)
        self.declare_parameter("check_period_s", 0.1)  # 10 Hz

        # For logging only; the actual namespace is inherited from the launch tree.
        self.declare_parameter("robot_namespace", "")

        # Inputs / Outputs
        self._sub = self.create_subscription(
            MultiGoal, self.get_parameter("input_topic").value, self._on_multi_goal, 10
        )
        self._pub = self.create_publisher(
            NavSatFix, self.get_parameter("output_topic").value, 10
        )

        # odom QoS: best-effort compatible with /mavros/local_position/odom,
        # but mavros_odom_relay re-publishes as RELIABLE so we still get data
        # even when subscribing RELIABLE.
        odom_qos = QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, odom_qos
        )

        # State
        self._queue: deque[Waypoint] = deque()
        self._current_goal_enu: tuple[float, float, float] | None = None
        self._consecutive_within = 0
        self._latest_pose: tuple[float, float] | None = None  # (east, north) in odom frame
        self._last_origin_warn_ns = 0  # throttling for origin-not-set warning

        # Periodic check timer for arrival
        period = self.get_parameter("check_period_s").value
        self._check_timer = self.create_timer(period, self._on_check_tick)

        self.get_logger().info(
            f"multi_goal_to_gp_goal: input={self.get_parameter('input_topic').value} -> "
            f"output={self.get_parameter('output_topic').value}, "
            f"odom={self.get_parameter('odom_topic').value} "
            f"(reach<{self.get_parameter('reach_threshold_m').value:.2f}m "
            f"x{self.get_parameter('consecutive_required').value}, {1.0/period:.0f}Hz)"
        )
        # Log the actual resolved topic names so namespace misconfiguration
        # is visible at startup.
        self.get_logger().info(
            f"  resolved: sub={self._sub.topic_name}, pub={self._pub.topic_name}, "
            f"odom_sub={self._odom_sub.topic_name}"
        )

    # ----- subscribers ----------------------------------------------------

    def _on_multi_goal(self, msg: MultiGoal) -> None:
        if not msg.is_gps_aid:
            self.get_logger().warn(
                "Received MultiGoal with is_gps_aid=False (local frame); cannot express as NavSatFix, skipping."
            )
            return

        new_points = [
            Waypoint(
                latitude=p.x_or_lat,
                longitude=p.y_or_lon,
                altitude=p.z_or_alt,
            )
            for p in msg.multi_goal_points
        ]
        if not new_points:
            self.get_logger().warn("Received MultiGoal with empty waypoint list; skipping.")
            return

        self._queue.extend(new_points)

        self.get_logger().info(
            f"Queued {len(new_points)} waypoint(s); queue size: {len(self._queue)}"
        )

        # If nothing is currently being executed, start the first waypoint.
        if self._current_goal_enu is None:
            self._advance_to_next()

    def _on_odom(self, msg: Odometry) -> None:
        # odom is in ENU (local) frame
        self._latest_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    # ----- periodic arrival check -----------------------------------------

    def _on_check_tick(self) -> None:
        if self._current_goal_enu is None or self._latest_pose is None:
            return

        goal_e, goal_n, _ = self._current_goal_enu
        pose_e, pose_n = self._latest_pose
        dist = math.hypot(pose_e - goal_e, pose_n - goal_n)

        threshold = self.get_parameter("reach_threshold_m").value
        required = self.get_parameter("consecutive_required").value

        if dist <= threshold:
            self._consecutive_within += 1
        else:
            self._consecutive_within = 0

        self.get_logger().debug(
            f"tick: dist={dist:.2f} thr={threshold:.2f} count={self._consecutive_within}/{required}"
        )

        if self._consecutive_within >= required:
            self.get_logger().info(
                f"Reached current waypoint (final dist={dist:.2f}m, threshold={threshold:.2f}m)"
            )
            self._consecutive_within = 0
            self._advance_to_next()

    # ----- queue management ----------------------------------------------

    def _advance_to_next(self) -> None:
        if not self._queue:
            self._current_goal_enu = None
            self.get_logger().info("Queue empty, idle.")
            return

        wp = self._queue.popleft()
        lat0 = self.get_parameter("reference_latitude").value
        lon0 = self.get_parameter("reference_longitude").value
        alt0 = self.get_parameter("reference_altitude").value
        if abs(lat0) < 1e-9 and abs(lon0) < 1e-9:
            # No origin configured yet — push back and retry next tick
            self._queue.appendleft(wp)
            self._current_goal_enu = None
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_origin_warn_ns > 5_000_000_000:
                self.get_logger().warn(
                    "reference_latitude/longitude not configured; deferring goal publication."
                )
                self._last_origin_warn_ns = now_ns
            return

        east, north, up = lla_to_enu(wp.latitude, wp.longitude, wp.altitude, lat0, lon0, alt0)
        self._current_goal_enu = (east, north, up)
        self._consecutive_within = 0

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.get_parameter("frame_id").value
        fix.latitude = wp.latitude
        fix.longitude = wp.longitude
        fix.altitude = wp.altitude
        fix.status.status = self.get_parameter("navsat_status").value
        fix.status.service = self.get_parameter("navsat_service").value

        self._pub.publish(fix)
        self.get_logger().info(
            f"Forwarded waypoint: lat={wp.latitude:.7f}, lon={wp.longitude:.7f}, alt={wp.altitude:.3f} "
            f"(ENU=({east:.2f}, {north:.2f}, {up:.2f}) m, remaining={len(self._queue)})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiGoalToGpGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()