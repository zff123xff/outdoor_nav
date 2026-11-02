#!/usr/bin/env python3
"""Sequential GPS waypoint dispatcher driven by navigation Action results."""

import math
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from msg_set_msgs.msg import MultiGoal
from action_msgs.msg import GoalStatus
from multi_goal_to_gp_goal.navigation_client import NavigationClient


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
        # odom_topic defaults to RELATIVE "odom" so the per-node namespace
        # (set via launch `namespace=robot_namespace`) prefixes the topic to
        # /{robot_namespace}/odom — matching the RELIABLE odom produced by
        # mavros_odom_relay in launch_real.sh. Pass an absolute path
        # explicitly when running un-namespaced.

        # Origin for ENU distance computation
        self.declare_parameter("reference_latitude", 0.0)
        self.declare_parameter("reference_longitude", 0.0)
        self.declare_parameter("reference_altitude", 0.0)

        # For logging only; the actual namespace is inherited from the launch tree.
        self.declare_parameter("robot_namespace", "")

        # Inputs / Outputs
        self._sub = self.create_subscription(
            MultiGoal, self.get_parameter("input_topic").value, self._on_multi_goal, 10
        )
        self._navigation = NavigationClient(self)

        # State
        self._queue: deque[Waypoint] = deque()
        self._current_goal_enu: tuple[float, float, float] | None = None
        self._current_waypoint: Waypoint | None = None
        self._patrol_points: tuple[Waypoint, Waypoint] | None = None
        self._pending_patrol: tuple[tuple[Waypoint, Waypoint], int, float] | None = None
        self._patrol_max_laps = 0
        self._patrol_hold_s = 0.0
        self._patrol_reached_points = 0
        self._holding_until_ns: int | None = None
        self._last_origin_warn_ns = 0  # throttling for origin-not-set warning

        self._check_timer = self.create_timer(0.1, self._on_check_tick)
        self.get_logger().info('GPS queue advances only on navigate_to_pose success')

    # ----- subscribers ----------------------------------------------------

    def _on_multi_goal(self, msg: MultiGoal) -> None:
        if msg.cancel_patrol:
            if self._patrol_points is not None:
                self._queue.clear()
                self._patrol_points = None
                self._current_goal_enu = None
                self._current_waypoint = None
                self._holding_until_ns = None
                self._navigation.cancel()
                self.get_logger().warn("Patrol canceled through ExecuteTask; planner stop requested.")
            elif self._pending_patrol is not None:
                self._pending_patrol = None
                self.get_logger().warn("Queued patrol canceled before it started.")
            else:
                self.get_logger().warn("Received cancel_patrol, but no pipeline patrol is active.")
            return

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

        if msg.is_patrol:
            if len(new_points) != 2:
                self.get_logger().error("Patrol MultiGoal must contain exactly two GPS endpoints; ignoring.")
                return
            if self._patrol_points is not None or self._pending_patrol is not None:
                self.get_logger().error("A patrol is already active or queued; rejecting another patrol.")
                return
            patrol_points = (new_points[0], new_points[1])
            max_laps = int(msg.patrol_max_laps)
            hold_s = max(0.0, float(msg.patrol_hold_time_at_endpoint_s))
            if self._queue or self._current_goal_enu is not None:
                self._pending_patrol = (patrol_points, max_laps, hold_s)
                self.get_logger().info("Patrol queued behind the active waypoint mission.")
            else:
                self._start_patrol(patrol_points, max_laps, hold_s)
            return

        if self._patrol_points is not None:
            self.get_logger().error("Patrol is active; rejecting a competing MultiGoal.")
            return

        self._queue.extend(new_points)

        self.get_logger().info(
            f"Queued {len(new_points)} waypoint(s); queue size: {len(self._queue)}"
        )

        # If nothing is currently being executed, start the first waypoint.
        if self._current_goal_enu is None:
            self._advance_to_next()

    def _on_check_tick(self):
        if self._holding_until_ns is not None:
            if self.get_clock().now().nanoseconds >= self._holding_until_ns:
                self._holding_until_ns = None
                self._advance_to_next()
        elif self._current_goal_enu is None and (self._queue or self._pending_patrol):
            self._advance_to_next()

    def _navigation_done(self, status):
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Navigation failed/canceled (status={status}); stopping the queue')
            self._queue.clear()
            self._pending_patrol = None
            self._patrol_points = None
            self._current_goal_enu = None
            self._current_waypoint = None
            self._holding_until_ns = None
            return
        if self._patrol_points is not None:
            self._on_patrol_reached()
        else:
            self._advance_to_next()

    # ----- queue management ----------------------------------------------

    def _start_patrol(
        self, points: tuple[Waypoint, Waypoint], max_laps: int, hold_s: float
    ) -> None:
        self._patrol_points = points
        self._patrol_max_laps = max_laps
        self._patrol_hold_s = hold_s
        self._patrol_reached_points = 0
        self._queue.extend(points)
        self.get_logger().info(
            f"Accepted patrol: max_laps={max_laps or 'infinite'}, endpoint_hold={hold_s:.2f}s"
        )
        self._advance_to_next()

    def _on_patrol_reached(self) -> None:
        """Advance ping-pong patrol state after the current endpoint is reached."""
        self._patrol_reached_points += 1
        if self._current_waypoint is not None:
            self._queue.append(self._current_waypoint)
        laps = max(0, (self._patrol_reached_points - 1) // 2)
        self.get_logger().info(
            f"Reached patrol endpoint {self._patrol_reached_points}; completed laps={laps}"
        )
        if self._patrol_max_laps > 0 and laps >= self._patrol_max_laps:
            self._queue.clear()
            self._patrol_points = None
            self._current_goal_enu = None
            self._current_waypoint = None
            self.get_logger().info("Patrol completed; waypoint pipeline is idle.")
            return
        if self._patrol_hold_s > 0.0:
            self._holding_until_ns = self.get_clock().now().nanoseconds + int(
                self._patrol_hold_s * 1e9)
            self.get_logger().info(f"Holding at patrol endpoint for {self._patrol_hold_s:.2f}s")
            return
        self._advance_to_next()

    def _advance_to_next(self) -> None:
        if not self._queue and self._pending_patrol is not None:
            points, max_laps, hold_s = self._pending_patrol
            self._pending_patrol = None
            self._start_patrol(points, max_laps, hold_s)
            return
        if not self._queue:
            self._current_goal_enu = None
            self._current_waypoint = None
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
        self._current_waypoint = wp
        self._current_goal_enu = (east, north, up)

        self._navigation.send(self._current_goal_enu, self._navigation_done)
        self.get_logger().info(
            f"Forwarded waypoint: lat={wp.latitude:.7f}, lon={wp.longitude:.7f}, alt={wp.altitude:.3f} "
            f"(ENU=({east:.2f}, {north:.2f}, {up:.2f}) m, remaining={len(self._queue)})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiGoalToGpGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: the context was already shut down (e.g.
        # launch sent SIGINT during shutdown) — not an error.
        pass
    finally:
        node.destroy_node()
        # A second rcl_shutdown raises RCLError and exits with code 1, which
        # launch reports as "process has died" — misleading for a clean exit.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
