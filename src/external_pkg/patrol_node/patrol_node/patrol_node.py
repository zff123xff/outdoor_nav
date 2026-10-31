#!/usr/bin/env python3

# Copyright 2026 outdoor_nav contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Ping-pong patrol along the line segment between two GPS points.

The node exposes a ROS 2 action (``patrol_node/action/Patrol``) whose goal
carries the two GPS endpoints of the patrol line.  On acceptance the node
fills a waypoint queue with ``[A, B]`` and dispatches the head of that queue
to ``gp_goal`` (sensor_msgs/NavSatFix), which the existing

    nav2_input_bridge -> straight_planner -> controller_server

pipeline turns into a nav2 goal.

Arrival is detected from the odom position the same way
``multi_goal_to_gp_goal`` does it: the odom position is compared against the
ENU projection of the current target and the target counts as reached after
``consecutive_required`` consecutive samples within ``reach_threshold_m``.
Whenever a point is reached it is pushed back onto the tail of the queue, so
the queue always holds exactly one pending target and the dispatched targets
alternate ``A, B, A, B ...`` -- the robot walks the segment back and forth
until the action goal is canceled or ``max_laps`` round trips are done.

The projection (``lla_to_enu``) and the arrival-detection parameters are
shared with ``multi_goal_to_gp_goal`` so that the ENU frame used for the
arrival check is exactly the one ``nav2_input_bridge`` publishes the goal in.

This is a *primitive*: the goal carries the identity of the task
(``mission_id`` / ``agent_id`` / ``capability_id`` / ``primitive_id``) and the
handover state (``current_pose``), and the result reports where the robot
ended up (``final_pose``), so a task dispatcher can compose it with other
primitives and hand the robot over without a hard stop.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from multi_goal_to_gp_goal.multi_goal_to_gp_goal import lla_to_enu
from nav_msgs.msg import Odometry
from patrol_node.action import Patrol
from patrol_node.waypoints import GeoPoint, PingPongQueue
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


class PatrolNode(Node):
    """Walk back and forth along the line between two GPS points."""

    def __init__(self) -> None:
        super().__init__('patrol_node')

        # --- topics -------------------------------------------------------
        self.declare_parameter('action_name', 'patrol')
        self.declare_parameter('goal_output_topic', 'gp_goal')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('stop_topic', 'stop_planner')
        self.declare_parameter('frame_id', 'gps')

        # NavSatStatus values published with every goal fix.
        self.declare_parameter('navsat_status', 1)   # NavSatStatus.STATUS_FIX
        self.declare_parameter('navsat_service', 1)  # NavSatStatus.SERVICE_GPS

        # ENU origin used for the arrival check. Must match nav2_input_bridge,
        # otherwise the arrival check compares against a different frame.
        self.declare_parameter('reference_latitude', 0.0)
        self.declare_parameter('reference_longitude', 0.0)
        self.declare_parameter('reference_altitude', 0.0)

        # Arrival detection (same defaults as multi_goal_to_gp_goal).
        self.declare_parameter('reach_threshold_m', 0.5)
        self.declare_parameter('consecutive_required', 5)
        self.declare_parameter('check_period_s', 0.1)
        self.declare_parameter('feedback_period_s', 0.5)

        # --- io -----------------------------------------------------------
        self._gp_pub = self.create_publisher(
            NavSatFix, self.get_parameter('goal_output_topic').value, 10)
        self._stop_pub = self.create_publisher(
            Bool, self.get_parameter('stop_topic').value, 10)

        # odom QoS: best-effort compatible with /mavros/local_position/odom;
        # mavros_odom_relay republishes it as RELIABLE, so we still get data.
        odom_qos = QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, odom_qos)

        # --- action -------------------------------------------------------
        self._action_server = ActionServer(
            self,
            Patrol,
            self.get_parameter('action_name').value,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

        # --- state --------------------------------------------------------
        self._queue = PingPongQueue(
            GeoPoint(0.0, 0.0, 0.0), GeoPoint(0.0, 0.0, 0.0))
        self._active = False
        self._end_reason = None
        self._end_future = None
        self._goal_handle = None
        self._target = None
        self._target_enu = None
        self._latest_pose = None      # (east, north) in the odom frame
        self._latest_pose_stamped = None
        self._consecutive_within = 0
        self._reached_points = 0
        self._segment_length_m = 0.0
        self._leg = 0                 # 0: heading to start_point, 1: to end_point
        self._last_feedback_ns = 0

        # per-goal parameters
        self._mission_id = ''
        self._capability_id = ''
        self._primitive_id = ''
        self._max_laps = 0
        self._hold_ns = 0
        self._hold_until_ns = 0
        self._holding = False
        self._keep_moving_on_end = False

        self._timer = self.create_timer(
            self.get_parameter('check_period_s').value, self._on_tick)

        self.get_logger().info(
            f'patrol_node: action={self.get_parameter("action_name").value} '
            f'gp_goal={self._gp_pub.topic_name} '
            f'odom={self._odom_sub.topic_name} '
            f'(reach<{self.get_parameter("reach_threshold_m").value:.2f}m '
            f'x{self.get_parameter("consecutive_required").value})'
        )

    # ----- action callbacks -------------------------------------------------

    def _on_goal(self, goal_request):
        """Accept a new patrol goal, unless one is already running."""
        if self._active:
            self.get_logger().warn(
                'A patrol is already running; rejecting the new goal.')
            return GoalResponse.REJECT

        if not self._origin_is_set():
            self.get_logger().error(
                'reference_latitude / reference_longitude are not configured; '
                'cannot run a patrol. Set them in the nav2 params yaml.')
            return GoalResponse.REJECT

        self._end_reason = None
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """Handle a cancel request: stop the patrol loop."""
        self.get_logger().info('Patrol cancel requested.')
        self._end_reason = 'canceled'
        self._resolve_end()
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        """
        Run the patrol until the goal is canceled or max_laps is reached.

        This callback has to be a coroutine: rclpy runs it on the executor,
        and a blocking callback would starve the odom/timer callbacks the
        arrival detection depends on.
        """
        request = goal_handle.request
        point_a = GeoPoint(request.start_point.latitude,
                           request.start_point.longitude,
                           request.start_point.altitude)
        point_b = GeoPoint(request.end_point.latitude,
                           request.end_point.longitude,
                           request.end_point.altitude)

        self._goal_handle = goal_handle
        self._end_future = Future()
        self._queue = PingPongQueue(point_a, point_b)
        self._target = None
        self._target_enu = None
        self._consecutive_within = 0
        self._reached_points = 0
        self._leg = 0
        self._holding = False
        self._hold_until_ns = 0
        self._segment_length_m = self._distance_between(point_a, point_b)

        # --- per-goal parameters ---
        self._mission_id = request.mission_id
        self._capability_id = request.capability_id or 'patrol_line'
        self._primitive_id = request.primitive_id
        self._max_laps = int(request.max_laps)
        self._hold_ns = int(max(0.0, request.hold_time_at_endpoint_s) * 1e9)
        self._keep_moving_on_end = bool(request.keep_moving_on_end)

        # Seed the pose from the handover pose so the arrival check works even
        # before the first odom message arrives.
        if self._latest_pose is None and request.current_pose is not None:
            self._latest_pose = (request.current_pose.pose.position.x,
                                 request.current_pose.pose.position.y)

        self._active = True
        self.get_logger().info(
            f'Starting patrol [{self._primitive_id or "-"}] '
            f'mission={self._mission_id or "-"} capability={self._capability_id} '
            f'start=({point_a.latitude:.7f}, {point_a.longitude:.7f}) '
            f'end=({point_b.latitude:.7f}, {point_b.longitude:.7f}) '
            f'segment={self._segment_length_m:.2f}m '
            f'max_laps={self._max_laps or "inf"} '
            f'hold={request.hold_time_at_endpoint_s:.1f}s'
        )

        self._dispatch_next()

        try:
            await self._end_future
        finally:
            self._active = False
            self._stop_robot()

        result = Patrol.Result()
        result.mission_id = self._mission_id
        result.capability_id = self._capability_id
        result.primitive_id = self._primitive_id
        result.reached_points = self._reached_points
        result.laps = self._laps()
        result.final_pose = self._final_pose()

        if self._end_reason == 'completed':
            result.success = True
            result.message = (
                f'patrol completed: {self._reached_points} arrival(s), '
                f'{result.laps} lap(s)')
            goal_handle.succeed()
        else:
            result.success = False
            result.message = (
                f'patrol {self._end_reason or "stopped"}: '
                f'{self._reached_points} arrival(s), {result.laps} lap(s)')
        self.get_logger().info(result.message)
        return result

    # ----- helpers ---------------------------------------------------------

    def _resolve_end(self):
        """Wake up the execute coroutine waiting on the end future."""
        future = self._end_future
        if future is not None and not future.done():
            future.set_result(None)

    def _origin_is_set(self):
        lat = self.get_parameter('reference_latitude').value
        lon = self.get_parameter('reference_longitude').value
        return not (abs(lat) < 1e-9 and abs(lon) < 1e-9)

    def _origin(self):
        return (
            self.get_parameter('reference_latitude').value,
            self.get_parameter('reference_longitude').value,
            self.get_parameter('reference_altitude').value,
        )

    def _distance_between(self, point_a: GeoPoint, point_b: GeoPoint) -> float:
        """Horizontal distance between two GPS points, in the ENU frame."""
        lat0, lon0, alt0 = self._origin()
        east_a, north_a, _ = lla_to_enu(
            point_a.latitude, point_a.longitude, point_a.altitude, lat0, lon0, alt0)
        east_b, north_b, _ = lla_to_enu(
            point_b.latitude, point_b.longitude, point_b.altitude, lat0, lon0, alt0)
        return math.hypot(east_b - east_a, north_b - north_a)

    def _laps(self) -> int:
        """Return the number of completed round trips (A->B->A is one lap)."""
        return max(0, (self._reached_points - 1) // 2)

    def _final_pose(self) -> PoseStamped:
        """Where the robot ended up, for the next primitive to pick up."""
        if self._latest_pose_stamped is not None:
            return self._latest_pose_stamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        return pose

    # ----- queue -----------------------------------------------------------

    def _dispatch_next(self):
        """Publish the next target of the queue to /gp_goal."""
        if len(self._queue) == 0:
            self._target = None
            self._target_enu = None
            return

        point = self._queue.take_next()
        lat0, lon0, alt0 = self._origin()
        east, north, up = lla_to_enu(
            point.latitude, point.longitude, point.altitude, lat0, lon0, alt0)

        self._target = point
        self._target_enu = (east, north, up)
        self._consecutive_within = 0

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.get_parameter('frame_id').value
        fix.latitude = point.latitude
        fix.longitude = point.longitude
        fix.altitude = point.altitude
        fix.status.status = self.get_parameter('navsat_status').value
        fix.status.service = self.get_parameter('navsat_service').value

        self._gp_pub.publish(fix)
        self.get_logger().info(
            f"Patrol target {'A' if self._leg == 0 else 'B'}: "
            f'lat={point.latitude:.7f}, lon={point.longitude:.7f}, '
            f'ENU=({east:.2f}, {north:.2f})m'
        )

    def _on_reached(self):
        """Handle the arrival at the current target."""
        reached = self._target
        self._reached_points += 1
        # Ping-pong: push the point we just reached back onto the tail, so the
        # next target is the other end of the segment.
        self._queue.on_reached(reached)
        self._leg = 1 - self._leg
        self._consecutive_within = 0
        self.get_logger().info(
            f'Reached target ({self._reached_points} arrival(s), '
            f'{self._laps()} lap(s)); heading to the other end.')

        # max_laps reached -> the primitive is done, hand over to the next one.
        if 0 < self._max_laps <= self._laps():
            self._end_reason = 'completed'
            self._resolve_end()
            return

        if self._hold_ns > 0:
            # Dwell at the endpoint so the next primitive can take over
            # smoothly.
            self._holding = True
            self._hold_until_ns = \
                self.get_clock().now().nanoseconds + self._hold_ns
            return

        self._dispatch_next()

    # ----- callbacks -------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        # odom is in the ENU (local) frame
        self._latest_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose_stamped = pose

    def _on_tick(self):
        """Arrival detection, dispatched at ``check_period_s``."""
        if not self._active:
            return

        now_ns = self.get_clock().now().nanoseconds

        if self._holding:
            # Dwell at the endpoint: no arrival check until the hold expires.
            if now_ns >= self._hold_until_ns:
                self._holding = False
                self._dispatch_next()
            self._publish_feedback(self._distance_to_target())
            return

        if self._target_enu is None or self._latest_pose is None:
            return

        goal_east, goal_north, _ = self._target_enu
        pose_east, pose_north = self._latest_pose
        distance = math.hypot(pose_east - goal_east, pose_north - goal_north)

        threshold = self.get_parameter('reach_threshold_m').value
        required = self.get_parameter('consecutive_required').value

        if distance <= threshold:
            self._consecutive_within += 1
        else:
            self._consecutive_within = 0

        if self._consecutive_within >= required:
            self._on_reached()
            distance = self._distance_to_target()

        self._publish_feedback(distance)

    def _distance_to_target(self) -> float:
        """Horizontal distance between the robot and the current target."""
        if self._target_enu is None or self._latest_pose is None:
            return 0.0
        goal_east, goal_north, _ = self._target_enu
        pose_east, pose_north = self._latest_pose
        return math.hypot(pose_east - goal_east, pose_north - goal_north)

    def _publish_feedback(self, distance: float) -> None:
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.get_parameter('feedback_period_s').value * 1e9)
        if now_ns - self._last_feedback_ns < period_ns:
            return
        self._last_feedback_ns = now_ns

        feedback = Patrol.Feedback()
        feedback.reached_points = self._reached_points
        feedback.laps = self._laps()
        feedback.phase = 'to_start' if self._leg == 0 else 'to_end'
        feedback.primitive_id = self._primitive_id
        feedback.distance_to_target_m = distance
        feedback.segment_length_m = self._segment_length_m
        if self._goal_handle is not None:
            self._goal_handle.publish_feedback(feedback)

    def _stop_robot(self):
        """Stop the patrol and, if configured, stop the robot as well."""
        self._queue.clear()
        self._target = None
        self._target_enu = None
        self._consecutive_within = 0
        self._holding = False

        if self._keep_moving_on_end:
            self.get_logger().info(
                'Patrol ended; keep_moving_on_end is set, so the robot keeps '
                'its last navigation goal for the next primitive to take over.')
            return

        stop = Bool()
        stop.data = True
        self._stop_pub.publish(stop)
        self.get_logger().info(
            f'Patrol ended; requested an immediate stop on '
            f'{self._stop_pub.topic_name}.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: the context was already shut down (e.g.
        # launch sent SIGINT during shutdown) — not an error.
        pass
    finally:
        node._resolve_end()
        node.destroy_node()
        # A second rcl_shutdown raises RCLError and exits with code 1, which
        # launch reports as "process has died" — misleading for a clean exit.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
