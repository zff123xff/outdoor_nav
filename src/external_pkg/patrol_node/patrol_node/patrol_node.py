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

"""GPS ping-pong patrol: advance endpoints only on successful navigation Action results."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from multi_goal_to_gp_goal.coordinates import lla_to_enu
from nav_msgs.msg import Odometry
from patrol_node.action import Patrol
from patrol_node.live_tf_buffer import LiveRobotTransformBuffer
from patrol_node.waypoints import GeoPoint, PingPongQueue, nearest_endpoint
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from multi_goal_to_gp_goal.navigation_client import NavigationClient
from tf2_ros import TransformException, TransformListener


class PatrolNode(Node):
    """Walk back and forth along the line between two GPS points."""

    def __init__(self) -> None:
        super().__init__('patrol_node')

        # --- topics -------------------------------------------------------
        self.declare_parameter('action_name', 'patrol')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')


        # ENU origin used for the arrival check. Must match nav2_input_bridge,
        # otherwise the arrival check compares against a different frame.
        self.declare_parameter('reference_latitude', 0.0)
        self.declare_parameter('reference_longitude', 0.0)
        self.declare_parameter('reference_altitude', 0.0)

        # Arrival detection (same defaults as multi_goal_to_gp_goal).
        self.declare_parameter('feedback_period_s', 0.5)

        # --- io -----------------------------------------------------------
        self._navigation = NavigationClient(self)

        # odom QoS: best-effort compatible with /mavros/local_position/odom;
        # mavros_odom_relay republishes it as RELIABLE, so we still get data.
        odom_qos = QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, odom_qos)
        namespace = self.get_namespace().strip('/')
        self._global_frame = self._resolve_frame(
            self.get_parameter('global_frame').value, namespace)
        self._robot_base_frame = self._resolve_frame(
            self.get_parameter('robot_base_frame').value, namespace)
        self._tf_buffer = LiveRobotTransformBuffer(self._global_frame, self._robot_base_frame)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_odom_received_monotonic = None
        self._last_tf_warning_monotonic = 0.0

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
        self._latest_global_yaw = None
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
            self.get_parameter('feedback_period_s').value, self._on_tick)

        self.get_logger().info('Patrol completion follows navigate_to_pose Action results')

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

        if self._robot_position_in_global_frame() is None:
            self.get_logger().error(
                'Fresh map -> base_link TF is required '
                'to select the nearest patrol endpoint.')
            return GoalResponse.REJECT

        self._end_reason = None
        self._active = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """Handle a cancel request: stop the patrol loop."""
        self.get_logger().info('Patrol cancel requested.')
        self._navigation.cancel()
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
        robot_xy = self._robot_position_in_global_frame()
        lat0, lon0, alt0 = self._origin()
        endpoint_xy = [lla_to_enu(point.latitude, point.longitude, point.altitude,
                                 lat0, lon0, alt0)[:2] for point in (point_a, point_b)]
        first_leg = nearest_endpoint(robot_xy, *endpoint_xy) if robot_xy is not None else 0

        self._goal_handle = goal_handle
        self._end_future = Future()
        self._queue = PingPongQueue(point_a, point_b, start_at_b=first_leg == 1)
        self._target = None
        self._target_enu = None
        self._reached_points = 0
        self._leg = first_leg
        self._holding = False
        self._hold_until_ns = 0
        self._segment_length_m = self._distance_between(point_a, point_b)
        self._last_feedback_ns = 0

        # --- per-goal parameters ---
        self._mission_id = request.mission_id
        self._capability_id = request.capability_id or 'patrol_line'
        self._primitive_id = request.primitive_id
        self._max_laps = int(request.max_laps)
        self._hold_ns = int(max(0.0, request.hold_time_at_endpoint_s) * 1e9)
        self._keep_moving_on_end = bool(request.keep_moving_on_end)

        self._active = True
        self.get_logger().info(
            f'Starting patrol [{self._primitive_id or "-"}] '
            f'mission={self._mission_id or "-"} capability={self._capability_id} '
            f'start=({point_a.latitude:.7f}, {point_a.longitude:.7f}) '
            f'end=({point_b.latitude:.7f}, {point_b.longitude:.7f}) '
            f'segment={self._segment_length_m:.2f}m '
            f'nearest_endpoint={"unavailable" if robot_xy is None else ("B" if first_leg else "A")} '
            f'max_laps={self._max_laps or "inf"} '
            f'hold={request.hold_time_at_endpoint_s:.1f}s'
        )

        if goal_handle.is_cancel_requested:
            self._end_reason = 'canceled'
            self._resolve_end()
        elif robot_xy is None:
            self._end_reason = 'localization_unavailable'
            self._resolve_end()
        else:
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
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
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

    @staticmethod
    def _resolve_frame(frame: str, namespace: str) -> str:
        """Qualify an unprefixed TF frame with this robot's namespace."""
        frame = frame.strip('/')
        if namespace and frame != namespace and not frame.startswith(namespace + '/'):
            return f'{namespace}/{frame}'
        return frame

    def _robot_position_in_global_frame(self):
        """Return fresh robot XY in the same map frame as projected GPS goals."""
        # TF already supplies the map-frame position and its timestamp. The
        # odom subscription only supplies the final-pose report; requiring a
        # separate receipt here rejects valid localization when that relay
        # topic is absent, even though Nav2 can navigate using this same TF.
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time())
        except TransformException as exc:
            self._warn_localization(
                f'Waiting for robot TF {self._global_frame} -> '
                f'{self._robot_base_frame}: {exc}')
            return None
        age = self._tf_buffer.robot_update_age()
        robot_xy = (transform.transform.translation.x, transform.transform.translation.y)
        if age is None or age > 2.0:
            self._warn_localization(
                f'Robot TF {self._global_frame} -> {self._robot_base_frame} '
                f'has no recent timestamp advancement (update_age_s={age}, limit 2.0s). '
                'Check dynamic TF publication.')
            return None
        if not all(math.isfinite(value) for value in robot_xy):
            self._warn_localization(
                f'Robot TF {self._global_frame} -> {self._robot_base_frame} '
                f'has non-finite position: {robot_xy}')
            return None
        q = transform.transform.rotation
        self._latest_global_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return robot_xy

    def _warn_localization(self, message):
        """Explain localization failures without flooding feedback logs."""
        now = time.monotonic()
        if now - self._last_tf_warning_monotonic >= 3.0:
            self.get_logger().warning(message)
            self._last_tf_warning_monotonic = now

    def _distance_between(self, point_a: GeoPoint, point_b: GeoPoint) -> float:
        """Horizontal distance between two GPS points, in the ENU frame."""
        lat0, lon0, alt0 = self._origin()
        east_a, north_a, _ = lla_to_enu(
            point_a.latitude, point_a.longitude, point_a.altitude, lat0, lon0, alt0)
        east_b, north_b, _ = lla_to_enu(
            point_b.latitude, point_b.longitude, point_b.altitude, lat0, lon0, alt0)
        return math.hypot(east_b - east_a, north_b - north_a)

    def _laps(self) -> int:
        """Count round trips from the selected nearest endpoint (A-B-A or B-A-B)."""
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
        """Send one correlated navigation Action for the next GPS endpoint."""
        if len(self._queue) == 0:
            self._target = None
            self._target_enu = None
            return

        robot_xy = self._robot_position_in_global_frame()
        if robot_xy is None:
            self._end_reason = 'localization_unavailable'
            self._resolve_end()
            return

        point = self._queue.take_next()
        lat0, lon0, alt0 = self._origin()
        east, north, up = lla_to_enu(
            point.latitude, point.longitude, point.altitude, lat0, lon0, alt0)

        self._target = point
        self._target_enu = (east, north, up)

        dx, dy = east - robot_xy[0], north - robot_xy[1]
        heading = (math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6
                   else self._latest_global_yaw)
        self._navigation.send(self._target_enu, self._navigation_done, heading=heading)
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
        # Keep odom for handover feedback; arrival uses TF in the Nav2 map frame.
        self._latest_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose_stamped = pose
        self._last_odom_received_monotonic = time.monotonic()

    def _navigation_done(self, status):
        if not self._active or self._end_reason is not None:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_reached()
        else:
            self._end_reason = ('navigation_canceled' if status == GoalStatus.STATUS_CANCELED else 'navigation_failed')
            self._resolve_end()

    def _on_tick(self):
        if not self._active:
            return
        if self._holding and self.get_clock().now().nanoseconds >= self._hold_until_ns:
            self._holding = False
            self._dispatch_next()
        self._publish_feedback(self._distance_to_target())

    def _distance_to_target(self) -> float:
        """Horizontal distance between the robot and the current target."""
        if self._target_enu is None:
            return float('inf')
        robot_xy = self._robot_position_in_global_frame()
        if robot_xy is None:
            return float('inf')
        goal_east, goal_north, _ = self._target_enu
        pose_east, pose_north = robot_xy
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
        if self._end_reason == 'completed':
            feedback.phase = 'completed'
        elif self._holding:
            feedback.phase = 'holding'
        else:
            feedback.phase = 'to_start' if self._leg == 0 else 'to_end'
        feedback.primitive_id = self._primitive_id
        feedback.distance_to_target_m = distance
        feedback.segment_length_m = self._segment_length_m
        if self._goal_handle is not None:
            self._goal_handle.publish_feedback(feedback)

    def _stop_robot(self):
        self._navigation.cancel()


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
