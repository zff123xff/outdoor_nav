"""Execute one GPS point through the navigation Action result."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from multi_goal_to_gp_goal.navigation_client import NavigationClient
from nav_msgs.msg import Odometry
from patrol_node.action import PointGoal
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from multi_goal_to_gp_goal.coordinates import lla_to_enu


class PointGoalNode(Node):
    """Execute one GPS goal, reporting arrival through Action feedback/result."""

    def __init__(self):
        super().__init__('point_goal_executor')
        self.declare_parameter('action_name', 'point_goal')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('reference_latitude', 0.0)
        self.declare_parameter('reference_longitude', 0.0)
        self.declare_parameter('reference_altitude', 0.0)
        self.declare_parameter('feedback_period_s', 0.5)

        self._navigation = NavigationClient(self)
        qos = QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, qos)

        namespace = self.get_namespace().strip('/')
        self._global_frame = self._resolve_frame(
            self.get_parameter('global_frame').value, namespace)
        self._base_frame = self._resolve_frame(
            self.get_parameter('robot_base_frame').value, namespace)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._active = False
        self._reason = None
        self._done_future = None
        self._goal_handle = None
        self._target_enu = None
        self._target = None
        self._latest_pose = None
        self._last_odom_monotonic = None
        self._last_feedback_ns = 0
        self._primitive_id = ''
        self._last_tf_warning = 0.0
        self._progress_start_xy = None
        self._progress_total_m = None

        self._action_server = ActionServer(
            self, PointGoal, self.get_parameter('action_name').value,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel)
        self._timer = self.create_timer(self.get_parameter('feedback_period_s').value, self._tick)
        self.get_logger().info(
            f'point_goal_executor: action={self.get_parameter("action_name").value} '
            f'navigation_action={self.get_parameter("navigation_action").value}')

    @staticmethod
    def _resolve_frame(frame, namespace):
        frame = frame.strip('/')
        if namespace and frame != namespace and not frame.startswith(namespace + '/'):
            return f'{namespace}/{frame}'
        return frame

    def _origin(self):
        return (self.get_parameter('reference_latitude').value,
                self.get_parameter('reference_longitude').value,
                self.get_parameter('reference_altitude').value)

    def _origin_is_set(self):
        lat, lon, _ = self._origin()
        return abs(lat) >= 1e-9 or abs(lon) >= 1e-9

    def _on_goal(self, request):
        if self._active:
            self.get_logger().warning('A point goal is already active; rejecting another goal.')
            return GoalResponse.REJECT
        p = request.target
        if (not math.isfinite(p.latitude) or not math.isfinite(p.longitude) or
                not math.isfinite(p.altitude) or not -90.0 <= p.latitude <= 90.0 or
                not -180.0 <= p.longitude <= 180.0):
            self.get_logger().error('Rejecting invalid GPS target.')
            return GoalResponse.REJECT
        if not self._origin_is_set():
            self.get_logger().error('reference_latitude/longitude must be configured.')
            return GoalResponse.REJECT
        self._active = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self._navigation.cancel()
        self._reason = 'canceled'
        self._resolve()
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        request = goal_handle.request
        p = request.target
        self._goal_handle = goal_handle
        self._primitive_id = request.primitive_id
        lat0, lon0, alt0 = self._origin()
        self._target_enu = lla_to_enu(p.latitude, p.longitude, p.altitude, lat0, lon0, alt0)
        self._target = (p.latitude, p.longitude, p.altitude)
        self._progress_start_xy = None
        self._progress_total_m = None
        self._enu_progress()  # Capture the fresh ENU start before dispatch.
        self._last_feedback_ns = 0
        self._reason = None
        self._done_future = Future()
        self._active = True

        if goal_handle.is_cancel_requested:
            self._reason = 'canceled'
            self._resolve()
        else:
            self._navigation.send(self._target_enu, self._navigation_done)
        self.get_logger().info(
            f'Accepted point goal {request.primitive_id}: '
            f'({p.latitude:.7f}, {p.longitude:.7f}, {p.altitude:.2f})')

        try:
            await self._done_future
        finally:
            self._active = False
            if self._reason == 'canceled':
                self._navigation.cancel()

        result = PointGoal.Result()
        result.success = self._reason == 'reached'
        result.message = 'target reached' if result.success else f'point goal {self._reason or "stopped"}'
        result.mission_id = request.mission_id
        result.capability_id = request.capability_id
        result.primitive_id = request.primitive_id
        result.final_pose = self._latest_pose or PoseStamped()
        if result.success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self._goal_handle = None
        return result

    def _resolve(self):
        future = self._done_future
        if future is not None and not future.done():
            future.set_result(None)

    def _on_odom(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose
        self._last_odom_monotonic = time.monotonic()

    def _robot_xy(self):
        if (self._last_odom_monotonic is None or
                time.monotonic() - self._last_odom_monotonic > 2.0):
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time())
        except TransformException as exc:
            now = time.monotonic()
            if now - self._last_tf_warning >= 3.0:
                self.get_logger().warning(f'Waiting for robot TF: {exc}')
                self._last_tf_warning = now
            return None
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        xy = transform.transform.translation.x, transform.transform.translation.y
        if age < -.5 or age > 2.0 or not all(math.isfinite(v) for v in xy):
            return None
        return xy

    def _navigation_done(self, status):
        if not self._active or self._reason == 'canceled':
            return
        self._reason = ('reached' if status == GoalStatus.STATUS_SUCCEEDED else
                        'navigation_canceled' if status == GoalStatus.STATUS_CANCELED else 'navigation_failed')
        self._publish_feedback(self._distance(), 'completed' if self._reason == 'reached' else 'failed',
                               self._reason == 'reached', force=True)
        self._resolve()

    def _tick(self):
        if self._active:
            self._publish_feedback(self._distance(), 'navigating', False)

    def _distance(self):
        if self._target_enu is None:
            return float('inf')
        xy = self._robot_xy()
        if xy is None:
            return float('inf')
        return math.hypot(xy[0] - self._target_enu[0], xy[1] - self._target_enu[1])

    def _enu_progress(self):
        if self._target_enu is None:
            return -1.0
        xy = self._robot_xy()
        if xy is None:
            return -1.0
        if self._progress_start_xy is None:
            self._progress_start_xy = xy
            self._progress_total_m = math.dist(xy, self._target_enu[:2])
        if self._progress_total_m <= 1e-6:
            return 0.0
        return min(99.0, 100.0 * math.dist(self._progress_start_xy, xy) / self._progress_total_m)

    def _publish_feedback(self, distance, phase, reached, force=False):
        if self._goal_handle is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.get_parameter('feedback_period_s').value * 1e9)
        if not force and now_ns - self._last_feedback_ns < period_ns:
            return
        self._last_feedback_ns = now_ns
        feedback = PointGoal.Feedback()
        feedback.phase = phase
        feedback.distance_to_target_m = distance
        feedback.reached = reached
        feedback.primitive_id = self._primitive_id
        feedback.progress = self._enu_progress()
        self._goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = PointGoalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._resolve()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
