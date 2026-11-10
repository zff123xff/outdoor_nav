"""Simulation target inspection with exclusive FollowPath ownership."""
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.time import Time
from task_interfaces.action import ApproachTarget
from task_interfaces.msg import InspectionArrival

from .targets import matches


def approach_pose(robot_xy, target_xy, standoff, height_difference=0.):
    """Stop before the observed object; retain position when already close enough."""
    dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        raise ValueError('Target direction is undefined at the robot position')
    if abs(height_difference) >= standoff:
        raise ValueError('Target height is outside the ground robot stand-off distance')
    horizontal_standoff = math.sqrt(standoff * standoff - height_difference * height_difference)
    travel = max(0., distance - horizontal_standoff)
    return (robot_xy[0] + travel * dx / distance,
            robot_xy[1] + travel * dy / distance, math.atan2(dy, dx))


def angle_error(desired, actual):
    return math.atan2(math.sin(desired - actual), math.cos(desired - actual))


class TargetApproach:
    """Lives in simulation_target_marker; generates and executes its navigation goal."""

    def __init__(self, node):
        self.node = node
        defaults = dict(approach_standoff_m=4.5, approach_max_distance_m=5.,
                        approach_yaw_tolerance_rad=.15, approach_stop_speed_mps=.05,
                        approach_stop_yaw_rate_rps=.05, approach_stop_hold_s=.5,
                        approach_timeout_s=120.)
        self.settings = {key: node.declare_parameter(key, value).value for key, value in defaults.items()}
        if any(not math.isfinite(value) or value <= 0 for value in self.settings.values()):
            raise ValueError('Target approach settings must be positive and finite')
        if self.settings['approach_standoff_m'] + .25 >= self.settings['approach_max_distance_m']:
            raise ValueError('Stand-off must leave more than 0.25m for navigation tolerance')
        self.client = ActionClient(node, FollowPath, 'follow_path')
        self.arrivals = node.create_publisher(InspectionArrival, 'targets/inspection_arrived', 10)
        self.server = ActionServer(node, ApproachTarget, 'targets/approach', self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.busy = False
        self.phase = 'idle'
        self.pending = None
        self.handle = None
        self.end = None
        self.outcome = None
        self.goal = None
        self.previous_pose = None
        self.stationary_since = None
        self.distance = math.inf
        self.heading_error = math.inf
        self.timer = node.create_timer(.1, self.tick)

    def target(self, target_id):
        track = self.node.tracks.items.get(target_id)
        now = self.node.get_clock().now().nanoseconds / 1e9
        if (track is None or now - track['stamp'] > self.node.settings['sensor_timeout_s'] or
                target_id in self.node.marked or not matches(self.node.targets, track['kind'],
                                                            self.node.class_ids.get(target_id)) or
                not all(math.isfinite(float(v)) for v in track['point'])):
            return None
        return track['point']

    def accept(self, request):
        if (self.busy or not self.node.inspection_enabled or not self.node.active or not request.task_id or
                self.target(request.target_id) is None):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.stop(False, 'Target inspection canceled')
        return CancelResponse.ACCEPT

    async def execute(self, goal):
        self.goal = goal
        self.end = Future()
        self.outcome = None
        self.phase = 'waiting_stop'
        self.started = time.monotonic()
        self.previous_pose = None
        self.stationary_since = None
        self.distance = self.heading_error = math.inf
        if goal.is_cancel_requested:
            self.stop(False, 'Target inspection canceled before execution')
        success, message = await self.end
        result = ApproachTarget.Result(success=success, message=message,
            target_id=goal.request.target_id, distance_m=self.distance)
        if goal.is_cancel_requested:
            goal.canceled()
        elif success:
            goal.succeed()
        else:
            goal.abort()
        self.busy = False
        self.phase = 'idle'
        self.goal = None
        return result

    def robot_pose(self):
        transform = self.node.tf.lookup_transform(self.node.frame, self.node.base_frame, Time())
        age = self.node.tf.robot_update_age()
        if age is None or age > self.node.settings['sensor_timeout_s']:
            raise ValueError('Robot TF has stopped updating')
        p, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if not all(math.isfinite(value) for value in (p.x, p.y, p.z, yaw)):
            raise ValueError('Nonfinite robot pose')
        return p.x, p.y, yaw, Time.from_msg(transform.header.stamp).nanoseconds, p.z

    def sample_stopped(self, pose, now):
        if self.previous_pose is not None and pose[3] != self.previous_pose[0][3]:
            old, since = self.previous_pose
            elapsed = now - since
            speed = math.hypot(pose[0] - old[0], pose[1] - old[1]) / max(elapsed, 1e-6)
            yaw_rate = abs(angle_error(pose[2], old[2])) / max(elapsed, 1e-6)
            if (speed <= self.settings['approach_stop_speed_mps'] and
                    yaw_rate <= self.settings['approach_stop_yaw_rate_rps']):
                if self.stationary_since is None:
                    self.stationary_since = now
            else:
                self.stationary_since = None
            self.previous_pose = (pose, now)
        elif self.previous_pose is None:
            self.previous_pose = (pose, now)
        return (self.stationary_since is not None and
                now - self.stationary_since >= self.settings['approach_stop_hold_s'] and
                self.node.tf.robot_update_age() <= .5)

    def suspended(self):
        status = self.node.search_status
        return (status.get('state') == 'suspended' and
                status.get('mission_id') == self.goal.request.task_id)

    def tick(self):
        if not self.busy or self.goal is None or self.end is None or self.end.done():
            return
        if self.outcome is not None:
            self.finish_if_drained()
            return
        now = time.monotonic()
        if not self.suspended() or not self.node.active:
            if self.phase == 'waiting_stop' and now - self.started < 5.:
                return  # Allow the latched suspension status to reach this process.
            self.stop(False, 'Search suspension/ownership was lost')
            return
        if self.phase != 'awaiting_confirmation' and now - self.started > self.settings['approach_timeout_s']:
            self.stop(False, 'Target approach timed out')
            return
        try:
            pose = self.robot_pose()
            stopped = self.sample_stopped(pose, now)
            target = self.target(self.goal.request.target_id)
            if target is None:
                self.distance = self.heading_error = math.inf
            else:
                dx, dy = float(target[0]) - pose[0], float(target[1]) - pose[1]
                self.distance = math.sqrt(dx * dx + dy * dy + (float(target[2]) - pose[4]) ** 2)
                self.heading_error = angle_error(math.atan2(dy, dx), pose[2])
            if self.phase == 'waiting_stop' and stopped and target is not None and self.client.server_is_ready():
                destination = approach_pose(pose, target, self.settings['approach_standoff_m'],
                                            float(target[2]) - pose[4])
                self.send_path(pose, destination)
            elif self.phase == 'settling' and stopped and target is not None:
                if (self.distance > self.settings['approach_max_distance_m'] or
                        abs(self.heading_error) > self.settings['approach_yaw_tolerance_rad']):
                    self.stop(False, 'Navigation ended outside target distance/heading tolerance')
                else:
                    self.phase = 'awaiting_confirmation'
            elif (self.phase == 'awaiting_confirmation' and
                  self.goal.request.task_id == '143' and stopped and target is not None):
                self.arrivals.publish(InspectionArrival(
                    task_id='143', target_id=self.goal.request.target_id,
                    target_position=Point(x=float(target[0]), y=float(target[1]), z=float(target[2])),
                    robot_position=Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[4])),
                    distance_m=float(self.distance), heading_error_rad=float(self.heading_error)))
                self.stop(True, 'Simulation observation arrival reported')
        except Exception as exc:
            self.stationary_since = None
            if self.phase in ('approaching', 'settling'):
                self.stop(False, 'Robot localization lost during target approach: '+str(exc))
            if self.phase == 'awaiting_confirmation':
                self.distance = self.heading_error = math.inf
            self.node.get_logger().warning(f'Target approach waiting for pose: {exc}', throttle_duration_sec=3.)
        self.goal.publish_feedback(ApproachTarget.Feedback(
            phase=self.phase, distance_m=self.distance, heading_error_rad=self.heading_error))

    def send_path(self, start, destination):
        path = Path()
        path.header.frame_id = self.node.frame
        path.header.stamp = self.node.get_clock().now().to_msg()
        length = math.hypot(destination[0] - start[0], destination[1] - start[1])
        count = max(1, math.ceil(length / .1))
        for index in range(count + 1):
            fraction = index / count
            pose = PoseStamped(header=path.header)
            pose.pose.position.x = start[0] + fraction * (destination[0] - start[0])
            pose.pose.position.y = start[1] + fraction * (destination[1] - start[1])
            pose.pose.orientation.z = math.sin(destination[2] / 2)
            pose.pose.orientation.w = math.cos(destination[2] / 2)
            path.poses.append(pose)
        self.phase = 'approaching'
        self.stationary_since = None
        self.node.get_logger().info(
            f'Target {self.goal.request.target_id}: approach goal x={destination[0]:.3f}, '
            f'y={destination[1]:.3f}, yaw={destination[2]:.3f}')
        self.pending = self.client.send_goal_async(FollowPath.Goal(
            path=path, controller_id='FollowTarget', goal_checker_id='target_goal_checker'))
        self.pending.add_done_callback(self.accepted)

    def accepted(self, future):
        self.pending = None
        try:
            handle = future.result()
            if not handle.accepted:
                self.stop(False, 'Target FollowPath was rejected')
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(self.result)
            if self.outcome is not None:
                handle.cancel_goal_async()
        except Exception as exc:
            self.stop(False, 'Target FollowPath acceptance failed: '+str(exc))
        self.finish_if_drained()

    def result(self, future):
        self.handle = None
        if self.outcome is None:
            try:
                status = future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.phase = 'settling'
                    self.stationary_since = None
                else:
                    self.stop(False, f'Target FollowPath ended with status={status}')
            except Exception as exc:
                self.stop(False, str(exc))
        self.finish_if_drained()

    def confirm(self, target_id):
        if (not self.busy or self.goal is None or self.phase != 'awaiting_confirmation' or
                target_id != self.goal.request.target_id or not self.suspended()):
            return False
        try:
            target = self.target(target_id)
            pose = self.robot_pose()
            if target is None or not self.sample_stopped(pose, time.monotonic()):
                return False
            dx, dy = float(target[0]) - pose[0], float(target[1]) - pose[1]
            if (math.sqrt(dx * dx + dy * dy + (float(target[2]) - pose[4]) ** 2) > self.settings['approach_max_distance_m'] or
                    abs(angle_error(math.atan2(dy, dx), pose[2])) > self.settings['approach_yaw_tolerance_rad']):
                return False
        except Exception:
            return False
        self.stop(True, 'Simulation confirmation received; target marked')
        return True

    def stop(self, success, message):
        if self.outcome is None:
            self.outcome = (success, message)
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.finish_if_drained()

    def finish_if_drained(self):
        if (self.outcome is not None and self.pending is None and self.handle is None and
                self.end is not None and not self.end.done()):
            self.end.set_result(self.outcome)
