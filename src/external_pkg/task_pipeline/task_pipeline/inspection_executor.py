"""Independent observation-only target visits, with drained navigation ownership."""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, FollowPath
from task_interfaces.action import InspectTargets
from task_interfaces.msg import TargetArray, InspectionArrival
from tf2_ros import TransformListener, TransformException
from patrol_node.live_tf_buffer import LiveRobotTransformBuffer
from task_pipeline.inspection_logic import angle_error, nearest, validate_batch, visit_pose


class InspectionExecutor(Node):
    def __init__(self):
        super().__init__('target_inspection_executor')
        defaults = dict(global_frame='', base_frame='', stand_off_m=9., max_range_m=10.,
                        heading_tolerance_rad=.15, settle_s=.5, dwell_s=2.5,
                        linear_stop_mps=.05, angular_stop_rps=.05, pose_max_age_s=.5,
                        target_timeout_s=120., assignment_timeout_s=10.,
                        action_name='target_inspection', assigned_topic='targets/assigned',
                        require_fresh_assignment=False)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.cfg.items():
            if key not in ('global_frame', 'base_frame', 'action_name', 'assigned_topic', 'require_fresh_assignment') and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        for key in ('action_name', 'assigned_topic'):
            if not self.cfg[key] or self.cfg[key].startswith(('/', '~')):
                raise ValueError(f'{key} must be a relative ROS name')
        if not 0 < self.cfg['stand_off_m'] < self.cfg['max_range_m'] <= 10.:
            raise ValueError('stand_off_m < max_range_m <= 10 required')
        if not 2. <= self.cfg['dwell_s'] <= 3.:
            raise ValueError('dwell_s must be between 2 and 3 seconds')
        prefix = self.get_namespace().strip('/')
        self.frame = self.cfg['global_frame'] or '/'.join(filter(None, (prefix, 'map')))
        self.base = self.cfg['base_frame'] or '/'.join(filter(None, (prefix, 'base_link')))
        self.tf = LiveRobotTransformBuffer(self.frame, self.base)
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.align = ActionClient(self, FollowPath, 'follow_path')
        self.arrivals = self.create_publisher(InspectionArrival, 'targets/inspection_arrived', 10)
        self.create_subscription(TargetArray, self.cfg['assigned_topic'], self.assign, 10)
        self.latest = None
        self.busy = False
        self.active = None
        self.child = None
        self.inflight = False
        self.canceling = False
        self.server = ActionServer(self, InspectTargets, self.cfg['action_name'], self.execute,
                                   goal_callback=self.goal, cancel_callback=self.cancel)
        self.create_timer(.05, self.tick)

    def assign(self, message):
        try:
            self.latest = validate_batch(message)
        except ValueError as exc:
            self.get_logger().warning(f'Assignment batch rejected: {exc}')

    def goal(self, request):
        if self.busy or request.task_id != '143':
            return GoalResponse.REJECT
        self.busy = True
        if self.cfg['require_fresh_assignment']:
            self.latest = None  # Private search visitor accepts only post-goal assignments.
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.canceling = True
        if self.child is not None:
            self.child.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        self.active = handle
        self.canceling = handle.is_cancel_requested
        self.pending = dict(self.latest) if self.latest is not None else None
        self.total = len(self.pending or {})
        self.inspected = self.failed = 0
        self.target_id = 0
        self.phase = 'assignment'
        self.deadline = time.monotonic() + self.cfg['assignment_timeout_s']
        self.waiter = Future()
        try:
            await self.waiter
            success = not self.canceling and self.failed == 0 and self.inspected > 0
            if self.canceling:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
            return InspectTargets.Result(success=success, inspected_count=self.inspected,
                failed_count=self.failed, message=f'inspection: {self.inspected} arrived, {self.failed} failed'
                + ('; canceled' if self.canceling else ''))
        finally:
            self.active = None
            self.busy = False

    def robot_pose(self):
        age = self.tf.robot_update_age()
        if age is None or age > self.cfg['pose_max_age_s']:
            return None
        try:
            transform = self.tf.lookup_transform(self.frame, self.base, Time())
        except TransformException:
            return None
        p, q = transform.transform.translation, transform.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v*v for v in values[3:]))
        if not all(math.isfinite(v) for v in values) or norm < 1e-6:
            return None
        x, y, z, w = (v/norm for v in values[3:])
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        stamp = Time.from_msg(transform.header.stamp).nanoseconds
        return (p.x, p.y, p.z, yaw, stamp)

    def pose_message(self, position):
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        pose.pose.orientation.z = math.sin(position[3]/2)
        pose.pose.orientation.w = math.cos(position[3]/2)
        return pose

    def start_child(self, client, goal, phase):
        if not client.server_is_ready():
            return False
        self.inflight = True
        self.phase = phase
        try:
            client.send_goal_async(goal).add_done_callback(self.accepted)
        except Exception as exc:
            self.inflight = False
            self.fail_target(str(exc))
        return True

    def accepted(self, future):
        self.inflight = False
        try:
            child = future.result()
            if not child.accepted:
                self.fail_target('navigation goal rejected')
                return
            self.child = child
            child.get_result_async().add_done_callback(self.completed)
            if self.canceling or self.phase == 'draining':
                child.cancel_goal_async()
        except Exception as exc:
            # No new goal is submitted following a transport error with uncertain ownership.
            self.canceling = True
            self.phase = 'transport_error'
            self.get_logger().error(f'Cannot establish navigation ownership: {exc}')

    def completed(self, future):
        try:
            result = future.result()
            if result.status not in (4, 5, 6):
                raise RuntimeError(f'non-terminal navigation result: {result.status}')
        except Exception as exc:
            self.canceling = True
            self.phase = 'transport_error'
            if self.child is not None:
                self.child.cancel_goal_async()
            self.get_logger().error(f'Cannot confirm navigation terminal result: {exc}')
            return
        self.child = None
        if self.canceling:
            return
        if self.phase == 'draining':
            self.fail_target('target timed out')
            return
        if result.status != 4:
            self.fail_target(f'navigation failed: status={result.status}')
            return
        self.phase = 'align_ready' if self.phase == 'approaching' else 'verify'
        pose = self.robot_pose()
        self.verification_stamp = pose[4] if pose else None
        self.previous_sample = None
        self.stopped_since = None

    def fail_target(self, reason):
        self.get_logger().warning(f'Target {self.target_id}: {reason}')
        self.failed += 1
        self.phase = 'select'
        self.deadline = time.monotonic() + self.cfg['target_timeout_s']

    def stopped(self, pose, now):
        previous = self.previous_sample
        if previous is None:
            self.previous_sample = (pose, now)
            return False
        old, received = previous
        if old[4] == pose[4]:
            return False
        elapsed = now-received
        self.previous_sample = (pose, now)
        moving = (elapsed <= 0 or elapsed > self.cfg['pose_max_age_s'] or
                  math.dist(old[:3], pose[:3])/elapsed > self.cfg['linear_stop_mps'] or
                  abs(angle_error(old[3], pose[3]))/elapsed > self.cfg['angular_stop_rps'])
        if moving:
            self.stopped_since = None
            return False
        if self.stopped_since is None:
            self.stopped_since = now
        return now-self.stopped_since >= self.cfg['settle_s']

    def tick(self):
        if self.active is None or self.waiter.done():
            return
        now = time.monotonic()
        if self.canceling:
            if self.child is None and not self.inflight and self.phase != 'transport_error':
                self.waiter.set_result(False)
            return
        self.active.publish_feedback(InspectTargets.Feedback(
            progress=100.*(self.inspected+self.failed)/max(1, self.total),
            phase=self.phase, target_id=self.target_id))
        if self.phase == 'assignment':
            if self.pending is None and self.latest is not None:
                self.pending = dict(self.latest)
                self.total = len(self.pending)
            if self.pending is not None:
                self.phase = 'select'
                self.deadline = now+self.cfg['target_timeout_s']
            elif now > self.deadline:
                self.failed += 1
                self.waiter.set_result(False)
            return
        if self.phase == 'dwell':
            if now >= self.dwell_until:
                self.phase = 'select'
                self.deadline = now+self.cfg['target_timeout_s']
            return
        if self.phase == 'select' and not self.pending:
            self.waiter.set_result(True)
            return
        if now > self.deadline:
            if self.phase == 'draining':
                return  # Retain ownership until the child's terminal result.
            if self.child is not None or self.inflight:
                self.phase = 'draining'
                if self.child is not None:
                    self.child.cancel_goal_async()
            elif self.phase == 'select':
                self.failed += len(self.pending)
                self.pending.clear()
            else:
                self.fail_target('no valid arrival before timeout')
            return
        pose = self.robot_pose()
        if pose is None:
            self.previous_sample = None
            self.stopped_since = None
            return
        if self.phase == 'select':
            self.target_id = nearest(pose, self.pending)
            self.target = self.pending.pop(self.target_id)
            self.deadline = now+self.cfg['target_timeout_s']
            self.phase = 'approach_ready'
            self.verification_stamp = None
        if self.phase == 'approach_ready':
            try:
                destination = visit_pose(pose, self.target, self.cfg['stand_off_m'], self.cfg['max_range_m'])
            except ValueError as exc:
                self.fail_target(str(exc))
                return
            if math.dist(pose[:3], self.target) <= self.cfg['max_range_m']:
                self.phase = 'align_ready'
            else:
                self.start_child(self.nav, NavigateToPose.Goal(pose=self.pose_message(destination)), 'approaching')
                return
        if self.phase == 'align_ready':
            if self.verification_stamp is not None and pose[4] <= self.verification_stamp:
                return
            dx, dy = self.target[0]-pose[0], self.target[1]-pose[1]
            yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else pose[3]
            desired = (*pose[:3], yaw)
            path = Path(header=self.pose_message(desired).header,
                        poses=[self.pose_message(desired)])
            self.start_child(self.align, FollowPath.Goal(path=path, controller_id='FollowTarget',
                goal_checker_id='target_goal_checker'), 'aligning')
            return
        if self.phase == 'verify':
            if self.verification_stamp is not None and pose[4] <= self.verification_stamp:
                return
            distance = math.dist(pose[:3], self.target)
            horizontal = math.hypot(self.target[1]-pose[1], self.target[0]-pose[0])
            error = (angle_error(math.atan2(self.target[1]-pose[1], self.target[0]-pose[0]), pose[3])
                     if horizontal > 1e-6 else 0.)
            if distance > self.cfg['max_range_m'] or abs(error) > self.cfg['heading_tolerance_rad']:
                self.fail_target('actual range or heading outside arrival tolerance')
                return
            if self.stopped(pose, now):
                self.arrivals.publish(InspectionArrival(task_id=self.active.request.task_id,
                    target_id=self.target_id, target_position=Point(x=self.target[0], y=self.target[1], z=self.target[2]),
                    robot_position=Point(x=pose[0], y=pose[1], z=pose[2]),
                    distance_m=distance, heading_error_rad=error))
                self.inspected += 1
                self.dwell_until = now+self.cfg['dwell_s']
                self.phase = 'dwell'

    def destroy_node(self):
        if self.child is not None:
            self.child.cancel_goal_async()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
