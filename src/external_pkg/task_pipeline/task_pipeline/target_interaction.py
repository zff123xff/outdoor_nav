"""Search suspension and simulation inspection, with one controller owner at a time."""
import math
import time

from action_msgs.srv import CancelGoal
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from std_srvs.srv import SetBool
from task_interfaces.action import ApproachTarget
from task_interfaces.msg import TargetEvent


class TargetInteraction:
    def __init__(self, node):
        self.node = node
        self.pause = node.create_client(SetBool, 'area_search/pause')
        self.approach = ActionClient(node, ApproachTarget, 'targets/approach')
        self.request_pub = node.create_publisher(TargetEvent, 'targets/request', 10)
        node.create_subscription(TargetEvent, 'targets/found', self.observed, 10)
        node.create_subscription(TargetEvent, 'targets/shoot', self.shoot, 10)
        # A dead marker cannot retire its FollowPath itself. Use the controller
        # cancel/result services before allowing a later task to be accepted.
        self.cancel_paths = node.create_client(CancelGoal, 'follow_path/_action/cancel_goal')
        self.path_result = node.create_client(FollowPath.Impl.GetResultService, 'follow_path/_action/get_result')
        self.pending = None
        self.handle = None
        self.operation = None
        self.recovery = None
        self.recovery_results = []
        self.generation = 0
        self.phase = 'idle'
        self.handled = set()
        self.target_id = None
        self.task_id = None
        self.stopping = False
        self.orphaned = False

    def begin(self, task):
        self.generation += 1
        self.task_id = task.task_id
        self.phase = 'searching'
        self.target_id = None
        self.handled.clear()
        self.stopping = False
        self.orphaned = False
        self.operation = None
        self.recovery = None
        self.recovery_results = []

    def enabled(self):
        task = self.node.current
        return (self.node.get_parameter('simulation_targets').value and
                self.node.get_parameter('target_interaction_enabled').value and
                task is not None and task.task_id in (142, 143) and
                self.node.goal_handle is not None and self.node.goal_handle.accepted)

    def observed(self, event):
        if (not self.enabled() or self.stopping or self.phase != 'searching' or
                event.command != TargetEvent.TARGET_FOUND or not event.target_id or
                event.target_id in self.handled or event.position.status.status < 0 or
                not all(math.isfinite(v) for v in (event.position.latitude,
                                                 event.position.longitude, event.position.altitude))):
            return
        self.target_id = event.target_id
        self.phase = 'pausing_search'
        self.deadline = time.monotonic() + 15.
        self.node.emit('target_detected', self.node.current, target_id=self.target_id,
                       phase=self.phase)

    def tick(self):
        if self.orphaned and (self.handle is not None or self.pending is not None) and self.recovery is None:
            self.start_recovery()
        if self.recovery is not None:
            self.poll_recovery()
        if self.stopping or not self.enabled():
            return
        if self.phase in ('pausing_search', 'starting_approach', 'resuming_search'):
            if time.monotonic() > self.deadline:
                self.fail(self.phase+' timed out')
                return
        if self.phase in ('pausing_search', 'resuming_search') and self.operation is None:
            if not self.pause.service_is_ready():
                return
            suspend = self.phase == 'pausing_search'
            token = self.generation
            try:
                self.operation = self.pause.call_async(SetBool.Request(data=suspend))
                self.operation.add_done_callback(lambda future: self.paused(future, suspend, token))
            except Exception as exc:
                self.fail('Cannot request search pause/resume: '+str(exc))
                return
        if self.phase == 'starting_approach' and self.pending is None:
            if not self.approach.server_is_ready():
                return
            self.phase = 'approaching_target'
            token = self.generation
            try:
                self.pending = self.approach.send_goal_async(
                    ApproachTarget.Goal(task_id=str(self.task_id), target_id=self.target_id),
                    feedback_callback=lambda message: self.feedback(message, token))
                self.pending.add_done_callback(lambda future: self.accepted(future, token))
            except Exception as exc:
                self.fail('Cannot request target approach: '+str(exc))

    def paused(self, future, suspended, token):
        if future is self.operation:
            self.operation = None
        if token != self.generation:
            return
        if self.stopping:
            return
        try:
            response = future.result()
            if not response.success:
                self.fail('Search pause/resume failed: '+response.message)
                return
        except Exception as exc:
            self.fail('Search pause/resume failed: '+str(exc))
            return
        self.phase = 'starting_approach' if suspended else 'searching'
        self.deadline = time.monotonic() + 15.
        self.node.emit('search_suspended' if suspended else 'search_resumed', self.node.current,
                       target_id=self.target_id, phase=self.phase)

    def accepted(self, future, token):
        if token != self.generation:
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
            return
        self.pending = None
        try:
            handle = future.result()
            if not handle.accepted:
                if not self.stopping:
                    self.fail('Target approach Action was rejected')
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(lambda future: self.completed(future, token))
            if self.stopping:
                handle.cancel_goal_async()
        except Exception as exc:
            self.fail('Target approach Action failed: '+str(exc))

    def feedback(self, message, token):
        if (token != self.generation or self.stopping or self.node.current is None or
                self.handle is None):
            return
        phase = message.feedback.phase
        if phase != self.phase:
            self.phase = phase
            distance = message.feedback.distance_m
            heading = message.feedback.heading_error_rad
            self.node.emit('target_state', self.node.current, target_id=self.target_id,
                           phase=phase, distance_m=distance if math.isfinite(distance) else None,
                           heading_error_rad=heading if math.isfinite(heading) else None)

    def completed(self, future, token):
        if token != self.generation:
            return
        self.handle = None
        if self.stopping:
            return
        try:
            wrapped = future.result()
            if wrapped.status != 4 or not wrapped.result.success:
                self.fail('Target inspection failed: '+wrapped.result.message)
                return
        except Exception as exc:
            self.fail('Target inspection failed: '+str(exc))
            return
        self.handled.add(self.target_id)
        self.node.emit('target_confirmed', self.node.current, target_id=self.target_id)
        self.phase = 'resuming_search'
        self.deadline = time.monotonic() + 15.

    def shoot(self, event):
        if (not self.enabled() or self.stopping or self.phase != 'awaiting_confirmation' or
                event.target_id != self.target_id or event.command not in ('', TargetEvent.REQUEST_SHOOT)):
            self.node.get_logger().warning('Ignoring target confirmation: no matching ready target',
                                           throttle_duration_sec=3.)
            return
        # The marker rechecks current range, heading, stopped state and target
        # freshness, and completes its Action only after applying the RViz mark.
        self.request_pub.publish(TargetEvent(target_id=self.target_id, command=TargetEvent.REQUEST_SHOOT))

    def fail(self, message):
        self.node._interaction_error = message
        self.node.get_logger().error(message)
        self.stop()
        if self.node.goal_handle is not None and self.node.goal_handle.accepted:
            self.node.goal_handle.cancel_goal_async()

    def stop(self, orphaned=False):
        self.stopping = True
        self.orphaned = self.orphaned or orphaned
        if self.handle is not None:
            self.handle.cancel_goal_async()
        if self.orphaned and (self.handle is not None or self.pending is not None) and self.recovery is None:
            self.start_recovery()

    def start_recovery(self):
        if self.cancel_paths.service_is_ready():
            self.recovery = self.cancel_paths.call_async(CancelGoal.Request())
        else:
            self.node.get_logger().error('Controller cancellation unavailable; keeping task ownership',
                                         throttle_duration_sec=3.)

    def poll_recovery(self):
        if not self.recovery.done():
            return
        if not self.recovery_results:
            try:
                response = self.recovery.result()
                if response.return_code not in (CancelGoal.Response.ERROR_NONE,
                                                 CancelGoal.Response.ERROR_GOAL_TERMINATED):
                    return
                for info in response.goals_canceling:
                    request = FollowPath.Impl.GetResultService.Request(goal_id=info.goal_id)
                    self.recovery_results.append(self.path_result.call_async(request))
            except Exception as exc:
                self.node.get_logger().error(f'Controller recovery failed: {exc}')
                return
            if not self.recovery_results:
                self.handle = self.pending = None
                self.recovery = None
                self.generation += 1
                return
        if all(future.done() for future in self.recovery_results):
            try:
                if not all(future.result().status in (4, 5, 6) for future in self.recovery_results):
                    return
            except Exception:
                return
            self.handle = self.pending = None
            self.recovery = None
            self.generation += 1

    def drained(self):
        return (self.handle is None and self.pending is None and self.recovery is None and
                self.operation is None)
