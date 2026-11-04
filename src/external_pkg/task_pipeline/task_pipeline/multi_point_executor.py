"""Multi-point primitive: sequential PointGoal actions with correlated results."""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.task import Future
from patrol_node.action import PointGoal
from task_interfaces.action import MultiPoint
from task_pipeline.task_schema import from_message
from task_interfaces.msg import Task


class MultiPointExecutor(Node):
    def __init__(self):
        super().__init__('multi_point_executor')
        self.busy = False
        self.active = None
        self.child = None
        self.canceling = False
        self.waiter = None
        self.pending = None
        self.client = ActionClient(self, PointGoal, 'point_goal')
        self.server = ActionServer(self, MultiPoint, 'multi_point', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel)
        self.create_timer(.1, self.pump)

    def goal(self, request):
        if self.busy:
            return GoalResponse.REJECT
        try:
            from_message(Task(task_id=request.task_id, task_type='multi_point', waypoints=request.waypoints))
        except ValueError:
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.canceling = True
        if self.child:
            self.child.cancel_goal_async()
        return CancelResponse.ACCEPT

    def pump(self):
        if not self.pending:
            return
        goal, deadline = self.pending
        if self.canceling:
            self.pending = None
            self.waiter.set_result(False)
        elif self.client.server_is_ready():
            self.pending = None
            try:
                self.client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)
            except Exception as exc:
                self.get_logger().error(str(exc))
                self.waiter.set_result(False)
        elif time.monotonic() > deadline:
            self.pending = None
            self.waiter.set_result(False)

    def accepted(self, future):
        try:
            self.child = future.result()
            if not self.child.accepted:
                self.waiter.set_result(False)
                return
            self.child.get_result_async().add_done_callback(self.completed)
            if self.canceling:
                self.child.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.waiter.set_result(False)

    def completed(self, future):
        self.child = None
        try:
            result = future.result()
            self.waiter.set_result(result.status == 4 and result.result.success)
        except Exception:
            self.waiter.set_result(False)

    def feedback(self, message):
        d = message.feedback.distance_to_target_m
        if math.isfinite(d):
            if self.initial_distance is None:
                self.initial_distance = d
            fraction = max(0., min(1., 1.-d/self.initial_distance)) if self.initial_distance > 1e-6 else 0.
            self.active.publish_feedback(MultiPoint.Feedback(
                progress=100.*(self.index+fraction)/self.total, phase=f'waypoint_{self.index+1}'))

    async def execute(self, handle):
        self.active = handle
        self.canceling = handle.is_cancel_requested
        self.total = len(handle.request.waypoints)
        success = True
        try:
            for index, point in enumerate(handle.request.waypoints):
                if self.canceling:
                    success = False
                    break
                self.index = index
                self.initial_distance = None
                self.waiter = Future()
                self.pending = (PointGoal.Goal(mission_id=handle.request.task_id,
                    primitive_id=f'{handle.request.task_id}_{index}', capability_id='point_goal', target=point),
                    time.monotonic()+30.)
                if not await self.waiter:
                    success = False
                    break
                handle.publish_feedback(MultiPoint.Feedback(
                    progress=100.*(index+1)/self.total, phase='waypoint_reached'))
            if handle.is_cancel_requested:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
            return MultiPoint.Result(success=success and not self.canceling,
                                     message='all waypoints reached' if success else 'waypoint failed or canceled')
        finally:
            self.active = None
            self.busy = False
            self.pending = None


def main(args=None):
    rclpy.init(args=args)
    node = MultiPointExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
