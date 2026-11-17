"""Single active task topic -> one primitive Action -> robot_task_status."""
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from patrol_node.action import Patrol, PointGoal
from task_interfaces.action import AreaSearch, MultiPoint, InspectTargets
from task_interfaces.msg import Task, TaskStatus, SearchArea
from task_pipeline.task_schema import from_message, distance, TASK_KINDS, nav_points
from task_pipeline.target_interaction import TargetInteraction
from yolo_detector.targets import DEFAULT_TARGETS_FILE, targets_path, write_targets


class PipelineDispatcher(Node):
    def __init__(self):
        super().__init__('pipeline_dispatcher')
        self.declare_parameter('robot_namespace', 'robot_1')
        self.declare_parameter('action_wait_timeout_s', 30.0)
        self.declare_parameter('inspection_search_mode', False)
        self.declare_parameter('simulation_targets', False)
        self.declare_parameter('target_interaction_enabled', False)
        self.declare_parameter('yolo_targets_file', DEFAULT_TARGETS_FILE)
        self.ns = str(self.get_parameter('robot_namespace').value).strip('/')
        self.yolo_targets_path = targets_path(
            self.get_namespace().strip('/') or self.ns,
            self.get_parameter('yolo_targets_file').value)
        self.action_clients = {
            'point_goal': ActionClient(self, PointGoal, 'point_goal'),
            'multi_point': ActionClient(self, MultiPoint, 'multi_point'),
            'patrol_line': ActionClient(self, Patrol, 'patrol'),
            'area_search': ActionClient(self, AreaSearch, 'area_search'),
            'target_inspection': ActionClient(self, InspectTargets, 'target_inspection'),
            'search_inspection': ActionClient(self, AreaSearch, 'search_inspection'),
        }
        self.current = None
        self.goal_handle = None
        self.sending = False
        self._ownership_error = ''
        self._pending_finish = None
        self.progress = 0
        self.fix = None
        self.start = None
        self.initial_distance = None
        self.status_pub = self.create_publisher(TaskStatus, 'robot_task_status', 10)
        self.events = self.create_publisher(String, 'task_feedback', 10)
        self.create_subscription(Task, 'task', self.receive, 10)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.on_fix,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.pump)
        self.create_timer(1.0, self.publish_status)
        self.target_interaction = TargetInteraction(self)
        self.create_timer(0.1, self.target_interaction.tick)

    def emit(self, event, task, **fields):
        self.events.publish(String(data=json.dumps(dict(
            event=event, task_id=str(task.task_id), **fields))))

    def receive(self, task):
        if task.cancel:
            self.cancel(task)
            return
        if self.current is not None or self._pending_finish is not None or self._ownership_error:
            self.emit('rejected', task, message='another task is still active')
            return
        try:
            from_message(task)
        except ValueError as exc:
            self.emit('rejected', task, message=str(exc))
            return
        self.current = task
        if task.task_id in (142, 143):
            self.target_interaction.begin(task)
        self.cancel_pending = None
        self.failures = 0
        self.progress = 1
        self.start = None
        self.initial_distance = None
        self.deadline = time.monotonic() + float(self.get_parameter('action_wait_timeout_s').value)
        if self.fix and time.monotonic() - self.fix[1] < 2.0:
            self.on_fix(self.fix[0])
        self.emit('accepted', task)
        self.publish_status()

    def on_fix(self, fix):
        if (fix.status.status < 0 or not all(math.isfinite(x) for x in
                (fix.latitude, fix.longitude)) or abs(fix.latitude) > 90 or abs(fix.longitude) > 180):
            self.fix = None
            return
        self.fix = (fix, time.monotonic())
        if self.current and self.current.task_id == 140:
            if self.start is None:
                self.start = fix
                self.initial_distance = distance(fix, nav_points(self.current.waypoints)[0])
            if self.initial_distance > 1e-6:
                remaining = distance(fix, nav_points(self.current.waypoints)[0])
                self.progress = max(self.progress, min(99, max(0, int(
                    100 * (1.0 - remaining / self.initial_distance)))))

    def pump(self):
        if self._pending_finish is not None:
            if self.target_interaction.drained():
                success, message = self._pending_finish
                self._pending_finish = None
                self._finish_now(success, message)
            return
        if self._ownership_error:
            return
        if self.current is None:
            return
        if self.sending or self.goal_handle is not None:
            return
        task = self.current
        kind = TASK_KINDS[task.task_id]
        points = nav_points(task.waypoints)
        simulation_observation = (kind == 'target_inspection' and
            self.get_parameter('simulation_targets').value and
            self.get_parameter('target_interaction_enabled').value)
        search_inspection = (kind == 'target_inspection' and
            self.get_parameter('inspection_search_mode').value and not simulation_observation)
        if search_inspection and (len(task.center) != 3 or min(task.length_m, task.width_m) < 1.):
            self.finish(False, '143 search inspection needs center and length_m/width_m >= 1 metre')
            return
        client = (self.action_clients['area_search'] if simulation_observation else
                  self.action_clients['search_inspection' if search_inspection else kind])
        if not client.server_is_ready():
            if time.monotonic() > self.deadline:
                self.finish(False, 'primitive Action server unavailable')
            return
        if kind == 'point_goal':
            goal = PointGoal.Goal(target=points[0])
        elif kind == 'patrol_line':
            goal = Patrol.Goal(start_point=points[0], end_point=points[1], max_laps=1)
        elif kind == 'multi_point':
            goal = MultiPoint.Goal(task_id=str(task.task_id), waypoints=points)
        elif kind == 'area_search' or search_inspection or simulation_observation:
            area = SearchArea(center=nav_points(task.center)[0], length_m=task.length_m,
                              width_m=task.width_m, yaw_deg=task.yaw_deg)
            goal = AreaSearch.Goal(task_id=str(task.task_id), area=area)
        else:
            goal = InspectTargets.Goal(task_id=str(task.task_id))
        if kind in ('area_search', 'target_inspection'):
            try:
                write_targets(self.yolo_targets_path, task.targets, task_id=str(task.task_id))
                self.get_logger().info(
                    f'{task.task_id} YOLO targets={list(task.targets)} -> {self.yolo_targets_path}')
            except (OSError, ValueError) as exc:
                self.finish(False, 'YOLO target file update failed: '+str(exc))
                return  # Do not dispatch with stale detection classes.
        if kind in ('point_goal', 'patrol_line'):
            goal.mission_id = str(task.task_id)
            goal.primitive_id = str(task.task_id)
            goal.agent_id = self.ns
            goal.capability_id = kind
        self.sending = True
        try:
            client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)
        except Exception as exc:
            self.ownership_error(exc)

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.finish(False, 'primitive Action rejected')
                return
            if getattr(self, 'cancel_pending', None) == self.current.task_id:
                self.goal_handle.cancel_goal_async()
            self.emit('executing', self.current)
            self.goal_handle.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.ownership_error(exc)

    def feedback(self, message):
        if self.current is None:
            return
        fb = message.feedback
        kind = TASK_KINDS[self.current.task_id]
        if kind in ('area_search', 'multi_point', 'target_inspection'):
            value = fb.progress
        elif kind == 'patrol_line':
            # Initial endpoint + outbound + return = three reached endpoints.
            value = fb.reached_points / 3.0 * 100
        elif kind == 'point_goal':
            if self.fix and time.monotonic() - self.fix[1] < 2.0:
                return  # Fresh valid GPS has priority over ENU feedback.
            value = fb.progress
        else:
            return
        if math.isfinite(value):
            if kind == 'area_search' and fb.phase in ('stopping', 'completed') and value >= 100:
                self.progress = 100
                self.publish_status()
                return
            self.progress = max(self.progress, min(99, max(0, int(value))))

    def result(self, future):
        try:
            wrapped = future.result()
            self.finish(wrapped.status == 4 and wrapped.result.success, wrapped.result.message)
        except Exception as exc:
            self.ownership_error(exc)

    def ownership_error(self, error):
        # A lost request/result response does not prove that the primitive stopped.
        self._ownership_error = str(error)
        self.get_logger().error('Task channel retained: cannot confirm primitive ownership: '+str(error))
        if self.goal_handle is not None and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
        self.emit('ownership_error', self.current, message=str(error))

    def publish_status(self):
        # Startup and idle heartbeat: the ID is a string in the public interface.
        status = TaskStatus(task_id="0", progress=0, failures=0,
                            param1=0.0, param2=0.0, param3=0.0)
        if self.current:
            status.task_id = str(self.current.mission_seq_id)
            status.progress = max(1, self.progress)
            status.failures = getattr(self, "failures", 0)
        self.status_pub.publish(status)

    def finish(self, success, message):
        if hasattr(self, 'target_interaction') and not self.target_interaction.drained():
            self.target_interaction.stop(orphaned=True)
            self._pending_finish = (success, message)
            return
        self._finish_now(success, message)

    def _finish_now(self, success, message):
        if self.cancel_pending:
            success, message = False, "canceled: " + message
        self.failures = 0 if success else 1
        self.progress = 100
        self.publish_status()
        self.emit('completed' if success else 'failed', self.current, message=message)
        self.current = None
        self.goal_handle = None
        self.sending = False
        self.cancel_pending = None

    def cancel(self, message):
        task_id = message.task_id
        if self.current is None or self.current.task_id != task_id:
            self.emit('rejected', message, message='no active task with matching task_id')
            return
        self.cancel_pending = task_id
        if self.goal_handle and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
        elif not self.sending:
            self.finish(False, 'canceled before dispatch')
        else:
            # The request may be in transit; cancel immediately on acceptance.
            self.emit('cancel_pending', self.current)

    def destroy_node(self):
        if self.goal_handle and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PipelineDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
