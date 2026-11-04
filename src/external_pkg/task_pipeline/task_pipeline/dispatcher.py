"""FIFO task topic -> exactly one primitive Action -> robot_task_status."""
from collections import deque
import json
import math
import time
import os
import signal
import subprocess
import sys
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from patrol_node.action import Patrol, PointGoal
from task_interfaces.action import AreaSearch, MultiPoint
from task_interfaces.msg import Task, TaskStatus
from task_pipeline.task_schema import from_message, distance
from yolo_detector.targets import targets_path, write_targets


class PipelineDispatcher(Node):
    def __init__(self):
        super().__init__('pipeline_dispatcher')
        self.declare_parameter('robot_namespace', 'robot_1')
        self.declare_parameter('action_wait_timeout_s', 30.0)
        self.declare_parameter('targets_file', '')
        self.declare_parameter('simulation_targets', False)
        self.declare_parameter('target_marker_config', '')
        self._target_marker_process = None
        self._marker_error = ''
        self.ns = str(self.get_parameter('robot_namespace').value).strip('/')
        self.action_clients = {
            'point_goal': ActionClient(self, PointGoal, 'point_goal'),
            'multi_point': ActionClient(self, MultiPoint, 'multi_point'),
            'patrol_line': ActionClient(self, Patrol, 'patrol'),
            'area_search': ActionClient(self, AreaSearch, 'area_search'),
        }
        self.targets_file = targets_path(self.get_namespace() or self.ns,
            self.get_parameter('targets_file').value)
        self.queue = deque()
        self.current = None
        self.goal_handle = None
        self.sending = False
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

    def emit(self, event, task, **fields):
        self.events.publish(String(data=json.dumps(dict(
            event=event, task_id=task.task_id, task_type=task.task_type, **fields))))

    def receive(self, task):
        try:
            from_message(task)
        except ValueError as exc:
            self.emit('rejected', task, message=str(exc))
            return
        if task.cancel:
            self.cancel(task)
            return
        self.queue.append(task)
        self.emit('queued', task)

    def on_fix(self, fix):
        if (fix.status.status < 0 or not all(math.isfinite(x) for x in
                (fix.latitude, fix.longitude)) or abs(fix.latitude) > 90 or abs(fix.longitude) > 180):
            self.fix = None
            return
        self.fix = (fix, time.monotonic())
        if self.current and self.current.task_type == 'point_goal':
            if self.start is None:
                self.start = fix
                self.initial_distance = distance(fix, self.current.waypoints[0])
            if self.initial_distance > 1e-6:
                self.progress = max(self.progress, min(99, int(
                    100 * distance(self.start, fix) / self.initial_distance)))

    def _start_target_marker(self):
        if not self.get_parameter('simulation_targets').value or self.current.task_type != 'area_search':
            return
        config = self.get_parameter('target_marker_config').value or os.path.join(
            get_package_share_directory('yolo_detector'), 'config', 'target_marker.yaml')
        sim_time = self.get_parameter('use_sim_time').value if self.has_parameter('use_sim_time') else False
        command = [sys.executable, '-m', 'yolo_detector.target_marker', '--ros-args',
                   '-r', '__ns:='+self.get_namespace(), '--params-file', str(config),
                   '-p', 'use_sim_time:='+str(bool(sim_time)).lower(),
                   '-p', 'targets_file:='+str(self.targets_file)]
        self._marker_error = ''
        self._target_marker_process = subprocess.Popen(command, start_new_session=True)
        self.get_logger().info(f'Started simulation marker for {self.current.task_id}')

    def _stop_target_marker(self):
        process = self._target_marker_process
        self._target_marker_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=1.)
            except ProcessLookupError:
                process.wait(timeout=1.)

    def pump(self):
        process = self._target_marker_process
        if process is not None and process.poll() is not None and not self._marker_error:
            self._marker_error = f'Simulation marker exited with code {process.returncode}'
            self.get_logger().error(self._marker_error)
            if self.goal_handle and self.goal_handle.accepted:
                self.goal_handle.cancel_goal_async()
            elif self.sending:
                self.cancel_pending = self.current.task_id
            else:
                self.finish(False, self._marker_error)
                return
        if self.current is None:
            if not self.queue:
                return
            self.current = self.queue.popleft()
            self.progress = 0
            try:
                write_targets(self.targets_file, self.current.targets, self.current.task_id)
            except (OSError, ValueError) as exc:
                self.finish(False, 'Cannot update detection targets: '+str(exc))
                return
            self.start = None
            self.initial_distance = None
            self.deadline = time.monotonic() + float(self.get_parameter('action_wait_timeout_s').value)
            if self.fix and time.monotonic()-self.fix[1] < 2.0:
                self.on_fix(self.fix[0])
        if self.sending or self.goal_handle is not None:
            return
        task = self.current
        client = self.action_clients[task.task_type]
        if not client.server_is_ready():
            if time.monotonic() > self.deadline:
                self.finish(False, 'primitive Action server unavailable')
            return
        if task.task_type == 'point_goal':
            goal = PointGoal.Goal(target=task.waypoints[0])
        elif task.task_type == 'patrol_line':
            goal = Patrol.Goal(start_point=task.waypoints[0], end_point=task.waypoints[1], max_laps=1)
        elif task.task_type == 'multi_point':
            goal = MultiPoint.Goal(task_id=task.task_id, waypoints=task.waypoints)
        else:
            goal = AreaSearch.Goal(task_id=task.task_id, area=task.area)
        if task.task_type in ('point_goal', 'patrol_line'):
            goal.mission_id = task.task_id
            goal.primitive_id = task.task_id
            goal.agent_id = self.ns
            goal.capability_id = task.task_type
        self.sending = True
        try:
            self._start_target_marker()
            client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)
        except Exception as exc:
            self.finish(False, str(exc))

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.finish(False, 'primitive Action rejected')
                return
            if getattr(self, 'cancel_pending', None) == self.current.task_id:
                self.goal_handle.cancel_goal_async()
                self.cancel_pending = None
            self.emit('executing', self.current)
            self.goal_handle.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.finish(False, str(exc))

    def feedback(self, message):
        if self.current is None:
            return
        fb = message.feedback
        kind = self.current.task_type
        if kind in ('area_search', 'multi_point'):
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
            self.progress = max(self.progress, min(99, max(0, int(value))))

    def result(self, future):
        try:
            wrapped = future.result()
            self.finish(wrapped.status == 4 and wrapped.result.success, wrapped.result.message)
        except Exception as exc:
            self.finish(False, str(exc))

    def publish_status(self):
        if self.current:
            self.status_pub.publish(TaskStatus(task_id=self.current.task_id,
                                               progress=self.progress, failures=0))

    def finish(self, success, message):
        # Every terminal Action result (including failure/cancel) completes this task.
        if self._marker_error:
            success, message = False, self._marker_error
        self.progress = 100
        self.publish_status()
        self.emit('completed' if success else 'failed', self.current, message=message)
        self._stop_target_marker()
        self._marker_error = ''
        self.current = None
        self.goal_handle = None
        self.sending = False
        self.cancel_pending = None

    def cancel(self, message):
        task_id = message.task_id
        for task in self.queue:
            if task.task_id == task_id:
                self.emit('failed', task, message='canceled while queued')
        self.queue = deque(t for t in self.queue if t.task_id != task_id)
        if self.current and self.current.task_id == task_id:
            if self.goal_handle and self.goal_handle.accepted:
                self.goal_handle.cancel_goal_async()
            elif not self.sending:
                self.finish(False, 'canceled before dispatch')
            else:
                # The request may be in transit; cancel immediately on acceptance.
                self.emit('cancel_pending', self.current)
                self.cancel_pending = task_id

    def destroy_node(self):
        if self.goal_handle and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
        self._stop_target_marker()
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
