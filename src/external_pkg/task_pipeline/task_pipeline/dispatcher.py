#!/usr/bin/env python3
"""ExecuteTask pipeline dispatcher for point, patrol and waypoint primitives."""

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

from execute_task_server.srv import ExecuteTask
from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
from patrol_node.action import Patrol, PointGoal


class PipelineDispatcher(Node):
    def __init__(self):
        super().__init__('pipeline_dispatcher')
        self.declare_parameter('agent_id', 'robot_1')
        self.declare_parameter('robot_namespace', 'robot_1')
        self.agent_id = str(self.get_parameter('agent_id').value)
        self.robot_namespace = str(self.get_parameter('robot_namespace').value).strip('/')
        if not self.agent_id or self.agent_id != self.robot_namespace:
            raise ValueError('agent_id must match robot_namespace for task routing')
        service = f'/agent/{self.agent_id}/execute_task'
        self._server = self.create_service(ExecuteTask, service, self._handle)
        self._feedback_pub = self.create_publisher(String, 'task_feedback', 10)
        self._waypoint_pub = self.create_publisher(MultiGoal, 'waypoint_goals', 10)
        self._area_request_pub = self.create_publisher(String, 'area_search/request', 10)
        self._area_active = False
        self._area_search_ready = False
        self._active_goals = {}
        self._active_goal_types = {}
        self._area_active_sub = self.create_subscription(
            Bool, 'area_search/active', lambda message: setattr(self, '_area_active', message.data), 1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._area_status_sub = self.create_subscription(
            String, 'area_search/status', self._area_status, latched)
        self._point_client = ActionClient(self, PointGoal, f'/{self.robot_namespace}/point_goal')
        self._patrol_client = ActionClient(self, Patrol, f'/{self.robot_namespace}/patrol')
        self.get_logger().info(
            f'Pipeline ready: {service} -> point_goal/patrol executors; '
            f'feedback={self._feedback_pub.topic_name}')

    def _emit(self, event, request, **fields):
        message = {
            'event': event,
            'mission_id': request.get('mission_id', ''),
            'agent_id': request.get('agent_id', self.agent_id),
            'capability_id': request.get('capability_id', ''),
            'primitive_id': request.get('primitive_id', ''),
            **fields,
        }
        self._feedback_pub.publish(String(data=json.dumps(message, ensure_ascii=False)))

    def _handle(self, request, response):
        task = {}
        try:
            task = json.loads(request.request_json)
            if not isinstance(task, dict):
                raise ValueError('request_json must contain a JSON object')
            agent = task.get('agent_id')
            if not isinstance(agent, str) or not agent:
                raise ValueError('task agent_id is required')
            if agent != self.agent_id:
                raise ValueError(f'task agent_id {agent!r} does not match {self.agent_id!r}')
            capability = task.get('capability_id', 'goto_waypoints')
            params = task.get('params', {})
            if not isinstance(params, dict):
                raise ValueError('params must be a JSON object')
            waypoints = params.get('waypoints', [])
            if self._area_active and capability not in ('cancel_area_search', 'cancel_task'):
                raise RuntimeError('area_search owns navigation; cancel it before another navigation task')

            if capability in ('cancel_patrol', 'cancel_task'):
                mission_to_cancel = str(params.get('mission_id', ''))
                if mission_to_cancel:
                    selected = [mission_to_cancel]
                elif capability == 'cancel_patrol':
                    selected = [key for key, value in self._active_goal_types.items() if value == 'patrol']
                else:
                    selected = list(self._active_goals)
                canceled = 0
                for mission_id in selected:
                    handle = self._active_goals.get(mission_id)
                    if handle is not None:
                        handle.cancel_goal_async()
                        canceled += 1
                if capability == 'cancel_patrol' and canceled == 0:
                    cancel = MultiGoal()
                    cancel.cancel_patrol = True
                    self._waypoint_pub.publish(cancel)
                response.response_json = json.dumps({'status': 'success', 'message': 'cancel requested',
                                                     'active_actions_canceled': canceled})
                self._emit('cancel_requested', task, active_actions_canceled=canceled)
                return response

            if capability == 'patrol_line':
                if params.get('waypoint_frame', 'gps') != 'gps' or len(waypoints) != 2:
                    raise ValueError('patrol_line requires exactly two GPS waypoints')
                client = self._patrol_client
                if not client.server_is_ready():
                    raise RuntimeError(f'Patrol Action server unavailable: {client._action_name}')
                goal = Patrol.Goal()
                goal.mission_id = str(task.get('mission_id', ''))
                goal.agent_id = self.agent_id
                goal.capability_id = capability
                goal.primitive_id = str(task.get('primitive_id', ''))
                goal.start_point = self._fix(waypoints[0])
                goal.end_point = self._fix(waypoints[1])
                goal.max_laps = int(params.get('max_laps', 1))
                goal.hold_time_at_endpoint_s = float(params.get('hold_time_at_endpoint_s', 0.0))
                goal.keep_moving_on_end = bool(params.get('keep_moving_on_end', False))
                self._send_action(client, goal, task, 'patrol')
            elif capability in ('goto_waypoints', 'point_goal') and len(waypoints) == 1:
                if params.get('waypoint_frame', 'gps') != 'gps':
                    raise ValueError('point_goal requires GPS waypoint after coordinate normalization')
                client = self._point_client
                if not client.server_is_ready():
                    raise RuntimeError(f'PointGoal Action server unavailable: {client._action_name}')
                goal = PointGoal.Goal()
                goal.mission_id = str(task.get('mission_id', ''))
                goal.agent_id = self.agent_id
                goal.capability_id = 'goto_waypoints'
                goal.primitive_id = str(task.get('primitive_id', ''))
                goal.target = self._fix(waypoints[0])
                self._send_action(client, goal, task, 'point_goal')
            elif capability in ('area_search', 'cancel_area_search'):
                if not self._area_search_ready or self._area_request_pub.get_subscription_count() == 0:
                    raise RuntimeError('area_search executor is not online in this namespace')
                if capability == 'area_search' and self._area_active:
                    raise RuntimeError('area_search executor is already active')
                self._area_request_pub.publish(String(data=json.dumps(task, ensure_ascii=False)))
                self._emit('accepted', task, message='routed to area_search executor')
                response.response_json = json.dumps({'status': 'success', 'mission_id': task.get('mission_id', ''),
                                                     'message': 'routed to area_search executor'})
                return response
            elif capability in ('goto_waypoints', 'takeoff', 'return'):
                self._publish_waypoints(task, params)
                response.response_json = json.dumps({'status': 'success', 'mission_id': task.get('mission_id', ''),
                                                     'message': 'published to waypoint executor'})
                self._emit('accepted', task, message='published to waypoint executor')
                return response
            else:
                raise ValueError(f'unsupported capability_id: {capability}')

            response.response_json = json.dumps({'status': 'accepted', 'mission_id': task.get('mission_id', ''),
                                                 'message': 'dispatched to action executor'})
            self._emit('accepted', task, message='dispatched to action executor')
        except Exception as exc:  # respond to caller instead of dropping service callback
            self.get_logger().error(f'ExecuteTask rejected: {exc}')
            response.response_json = json.dumps({'status': 'error', 'message': str(exc)})
            self._emit('rejected', task, message=str(exc))
        return response

    @staticmethod
    def _fix(point):
        fix = NavSatFix()
        fix.latitude = float(point['lat'])
        fix.longitude = float(point['lon'])
        fix.altitude = float(point.get('alt', 0.0))
        return fix

    def _area_status(self, message):
        try:
            self._area_search_ready = isinstance(json.loads(message.data), dict)
        except (TypeError, ValueError):
            self._area_search_ready = False

    def _publish_waypoints(self, task, params):
        frame = params.get('waypoint_frame', 'gps')
        if frame not in ('gps', 'local'):
            raise ValueError('waypoint_frame must be gps or local')
        points = params.get('waypoints')
        if not isinstance(points, list) or not points:
            raise ValueError('waypoints must be a non-empty list')
        msg = MultiGoal()
        msg.is_gps_aid = frame == 'gps'
        msg.is_gps_hgt = frame == 'gps'
        msg.ctl_mode = 1
        for point in points:
            output = MultiGoalPoint()
            if frame == 'gps':
                output.x_or_lat = float(point['lat'])
                output.y_or_lon = float(point['lon'])
                output.z_or_alt = float(point.get('alt', 0.0))
            else:
                output.x_or_lat = float(point['x'])
                output.y_or_lon = float(point['y'])
                output.z_or_alt = float(point.get('z', 0.0))
            output.yaw = float(point.get('yaw', 0.0))
            output.vel = float(point.get('vel', 1.0))
            msg.multi_goal_points.append(output)
        self._waypoint_pub.publish(msg)

    def _send_action(self, client, goal, task, kind):
        send_future = client.send_goal_async(
            goal, feedback_callback=lambda msg: self._action_feedback(msg, task, kind))
        send_future.add_done_callback(lambda future: self._goal_response(future, task, kind))

    def _goal_response(self, future, task, kind):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._emit('rejected', task, message=f'{kind} action goal rejected')
                return
            self._active_goals[str(task.get('mission_id', ''))] = handle
            self._active_goal_types[str(task.get('mission_id', ''))] = kind
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda done: self._action_result(done, task, kind))
        except Exception as exc:
            self._emit('failed', task, message=str(exc))

    def _action_feedback(self, message, task, kind):
        fb = message.feedback
        payload = {'executor': kind}
        if kind == 'patrol':
            payload.update(phase=fb.phase, reached_points=fb.reached_points, laps=fb.laps,
                           distance_to_target_m=fb.distance_to_target_m)
        else:
            payload.update(phase=fb.phase, reached=fb.reached,
                           distance_to_target_m=fb.distance_to_target_m)
        self._emit('feedback', task, feedback=payload)

    def _action_result(self, future, task, kind):
        try:
            wrapped = future.result()
            result = wrapped.result
            payload = {'executor': kind, 'success': result.success, 'message': result.message}
            if kind == 'patrol':
                payload.update(reached_points=result.reached_points, laps=result.laps)
            event = 'canceled' if wrapped.status == GoalStatus.STATUS_CANCELED else (
                'completed' if result.success else 'failed')
            self._emit(event, task, result=payload)
            self._active_goals.pop(str(task.get('mission_id', '')), None)
            self._active_goal_types.pop(str(task.get('mission_id', '')), None)
        except Exception as exc:
            self._emit('failed', task, message=str(exc))
            self._active_goals.pop(str(task.get('mission_id', '')), None)
            self._active_goal_types.pop(str(task.get('mission_id', '')), None)


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


if __name__ == '__main__':
    main()
