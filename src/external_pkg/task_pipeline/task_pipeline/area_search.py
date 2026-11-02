"""Pipeline rectangle exploration executor. Idle until ExecuteTask routes a request."""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from .area_search_core import (ExplorationMap, natural_key, rectangle_for_task,
                               task_signature, validate_task)

RUNNING = ('waiting_team', 'waiting_consensus',
           'planning_entry', 'entering', 'planning', 'executing')

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def transform_points(transform, points):
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform.transform.translation
    origin = np.array([t.x, t.y, t.z])
    return origin, points @ rotation.T + origin


class AreaSearch(Node):
    def __init__(self):
        super().__init__('area_search')
        self.ns = self.get_namespace().strip('/')
        if not self.ns or '/' in self.ns:
            raise ValueError('area_search needs a single robot namespace')
        self.declare_parameter('config_file', '')
        with open(self.get_parameter('config_file').value, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.frame = self.config.get('map_frame', '{namespace}/map').format(namespace=self.ns)
        self.base = self.config.get('base_frame', '{namespace}/base_link').format(namespace=self.ns)
        self.height_frame = self.config.get('height_reference_frame', self.base).format(namespace=self.ns)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.active_pub = self.create_publisher(Bool, 'area_search/active', LATCHED)
        self.status_pub = self.create_publisher(String, 'area_search/status', LATCHED)
        self.feedback_pub = self.create_publisher(String, 'task_feedback', 10)
        self.assignment_pub = self.create_publisher(String, 'area_search/assignment', LATCHED)
        self.region_pub = self.create_publisher(PolygonStamped, 'area_search/region', LATCHED)
        self.assigned_pub = self.create_publisher(PolygonStamped, 'area_search/assigned_region', LATCHED)
        self.map_pub = self.create_publisher(OccupancyGrid, 'area_search/map', LATCHED)
        self.path_pub = self.create_publisher(Path, 'area_search/path', 10)
        self.create_subscription(String, 'area_search/request', self.request, 10)
        self.create_subscription(Bool, 'stop_planner', self.stop, 10)
        self.create_service(Trigger, 'area_search/cancel', self.cancel_service)
        topics = self.config.get('topics', {})
        self.create_subscription(PointCloud2, topics.get('cloud', 'livox/lidar'), self.cloud, qos_profile_sensor_data)
        self.action = ActionClient(self, FollowPath, topics.get('follow_path', 'follow_path'))
        self.lifecycle = self.create_client(GetState, 'controller_server/get_state')
        self.manager = self.create_client(Trigger, 'lifecycle_manager_navigation/is_active')
        self.parameters = self.create_client(GetParameters, 'nav2_input_bridge/get_parameters')
        self.origin = None; self.origin_future = None; self.ready_future = None
        self.origin_requested_at = 0.0; self.ready_requested_at = 0.0
        self.controller_active = False; self.ready_at = 0.0
        self.manager_future = None; self.manager_requested_at = 0.0
        self.navigation_active = False; self.navigation_ready_at = 0.0
        self.nav_subs = {}; self.nav_seen = {}; self.proposal_subs = {}; self.proposals = {}
        self.task = None; self.grid = None; self.state = 'idle'; self.team = ()
        self.last_roster = (); self.roster_since = time.monotonic(); self.started = 0.0
        self.region = None; self.assigned = None; self.signature = ''
        self.goal_handle = None; self.goal_future = None; self.generation = 0
        self.path_purpose = 'search'
        self.replan_pending = False; self.active_path = []
        self.target = None; self.goal_started = 0.0; self.last_plan = 0.0
        self.last_proposal = 0.0
        self.pending_cloud = None; self.last_cloud_error = 'waiting for point cloud'
        self.last_cloud = 0.0; self.last_map_update = 0.0; self.last_map_publish = 0.0
        self.blacklist = {}; self.failures = 0; self.empty_count = 0
        self.ratio = 0.0; self.frontier_count = 0; self.last_empty_update = -1
        self.active_pub.publish(Bool(data=False))
        self.last_active_heartbeat = 0.0
        self.create_timer(0.1, self.process_cloud, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.publish_progress_feedback, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.report('idle', 'Waiting for pipeline ExecuteTask capability_id=area_search')

    def report(self, state, reason=''):
        changed = state != self.state
        self.state = state
        metrics = self.grid.area_metrics(self.assigned) if self.grid and self.assigned else {
            'searched_area_m2': 0.0, 'assigned_area_m2': 0.0, 'observed_ratio': 0.0}
        data = dict(state=state, reason=reason, robot_namespace=self.ns,
                    mission_id=self.task['mission_id'] if self.task else '',
                    team=list(self.team), **metrics,
                    frontier_clusters=self.frontier_count)
        if self.assigned:
            data['assigned_region'] = self.assigned.__dict__
        self.status_pub.publish(String(data=json.dumps(data)))
        if self.task:
            terminal_events = {'completed': 'completed', 'partial': 'partial', 'failed': 'failed',
                               'rejected': 'rejected', 'cancelled': 'canceled', 'paused': 'paused'}
            event = terminal_events.get(state, 'feedback')
            if state in ('cancelled', 'paused') and (self.goal_handle or self.goal_future):
                event = 'cancel_requested'
            self._publish_task_feedback(event, data)
        if changed or reason:
            self.get_logger().info(json.dumps(data, ensure_ascii=False))

    def _publish_task_feedback(self, event, status):
        data = {
            'event': event,
            'mission_id': status.get('mission_id', ''),
            'agent_id': status.get('agent_id', self.ns),
            'capability_id': status.get('capability_id', 'area_search'),
            'primitive_id': status.get('primitive_id', self.task.get('primitive_id', '') if self.task else ''),
            'executor': 'area_search',
            'phase': status.get('state', self.state),
            'reason': status.get('reason', ''),
            'searched_area_m2': float(status.get('searched_area_m2', 0.0)),
            'assigned_area_m2': float(status.get('assigned_area_m2', 0.0)),
            'observed_ratio': float(status.get('observed_ratio', 0.0)),
            'inferred_obstacle_area_m2': float(status.get('inferred_obstacle_area_m2', 0.0)),
            'direct_observed_area_m2': float(status.get('direct_observed_area_m2', 0.0)),
            'frontier_clusters': int(status.get('frontier_clusters', 0)),
        }
        if event in ('completed', 'partial', 'failed', 'canceled', 'paused', 'rejected'):
            data['result'] = {
                'success': event == 'completed',
                'state': data['phase'],
                'searched_area_m2': data['searched_area_m2'],
                'assigned_area_m2': data['assigned_area_m2'],
                'observed_ratio': data['observed_ratio'],
                'inferred_obstacle_area_m2': data['inferred_obstacle_area_m2'],
                'direct_observed_area_m2': data['direct_observed_area_m2'],
                'message': data['reason'],
            }
        self.feedback_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def publish_progress_feedback(self):
        if not self.task or self.state in ('idle', 'completed', 'partial', 'failed', 'rejected', 'cancelled', 'paused'):
            return
        metrics = self.grid.area_metrics(self.assigned) if self.grid and self.assigned else {
            'searched_area_m2': 0.0, 'assigned_area_m2': 0.0, 'observed_ratio': 0.0}
        self._publish_task_feedback('feedback', {
            'mission_id': self.task['mission_id'], 'state': self.state,
            'reason': 'area search progress', 'frontier_clusters': self.frontier_count,
            **metrics,
        })

    def request(self, msg):
        data = {}
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('request must be a JSON object')
            if data.get('capability_id') == 'cancel_area_search':
                self.pause('cancelled', 'pipeline cancellation'); return
            settings = json.loads(json.dumps(self.config))
            supplied = data['params']
            settings.update(supplied)
            settings['options'] = {**self.config.get('options', {}), **supplied.get('options', {})}
            task = validate_task(settings)
            if task['robot_namespace'] != self.ns or data.get('agent_id') != self.ns:
                raise ValueError('task namespace/agent does not match this executor')
            if self.state in RUNNING:
                raise ValueError('search already running; cancel it before submitting a new task')
            if self.goal_future is not None or self.goal_handle is not None:
                raise ValueError('previous FollowPath cancellation still pending')
            if task.get('topics', {}) != self.config.get('topics', {}):
                raise ValueError('task topics must match executor config_file; restart pipeline for topic changes')
            for key, fallback in [('map_frame', '{namespace}/map'), ('base_frame', '{namespace}/base_link')]:
                if task.get(key, fallback).format(namespace=self.ns) != self.config.get(key, fallback).format(namespace=self.ns):
                    raise ValueError('task frames must match executor config_file')
            self.task = task; self.grid = None; self.team = (); self.signature = ''
            self.region = None; self.assigned = None; self.ratio = 0; self.frontier_count = 0
            self.blacklist.clear(); self.failures = 0; self.empty_count = 0
            self.last_empty_update = -1; self.last_cloud = 0; self.last_map_update = 0
            self.pending_cloud = None; self.last_cloud_error = 'waiting for point cloud'
            self.last_roster = (); self.roster_since = time.monotonic(); self.started = self.roster_since
            self.path_purpose = 'search'; self.target = None
            self.replan_pending = False; self.active_path = []
            self.generation += 1
            self.publish_region_preview()
            self.report('waiting_team', 'Collecting live nav_run heartbeats; no motion yet')
        except (ValueError, KeyError, TypeError) as exc:
            # Rejected submissions must not mutate an already executing mission.
            self.get_logger().error(f'Rejected area search request: {exc}')
            params = data.get('params', {}) if isinstance(data, dict) else {}
            mission_id = data.get('mission_id', params.get('mission_id', '')) if isinstance(params, dict) else data.get('mission_id', '')
            self._publish_task_feedback('rejected', {
                'state': 'rejected', 'reason': str(exc), 'mission_id': mission_id,
                'agent_id': data.get('agent_id', self.ns) if isinstance(data, dict) else self.ns,
                'primitive_id': data.get('primitive_id', '') if isinstance(data, dict) else '',
                'searched_area_m2': 0.0, 'assigned_area_m2': 0.0,
                'observed_ratio': 0.0, 'frontier_clusters': 0,
            })

    def stop(self, msg):
        if msg.data and self.state in RUNNING:
            self.pause('paused', 'stop_planner: manual/target-detected stop; explicit resubmission required')

    def cancel_service(self, request, response):
        self.pause('cancelled', 'operator cancellation')
        response.success = True; response.message = 'Search cancelled; current owned FollowPath cancellation requested'
        return response

    def pause(self, state, reason):
        self.replan_pending = False
        self.generation += 1
        self.interruption_reason = reason
        self.pending_cloud = None
        self.report(state, reason)
        if self.goal_handle:
            handle = self.goal_handle
            future = handle.cancel_goal_async()
            # Keep ownership locked until this goal's terminal result is received.
            future.add_done_callback(lambda _: None)
        if self.goal_handle is None and self.goal_future is None:
            self.active_pub.publish(Bool(data=False))

    def discover(self, now):
        for topic, types in self.get_topic_names_and_types():
            if topic.endswith('/nav_run') and types == ['std_msgs/msg/Bool'] and topic.count('/') == 2:
                ns = topic.split('/')[1]
                if ns not in self.nav_subs:
                    # Volatile reader: demand fresh heartbeats, not a cached old true.
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                    self.nav_subs[ns] = self.create_subscription(Bool, topic,
                        lambda msg, n=ns: self.nav_seen.__setitem__(n, (msg.data, time.monotonic())), qos)
                    self.proposal_subs[ns] = self.create_subscription(String, f'/{ns}/area_search/assignment',
                        lambda msg, n=ns: self.receive_proposal(n, msg), LATCHED)
        timeout = self.task['options']['heartbeat_timeout_s'] if self.task else 4.0
        return tuple(sorted((ns for ns, (value, stamp) in self.nav_seen.items()
                             if value and now-stamp < timeout and self.count_publishers('/'+ns+'/nav_run') > 0), key=natural_key))

    def receive_proposal(self, ns, msg):
        try:
            data = json.loads(msg.data)
            self.proposals[ns] = (data, time.monotonic())
        except (ValueError, TypeError):
            pass

    def query_navigation(self, now):
        # DDS discovery can succeed while an individual startup RPC reply is
        # lost. Bound pending queries rather than waiting on one forever.
        for field, started, client in [('origin_future', 'origin_requested_at', self.parameters),
                                       ('ready_future', 'ready_requested_at', self.lifecycle),
                                       ('manager_future', 'manager_requested_at', self.manager)]:
            future = getattr(self, field)
            if future is not None and now-getattr(self, started) > 2.0:
                setattr(self, field, None)
                client.remove_pending_request(future)
                future.cancel()
        if self.origin is None and self.origin_future is None and self.parameters.service_is_ready():
            request = GetParameters.Request()
            request.names = ['reference_latitude', 'reference_longitude', 'reference_altitude', 'goal_output_frame']
            self.origin_future = self.parameters.call_async(request)
            self.origin_requested_at = now
            self.origin_future.add_done_callback(self.got_origin)
        if now-self.ready_requested_at > 1.0 and self.ready_future is None and self.lifecycle.service_is_ready():
            self.ready_requested_at = now
            self.ready_future = self.lifecycle.call_async(GetState.Request())
            self.ready_future.add_done_callback(self.got_ready)
        if now-self.manager_requested_at > 1.0 and self.manager_future is None and self.manager.service_is_ready():
            self.manager_requested_at = now
            self.manager_future = self.manager.call_async(Trigger.Request())
            self.manager_future.add_done_callback(self.got_navigation_ready)

    def got_navigation_ready(self, future):
        if self.manager_future is not future:
            return
        self.manager_future = None
        try:
            self.navigation_active = future.result().success
            if self.navigation_active:
                self.navigation_ready_at = time.monotonic()
        except (RuntimeError, AttributeError):
            self.navigation_active = False

    def got_origin(self, future):
        if self.origin_future is not future:
            return
        self.origin_future = None
        try:
            values = future.result().values
            if len(values) != 4 or any(v.type not in (2, 3) for v in values[:3]):
                raise ValueError('navigation origin parameters are unavailable')
            origin = tuple(v.double_value if v.type == 3 else float(v.integer_value) for v in values[:3])
            if not all(math.isfinite(x) for x in origin) or abs(origin[0]) > 90 or abs(origin[1]) > 180:
                raise ValueError('invalid navigation GPS origin')
            if values[3].string_value.lstrip('/') != self.frame:
                raise ValueError(f'navigation frame {values[3].string_value} != search frame {self.frame}')
            self.origin = origin
            self.publish_region_preview()
        except (ValueError, RuntimeError, AttributeError) as exc:
            self.get_logger().error(str(exc))

    def got_ready(self, future):
        if self.ready_future is not future:
            return
        self.ready_future = None
        try:
            self.controller_active = future.result().current_state.id == 3
            if self.controller_active:
                self.ready_at = time.monotonic()
        except (RuntimeError, AttributeError):
            self.controller_active = False

    def pose(self):
        tf = self.tf.lookup_transform(self.frame, self.base, Time())
        age = (self.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if age < -0.5 or age > self.task['options']['sensor_timeout_s']:
            raise ValueError('robot TF is stale or in the future')
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return t.x, t.y, yaw

    def cloud(self, msg):
        if self.grid is None or self.state not in RUNNING:
            return
        # Keep the selected scan until its exact-time TF arrives. Replacing it
        # with every newer scan would perpetually chase TF into the future.
        if self.pending_cloud is not None:
            return
        if time.monotonic()-self.last_map_update < self.task['options']['map_update_period_s']:
            return
        self.pending_cloud = msg

    def process_cloud(self):
        if self.grid is None or self.state not in RUNNING:
            self.pending_cloud = None
            return
        msg = self.pending_cloud
        if msg is None:
            return
        try:
            age = (self.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
            if age < -0.5 or age > self.task['options']['sensor_timeout_s']:
                self.last_cloud_error = f'point cloud timestamp out of range: age={age:.3f}s'
                self.pending_cloud = None
                return
            try:
                tf = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                height_tf = (self.tf.lookup_transform(
                    self.height_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                    if self.task['options'].get('use_imu_ground_tilt', True) else None)
            except TransformException as exc:
                self.last_cloud_error = 'waiting for point cloud TF: '+str(exc)
                return  # timer retries while TF callbacks continue to run
            points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
            if points:
                # Keep all hits for column minima; the map limits ray tracing.
                data = np.array([tuple(p) for p in points], dtype=float)
                data = data[np.all(np.isfinite(data), axis=1)]
                origin, endpoints = transform_points(tf, data)
                if height_tf is not None:
                    local_origin, local_points = transform_points(height_tf, data)
                    self.grid.integrate(tuple(origin), endpoints,
                                        origin_height=local_origin[2], endpoint_heights=local_points[:, 2])
                else:
                    self.grid.integrate(tuple(origin), endpoints)
            else:
                self.grid.updates += 1  # fresh empty scan, no free-space evidence
            self.last_cloud = time.monotonic(); self.last_map_update = self.last_cloud
            self.last_cloud_error = ''
            self.pending_cloud = None
            x, y, yaw = self.pose(); self.grid.seed_footprint(x, y, yaw)
        except (TransformException, ValueError, TypeError, IndexError) as exc:
            self.last_cloud_error = 'point cloud processing failed: '+str(exc)
            self.pending_cloud = None
            self.get_logger().debug('Skipping cloud: '+str(exc))

    def publish_region_preview(self):
        if self.task and (self.task['coordinate_mode'] == 'enu' or self.origin is not None):
            self.region_pub.publish(self.polygon(rectangle_for_task(self.task, self.origin)))

    def polygon(self, rectangle):
        msg = PolygonStamped(); msg.header.frame_id = self.frame; msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in rectangle.corners()]
        return msg

    def publish_map(self):
        msg = OccupancyGrid(); msg.header.frame_id = self.frame; msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.res; msg.info.width = self.grid.w; msg.info.height = self.grid.h
        msg.info.origin.position.x = self.grid.ox; msg.info.origin.position.y = self.grid.oy
        msg.info.origin.orientation.w = 1.0; msg.data = self.grid.planar().flatten().tolist()
        self.map_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if (self.state in ('planning_entry', 'entering', 'planning', 'executing') or self.goal_handle or self.goal_future) and now-self.last_active_heartbeat > .8:
            self.active_pub.publish(Bool(data=True)); self.last_active_heartbeat = now
        roster = self.discover(now); self.query_navigation(now)
        if self.state not in RUNNING:
            return
        options = self.task['options']
        if roster != self.last_roster:
            self.last_roster = roster; self.roster_since = now
        if self.team and roster != self.team:
            self.pause('paused', 'nav_run membership changed; cancel/reissue the same task to the new team'); return
        if self.state == 'waiting_team':
            if now-self.started > options['team_timeout_s']:
                self.pause('failed', 'timeout waiting for stable nav_run team/navigation origin'); return
            if self.ns not in roster or now-self.roster_since < options['team_stable_s'] or self.origin is None:
                return
            self.team = roster
            self.region = rectangle_for_task(self.task, self.origin)
            self.assigned = self.region.partition(roster, self.ns)
            if self.assigned.length <= 2*options['robot_radius_m']:
                self.pause('failed', 'too many robots: assigned strip narrower than safety footprint'); return
            try:
                self.grid = ExplorationMap(self.region, options)
            except ValueError as exc:
                self.pause('failed', str(exc)); return
            self.signature = task_signature(self.task, self.origin, roster)
            self.assignment_pub.publish(String(data=json.dumps(dict(
                signature=self.signature, team=list(roster), mission_id=self.task['mission_id'], origin=self.origin))))
            self.assigned_pub.publish(self.polygon(self.assigned))
            self.report('waiting_consensus', 'Waiting for every nav_run participant to submit the same region task')
        if self.signature and now-self.last_proposal > 1.0:
            self.assignment_pub.publish(String(data=json.dumps(dict(signature=self.signature,
                team=list(self.team), mission_id=self.task['mission_id'], origin=self.origin))))
            self.last_proposal = now
        if self.state == 'waiting_consensus':
            if now-self.started > options['team_timeout_s']:
                self.pause('failed', 'participants did not agree on mission/region/origin/roster'); return
            if any(self.proposals.get(n, ({}, 0))[0].get('signature') != self.signature or self.proposals[n][1] < self.started for n in self.team):
                return
            try:
                pose = self.pose()
            except (TransformException, ValueError):
                return
            if (not self.controller_active or not self.action.server_is_ready() or now-self.ready_at > 3
                    or not self.navigation_active or now-self.navigation_ready_at > 3):
                return
            if self.last_cloud == 0 or now-self.last_cloud > options['sensor_timeout_s']:
                return
            next_state = 'planning' if self.region.contains(*pose[:2]) else 'planning_entry'
            self.start_navigation(pose, next_state)
        else:
            if not self.controller_active or now-self.ready_at > 3:
                self.pause('paused', 'controller_server is not freshly confirmed active'); return
            if not self.navigation_active or now-self.navigation_ready_at > 3:
                self.pause('paused', 'navigation lifecycle manager is not freshly confirmed active'); return
            try:
                pose = self.pose()
            except (TransformException, ValueError) as exc:
                self.pause('paused', str(exc)); return
            if now-self.last_cloud > options['sensor_timeout_s']:
                self.pause('paused', 'sensor data timeout: '+
                           (self.last_cloud_error or 'no recently processed point cloud')); return
            if self.goal_handle and now-self.goal_started > options['goal_timeout_s']:
                self.pause('failed', 'FollowPath execution timeout'); return
        if self.grid and now-self.last_map_publish >= options['map_update_period_s']:
            self.publish_map(); self.last_map_publish = now
        if (self.state == 'entering' and self.goal_handle is not None and
                self.goal_future is None and not self.replan_pending and
                self.region.contains(*pose[:2])):
            self.request_replan('robot entered search region; select frontier while moving')
            return
        if (self.state == 'executing' and self.goal_handle is not None and
                self.goal_future is None and not self.replan_pending and
                now-self.last_plan >= options['planning_period_s']):
            self.last_plan = now
            self.blacklist = {p: t for p, t in self.blacklist.items() if t > now}
            path, self.ratio, self.frontier_count, target = self.grid.plan(
                pose, self.assigned, self.blacklist, current_target=self.target)
            if self.ratio >= options['completion_observed_ratio']:
                self.pause('completed', 'assigned-region observed-ratio target reached'); return
            if len(path) < 2 or target != self.target:
                self.request_replan('current frontier lost value or a better frontier is available')
            elif not self.grid.path_is_valid(pose, self.active_path):
                self.request_replan('current path invalidated by updated observations')
            elif (self.active_path and
                    math.dist(pose[:2], self.active_path[-1]) <= options['path_refresh_distance_m'] and
                    math.dist(path[-1], self.active_path[-1]) >= options['min_progress_distance_m']):
                # The selected viewpoint may be farther away than the clipped
                # local segment. Extend it before reaching the segment endpoint.
                self.request_replan('extend rolling path before reaching current segment endpoint')
            return
        if self.state == 'planning_entry' and now-self.last_plan >= options['planning_period_s'] and self.goal_future is None and self.goal_handle is None:
            self.last_plan = now
            # The closest boundary direction, with footprint clearance at the
            # endpoint. Entry uses the tighter checker from navigation YAML.
            target = self.region.closest_point(*pose[:2],
                margin=options['robot_radius_m']+options['resolution_m'])
            count = max(1, math.ceil(math.dist(pose[:2], target)/options['resolution_m']))
            points = [(pose[0]+(target[0]-pose[0])*i/count,
                       pose[1]+(target[1]-pose[1])*i/count) for i in range(count+1)]
            self.path_purpose = 'entry'; self.target = None
            self.send_path(points)
        if self.state == 'planning' and now-self.last_plan >= options['planning_period_s'] and self.goal_future is None and self.goal_handle is None:
            self.last_plan = now
            self.blacklist = {p: t for p, t in self.blacklist.items() if t > now}
            path, self.ratio, self.frontier_count, target = self.grid.plan(pose, self.assigned, self.blacklist)
            if self.ratio >= options['completion_observed_ratio']:
                self.pause('completed', 'assigned-region observed-ratio target reached'); return
            if len(path) < 2:
                diagnostics = getattr(self.grid, 'plan_diagnostics', {})
                detail = json.dumps(diagnostics, ensure_ascii=False, sort_keys=True)
                if self.grid.updates != self.last_empty_update:
                    self.empty_count += 1; self.last_empty_update = self.grid.updates
                if self.empty_count >= options['no_frontier_confirmations']:
                    self.pause('partial', 'no reachable usable frontier; remaining unknown is not reported as covered; '
                               'frontier_diagnostics='+detail)
                else:
                    self.report('planning', 'waiting for more observations / reachable frontier; '
                                'frontier_diagnostics='+detail)
                return
            self.path_purpose = 'search'
            self.empty_count = 0; self.target = target
            self.send_path(path)

    def request_replan(self, reason):
        # Retain search ownership; submit no replacement until the old Action
        # reaches its terminal result. A timer then plans from the fresh pose.
        self.replan_pending = True
        self.report('executing', 'replanning: '+reason+'; cancelling current segment')
        try:
            handle = self.goal_handle
            future = handle.cancel_goal_async()
            future.add_done_callback(lambda response: self.replan_cancel_response(response, handle))
        except RuntimeError as exc:
            self.replan_pending = False
            self.pause('failed', 'cannot cancel segment for replanning: '+str(exc))

    def replan_cancel_response(self, future, handle):
        if not self.replan_pending or self.goal_handle is not handle:
            return
        try:
            accepted = bool(future.result().goals_canceling)
        except (RuntimeError, AttributeError) as exc:
            self.replan_pending = False
            self.pause('failed', 'replanning cancellation response failed: '+str(exc)); return
        if not accepted and self.goal_handle is not None:
            self.replan_pending = False
            self.report('executing', 'replanning cancellation not accepted; retaining current segment')

    def start_navigation(self, pose, next_state):
        # The rectangle is exploration metadata, never a costmap obstacle.
        self.region_pub.publish(self.polygon(self.region))
        self.active_pub.publish(Bool(data=True))
        # Allow the straight-planner ownership gate to cancel its old goal.
        self.last_plan = time.monotonic()
        self.report(next_state, 'Team agreed; entering search region' if next_state == 'planning_entry'
                    else 'Team agreed; selecting an observed frontier')

    def send_path(self, points):
        self.active_path = list(points)
        path = Path(); path.header.frame_id = self.frame; path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            msg = PoseStamped(); msg.header = path.header; msg.pose.position.x = x; msg.pose.position.y = y
            a, b = (points[i], points[i+1]) if i+1 < len(points) else (points[i-1], points[i])
            yaw = math.atan2(b[1]-a[1], b[0]-a[0]); msg.pose.orientation.z = math.sin(yaw/2); msg.pose.orientation.w = math.cos(yaw/2)
            path.poses.append(msg)
        goal = FollowPath.Goal(); goal.path = path
        goal.controller_id = self.config.get('controller_id', 'FollowPath')
        goal.goal_checker_id = (self.config.get('entry_goal_checker_id', 'area_entry_goal_checker')
                                if self.path_purpose == 'entry' else
                                self.config.get('goal_checker_id', 'general_goal_checker'))
        self.path_pub.publish(path)
        if (self.path_purpose == 'search' and
                math.dist(points[0], points[-1]) < self.task['options']['min_goal_distance_m']):
            goal.goal_checker_id = self.config.get('short_goal_checker_id', 'area_entry_goal_checker')
        generation = self.generation
        self.goal_started = time.monotonic()
        self.goal_future = self.action.send_goal_async(goal)
        self.goal_future.add_done_callback(lambda future: self.goal_response(future, generation))
        self.report('entering' if self.path_purpose == 'entry' else 'executing',
                    'FollowPath submitted; awaiting acceptance/result')

    def goal_response(self, future, generation):
        self.goal_future = None
        try:
            handle = future.result()
        except (RuntimeError, AttributeError) as exc:
            if generation != self.generation:
                self._finish_interrupted_action(str(exc))
            else:
                self.pause('failed', str(exc))
            return
        if not handle.accepted:
            if generation != self.generation:
                self.active_pub.publish(Bool(data=False))
                self._finish_interrupted_action('FollowPath goal was not accepted')
            if generation == self.generation:
                self.path_failed('FollowPath rejected')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda result: self.goal_result(result, generation, handle))
        if generation != self.generation:
            handle.cancel_goal_async()

    def goal_result(self, future, generation, handle):
        if self.goal_handle is handle:
            self.goal_handle = None
        if generation != self.generation:
            self.active_pub.publish(Bool(data=False))
            self._finish_interrupted_action('FollowPath cancellation reached terminal state')
            return
        try:
            status = future.result().status
        except (RuntimeError, AttributeError) as exc:
            self.pause('failed', str(exc)); return
        if getattr(self, 'replan_pending', False):
            self.replan_pending = False
            self.last_plan = 0.0
            self.report('planning', 'previous segment stopped; reselect frontier from fresh observations')
            return
        if status == 4:
            self.failures = 0
            if self.path_purpose == 'entry':
                try:
                    pose = self.pose()
                except (TransformException, ValueError) as exc:
                    self.pause('paused', str(exc)); return
                if not self.region.contains(*pose[:2]):
                    self.pause('failed', 'entry Action succeeded but robot center is still outside region'); return
                self.path_purpose = 'search'
                self.last_plan = 0.0
                self.report('planning', 'Entered search region; select frontier from observations')
            else:
                self.report('planning', 'current segment succeeded; update frontier from observations')
        else:
            self.path_failed(f'FollowPath terminal status={status}')

    def _finish_interrupted_action(self, reason):
        if self.state in ('cancelled', 'paused'):
            cause = getattr(self, 'interruption_reason', '')
            self.report(self.state, cause+'; '+reason if cause else reason)

    def path_failed(self, reason):
        self.failures += 1
        if self.target:
            self.blacklist[self.target] = time.monotonic()+self.task['options']['frontier_cooldown_s']
        if self.failures >= self.task['options']['max_path_failures']:
            self.pause('failed', reason)
        else:
            self.report('planning_entry' if self.path_purpose == 'entry' else 'planning',
                        reason+'; cooling down target and replanning')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AreaSearch(); rclpy.spin(node)
    except KeyboardInterrupt:
        if node and rclpy.ok():
            node.pause('cancelled', 'executor interrupted')
            until = time.monotonic()+2
            while rclpy.ok() and time.monotonic() < until and (node.goal_handle or node.goal_future):
                rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
