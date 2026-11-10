"""area_search_co_search: task ownership and Nav2 execution, no exploration policy."""
import copy
import json
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import tempfile
import time

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from task_interfaces.action import AreaSearch
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from geometry_msgs.msg import PolygonStamped, Point32
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener
from .task_geometry import validate_task, rectangle_for_task
from .configuration import coverage_threshold, search_timeout_coefficient, engine_parameters, native_remappings

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
TERMINAL = {'completed', 'partial', 'failed', 'cancelled', 'paused', 'rejected'}


class CoSearchExecutor(Node):
    def __init__(self):
        super().__init__('area_search_co_search')
        self.ns = self.get_namespace().strip('/')
        if not self.ns or '/' in self.ns:
            raise ValueError('area_search_co_search requires a single robot namespace')
        self.declare_parameter('config_file', '')
        self.config = yaml.safe_load(FilePath(self.get_parameter('config_file').value).read_text())
        self.frame = self.config.get('map_frame', '{namespace}/map').format(namespace=self.ns)
        self.base = self.config.get('base_frame', '{namespace}/base_link').format(namespace=self.ns)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.tf_health_sample = None
        self.tf_health_log = 0.
        self.tf_health_bad = False
        self.tf_health_started = time.monotonic()
        self.active_pub = self.create_publisher(Bool, 'area_search/active', LATCHED)
        self.status_pub = self.create_publisher(String, 'area_search/status', LATCHED)
        self.region_pub = self.create_publisher(PolygonStamped, 'area_search/region', LATCHED)
        self.assigned_pub = self.create_publisher(PolygonStamped, 'area_search/assigned_region', LATCHED)
        self.map_pub = self.create_publisher(OccupancyGrid, 'area_search/map', LATCHED)
        self.path_pub = self.create_publisher(Path, 'area_search/path', 10)
        self.enable_pub = self.create_publisher(Bool, 'co_search/enable', 1)
        self.entry_failure_pub = self.create_publisher(Bool, 'co_search/entry_failed', 1)
        self.entry_waiting = False
        self.primitive_handle = None
        self.primitive_future = None
        self.primitive_busy = False
        self.primitive_server = ActionServer(self, AreaSearch, 'area_search', self.execute_primitive,
            goal_callback=self.primitive_goal, cancel_callback=self.primitive_cancel)
        self.create_subscription(Bool, 'stop_planner', lambda msg: self.stop('cancelled', 'stop_planner') if msg.data else None, 10)
        self.create_service(Trigger, 'area_search/cancel', self.cancel)
        self.suspend_requested = False
        self.suspended_since = None
        self.pause_future = None
        self.path_min_stamp_ns = 0
        self.create_service(SetBool, 'area_search/pause', self.pause,
                            callback_group=ReentrantCallbackGroup())
        self.create_subscription(Path, 'co_search/path', self.path, 1)
        self.create_subscription(String, 'co_search/engine_state', self.engine_state, LATCHED)
        self.create_subscription(OccupancyGrid, 'co_search/native/sdf_map/occupancy_all', self.map, qos_profile_sensor_data)
        self.action = ActionClient(self, FollowPath, 'follow_path')
        self.controller = self.create_client(GetState, 'controller_server/get_state')
        self.manager = self.create_client(Trigger, 'lifecycle_manager_navigation/is_active')
        self.origin_client = self.create_client(GetParameters, 'nav2_input_bridge/get_parameters')
        self.origin = None; self.ready = {}; self.calls = {}
        self.create_subscription(NavSatFix, 'navigation/gps_origin', self.update_origin, LATCHED)
        self.task = None; self.region = None; self.state = 'idle'; self.process = None
        self.temp = None; self.log = None; self.start_time = 0.; self.last_feedback = 0.
        self.metrics = {}; self.engine = {}; self.enabled = False
        self.coverage_threshold = 0.
        self.search_started = None
        self.search_time_limit = 0.
        self.handles = {}; self.pending = {}; self.sequence = 0; self.generation = 0
        self.finishing = None; self.last_engine = 0.
        self.exhaustion_count = 0; self.exhaustion_cloud = None
        self.create_timer(.2, self.tick)
        self.active_pub.publish(Bool(data=False))
        self.report('idle', 'area_search_co_search ready; native single-robot search backend')

    def report(self, state, reason=''):
        self.state = state
        data = dict(state=state, reason=reason, search_controller='co_search', executor='area_search_co_search',
                    robot_namespace=self.ns, mission_id=self.task.get('mission_id', '') if self.task else '',
                    engine_phase=self.engine.get('phase', ''),
                    entry_target_map=self.engine.get('entry_target_map'),
                    frontier_clusters=int(self.engine.get('frontier_clusters', 0)), **self.metrics)
        self.status_pub.publish(String(data=json.dumps(data)))
        if self.primitive_handle:
            self.primitive_handle.publish_feedback(AreaSearch.Feedback(
                progress=(100. if state == 'completed' or
                          (self.finishing and self.finishing[0] == 'completed') else
                          min(99., 100. * float(self.metrics.get('observed_ratio', 0.)))), phase=state))
            if state in TERMINAL and self.primitive_future and not self.primitive_future.done():
                self.primitive_future.set_result((state, reason))
        if reason: self.get_logger().info(json.dumps(data, ensure_ascii=False))

    def primitive_goal(self, request):
        a = request.area
        values = (a.center.latitude, a.center.longitude, a.center.altitude,
                  a.length_m, a.width_m, a.yaw_deg)
        if (self.primitive_busy or not self.navigation_ready() or self.origin is None or
                not request.task_id or not all(math.isfinite(v) for v in values) or
                abs(a.center.latitude) > 90 or abs(a.center.longitude) > 180 or
                min(a.length_m, a.width_m) < 1.):
            return GoalResponse.REJECT
        self.primitive_busy = True
        return GoalResponse.ACCEPT

    def primitive_cancel(self, handle):
        self.stop('cancelled', 'Action cancellation')
        return CancelResponse.ACCEPT

    async def execute_primitive(self, handle):
        self.primitive_handle = handle
        self.primitive_future = Future()
        a = handle.request.area
        task = copy.deepcopy(self.config)
        task.update(robot_namespace=self.ns, coordinate_mode='gps', mission_id=handle.request.task_id,
                    center_gps=dict(latitude=a.center.latitude, longitude=a.center.longitude,
                                    altitude=a.center.altitude),
                    rectangle=dict(length_m=a.length_m, width_m=a.width_m, yaw_deg=a.yaw_deg))
        try:
            self.request(String(data=json.dumps(dict(agent_id=self.ns, capability_id='area_search',
                mission_id=handle.request.task_id, primitive_id=handle.request.task_id, params=task))))
            if handle.is_cancel_requested:
                self.stop('cancelled', 'Action canceled before execution')
            state, reason = await self.primitive_future
            if handle.is_cancel_requested:
                handle.canceled()
            elif state == 'completed':
                handle.succeed()
            else:
                handle.abort()
            return AreaSearch.Result(success=state == 'completed', message=reason)
        finally:
            self.primitive_handle = None
            self.primitive_future = None
            self.primitive_busy = False

    def navigation_ready(self):
        now = time.monotonic()
        return self.action.server_is_ready() and all(self.ready.get(key, (False, 0))[0] and
            now-self.ready[key][1] < 3. for key in ('controller', 'manager'))

    def navigation_readiness_detail(self):
        now = time.monotonic()
        parts = ['follow_path_action=%s' % self.action.server_is_ready()]
        for key in ('controller', 'manager'):
            ready, stamp = self.ready.get(key, (False, 0.))
            age = now-stamp if stamp else float('inf')
            parts.append('%s_ready=%s_age_s=%.2f' % (key, ready, age))
        return ' '.join(parts)

    def update_origin(self, message):
        values = (message.latitude, message.longitude, message.altitude)
        if (message.status.status < 0 or not all(math.isfinite(v) for v in values) or
                abs(values[0]) > 90. or abs(values[1]) > 180.):
            return
        self.origin = values  # Existing region/path remain ENU; used for future GPS task conversion.

    def poll_readiness(self):
        now = time.monotonic()
        for key, client, request in (('controller', self.controller, GetState.Request()),
                                      ('manager', self.manager, Trigger.Request())):
            if key in self.calls:
                future, since = self.calls[key]
                if now-since > 3.:
                    self.calls.pop(key); self.ready[key] = (False, now)
                continue
            if not client.service_is_ready():
                self.ready[key] = (False, now); continue
            if now-self.ready.get(key, (False, 0))[1] < 1.: continue
            future = client.call_async(request); self.calls[key] = (future, now)
            def done(result, name=key):
                if self.calls.get(name, (None,))[0] is not result: return
                self.calls.pop(name, None)
                try:
                    response = result.result()
                    ok = response.current_state.id == 3 if name == 'controller' else response.success
                except Exception: ok = False
                self.ready[name] = (ok, time.monotonic())
            future.add_done_callback(done)
        if self.origin is None and 'origin' not in self.calls and self.origin_client.service_is_ready():
            request = GetParameters.Request(); request.names = ['reference_latitude', 'reference_longitude', 'reference_altitude']
            future = self.origin_client.call_async(request); self.calls['origin'] = (future, now)
            def got_origin(result):
                self.calls.pop('origin', None)
                try:
                    values = result.result().values
                    if self.origin is None and len(values) == 3 and all(v.type in (2, 3) for v in values):
                        self.origin = tuple(v.double_value if v.type == 3 else float(v.integer_value) for v in values)
                except Exception: pass
            future.add_done_callback(got_origin)

    def request(self, message):
        try:
            request = json.loads(message.data)
            if request.get('agent_id') != self.ns: raise ValueError('agent_id must match robot namespace')
            if request.get('capability_id') == 'cancel_area_search':
                self.stop('cancelled', 'pipeline cancellation'); return
            if self.task and self.state not in TERMINAL or self.handles or self.pending or self.finishing:
                raise ValueError('area_search_co_search is already active or stopping')
            if not self.navigation_ready(): raise ValueError('controller and navigation lifecycle manager must be active')
            task = validate_task(request.get('params', {}))
            threshold = coverage_threshold(task)
            coefficient = search_timeout_coefficient(task)
            if task['robot_namespace'] != self.ns: raise ValueError('task namespace mismatch')
            if task.get('search_controller', self.config.get('search_controller')) != 'co_search':
                raise ValueError('this executor requires search_controller: co_search')
            task.update(mission_id=request.get('mission_id', task.get('mission_id', '')),
                        primitive_id=request.get('primitive_id', 'area_search_001'))
            if task['coordinate_mode'] == 'gps' and self.origin is None:
                raise ValueError('navigation GPS origin is not ready')
            region = rectangle_for_task(task, self.origin)
            pose = self.tf.lookup_transform(self.frame, self.base, Time())
            age, detail = self.tf_diagnostics(pose, task['options']['sensor_timeout_s'])
            self.get_logger().info('Area search startup TF: '+detail)
            if age < -.5 or age > task['options']['sensor_timeout_s']:
                raise ValueError('robot TF is stale: '+detail)
            # Only the fully ported native search engine owns area navigation.
            self.cleanup_process()
            self.generation += 1; self.sequence = 0; self.engine = {}; self.metrics = {}
            self.task = task; self.region = region; self.enabled = False
            self.coverage_threshold = threshold
            self.search_started = None
            self.search_time_limit = coefficient * region.length * region.width
            self.entry_waiting = False
            self.exhaustion_count = 0; self.exhaustion_cloud = None
            self.start_time = time.monotonic(); self.last_engine = self.start_time
            self.temp = tempfile.TemporaryDirectory(prefix='co_search_task_')
            share = get_package_share_directory('co_search_nav'); prefix = get_package_prefix('co_search_nav')
            parameters = engine_parameters(task, region, self.frame, self.temp.name, share, prefix,
                (pose.transform.translation.x, pose.transform.translation.y))
            parameters['use_sim_time'] = bool(self.get_parameter('use_sim_time').value)
            path = FilePath(self.temp.name)/'parameters.yaml'
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            command = [str(FilePath(prefix)/'lib/co_search_nav/search_engine'), '--ros-args',
                       '-r', '__ns:=/'+self.ns, '--params-file', str(path)]
            for source, target in native_remappings(self.ns): command += ['-r', source+':='+target]
            log_path = FilePath(self.temp.name)/'engine.log'
            self.log = log_path.open('w')
            self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            self.get_logger().info('native engine log: '+str(log_path))
            self.publish_region()
            self.active_pub.publish(Bool(data=True)); self.report('starting', 'starting fully ported co-search mapping, planning and optimization')
        except Exception as exc:
            self.get_logger().error(str(exc))
            if self.primitive_future and not self.primitive_future.done():
                self.primitive_future.set_result(('rejected', str(exc)))

    def publish_region(self):
        msg = PolygonStamped(); msg.header.frame_id = self.frame; msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.) for x, y in self.region.corners()]
        self.region_pub.publish(msg); self.assigned_pub.publish(msg)

    def map(self, message):
        if not self.task or self.state in TERMINAL or self.finishing or self.region is None: return
        res = message.info.resolution
        if res <= 0 or not message.info.width or not message.info.height: return
        grid = np.asarray(message.data).reshape(message.info.height, message.info.width)
        xx = message.info.origin.position.x+(np.arange(message.info.width)+.5)*res
        yy = message.info.origin.position.y+(np.arange(message.info.height)+.5)*res
        own = (np.abs(yy[:, None]) < self.region.width/2) & (np.abs(xx[None, :]) < self.region.length/2)
        # The native publisher includes map storage beyond its exploration box.
        # Publish the task rectangle only, keeping measured cell values intact.
        xs = np.flatnonzero(np.any(own, axis=0)); ys = np.flatnonzero(np.any(own, axis=1))
        if not len(xs) or not len(ys): return
        x0, x1 = int(xs[0]), int(xs[-1])+1; y0, y1 = int(ys[0]), int(ys[-1])+1
        msg = copy.deepcopy(message); msg.header.frame_id = self.frame
        msg.info.width = x1-x0; msg.info.height = y1-y0
        msg.data = grid[y0:y1, x0:x1].ravel().tolist()
        x, y = self.region.world(message.info.origin.position.x+x0*res,
                                 message.info.origin.position.y+y0*res)
        msg.info.origin.position.x = x; msg.info.origin.position.y = y
        msg.info.origin.orientation.z = math.sin(self.region.yaw/2); msg.info.origin.orientation.w = math.cos(self.region.yaw/2)
        self.map_pub.publish(msg)
        observed = int(np.count_nonzero(own & (grid >= 0)))
        # Use the exact same cell-centre mask for numerator and denominator,
        # including rectangles whose dimensions are not multiples of resolution.
        total = max(1, int(np.count_nonzero(own)))
        self.metrics = dict(searched_area_m2=observed*res*res, assigned_area_m2=total*res*res,
                            observed_ratio=min(1., observed/total), direct_observed_area_m2=observed*res*res,
                            inferred_obstacle_area_m2=0., coverage_threshold=self.coverage_threshold)

    def check_coverage_completion(self):
        if (not self.enabled or self.finishing or self.suspend_requested or
                self.search_started is None or self.coverage_threshold <= 0):
            return
        elapsed = time.monotonic() - self.search_started
        self.metrics.update(search_elapsed_s=elapsed, search_time_limit_s=self.search_time_limit)
        ratio = self.metrics.get('observed_ratio', 0.)
        if elapsed > self.search_time_limit and ratio >= self.coverage_threshold:
            self.stop('completed', 'search time %.2fs > %.2fs and observed coverage %.2f%% >= %.2f%%' %
                      (elapsed, self.search_time_limit, 100*ratio, 100*self.coverage_threshold))

    def engine_state(self, message):
        if not self.task or self.state in TERMINAL or self.finishing: return
        try: self.engine = json.loads(message.data)
        except ValueError: return
        self.last_engine = time.monotonic()
        if self.suspend_requested:
            return  # Keep the native map alive without re-enabling exploration.
        phase = self.engine.get('phase')
        if phase == 'entry_failed':
            self.stop('failed', self.engine.get('reason', 'no reachable area entry'))
            return
        if phase in ('entry_waiting', 'entry_retry'):
            if not self.entry_waiting:
                self.entry_waiting = True
                self.path_min_stamp_ns = self.get_clock().now().nanoseconds
                for handle in self.handles.values():
                    handle.cancel_goal_async()
                self.report('planning', self.engine.get('reason', 'selecting another area entry'))
            return
        if phase != 'exhausted':
            self.exhaustion_count = 0; self.exhaustion_cloud = None
        if phase == 'ready' and not self.enabled and self.navigation_ready():
            if self.search_started is None:
                self.search_started = time.monotonic()
            self.enabled = True; self.enable_pub.publish(Bool(data=True)); self.report('planning', 'native map ready; enabling single-robot exploration')
        elif phase == 'exhausted' and self.enabled and self.navigation_ready():
            clouds = self.engine.get('clouds', 0)
            if self.engine.get('reachable_frontiers', 0) > 0:
                self.exhaustion_count = 0
            elif clouds and clouds != self.exhaustion_cloud:
                self.exhaustion_cloud = clouds; self.exhaustion_count += 1
            confirmations = self.task.get('co_search', {}).get('exhaustion_confirmations', 3)
            if self.coverage_threshold <= 0 and self.exhaustion_count >= max(1, int(confirmations)) and self.metrics:
                self.stop('completed', 'native co-search confirmed all reachable frontiers exhausted; returning actual coverage')
        elif phase == 'waiting_sensor' and self.enabled:
            self.stop('paused', self.engine.get('reason', 'sensor timeout'))

    def path(self, message):
        if (not self.task or not self.enabled or self.finishing or self.state in TERMINAL or
                self.suspend_requested): return
        if Time.from_msg(message.header.stamp).nanoseconds < self.path_min_stamp_ns:
            return  # Do not replay a queued pre-suspension path after the robot moved.
        if not self.navigation_ready(): self.stop('paused', 'navigation is no longer active'); return
        if message.header.frame_id != self.frame or len(message.poses) < 2:
            self.stop('failed', 'invalid search path frame/length'); return
        if any(not math.isfinite(v) for p in message.poses for v in (p.pose.position.x, p.pose.position.y)):
            self.stop('failed', 'nonfinite search path'); return
        self.path_pub.publish(message)
        self.entry_waiting = False
        self.sequence += 1; sequence = self.sequence; generation = self.generation
        goal = FollowPath.Goal(); goal.path = message; goal.controller_id = 'FollowPath'
        goal.goal_checker_id = self.task.get('goal_checker_id', self.config.get('goal_checker_id', 'general_goal_checker'))
        future = self.action.send_goal_async(goal); self.pending[sequence] = future
        future.add_done_callback(lambda result: self.accepted(result, generation, sequence))

    def accepted(self, future, generation, sequence):
        self.pending.pop(sequence, None)
        try: handle = future.result()
        except Exception as exc: self.stop('failed', 'FollowPath request failed: '+str(exc)); return
        if not handle.accepted:
            if not self.finishing and generation == self.generation and sequence == self.sequence:
                self.stop('failed', 'FollowPath rejected search path')
            self.finish_pause_if_stopped()
            return
        self.handles[id(handle)] = handle
        handle.get_result_async().add_done_callback(lambda result: self.result(result, handle, generation, sequence))
        if self.finishing or generation != self.generation or self.entry_waiting:
            handle.cancel_goal_async()
        elif sequence == self.sequence:
            self.report('executing', 'native optimized B-spline -> existing FollowPath controller')
        self.finish_pause_if_stopped()

    def result(self, future, handle, generation, sequence):
        self.handles.pop(id(handle), None)
        if self.finishing: self.finish_if_stopped(); return
        self.finish_pause_if_stopped()
        if generation != self.generation or sequence != self.sequence: return
        try:
            result = future.result()
            if result.status == 4: self.report('planning', 'segment reached; native exploration continues')
            elif result.status == 6 and self.engine.get('phase') == 'entering' and self.navigation_ready():
                self.entry_waiting = True
                self.path_min_stamp_ns = self.get_clock().now().nanoseconds
                self.entry_failure_pub.publish(Bool(data=True))
                self.report('planning', 'entry FollowPath aborted; requesting an alternative entry')
            elif result.status != 5: self.stop('failed', 'FollowPath terminal status='+str(result.status))
        except Exception as exc: self.stop('failed', str(exc))

    def cancel(self, request, response):
        self.stop('cancelled', 'cancel service'); response.success = True; response.message = 'cancellation requested'; return response

    async def pause(self, request, response):
        """Keep the map and AreaSearch Action, releasing only FollowPath ownership."""
        if not self.task or self.state in TERMINAL or self.finishing:
            response.success = False
            response.message = 'No resumable area search is active'
            return response
        if not request.data:
            if self.state != 'suspended' or self.handles or self.pending:
                response.success = False
                response.message = 'Search has not finished suspending'
                return response
            self.start_time += time.monotonic() - self.suspended_since
            if self.search_started is not None:
                self.search_started += time.monotonic() - self.suspended_since
            self.suspend_requested = False
            self.suspended_since = None
            self.pause_future = None
            self.path_min_stamp_ns = self.get_clock().now().nanoseconds
            self.enabled = True
            self.enable_pub.publish(Bool(data=True))
            self.report('planning', 'resuming retained native search map')
            response.success = True
            response.message = 'Search resumed'
            return response
        if not self.suspend_requested:
            self.suspend_requested = True
            self.suspended_since = time.monotonic()
            self.pause_future = Future()
            self.generation += 1
            self.enabled = False
            self.enable_pub.publish(Bool(data=False))
            for handle in list(self.handles.values()):
                handle.cancel_goal_async()
            self.report('suspending', 'waiting for owned FollowPath goals to stop')
            self.finish_pause_if_stopped()
        response.success = await self.pause_future
        response.message = 'Search suspended; FollowPath released' if response.success else 'Search ended while suspending'
        return response

    def finish_pause_if_stopped(self):
        if (self.suspend_requested and not self.finishing and not self.handles and not self.pending):
            if self.state != 'suspended':
                self.report('suspended', 'native map retained; FollowPath released')
            if self.pause_future is not None and not self.pause_future.done():
                self.pause_future.set_result(True)

    def stop(self, state, reason):
        if self.finishing or not self.task or self.state in TERMINAL: return
        self.suspend_requested = False
        if self.pause_future is not None and not self.pause_future.done():
            self.pause_future.set_result(False)
        self.pause_future = None
        self.suspended_since = None
        self.enabled = False; self.enable_pub.publish(Bool(data=False)); self.finishing = (state, reason)
        for handle in list(self.handles.values()): handle.cancel_goal_async()
        self.report('stopping', reason)
        self.finish_if_stopped()

    def finish_if_stopped(self):
        if self.finishing and not self.handles and not self.pending:
            state, reason = self.finishing; self.finishing = None
            self.cleanup_process(); self.report(state, reason); self.active_pub.publish(Bool(data=False))

    def cleanup_process(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try: self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL); self.process.wait(timeout=2)
        self.process = None
        if self.log: self.log.close(); self.log = None
        # Keep engine diagnostics after a task; clean only on the next task or exit.
        if self.temp:
            directory = FilePath(self.temp.name)
            log = directory/'engine.log'
            if log.exists():
                destination = FilePath(tempfile.gettempdir())/('co_search_'+self.ns+'_last_engine.log')
                destination.write_bytes(log.read_bytes())
            self.temp.cleanup(); self.temp = None

    def tf_diagnostics(self, pose, timeout):
        now = self.get_clock().now().nanoseconds/1e9
        stamp = Time.from_msg(pose.header.stamp).nanoseconds/1e9
        sample = self.tf_health_sample
        gap = time.monotonic()-sample[0] if sample else 0.
        delta = stamp-sample[1] if sample else 0.
        age = now-stamp
        detail = ('lookup=%s<-%s ros_now=%.9f tf_stamp=%.9f age_s=%.6f '
                  'allowed_age_s=[-0.5,%.3f] wall_now=%.9f monitor_gap_s=%.3f '
                  'tf_stamp_delta_s=%.6f use_sim_time=%s pid=%d' %
                  (self.frame, self.base, now, stamp, age, timeout, time.time(),
                   gap, delta, self.get_parameter('use_sim_time').value, os.getpid()))
        return age, detail

    def monitor_tf(self):
        now = time.monotonic()
        if self.tf_health_sample and now-self.tf_health_sample[0] < 1.:
            return
        timeout = self.config.get('options', {}).get('sensor_timeout_s', 3.)
        try:
            pose = self.tf.lookup_transform(self.frame, self.base, Time())
            age, detail = self.tf_diagnostics(pose, timeout)
            bad = age < -.5 or age > timeout
            gap = now-self.tf_health_sample[0] if self.tf_health_sample else 0.
            if (bad or gap > 2.) and now-self.tf_health_log >= 5.:
                self.get_logger().warning('Area search TF health: '+detail)
                self.tf_health_log = now
            elif self.tf_health_bad and not bad:
                self.get_logger().info('Area search TF recovered: '+detail)
            self.tf_health_bad = bad
            self.tf_health_sample = (now, Time.from_msg(pose.header.stamp).nanoseconds/1e9)
        except Exception as exc:
            if now-self.tf_health_started >= 10. and now-self.tf_health_log >= 5.:
                self.get_logger().warning('Area search TF unavailable: lookup=%s<-%s ros_now=%.9f error=%s' %
                    (self.frame, self.base, self.get_clock().now().nanoseconds/1e9, exc))
                self.tf_health_log = now
            self.tf_health_bad = True

    def tick(self):
        self.monitor_tf()
        self.poll_readiness()
        if not self.task or self.state in TERMINAL: return
        now = time.monotonic()
        self.active_pub.publish(Bool(data=True))
        if self.finishing: self.finish_if_stopped(); return
        if self.process and self.process.poll() is not None:
            self.stop('failed', 'native engine exited; see /tmp/co_search_'+self.ns+'_last_engine.log'); return
        if self.suspend_requested:
            self.finish_pause_if_stopped()
            if not self.navigation_ready() or now-self.last_engine > 15.:
                self.stop('failed', 'navigation readiness or native heartbeat lost during suspension')
            elif now-self.last_feedback > 1.:
                self.last_feedback = now; self.report(self.state)
            return
        if now-self.start_time > float(self.task.get('co_search', {}).get('startup_timeout_s', 60.)) and not self.enabled:
            self.stop('failed', 'native engine startup/map timeout'); return
        if self.enabled:
            navigation_ok = self.navigation_ready()
            heartbeat_age = now-self.last_engine
            if not navigation_ok or heartbeat_age > 15.:
                causes = []
                if not navigation_ok:
                    causes.append('navigation readiness lost ('+self.navigation_readiness_detail()+')')
                if heartbeat_age > 15.:
                    causes.append('native engine heartbeat timeout age_s=%.2f phase=%s' %
                                  (heartbeat_age, self.engine.get('phase', 'unknown')))
                reason = '; '.join(causes)
                self.get_logger().error('[co_search_watchdog] mission_id=%s %s' %
                                        (self.task.get('mission_id', ''), reason))
                self.stop('paused', reason)
                return
        self.check_coverage_completion()
        if self.finishing or self.state in TERMINAL: return
        if now-self.last_feedback > 1.:
            self.last_feedback = now; self.report(self.state)


def main():
    rclpy.init(); node = None
    try:
        node = CoSearchExecutor(); rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        if node:
            # Shutdown does not release ownership before cancel requests are sent.
            for handle in list(node.handles.values()):
                if rclpy.ok(): handle.cancel_goal_async()
            node.cleanup_process(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
