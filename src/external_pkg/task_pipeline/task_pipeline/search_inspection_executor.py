"""Simulation observation: retain area-search state, pause, visit, then resume.

Only observation arrivals are produced. Navigation ownership transfers after
pause acknowledgement and returns after the visitor's terminal Action result.
"""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from task_interfaces.action import AreaSearch, InspectTargets
from task_interfaces.msg import TargetArray, TargetInfo
from visualization_msgs.msg import Marker, MarkerArray


def marker_observation(marker, frame, now, since, max_age):
    """Reject stale/foreign/invalid markers; positions are already in the map."""
    stamp = marker.header.stamp.sec + marker.header.stamp.nanosec / 1e9
    point = marker.pose.position
    xyz = (point.x, point.y, point.z)
    if (marker.action != Marker.ADD or marker.type != Marker.SPHERE or marker.id <= 0
            or marker.header.frame_id != frame or stamp <= 0 or stamp < since-.05
            or abs(now-stamp) > max_age or not all(math.isfinite(v) for v in xyz)):
        return None
    return dict(key=(marker.ns, marker.id), target_id=marker.id, point=xyz, stamp=stamp)


def observed_before(point, handled, radius):
    return any(math.dist(point, old) <= radius for old in handled)


class SearchInspectionExecutor(Node):
    def __init__(self):
        super().__init__('search_inspection_executor')
        defaults = dict(global_frame='', marker_topic='targets/markers', marker_max_age_s=1.5,
                        observation_merge_m=1.5, search_service_timeout_s=15.)
        self.cfg = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        for name in ('marker_max_age_s', 'observation_merge_m', 'search_service_timeout_s'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(f'{name} must be positive and finite')
        namespace = self.get_namespace().strip('/')
        self.frame = self.cfg['global_frame'] or '/'.join(filter(None, (namespace, 'map')))
        self.search = ActionClient(self, AreaSearch, 'area_search')
        self.visitor = ActionClient(self, InspectTargets, 'search_target_inspection')
        self.pause_client = self.create_client(SetBool, 'area_search/pause')
        self.assignment = self.create_publisher(TargetArray, 'targets/search_assigned', 10)
        self.create_subscription(MarkerArray, self.cfg['marker_topic'], self.markers, 10)
        self.server = ActionServer(self, AreaSearch, 'search_inspection', self.execute,
                                   goal_callback=self.goal, cancel_callback=self.cancel)
        self.active = None
        self.busy = False
        self.create_timer(.05, self.tick)

    def goal(self, request):
        area = request.area
        values = (area.center.latitude, area.center.longitude, area.center.altitude,
                  area.length_m, area.width_m, area.yaw_deg)
        if (self.busy or request.task_id != '143' or not all(math.isfinite(v) for v in values)
                or abs(area.center.latitude) > 90 or abs(area.center.longitude) > 180
                or min(area.length_m, area.width_m) < 1.):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        if self.active is not None:
            self.canceling = True
            self.cancel_children()
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        self.active = handle
        self.canceling = handle.is_cancel_requested
        self.phase = 'starting_search'
        self.search_handle = self.visit_handle = None
        self.search_sending = self.visit_sending = False
        self.search_result = None
        self.pause_future = None
        self.fatal = ''
        self.uncertain = set()
        self.cancel_sent = set()
        self.candidate = None
        self.handled = []
        self.inspected = self.failed = 0
        self.progress = 0.
        self.since = self.get_clock().now().nanoseconds / 1e9
        self.deadline = time.monotonic() + self.cfg['search_service_timeout_s']
        self.waiter = Future()
        try:
            await self.waiter
            success = (not self.canceling and not self.fatal and self.search_result is not None
                       and self.search_result[0] and self.failed == 0)
            message = (f'search observation: {self.inspected} arrived, {self.failed} failed; '
                       + (self.fatal or (self.search_result[1] if self.search_result else 'canceled')))
            if self.canceling:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
            return AreaSearch.Result(success=success, message=message)
        finally:
            self.active = None
            self.busy = False

    def markers(self, message):
        if self.active is None or self.canceling or self.fatal:
            return
        if self.phase not in ('searching', 'pausing_search', 'visitor_ready'):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        for marker in message.markers:
            if marker.action == Marker.DELETEALL or (self.candidate is not None
                    and marker.action == Marker.DELETE and
                    (marker.ns, marker.id) == self.candidate['key']):
                self.candidate = None
        for marker in sorted(message.markers, key=lambda m: m.id):
            observation = marker_observation(marker, self.frame, now, self.since, self.cfg['marker_max_age_s'])
            if observation is None or observed_before(observation['point'], self.handled, self.cfg['observation_merge_m']):
                continue
            if self.phase == 'searching' and self.candidate is None:
                self.candidate = observation
                return
            if self.candidate is not None and (observation['key'] == self.candidate['key']
                    or math.dist(observation['point'], self.candidate['point']) <= self.cfg['observation_merge_m']):
                self.candidate = observation
                return

    def candidate_fresh(self):
        return (self.candidate is not None and abs(self.get_clock().now().nanoseconds / 1e9
                - self.candidate['stamp']) <= self.cfg['marker_max_age_s'])

    def stop(self, reason):
        if not self.fatal:
            self.fatal = reason
            self.get_logger().error(reason)
        self.phase = 'stopping'
        self.cancel_children()

    def transport_error(self, reason, owner):
        # A lost Action response cannot prove that an accepted child stopped.
        self.uncertain.add(owner)
        self.stop('Navigation ownership uncertain: '+str(reason))
        self.phase = 'transport_error'

    def cancel_children(self):
        for handle in (self.search_handle, self.visit_handle):
            key = tuple(handle.goal_id.uuid) if handle is not None else None
            if handle is not None and key not in self.cancel_sent:
                self.cancel_sent.add(key)
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error(f'Cannot request child cancellation: {exc}')

    def search_accepted(self, future):
        self.search_sending = False
        try:
            handle = future.result()
            self.uncertain.discard('search')
            if not handle.accepted:
                self.search_result = (False, 'area search rejected')
                self.stop('area search rejected')
                return
            self.search_handle = handle
            handle.get_result_async().add_done_callback(self.search_completed)
            if self.canceling or self.fatal:
                self.cancel_children()
            else:
                self.phase = 'searching'
        except Exception as exc:
            self.transport_error(exc, 'search')

    def search_feedback(self, message):
        if self.active is not None and math.isfinite(message.feedback.progress):
            self.progress = max(self.progress, min(99., max(0., message.feedback.progress)))

    def search_completed(self, future):
        try:
            wrapped = future.result()
            if wrapped.status not in (4, 5, 6):
                raise RuntimeError(f'non-terminal area result {wrapped.status}')
        except Exception as exc:
            self.transport_error(exc, 'search')
            return
        self.uncertain.discard('search')
        self.search_handle = None
        self.search_result = (wrapped.status == 4 and wrapped.result.success, wrapped.result.message)
        if not self.search_result[0] and not self.canceling:
            self.stop('area search ended: '+wrapped.result.message)
        elif self.phase != 'visiting':
            self.phase = 'finishing'

    def call_pause(self, paused):
        self.phase = 'pausing_search' if paused else 'resuming_search'
        self.deadline = time.monotonic() + self.cfg['search_service_timeout_s']
        try:
            self.pause_future = self.pause_client.call_async(SetBool.Request(data=paused))
            self.pause_future.add_done_callback(lambda f: self.pause_completed(f, paused))
        except Exception as exc:
            self.transport_error(exc, 'pause')

    def pause_completed(self, future, paused):
        self.pause_future = None
        try:
            response = future.result()
            self.uncertain.discard('pause')
        except Exception as exc:
            self.transport_error('search pause/resume response failed: '+str(exc), 'pause')
            return
        if self.canceling or self.fatal:
            return
        if self.search_result is not None:
            self.phase = 'finishing'
        elif not response.success:
            self.stop('search pause/resume rejected: '+response.message)
        else:
            self.phase = 'visitor_ready' if paused else 'searching'
            self.deadline = time.monotonic() + self.cfg['search_service_timeout_s']

    def visit_accepted(self, future):
        self.visit_sending = False
        try:
            handle = future.result()
            self.uncertain.discard('visit')
            if not handle.accepted:
                self.finish_visit(0, 1)
                return
            self.visit_handle = handle
            handle.get_result_async().add_done_callback(self.visit_completed)
            if self.canceling or self.fatal:
                self.cancel_children()
                return
            # The private visitor clears its prior assignment in its goal
            # callback, so this post-acceptance publication belongs to this visit.
            observation = self.candidate
            self.assignment.publish(TargetArray(targets=[TargetInfo(
                target_id=observation['target_id'], target_position=Point(
                    x=observation['point'][0], y=observation['point'][1], z=observation['point'][2]))]))
        except Exception as exc:
            self.transport_error(exc, 'visit')

    def visit_completed(self, future):
        try:
            wrapped = future.result()
            if wrapped.status not in (4, 5, 6):
                raise RuntimeError(f'non-terminal visitor result {wrapped.status}')
        except Exception as exc:
            self.transport_error(exc, 'visit')
            return
        self.uncertain.discard('visit')
        self.visit_handle = None
        if self.canceling or self.fatal:
            return
        success = wrapped.status == 4 and wrapped.result.success
        self.finish_visit(wrapped.result.inspected_count if success else 0,
                          0 if success else max(1, wrapped.result.failed_count))

    def finish_visit(self, inspected, failed):
        self.inspected += inspected
        self.failed += failed
        if self.candidate is not None:
            self.handled.append(self.candidate['point'])
        self.candidate = None
        self.phase = 'resume_ready' if self.search_result is None else 'finishing'
        self.deadline = time.monotonic() + self.cfg['search_service_timeout_s']

    def drained(self):
        return not (self.search_handle or self.visit_handle or self.search_sending
                    or self.visit_sending or self.pause_future or self.uncertain)

    def tick(self):
        if self.active is None or self.waiter.done():
            return
        now = time.monotonic()
        if self.active.is_cancel_requested:
            self.canceling = True
        if self.canceling or self.fatal:
            self.cancel_children()
            if self.drained():
                self.waiter.set_result(False)
            return
        self.active.publish_feedback(AreaSearch.Feedback(progress=self.progress, phase=self.phase))
        if self.pause_future is not None and now > self.deadline:
            self.stop('search pause/resume timed out; waiting for child terminal results')
            return
        if (self.search_sending or self.visit_sending) and now > self.deadline:
            self.transport_error('child goal acceptance timed out', 'search' if self.search_sending else 'visit')
            return
        if self.phase == 'starting_search' and not self.search_sending:
            if not self.search.server_is_ready():
                if now > self.deadline:
                    self.stop('area search Action unavailable')
                return
            self.search_sending = True
            try:
                self.search.send_goal_async(AreaSearch.Goal(task_id='143', area=self.active.request.area),
                    feedback_callback=self.search_feedback).add_done_callback(self.search_accepted)
            except Exception as exc:
                self.search_sending = False
                self.transport_error(exc, 'search')
        elif self.phase == 'searching' and self.candidate is not None:
            if not self.candidate_fresh():
                self.candidate = None
                return
            self.phase = 'pause_ready'
            self.deadline = now+self.cfg['search_service_timeout_s']
        elif self.phase in ('pause_ready', 'resume_ready'):
            if self.pause_client.service_is_ready():
                self.call_pause(self.phase == 'pause_ready')
            elif now > self.deadline:
                self.stop('area search pause service unavailable')
        elif self.phase == 'visitor_ready':
            if not self.candidate_fresh():
                self.candidate = None
                self.phase = 'resume_ready'
                self.deadline = now+self.cfg['search_service_timeout_s']
            elif not self.visitor.server_is_ready() or self.assignment.get_subscription_count() == 0:
                if now > self.deadline:
                    self.stop('private observation visitor unavailable')
            else:
                self.visit_sending = True
                self.deadline = now+self.cfg['search_service_timeout_s']
                self.phase = 'visiting'
                try:
                    self.visitor.send_goal_async(InspectTargets.Goal(task_id='143')).add_done_callback(self.visit_accepted)
                except Exception as exc:
                    self.visit_sending = False
                    self.transport_error(exc, 'visit')
        elif self.phase == 'finishing' and self.drained():
            self.waiter.set_result(True)

    def destroy_node(self):
        if self.active is not None:
            self.cancel_children()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SearchInspectionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
