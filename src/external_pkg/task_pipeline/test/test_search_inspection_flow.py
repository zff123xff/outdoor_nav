"""Real DDS/Action/service handoff using simulated search and observation workers.

No robot, Nav2 motion server, or actuator is started by these tests.
"""
import time
from types import SimpleNamespace as NS

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
from task_interfaces.action import AreaSearch, InspectTargets
from task_interfaces.msg import SearchArea, TargetArray, InspectionArrival
from visualization_msgs.msg import Marker, MarkerArray
from task_pipeline.search_inspection_executor import SearchInspectionExecutor


class SimulatedWorkers(Node):
    def __init__(self):
        super().__init__('simulated_observation_workers')
        self.events = []
        self.area_handle = self.visit_handle = None
        self.area_gate = self.pause_gate = self.visit_gate = self.assignment_gate = None
        self.paused = self.visit_active = False
        self.batch = None
        self.pause_success = True
        self.visits = 0
        self.group = ReentrantCallbackGroup()
        self.area_server = ActionServer(self, AreaSearch, 'area_search', self.search,
            cancel_callback=self.cancel_search, callback_group=self.group)
        self.visit_server = ActionServer(self, InspectTargets, 'search_target_inspection', self.visit,
            goal_callback=self.visit_goal, cancel_callback=self.cancel_visit, callback_group=self.group)
        self.create_service(SetBool, 'area_search/pause', self.pause, callback_group=self.group)
        self.create_subscription(TargetArray, 'targets/search_assigned', self.assign, 10,
                                 callback_group=self.group)
        self.arrivals = self.create_publisher(InspectionArrival, 'targets/inspection_arrived', 10)

    async def search(self, handle):
        self.area_handle = handle
        self.area_gate = Future()
        self.events.append('search_started')
        if not handle.is_cancel_requested:
            await self.area_gate
        if handle.is_cancel_requested:
            handle.canceled(); success = False
        else:
            handle.succeed(); success = True
        self.events.append('search_terminal')
        return AreaSearch.Result(success=success, message='simulated search finished')

    def cancel_search(self, handle):
        self.events.append('cancel_search')
        if self.area_gate is not None and not self.area_gate.done():
            self.area_gate.set_result(False)
        return CancelResponse.ACCEPT

    async def pause(self, request, response):
        if request.data:
            self.pause_gate = Future()
            self.events.append('pause_requested')
            await self.pause_gate  # Test deliberately delays navigation ownership release.
            self.paused = True
            self.events.append('pause_acknowledged')
        else:
            assert not self.visit_active, 'Search resumed before visitor terminal result'
            self.paused = False
            self.events.append('resumed')
        response.success = self.pause_success
        return response

    def visit_goal(self, request):
        self.batch = None  # Same private post-acceptance assignment contract as the real visitor.
        return GoalResponse.ACCEPT

    def assign(self, message):
        self.batch = message
        if self.assignment_gate is not None and not self.assignment_gate.done():
            self.assignment_gate.set_result(True)

    async def visit(self, handle):
        assert self.paused, 'Observation started before area search pause acknowledgement'
        self.visit_handle = handle
        self.visit_active = True
        self.visits += 1
        self.visit_gate = Future()
        self.events.append('visit_started')
        if self.batch is None:
            self.assignment_gate = Future()
            await self.assignment_gate
        assert self.batch and len(self.batch.targets) == 1
        self.events.append('assignment_received')
        if not handle.is_cancel_requested:
            await self.visit_gate  # Includes the simulated observation dwell interval.
        self.visit_active = False
        if handle.is_cancel_requested:
            handle.canceled()
            return InspectTargets.Result(success=False, failed_count=1, message='canceled')
        target = self.batch.targets[0]
        self.arrivals.publish(InspectionArrival(task_id='143', target_id=target.target_id,
            target_position=target.target_position, robot_position=Point(), distance_m=4., heading_error_rad=0.))
        handle.succeed()
        self.events.append('visit_terminal')
        return InspectTargets.Result(success=True, inspected_count=1, message='observation done')

    def cancel_visit(self, handle):
        self.events.append('cancel_visit')
        for future in (self.assignment_gate, self.visit_gate):
            if future is not None and not future.done():
                future.set_result(False)
        return CancelResponse.ACCEPT


class Harness:
    def __init__(self, executor, coordinator, workers, peer):
        self.executor, self.coordinator, self.workers, self.peer = executor, coordinator, workers, peer
        self.client = ActionClient(peer, AreaSearch, 'search_inspection')
        self.publisher = peer.create_publisher(MarkerArray, 'targets/markers', 10)
        self.arrivals = []
        self.subscription = peer.create_subscription(InspectionArrival, 'targets/inspection_arrived', self.arrivals.append, 10)
        self.handle = self.result = None

    def wait(self, predicate, timeout=5.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
        assert predicate(), (getattr(self.coordinator, 'phase', None), self.workers.events)

    def spin(self, seconds=.2):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)

    def start(self):
        self.wait(self.client.server_is_ready)
        goal = AreaSearch.Goal(task_id='143', area=SearchArea(
            center=NavSatFix(latitude=31., longitude=120.), length_m=10., width_m=8.))
        future = self.client.send_goal_async(goal)
        self.wait(future.done)
        self.handle = future.result(); assert self.handle.accepted
        self.result = self.handle.get_result_async()
        self.wait(lambda: self.coordinator.phase == 'searching')
        self.wait(lambda: self.publisher.get_subscription_count() > 0)

    def marker(self, key=1, x=4., old=False):
        marker = Marker(); marker.id = key; marker.ns = 'yolo_targets'; marker.type = Marker.SPHERE
        marker.header.frame_id = 'search_fixture/map'
        marker.header.stamp = self.peer.get_clock().now().to_msg()
        if old: marker.header.stamp.sec -= 10
        marker.pose.position = Point(x=x)
        self.publisher.publish(MarkerArray(markers=[marker]))

    def pause_search(self):
        self.marker()
        self.wait(lambda: self.workers.pause_gate is not None)

    def release_pause(self):
        self.marker()  # Refresh the selected point before navigation ownership transfers.
        self.workers.pause_gate.set_result(True)
        self.wait(lambda: self.workers.visits == 1 and self.workers.batch is not None)


@pytest.fixture
def flow():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/search_fixture', '-p', 'search_service_timeout_s:=3.0'])
    executor = SingleThreadedExecutor()
    coordinator = SearchInspectionExecutor()
    workers = SimulatedWorkers()
    peer = Node('search_observation_peer')
    for node in (coordinator, workers, peer): executor.add_node(node)
    harness = Harness(executor, coordinator, workers, peer)
    try:
        yield harness
    finally:
        if harness.handle is not None and (harness.result is None or not harness.result.done()):
            harness.handle.cancel_goal_async()
            for future in (workers.area_gate, workers.pause_gate, workers.assignment_gate, workers.visit_gate):
                if future is not None and not future.done(): future.set_result(False)
            harness.spin(.3)
        harness.client.destroy()
        workers.area_server.destroy(); workers.visit_server.destroy(); coordinator.server.destroy()
        for node in (coordinator, workers, peer):
            executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.try_shutdown()


def test_pause_visit_dwell_resume_and_spatial_dedup(flow):
    flow.start()
    flow.marker(old=True); flow.spin()
    assert flow.workers.pause_gate is None
    flow.pause_search(); flow.spin()
    assert flow.workers.visits == 0, 'Visit raced ahead of pause acknowledgement'
    flow.release_pause(); flow.spin()
    assert 'resumed' not in flow.workers.events, 'Search resumed before observation/dwell ended'
    flow.workers.visit_gate.set_result(True)
    flow.wait(lambda: 'resumed' in flow.workers.events and flow.coordinator.phase == 'searching')
    flow.wait(lambda: len(flow.arrivals) == 1)
    assert flow.arrivals[0].task_id == '143'
    flow.marker(key=99, x=4.2); flow.spin(.3)
    assert flow.workers.visits == 1
    assert flow.workers.events.count('pause_requested') == 1
    flow.workers.area_gate.set_result(True)
    flow.wait(flow.result.done)
    assert flow.result.result().status == 4
    assert flow.result.result().result.success
    events = flow.workers.events
    assert events.index('pause_acknowledged') < events.index('visit_started') < events.index('visit_terminal') < events.index('resumed')


def test_cancel_while_pause_pending_waits_for_pause_and_child_terminal(flow):
    flow.start(); flow.pause_search()
    future = flow.handle.cancel_goal_async(); flow.wait(future.done)
    flow.wait(lambda: 'search_terminal' in flow.workers.events)
    flow.spin(.15)
    assert not flow.result.done(), 'Parent released while pause service remained in flight'
    assert flow.workers.visits == 0
    flow.workers.pause_gate.set_result(True)
    flow.wait(flow.result.done)
    assert flow.result.result().status == 5
    assert 'resumed' not in flow.workers.events


def test_cancel_during_observation_cancels_both_owned_actions(flow):
    flow.start(); flow.pause_search(); flow.release_pause()
    future = flow.handle.cancel_goal_async(); flow.wait(future.done)
    flow.wait(flow.result.done)
    assert flow.result.result().status == 5
    assert 'cancel_search' in flow.workers.events and 'cancel_visit' in flow.workers.events
    assert 'resumed' not in flow.workers.events


def test_search_completes_normally_without_observations(flow):
    flow.start(); flow.workers.area_gate.set_result(True)
    flow.wait(flow.result.done)
    assert flow.result.result().result.success
    assert flow.workers.visits == 0


def test_pause_rejection_never_starts_observation(flow):
    flow.start(); flow.pause_search()
    flow.workers.pause_success = False
    flow.workers.pause_gate.set_result(True)
    flow.wait(flow.result.done)
    assert flow.result.result().status == 6
    assert flow.workers.visits == 0
    assert 'cancel_search' in flow.workers.events


def test_stale_observation_after_pause_resumes_without_visit(flow):
    flow.start(); flow.pause_search()
    flow.spin(1.6)
    flow.workers.pause_gate.set_result(True)
    flow.wait(lambda: 'resumed' in flow.workers.events)
    assert flow.workers.visits == 0
    flow.workers.area_gate.set_result(True)
    flow.wait(flow.result.done)
    assert flow.result.result().result.success
