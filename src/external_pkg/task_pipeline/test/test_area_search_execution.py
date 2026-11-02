"""Async cancellation invariants without TF, robot, controller or motion."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
from pathlib import Path
import yaml
import pytest

pytest.importorskip('rclpy')
from task_pipeline.area_search import AreaSearch
from task_pipeline.area_search_core import Rectangle, validate_task


def fake_node():
    return NS(generation=7, state='executing', path_purpose='search',
              active_pub=NS(publish=Mock()),
              goal_handle=None, goal_future=None, report=Mock(), pause=Mock(),
              path_failed=Mock(), _finish_interrupted_action=Mock())


def test_pause_does_not_release_ownership_before_pending_acceptance():
    n=fake_node(); n.goal_future=object()
    AreaSearch.pause(n,'paused','operator stop')
    assert n.generation==8
    n.active_pub.publish.assert_not_called()


def test_late_acceptance_after_cancel_is_cancelled_and_waited():
    n=fake_node(); handle=NS(accepted=True, get_result_async=Mock(),cancel_goal_async=Mock())
    future=NS(result=lambda:handle)
    AreaSearch.goal_response(n,future,6)
    handle.cancel_goal_async.assert_called_once()
    assert n.goal_handle is handle
    n.active_pub.publish.assert_not_called()
    AreaSearch.goal_result(n,NS(result=lambda:NS(status=5)),6,handle)
    assert n.goal_handle is None
    assert n.active_pub.publish.call_args.args[0].data is False
    n.report.assert_not_called()  # old terminal results must not restart planning


def test_cancelled_pending_request_rejected_releases_ownership():
    n=fake_node()
    AreaSearch.goal_response(n,NS(result=lambda:NS(accepted=False)),6)
    assert n.active_pub.publish.call_args.args[0].data is False
    n.path_failed.assert_not_called()


def test_real_current_success_returns_to_planning():
    n=fake_node(); handle=object();n.goal_handle=handle
    AreaSearch.goal_result(n,NS(result=lambda:NS(status=4)),7,handle)
    assert n.goal_handle is None
    assert n.report.call_args.args[0]=='planning'


def test_action_failure_is_not_reported_as_arrival():
    n=fake_node();handle=object();n.goal_handle=handle
    AreaSearch.goal_result(n,NS(result=lambda:NS(status=6)),7,handle)
    n.path_failed.assert_called_once()
    n.report.assert_not_called()


def tick_node(pose, state='waiting_consensus'):
    now = time.monotonic()
    config = Path(__file__).parents[1] / 'config/area_search.yaml'
    task = validate_task(yaml.safe_load(config.read_text()))
    node = fake_node()
    node.__dict__.update(
        state=state, task=task, region=Rectangle(10, 0, 20, 16),
        discover=lambda _: ('robot_1',), query_navigation=Mock(),
        last_roster=('robot_1',), team=('robot_1',),
        signature='same-task', last_proposal=now, started=now-1,
        proposals={'robot_1': ({'signature': 'same-task'}, now)},
        pose=lambda: pose, controller_active=True, ready_at=now,
        navigation_active=True, navigation_ready_at=now,
        action=NS(server_is_ready=lambda: True), last_cloud=now,
        start_navigation=Mock(), grid=None, last_plan=now,
        last_active_heartbeat=now, send_path=Mock(),
    )
    return node


@pytest.mark.parametrize('pose,phase', [
    ((-3, 0, 0), 'planning_entry'),
    ((-3, 10, 0), 'planning_entry'),
    ((0, 0, 0), 'planning'),
    ((0.1, 0, 0), 'planning'),
    ((10, 0, 0), 'planning'),
])
def test_start_selects_entry_only_when_center_is_outside(pose, phase):
    node = tick_node(pose)
    AreaSearch.tick(node)
    node.pause.assert_not_called()
    node.start_navigation.assert_called_once_with(pose, phase)
    node.send_path.assert_not_called()  # ownership handover precedes motion


def test_entry_path_points_to_nearest_side_with_footprint_clearance():
    node = tick_node((-3, 2, 0), 'planning_entry')
    node.last_plan = 0
    AreaSearch.tick(node)
    node.pause.assert_not_called()
    points = node.send_path.call_args.args[0]
    assert points[0] == (-3, 2)
    options = node.task['options']
    assert points[-1] == pytest.approx((options['robot_radius_m']+options['resolution_m'], 2))
    assert all(y == pytest.approx(2) for x, y in points)
    assert node.path_purpose == 'entry'


def test_entry_success_requires_inside_pose_then_selects_frontier():
    node = tick_node((0.8, 2, 0), 'entering')
    node.path_purpose = 'entry'
    handle = object(); node.goal_handle = handle
    AreaSearch.goal_result(node, NS(result=lambda: NS(status=4)), 7, handle)
    node.start_navigation.assert_not_called()
    assert node.report.call_args.args[0] == 'planning'
    assert node.path_purpose == 'search'


def test_entry_success_outside_does_not_start_frontier_search():
    node = tick_node((-0.1, 2, 0), 'entering')
    node.path_purpose = 'entry'
    handle = object(); node.goal_handle = handle
    AreaSearch.goal_result(node, NS(result=lambda: NS(status=4)), 7, handle)
    node.start_navigation.assert_not_called()
    assert node.pause.call_args.args[0] == 'failed'


def test_entry_failure_does_not_advance_to_frontier_search():
    node = tick_node((-3, 2, 0), 'entering')
    node.path_purpose = 'entry'
    handle = object(); node.goal_handle = handle
    AreaSearch.goal_result(node, NS(result=lambda: NS(status=6)), 7, handle)
    node.path_failed.assert_called_once()
    node.start_navigation.assert_not_called()


def test_start_navigation_publishes_region_without_costmap_acknowledgement():
    node = tick_node((0.05, 2, 0))
    polygon = object()
    node.polygon = Mock(return_value=polygon)
    node.region_pub = NS(publish=Mock())
    AreaSearch.start_navigation(node, (0.05, 2, 0), 'planning')
    node.polygon.assert_called_once_with(node.region)
    node.region_pub.publish.assert_called_once_with(polygon)
    assert node.active_pub.publish.call_args.args[0].data is True
    assert node.report.call_args.args[0] == 'planning'


@pytest.mark.parametrize('purpose,checker,phase', [
    ('entry', 'area_entry_goal_checker', 'entering'),
    ('search', 'general_goal_checker', 'executing'),
])
def test_path_uses_navigation_checker_for_its_phase(purpose, checker, phase):
    node = tick_node((-3, 2, 0))
    from builtin_interfaces.msg import Time
    node.frame = 'robot_1/map'; node.config = {}
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
    node.path_purpose = purpose; node.path_pub = NS(publish=Mock())
    node.action.send_goal_async = Mock(return_value=NS(add_done_callback=Mock()))
    AreaSearch.send_path(node, [(-3.0, 2.0), (0.85, 2.0)])
    goal = node.action.send_goal_async.call_args.args[0]
    assert goal.goal_checker_id == checker
    assert goal.path.poses[-1].pose.position.x == pytest.approx(0.85)
    assert node.report.call_args.args[0] == phase


def test_short_search_step_uses_tight_checker_instead_of_immediate_arrival():
    node = tick_node((10, 0, 0))
    from builtin_interfaces.msg import Time
    node.frame = 'robot_1/map'; node.config = {}
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
    node.path_purpose = 'search'; node.path_pub = NS(publish=Mock())
    node.action.send_goal_async = Mock(return_value=NS(add_done_callback=Mock()))
    AreaSearch.send_path(node, [(10.0, 0.0), (10.25, 0.0)])
    goal = node.action.send_goal_async.call_args.args[0]
    assert goal.goal_checker_id == 'area_entry_goal_checker'


def test_entry_remains_cancellable_while_waiting_for_action_acceptance():
    node = tick_node((-3, 2, 0), 'entering')
    node.goal_future = object()
    AreaSearch.stop(node, NS(data=True))
    assert node.pause.call_args.args[0] == 'paused'


def test_near_edge_during_execution_has_no_radius_based_pause():
    node = tick_node((0.05, 2, 0), 'executing')
    AreaSearch.tick(node)
    node.pause.assert_not_called()


def executing_node():
    node = tick_node((10, 0, 0), 'executing')
    node.assigned = node.region
    node.last_plan = 0.0; node.last_map_publish = time.monotonic()
    node.goal_handle = NS(cancel_goal_async=Mock(return_value=NS(add_done_callback=Mock())))
    node.goal_started = time.monotonic()
    node.replan_pending = False; node.target = (4, 5)
    node.active_path = [(10, 0), (11, 0)]
    node.blacklist = {}; node.request_replan = Mock()
    node.grid = NS(plan=Mock(return_value=([(10, 0), (11, 0)], .5, 2, node.target)),
                   path_is_valid=Mock(return_value=True))
    return node


def test_execution_reevaluates_without_cancelling_a_useful_goal():
    node = executing_node()
    AreaSearch.tick(node)
    node.grid.plan.assert_called_once_with((10, 0, 0), node.assigned, {}, current_target=node.target)
    node.request_replan.assert_not_called()
    node.send_path.assert_not_called()


def test_execution_extends_same_viewpoint_path_before_segment_arrival():
    node = executing_node()
    node.grid.plan.return_value = ([(10, 0), (14, 0)], .5, 2, node.target)
    AreaSearch.tick(node)
    node.request_replan.assert_called_once()
    assert 'extend rolling path' in node.request_replan.call_args.args[0]
    node.send_path.assert_not_called()  # replacement waits for cancellation acknowledgement


def test_execution_does_not_refresh_distant_segment_endpoint():
    node = executing_node()
    node.active_path = [(10, 0), (13, 0)]
    node.grid.plan.return_value = ([(10, 0), (14, 0)], .5, 2, node.target)
    AreaSearch.tick(node)
    node.request_replan.assert_not_called()


def test_search_replans_from_outside_pose_without_forcing_entry_again():
    node = executing_node(); node.state = 'planning'; node.goal_handle = None
    node.pose = lambda: (-.5, 2, 0)
    AreaSearch.tick(node)
    node.start_navigation.assert_not_called()
    node.grid.plan.assert_called_once_with((-.5, 2, 0), node.assigned, {})
    node.send_path.assert_called_once()


def test_entry_selects_frontier_as_soon_as_center_enters_region():
    node = executing_node()
    node.state = 'entering'; node.path_purpose = 'entry'
    AreaSearch.tick(node)
    node.request_replan.assert_called_once()
    assert 'entered search region' in node.request_replan.call_args.args[0]
    node.grid.plan.assert_not_called()  # finish cancellation before selecting a new action


def test_map_publication_uses_configured_cadence_during_motion():
    node = executing_node()
    node.publish_map = Mock()
    node.last_map_publish = time.monotonic() - .4
    node.last_plan = time.monotonic()
    AreaSearch.tick(node)
    node.publish_map.assert_called_once()
    node.grid.plan.assert_not_called()


@pytest.mark.parametrize('change', ['target', 'blocked', 'no_frontier'])
def test_execution_requests_replan_before_arrival(change):
    node = executing_node()
    if change == 'target':
        node.grid.plan.return_value = ([(10, 0), (12, 0)], .5, 2, (8, 5))
    elif change == 'blocked':
        node.grid.path_is_valid.return_value = False
    else:
        node.grid.plan.return_value = ([], .5, 0, None)
    AreaSearch.tick(node)
    node.request_replan.assert_called_once()
    node.send_path.assert_not_called()


def test_awaiting_cancel_does_not_plan_or_send_replacement():
    node = executing_node(); node.replan_pending = True
    AreaSearch.tick(node)
    node.grid.plan.assert_not_called()
    node.send_path.assert_not_called()


def test_replan_retains_ownership_until_old_goal_terminal_result():
    node = executing_node(); handle = node.goal_handle
    node.replan_cancel_response = Mock()
    AreaSearch.request_replan(node, 'updated map')
    assert node.replan_pending
    handle.cancel_goal_async.assert_called_once()
    node.send_path.assert_not_called()
    node.active_pub.publish.assert_not_called()
    AreaSearch.goal_result(node, NS(result=lambda: NS(status=5)), node.generation, handle)
    assert not node.replan_pending and node.goal_handle is None and node.last_plan == 0
    assert node.report.call_args.args[0] == 'planning'
    node.path_failed.assert_not_called()
    node.active_pub.publish.assert_not_called()


def test_stop_during_replan_cannot_restart_planning_from_old_result():
    node = executing_node(); handle = node.goal_handle
    node.replan_pending = True; generation = node.generation
    AreaSearch.pause(node, 'paused', 'operator stop')
    AreaSearch.goal_result(node, NS(result=lambda: NS(status=5)), generation, handle)
    assert not node.replan_pending
    assert node.report.call_args.args[0] == 'paused'
    assert node.active_pub.publish.call_args.args[0].data is False
    node.send_path.assert_not_called()


def test_replan_cancel_rejection_retains_current_segment():
    node = executing_node(); node.replan_pending = True
    AreaSearch.replan_cancel_response(node, NS(result=lambda: NS(goals_canceling=[])), node.goal_handle)
    assert not node.replan_pending
    assert node.report.call_args.args[0] == 'executing'
    node.send_path.assert_not_called()


def test_late_cancellation_response_cannot_affect_replacement_goal():
    node = executing_node(); node.replan_pending = True
    AreaSearch.replan_cancel_response(node, NS(result=lambda: NS(goals_canceling=[])), object())
    assert node.replan_pending
    node.report.assert_not_called()


def test_lost_navigation_state_response_is_retried_without_refreshing_readiness():
    now = time.monotonic(); old = Mock(); new = Mock()
    client = NS(remove_pending_request=Mock(), service_is_ready=lambda: True,
                call_async=Mock(return_value=new))
    node = NS(parameters=Mock(), lifecycle=client, origin=(0, 0, 0), origin_future=None,
              origin_requested_at=0.0, ready_future=old, ready_requested_at=now-3,
              ready_at=now-10, got_ready=Mock(), manager=Mock(), manager_future=None,
              manager_requested_at=now, got_navigation_ready=Mock())
    AreaSearch.query_navigation(node, now)
    client.remove_pending_request.assert_called_once_with(old)
    old.cancel.assert_called_once()
    assert node.ready_future is new and node.ready_at == now-10


def test_late_state_response_cannot_overwrite_fresh_query():
    node = NS(ready_future=object(), controller_active=False, ready_at=0.0)
    AreaSearch.got_ready(node, NS(result=lambda: NS(current_state=NS(id=3))))
    assert node.controller_active is False and node.ready_at == 0.0


def test_controller_active_does_not_start_search_until_manager_is_active():
    node = tick_node((10, 0, 0)); node.navigation_active = False
    AreaSearch.tick(node)
    node.start_navigation.assert_not_called()
    node.send_path.assert_not_called()


def test_lost_manager_readiness_during_motion_pauses_search():
    node = executing_node(); node.navigation_ready_at = time.monotonic()-4
    AreaSearch.tick(node)
    assert 'lifecycle manager' in node.pause.call_args.args[1]
    node.grid.plan.assert_not_called()


def test_manager_readiness_requires_real_response_and_ignores_old_response():
    node = NS(manager_future=object(), navigation_active=False, navigation_ready_at=0.)
    AreaSearch.got_navigation_ready(node, NS(result=lambda: NS(success=True)))
    assert not node.navigation_active
    future = NS(result=lambda: NS(success=True)); node.manager_future = future
    AreaSearch.got_navigation_ready(node, future)
    assert node.navigation_active and node.navigation_ready_at > 0


@pytest.mark.parametrize('mode,origin,published', [
    ('enu', None, True), ('gps', None, False), ('gps', (0.0, 0.0, 0.0), True),
])
def test_region_preview_is_available_before_team_or_controller_readiness(mode, origin, published):
    node = tick_node((0, 0, 0), 'waiting_team')
    node.task['coordinate_mode'] = mode
    node.origin = origin
    node.region_pub = NS(publish=Mock())
    node.polygon = Mock(return_value=object())
    AreaSearch.publish_region_preview(node)
    assert node.region_pub.publish.called is published
