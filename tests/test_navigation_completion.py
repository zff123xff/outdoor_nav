"""Protocol regressions: completion belongs to correlated navigation results."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
from concurrent.futures import Future
import tempfile
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/external_pkg/multi_goal_to_gp_goal'))
import patrol_node
patrol_node.__path__.insert(0, str(ROOT/'src/external_pkg/patrol_node/patrol_node'))
from action_msgs.msg import GoalStatus
from multi_goal_to_gp_goal.navigation_client import NavigationClient
from multi_goal_to_gp_goal.multi_goal_to_gp_goal import MultiGoalToGpGoal
from patrol_node.point_goal_node import PointGoalNode
from patrol_node.patrol_node import PatrolNode
import asyncio
import pytest
from geometry_msgs.msg import PoseStamped
from multi_goal_to_gp_goal.multi_goal_to_gp_goal import lla_to_enu


def make_client():
    parameters = {'navigation_action': 'navigate_to_pose', 'global_frame': 'map'}
    node = Mock()
    node.has_parameter.side_effect = lambda name: name in parameters
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value=parameters[name])
    node.get_namespace.return_value = '/test_robot'
    node.get_clock.return_value.now.return_value.to_msg.return_value = __import__('builtin_interfaces.msg', fromlist=['Time']).Time()
    with patch('multi_goal_to_gp_goal.navigation_client.ActionClient') as factory:
        client = NavigationClient(node)
    transport = factory.return_value
    transport.server_is_ready.return_value = True
    responses = []
    def send(goal):
        future = Future(); responses.append((goal, future)); return future
    transport.send_goal_async.side_effect = send
    return client, transport, responses


def handle():
    result = Future()
    goal = Mock(accepted=True)
    goal.get_result_async.return_value = result
    return goal, result


def test_only_matching_success_completes_request():
    client, transport, requests = make_client()
    done = Mock()
    client.send((0.0, 0.0, 0.0), done)
    accepted, result = handle()
    requests[0][1].set_result(accepted)
    for _ in range(5):client._pump()
    done.assert_not_called()  # Even a zero-distance goal has no task-side shortcut.
    assert len(requests) == 1
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    done.assert_called_once_with(GoalStatus.STATUS_SUCCEEDED)


def test_late_acceptance_is_canceled_and_does_not_complete_new_goal():
    client, transport, requests = make_client()
    old_done, new_done = Mock(), Mock()
    client.send((1, 0, 0), old_done)
    client.cancel()
    client.send((2, 0, 0), new_done)
    assert len(requests) == 1  # Wait for retirement before issuing another goal.
    stale, stale_result = handle()
    requests[0][1].set_result(stale)
    stale.cancel_goal_async.assert_called_once()
    client._pump(); assert len(requests) == 1
    stale_result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    client._pump(); assert len(requests) == 2
    old_done.assert_not_called(); new_done.assert_not_called()
    current, current_result = handle()
    requests[1][1].set_result(current)
    current_result.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    new_done.assert_called_once_with(GoalStatus.STATUS_ABORTED)


def test_waypoint_queue_does_not_advance_on_failure():
    queue = SimpleNamespace(_queue=[1, 2], _pending_patrol=object(), _patrol_points=None,
                            _current_goal_enu=(0, 0, 0), _current_waypoint=object(),
                            _holding_until_ns=None, get_logger=Mock(), _advance_to_next=Mock())
    MultiGoalToGpGoal._navigation_done(queue, GoalStatus.STATUS_ABORTED)
    queue._advance_to_next.assert_not_called()
    assert queue._queue == [] and queue._current_goal_enu is None


def test_task_consumers_follow_status_not_distance():
    point = SimpleNamespace(_active=True, _reason=None, _distance=lambda: 99.0,
                            _publish_feedback=Mock(), _resolve=Mock())
    PointGoalNode._navigation_done(point, GoalStatus.STATUS_SUCCEEDED)
    assert point._reason == 'reached'
    point._reason = None
    PointGoalNode._navigation_done(point, GoalStatus.STATUS_ABORTED)
    assert point._reason == 'navigation_failed'
    patrol = SimpleNamespace(_active=True, _end_reason=None, _on_reached=Mock(), _resolve_end=Mock())
    PatrolNode._navigation_done(patrol, GoalStatus.STATUS_CANCELED)
    patrol._on_reached.assert_not_called()
    assert patrol._end_reason == 'navigation_canceled'


@pytest.mark.parametrize('robot_xy,order', [((0, 1), [0, 1, 0]), ((0, 10), [1, 0, 1])])
def test_patrol_selects_nearest_endpoint_and_completes_one_round_trip(robot_xy, order):
    origin = (40.9445353, 113.172819, 1298.734571)
    a = SimpleNamespace(latitude=origin[0], longitude=origin[1], altitude=origin[2])
    b = SimpleNamespace(latitude=origin[0]+0.0001, longitude=origin[1], altitude=origin[2])
    request = SimpleNamespace(start_point=a, end_point=b, mission_id='test_patrol',
                              capability_id='patrol_line', primitive_id='patrol_1',
                              max_laps=1, hold_time_at_endpoint_s=0.0, keep_moving_on_end=False)
    goal = Mock(request=request, is_cancel_requested=False)
    node = SimpleNamespace(_robot_position_in_global_frame=lambda: robot_xy,
                           _origin=lambda: origin, _distance_between=Mock(return_value=11.1),
                           _end_reason=None, _navigation=Mock(), _stop_robot=Mock(),
                           _final_pose=lambda: PoseStamped(), get_logger=Mock())
    targets = []
    node._laps = lambda: PatrolNode._laps(node)
    node._resolve_end = lambda: PatrolNode._resolve_end(node)
    node._dispatch_next = lambda: PatrolNode._dispatch_next(node)
    node._on_reached = lambda: PatrolNode._on_reached(node)
    def navigate(position, done, heading):
        targets.append(position)
        assert heading == pytest.approx(__import__('math').atan2(
            position[1]-robot_xy[1], position[0]-robot_xy[0]))
        done(GoalStatus.STATUS_SUCCEEDED)
    node._navigation.send.side_effect = navigate
    node._navigation_done = lambda status: PatrolNode._navigation_done(node, status)
    result = asyncio.run(PatrolNode._execute(node, goal))
    points = [lla_to_enu(p.latitude, p.longitude, p.altitude, *origin) for p in (a, b)]
    assert targets == [points[index] for index in order]
    assert result.success and result.reached_points == 3 and result.laps == 1
    goal.succeed.assert_called_once()


def test_patrol_rejects_goal_without_fresh_localization():
    from rclpy.action import GoalResponse
    node = SimpleNamespace(_active=False, _origin_is_set=lambda: True,
                           _robot_position_in_global_frame=lambda: None, get_logger=Mock())
    assert PatrolNode._on_goal(node, Mock()) == GoalResponse.REJECT
    assert not node._active


def test_navigation_client_preserves_patrol_travel_heading():
    import math
    client, transport, requests = make_client()
    client.send((0.0, -10.0, 0.0), Mock(), heading=-math.pi/2)
    orientation = requests[0][0].pose.pose.orientation
    assert orientation.z == pytest.approx(-math.sqrt(0.5))
    assert orientation.w == pytest.approx(math.sqrt(0.5))
