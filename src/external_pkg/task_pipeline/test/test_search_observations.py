"""Observation validation and mode dispatch without robot motion."""
import pytest
from visualization_msgs.msg import Marker
from task_pipeline.search_inspection_executor import marker_observation, observed_before


def sample():
    marker = Marker()
    marker.ns = 'yolo_targets'; marker.id = 1; marker.type = Marker.SPHERE
    marker.header.frame_id = 'robot_1/map'; marker.header.stamp.sec = 100
    marker.pose.position.x = 3.; marker.pose.position.y = 4.
    return marker


def test_fresh_marker_and_spatial_dedup_survive_id_change():
    marker = sample()
    result = marker_observation(marker, 'robot_1/map', 100.1, 99., 1.5)
    assert result['point'] == (3., 4., 0.)
    assert observed_before((3.2, 4., 0.), [result['point']], 1.)
    assert not observed_before((8., 4., 0.), [result['point']], 1.)


@pytest.mark.parametrize('failure', ['old', 'previous_task', 'future', 'frame', 'nan', 'delete', 'zero_id'])
def test_invalid_observations_are_ignored(failure):
    marker = sample(); now = 100.1; since = 99.
    if failure == 'old': now = 103.
    elif failure == 'previous_task': since = 101.
    elif failure == 'future': now = 97.
    elif failure == 'frame': marker.header.frame_id = 'other/map'
    elif failure == 'nan': marker.pose.position.x = float('nan')
    elif failure == 'delete': marker.action = Marker.DELETE
    elif failure == 'zero_id': marker.id = 0
    assert marker_observation(marker, 'robot_1/map', now, since, 1.5) is None


@pytest.mark.parametrize('fresh', [False, True])
def test_original_visitor_keeps_snapshot_and_private_visitor_requires_new_assignment(fresh):
    from types import SimpleNamespace as NS
    from task_pipeline.inspection_executor import InspectionExecutor
    from rclpy.action import GoalResponse
    old = {1: (1., 0., 0.)}
    node = NS(busy=False, latest=old, cfg={'require_fresh_assignment': fresh})
    assert InspectionExecutor.goal(node, NS(task_id='143')) == GoalResponse.ACCEPT
    assert node.latest == (None if fresh else old)


def test_search_inspection_enu_example_converts_center_and_preserves_143(tmp_path):
    from pathlib import Path
    from task_pipeline.examples.sender import load_example
    source = Path(__file__).resolve().parents[1]/'config/examples/enu/search_inspection.yaml'
    namespace, task = load_example(source, 'target_inspection', origin=(31., 120., 0.))
    assert namespace == 'robot_1' and task['task_id'] == 143
    assert task['center'] == [31., 120., 0.]
    assert task['length_m'] == task['width_m'] == 10.
