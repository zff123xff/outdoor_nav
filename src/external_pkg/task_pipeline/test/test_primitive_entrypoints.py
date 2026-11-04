"""All navigation primitive senders use the same direct pipeline client."""
from pathlib import Path

import pytest

from task_pipeline.examples import topic_client
from task_pipeline.examples.sender import send_area_search, send_patrol, send_point_goal, send_multi_point


CONFIG = Path(__file__).resolve().parents[1] / 'config'
SENDERS = [
    (send_point_goal, 'examples/enu/point_goal.yaml', 'point_goal'),
    (send_patrol, 'examples/gps/patrol_line.yaml', 'patrol_line'),
    (send_area_search, 'examples/enu/area_search.yaml', 'area_search'),
    (send_multi_point, 'examples/gps/multi_point.yaml', 'multi_point'),
]


@pytest.mark.parametrize('sender,filename,capability', SENDERS)
def test_direct_pipeline_submission(monkeypatch, sender, filename, capability):
    calls = []
    monkeypatch.setattr(topic_client, 'resolve_navigation_origin',
                        lambda *args: (40.9445353, 113.172819, 1298.7345711102703))
    monkeypatch.setattr(topic_client, 'submit', lambda *args: calls.append(args) or 17)
    assert sender(['--file', str(CONFIG / filename), '--namespace', 'robot_2']) == 17
    task, namespace, timeout, check, cancel, ros_args = calls[0]
    assert namespace == 'robot_2'
    assert task['task_id']
    assert task['task_type'] == capability
    assert not check
    if capability == 'point_goal':
        assert 'latitude' in task['waypoints'][0]


@pytest.mark.parametrize('sender,filename,capability', SENDERS)
def test_check_needs_only_pipeline_service(monkeypatch, sender, filename, capability):
    def unexpected_origin_read(*args):
        raise AssertionError('--check must not read navigation parameters or submit a goal')

    calls = []
    monkeypatch.setattr(topic_client, 'resolve_navigation_origin', unexpected_origin_read)
    monkeypatch.setattr(topic_client, 'submit', lambda *args: calls.append(args) or 0)
    assert sender(['--file', str(CONFIG / filename), '--check']) == 0
    assert calls[0][3] is True


def test_area_cancel_uses_pipeline(monkeypatch):
    calls = []
    monkeypatch.setattr(topic_client, 'submit', lambda *args: calls.append(args) or 0)
    assert send_area_search(['--file', str(CONFIG / 'examples/enu/area_search.yaml'), '--cancel']) == 0
    assert calls[0][4] is True


@pytest.mark.parametrize('frame', ['enu', 'gps'])
@pytest.mark.parametrize('sender,filename,capability', SENDERS)
def test_coordinate_option_selects_matching_yaml(monkeypatch, sender, filename, capability, frame):
    import ament_index_python.packages
    from task_pipeline.examples.sender import load_example

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda name: str(CONFIG.parent))
    origins, calls = [], []
    origin = (40.9445353, 113.172819, 1298.7345711102703)
    monkeypatch.setattr(topic_client, 'resolve_navigation_origin',
                        lambda *args: origins.append(args) or origin)
    monkeypatch.setattr(topic_client, 'submit', lambda *args: calls.append(args) or 0)
    assert sender(['--coordinate-frame', frame]) == 0
    expected_ns, expected_task = load_example(CONFIG/'examples'/frame/(capability+'.yaml'),
                                              capability, origin=origin if frame == 'enu' else None)
    assert calls[0][0] == expected_task
    assert calls[0][1] == expected_ns
    assert len(origins) == (1 if frame == 'enu' else 0)


def test_explicit_file_overrides_coordinate_selection(monkeypatch):
    calls = []
    def unexpected_origin_read(*args):
        raise AssertionError('Explicit GPS file must not request an ENU origin')
    monkeypatch.setattr(topic_client, 'resolve_navigation_origin', unexpected_origin_read)
    monkeypatch.setattr(topic_client, 'submit', lambda *args: calls.append(args) or 0)
    assert send_point_goal(['--coordinate-frame', 'enu', '--file',
                            str(CONFIG/'examples/gps/point_goal.yaml')]) == 0
    assert calls[0][0]['task_type'] == 'point_goal'
