"""All navigation primitive senders use the same direct pipeline client."""
from pathlib import Path

import pytest

from task_pipeline import send_area_search, send_patrol, send_point_goal, service_client


CONFIG = Path(__file__).resolve().parents[1] / 'config'
SENDERS = [
    (send_point_goal, 'point_goal.yaml', 'goto_waypoints'),
    (send_patrol, 'patrol_goal.yaml', 'patrol_line'),
    (send_area_search, 'area_search.yaml', 'area_search'),
]


@pytest.mark.parametrize('sender,filename,capability', SENDERS)
def test_direct_pipeline_submission(monkeypatch, sender, filename, capability):
    calls = []
    monkeypatch.setattr(service_client, 'resolve_navigation_origin',
                        lambda *args: (40.9445353, 113.172819, 1298.7345711102703))
    monkeypatch.setattr(service_client, 'submit', lambda *args: calls.append(args) or 17)
    assert sender.main(['--file', str(CONFIG / filename), '--namespace', 'robot_2']) == 17
    task, service, namespace, timeout, check, ros_args = calls[0]
    assert service == '/agent/robot_2/execute_task'
    assert namespace == task['agent_id'] == 'robot_2'
    assert task['primitive_id'] and task['mission_id']
    assert task['capability_id'] == capability
    assert not check
    if capability == 'goto_waypoints':
        assert task['params']['waypoint_frame'] == 'gps'


@pytest.mark.parametrize('sender,filename,capability', SENDERS)
def test_check_needs_only_pipeline_service(monkeypatch, sender, filename, capability):
    def unexpected_origin_read(*args):
        raise AssertionError('--check must not read navigation parameters or submit a goal')

    calls = []
    monkeypatch.setattr(service_client, 'resolve_navigation_origin', unexpected_origin_read)
    monkeypatch.setattr(service_client, 'submit', lambda *args: calls.append(args) or 0)
    assert sender.main(['--file', str(CONFIG / filename), '--check']) == 0
    assert calls[0][4] is True


def test_area_cancel_uses_pipeline(monkeypatch):
    calls = []
    monkeypatch.setattr(service_client, 'submit', lambda *args: calls.append(args) or 0)
    assert send_area_search.main(['--file', str(CONFIG / 'area_search.yaml'), '--cancel']) == 0
    assert calls[0][0]['capability_id'] == 'cancel_area_search'
