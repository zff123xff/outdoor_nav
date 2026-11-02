"""Verify patrol YAML becomes an ExecuteTask pipeline request."""

from pathlib import Path

import pytest
import yaml

from task_pipeline.send_patrol import load_task, main


TASK_FILE = Path(__file__).resolve().parents[1] / 'config/patrol_goal.yaml'


def test_task_round_trip():
    service, task, _ = load_task(TASK_FILE)
    assert service == '/agent/robot_1/execute_task'
    assert task['capability_id'] == 'patrol_line'
    assert task['params']['waypoint_frame'] == 'gps'
    assert len(task['params']['waypoints']) == 2
    assert task['params']['max_laps'] == 1
    assert task['primitive_id'] == 'patrol_001'
    assert task['params']['hold_time_at_endpoint_s'] == 0.0


def test_namespace_override_selects_matching_action():
    service, task, _ = load_task(TASK_FILE, namespace_override='robot_2')
    assert service == '/agent/robot_2/execute_task'
    assert task['agent_id'] == 'robot_2'


def test_dry_run_does_not_call_ros(capsys):
    assert main([str(TASK_FILE), '--dry-run']) == 0
    output = capsys.readouterr().out
    assert 'ExecuteTask 服务: /agent/robot_1/execute_task' in output
    assert '"capability_id": "patrol_line"' in output


@pytest.mark.parametrize('value', [float('nan'), float('inf'), 91, True, '30.0'])
def test_reject_invalid_latitude(tmp_path, value):
    task = yaml.safe_load(TASK_FILE.read_text())
    task['task']['params']['waypoints'][0]['gps']['lat'] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(task))
    with pytest.raises(ValueError):
        load_task(path)
