"""Validate point-goal YAML normalization for its Action executor."""

from pathlib import Path

import pytest
import yaml

from task_pipeline.send_point_goal import load_task, main


TASK_FILE = Path(__file__).resolve().parents[1] / 'config/point_goal.yaml'


def test_enu_task_becomes_pipeline_request():
    service, task, _ = load_task(
        TASK_FILE, nav_origin=(40.9445353, 113.172819, 1298.7345711102703))
    assert service == '/agent/robot_1/execute_task'
    assert task['capability_id'] == 'goto_waypoints'
    assert task['params']['waypoints'][0]['lat'] == pytest.approx(40.9445452, abs=2e-7)
    assert task['params']['waypoints'][0]['lon'] == pytest.approx(113.172819, abs=2e-7)
    assert task['params']['waypoints'][0]['alt'] == pytest.approx(1298.734571, abs=0.01)


def test_namespace_override_selects_matching_action():
    service, task, _ = load_task(TASK_FILE, namespace_override='robot_2')
    assert service == '/agent/robot_2/execute_task'
    assert task['agent_id'] == 'robot_2'


def test_dry_run_does_not_call_ros(capsys):
    assert main(['--file', str(TASK_FILE), '--dry-run']) == 0
    output = capsys.readouterr().out
    assert 'ExecuteTask 服务: /agent/robot_1/execute_task' in output
    assert '"waypoints"' in output


def test_rejects_multiple_waypoints(tmp_path):
    config = yaml.safe_load(TASK_FILE.read_text())
    config['task']['params']['waypoints'].append(config['task']['params']['waypoints'][0])
    path = tmp_path / 'multi.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='exactly one point'):
        load_task(path)
