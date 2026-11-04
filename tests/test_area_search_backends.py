"""The same area_search primitive selects exactly one named implementation."""
import importlib.util
from pathlib import Path
from unittest.mock import patch
import pytest
import yaml
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('search_pipeline', ROOT/'src/robot_launch/robot_launch/bringup/pipeline.py')
pipeline = importlib.util.module_from_spec(spec); spec.loader.exec_module(pipeline)


def test_only_co_search_executor_is_started(tmp_path):
    config = tmp_path/'task.yaml'; config.write_text('search_controller: co_search\n')
    context = LaunchContext(); context.launch_configurations.update(
        area_search_config=str(config), robot_namespace='robot_1', use_sim_time='false')
    with patch.object(pipeline, 'Node', side_effect=lambda **kwargs: kwargs):
        nodes = pipeline._area_search_executor(context)
    assert len(nodes) == 1
    assert nodes[0]['package'] == 'co_search_nav'
    assert nodes[0]['executable'] == 'co_search_executor'
    assert nodes[0]['name'] == 'area_search_co_search'


@pytest.mark.parametrize('controller', ['legacy', 'typo'])
def test_removed_or_unknown_controller_is_rejected(tmp_path, controller):
    config = tmp_path/'task.yaml'; config.write_text('search_controller: '+controller+'\n')
    context = LaunchContext(); context.launch_configurations.update(area_search_config=str(config))
    with pytest.raises(ValueError, match='search_controller'):
        pipeline._area_search_executor(context)


def test_area_sender_uses_gps_topic_schema():
    from task_pipeline.examples.sender import load_example
    ns, task = load_example(ROOT/'src/external_pkg/task_pipeline/config/examples/enu/area_search.yaml',
        'area_search', 'test_robot', (40., 113., 1200.))
    assert ns == 'test_robot'
    assert task['task_type'] == 'area_search'
    assert task['area']['center']['latitude'] > 39.


def test_legacy_executable_and_modules_are_removed():
    package = ROOT/'src/external_pkg/task_pipeline'
    assert 'task_pipeline.area_search:main' not in (package/'setup.py').read_text()
    for name in ('area_search.py', 'area_search_core.py', 'path_refinement.py'):
        assert not (package/'task_pipeline'/name).exists()
