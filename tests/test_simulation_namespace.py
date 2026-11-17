"""All simulation-launch siblings must share the resolved robot namespace."""
import importlib.util
from pathlib import Path
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('simulation_launch_under_test', ROOT/'src/robot_launch/launch/simulation.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('override,configured,expected', [
    ('', None, 'robot_1'),
    ('', 'field_robot', 'field_robot'),
    ('robot_2', 'field_robot', 'robot_2'),
])
def test_fallback_namespace_is_published_to_pipeline_and_visualization(tmp_path, override, configured, expected):
    config = {} if configured is None else {'launch_config': {'ros__parameters': {'robot_namespace': configured}}}
    path = tmp_path/'params.yaml'; path.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    targets = tmp_path/'targets.yaml'
    targets.write_text('task_id: stale\ntargets: [person]\n')
    context.launch_configurations.update(params_file=str(path), robot_namespace=override,
                                        targets_file=str(targets))
    actions = module.bringup(context)
    actions[0].execute(context)
    assert LaunchConfiguration('robot_namespace').perform(context) == expected
    assert yaml.safe_load(targets.read_text()) == {'task_id': 'stale', 'targets': ['person']}
    assert LaunchConfiguration('targets_file').perform(context) == str(targets)
    # Subsequent sibling includes and OpaqueFunctions read the parent context.
    context._push_launch_configurations()
    context._pop_launch_configurations()
    assert LaunchConfiguration('robot_namespace').perform(context) == expected


def test_simulation_defaults_to_video_and_detector_autostart():
    arguments = {action.name: action for action in module.generate_launch_description().entities
                 if isinstance(action, DeclareLaunchArgument)}
    assert 'yolo' in arguments
    assert 'video_stream' in arguments
    assert 'target_interaction_enabled' not in arguments
    assert 'inspection_udp' in arguments
    assert 'use_ue_gps_origin' in arguments
    assert 'origin_source' not in arguments  # No wait-for-origin startup gate.
    assert 'origin_timeout_s' not in arguments
