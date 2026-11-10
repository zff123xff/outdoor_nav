"""YOLO launch preserves the GPU interpreter and shared task settings."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'yolo_launch_under_test', ROOT/'src/yolo_detector/launch/yolo.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_detector_uses_gpu_python_robot_namespace_clock_and_shared_targets():
    context = LaunchContext()
    context.launch_configurations.update(
        namespace='robot_2', config_file='/tmp/model.yaml',
        python_executable='/tmp/gpu/bin/python', use_sim_time='true',
        targets_file='/tmp/robot_2_targets.yaml')
    with patch.object(module, 'Node') as create:
        module._create_detector(context)
    options = create.call_args.kwargs
    assert options['namespace'].perform(context) == 'robot_2'
    assert options['prefix'].perform(context) == '/tmp/gpu/bin/python'
    assert options['parameters'][0].perform(context) == '/tmp/model.yaml'
    assert options['parameters'][1]['use_sim_time'].evaluate(context) is True
    assert options['parameters'][2]['targets_file'] == '/tmp/robot_2_targets.yaml'


def test_standalone_detector_preserves_targets_path_from_config():
    context = LaunchContext()
    context.launch_configurations.update(targets_file='')
    with patch.object(module, 'Node') as create:
        module._create_detector(context)
    assert len(create.call_args.kwargs['parameters']) == 2
