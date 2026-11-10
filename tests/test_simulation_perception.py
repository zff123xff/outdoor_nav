"""Simulation perception composition uses the resolved namespace and scoped launch arguments."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from yolo_detector.targets import DEFAULT_TARGETS_FILE

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sim_perception_under_test', ROOT/'src/robot_launch/launch/simulation.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def context(yolo='true', video='true'):
    ctx = LaunchContext()
    ctx.launch_configurations.update(robot_namespace='field_robot', use_sim_time='true',
        yolo=yolo, video_stream=video, yolo_config='/tmp/detector.yaml',
        video_config='/tmp/stream.yaml', video_url='rtmp://localhost/live/test',
        yolo_python='/tmp/gpu/bin/python', yolo_targets_file=DEFAULT_TARGETS_FILE)
    return ctx


@pytest.mark.parametrize('yolo,video,count', [('true', 'true', 2), ('false', 'true', 1),
                                            ('true', 'false', 1), ('false', 'false', 0)])
def test_perception_optional_nodes_are_scoped_and_share_transport(yolo, video, count):
    with patch.object(module, 'IncludeLaunchDescription') as include, patch.object(module, 'GroupAction') as group:
        actions = module.perception(context(yolo, video))
    assert len(actions) == group.call_count == include.call_count == count
    for call in include.call_args_list:
        args = dict(call.kwargs['launch_arguments'])
        assert args['namespace'] == 'field_robot'
        assert args['use_sim_time'] == 'true'
        if 'python_executable' in args:
            assert args['python_executable'] == '/tmp/gpu/bin/python'
            assert args['config_file'] == '/tmp/detector.yaml'
            assert args['targets_file'] == DEFAULT_TARGETS_FILE
        else:
            assert args['config_file'] == '/tmp/stream.yaml'
            assert args['rtmp_url'] == 'rtmp://localhost/live/test'


def test_default_startup_enables_perception_with_common_target_path():
    ctx = LaunchContext()
    args = {action.name: action for action in module.generate_launch_description().entities
            if isinstance(action, DeclareLaunchArgument)}
    assert perform_substitutions(ctx, args['yolo'].default_value) == 'true'
    assert perform_substitutions(ctx, args['video_stream'].default_value) == 'true'
    assert perform_substitutions(ctx, args['yolo_targets_file'].default_value) == DEFAULT_TARGETS_FILE


def test_custom_target_file_is_passed_to_detector():
    ctx = context('true', 'false')
    ctx.launch_configurations['yolo_targets_file'] = '/tmp/custom/classes.yaml'
    with patch.object(module, 'IncludeLaunchDescription') as include:
        module.perception(ctx)
    assert dict(include.call_args.kwargs['launch_arguments'])['targets_file'] == '/tmp/custom/classes.yaml'


def test_full_hd_video_detector_and_calibration_are_consistent():
    import yaml
    detector = yaml.safe_load((ROOT/'src/yolo_detector/config/yolo.yaml').read_text())['/**']['ros__parameters']
    video = yaml.safe_load((ROOT/'src/external_pkg/rtmp_image_bridge/config/rtmp.yaml').read_text())['/**']['ros__parameters']
    assert (video['width'], video['height']) == (1920, 1080)
    assert (detector['expected_width'], detector['expected_height']) == (1920, 1080)
    assert (detector['camera_width'], detector['camera_height']) == (1920, 1080)
    assert (detector['model_input_width'], detector['model_input_height']) == (640, 480)
    assert detector['image_topic'] == video['image_topic']
    assert detector['expected_encoding'] == video['encoding']
    assert detector['targets_file'] == DEFAULT_TARGETS_FILE
