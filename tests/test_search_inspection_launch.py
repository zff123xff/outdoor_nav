"""The optional search-observation mode leaves assigned-target inspection available."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from robot_launch.bringup import pipeline


@pytest.mark.parametrize('enabled', ['false', 'true'])
def test_mode_reaches_dispatcher_and_only_enables_private_workers_when_selected(enabled):
    context = LaunchContext()
    context.launch_configurations.update(inspection_search_mode=enabled)
    with patch.object(pipeline, 'Node') as create:
        pipeline.generate_launch_description()
    nodes = [call.kwargs for call in create.call_args_list]
    dispatcher = next(n for n in nodes if n['executable'] == 'pipeline_dispatcher')
    assert dispatcher['parameters'][0]['inspection_search_mode'].evaluate(context) == (enabled == 'true')
    visitors = [n for n in nodes if n['executable'] == 'target_inspection_executor']
    assert len(visitors) == 2
    original = next(n for n in visitors if 'condition' not in n)
    private = next(n for n in visitors if 'condition' in n)
    assert 'name' not in original
    assert private['condition'].evaluate(context) == (enabled == 'true')
    assert private['parameters'][1]['action_name'] == 'search_target_inspection'
    assert private['parameters'][1]['assigned_topic'] == 'targets/search_assigned'
    assert private['parameters'][1]['require_fresh_assignment'] is True
    coordinator = next(n for n in nodes if n['executable'] == 'search_inspection_executor')
    assert coordinator['condition'].evaluate(context) == (enabled == 'true')


@pytest.mark.parametrize('entry', ['simulation', 'real'])
def test_public_launches_default_to_original_mode(entry):
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('search_mode_'+entry, root/f'src/robot_launch/launch/{entry}.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name:'/tmp/'+name):
        description = module.generate_launch_description()
    arguments = {a.name:a for a in description.entities if isinstance(a, DeclareLaunchArgument)}
    assert perform_substitutions(LaunchContext(), arguments['inspection_search_mode'].default_value) == 'false'
