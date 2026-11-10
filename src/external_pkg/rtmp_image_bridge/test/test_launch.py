import importlib.util
from pathlib import Path
from unittest.mock import patch
from launch import LaunchContext

spec=importlib.util.spec_from_file_location('rtmp_launch_under_test',
    Path(__file__).parents[1]/'launch/rtmp.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def context(**overrides):
    context=LaunchContext()
    context.launch_configurations.update(namespace='robot_2',config_file='/tmp/rtmp.yaml',
        rtmp_url='',mode='',image_topic='',use_sim_time='false',**overrides)
    return context


def test_yaml_url_is_preserved_when_no_launch_override():
    with patch.object(module,'Node') as create:module._create_bridge(context())
    params=create.call_args.kwargs['parameters']
    assert params[0].perform(context())=='/tmp/rtmp.yaml'
    assert 'rtmp_url' not in params[1] and 'mode' not in params[1]


def test_explicit_launch_url_and_namespace_override_yaml():
    ctx=context()
    ctx.launch_configurations.update(rtmp_url='rtmp://192.168.1.186/live/robot_2',mode='listen',
                                     image_topic='camera/rtmp',use_sim_time='true')
    with patch.object(module,'Node') as create:module._create_bridge(ctx)
    options=create.call_args.kwargs
    assert options['namespace'].perform(ctx)=='robot_2'
    params=options['parameters'][1]
    assert params['rtmp_url'].evaluate(ctx)=='rtmp://192.168.1.186/live/robot_2'
    assert params['mode'].evaluate(ctx)=='listen'
    assert params['image_topic'].evaluate(ctx)=='camera/rtmp'
    assert params['use_sim_time'].evaluate(ctx) is True
