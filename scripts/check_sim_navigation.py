#!/usr/bin/env python3
"""Check shared navigation parameters, both ROS clocks and namespace isolation."""
from pathlib import Path
from unittest.mock import patch
import tempfile
import yaml
import rclpy
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from robot_launch.bringup import nav2, visualization

source = Path(__file__).resolve().parents[1]/'src/robot_launch/config/go2w_nav2.yaml'
expected = yaml.safe_load(source.read_text())['nav2_input_bridge']['ros__parameters']
for namespace in ('robot', 'robot_1', 'field_robot'):
    for use_clock in (False, True):
        context = LaunchContext()
        context.launch_configurations.update(robot_namespace=namespace, params_file=str(source),
                                             use_sim_time=str(use_clock).lower(), rviz='true')
        rewritten = Path(nav2._rewrite_params_with_namespace(
            context, LaunchConfiguration('params_file'), LaunchConfiguration('robot_namespace')))
        try:
            params = yaml.safe_load(rewritten.read_text())
            bridge = params['/'+namespace+'/nav2_input_bridge']['ros__parameters']
            costmap = params['/'+namespace+'/local_costmap']['local_costmap']['ros__parameters']
            pointcloud = costmap['obstacle_layer']['pointcloud']
            configured_cloud = yaml.safe_load(source.read_text())['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['pointcloud']
            assert pointcloud['observation_persistence'] == configured_cloud['observation_persistence']
            assert 'observation_keep_time' not in pointcloud
            assert pointcloud['height_reference_frame'] == namespace+'/base_link'
            assert pointcloud['height_reference_ground_z'] == configured_cloud['height_reference_ground_z']
            assert '/'+namespace+'/ground_tilt_filter' not in params
            assert costmap['global_frame'] == namespace+'/map'
            assert costmap['robot_base_frame'] == namespace+'/base_link'
            assert costmap['use_sim_time'] is use_clock
            assert bridge['use_sim_time'] is use_clock
            for key in ('reference_latitude', 'reference_longitude', 'reference_altitude'):
                assert bridge[key] == expected[key]
            rclpy.init(args=['--ros-args', '--params-file', str(rewritten)])
            rclpy.shutdown()
        finally:
            rewritten.unlink()
    with patch.object(visualization, 'Node') as create:
        visualization.bringup(context)
    assert len(create.call_args_list) == 2
    merger = create.call_args_list[0].kwargs
    assert merger['package'] == 'robot_launch' and merger['executable'] == 'visualization_tf'
    assert merger['condition'].evaluate(context)
    rviz = create.call_args.kwargs
    assert rviz['remappings'] == [('/tf', '/' + namespace + '/visualization/tf'),
                                   ('/tf_static', '/' + namespace + '/visualization/tf_static')]
    assert rviz['condition'].evaluate(context)
    context.launch_configurations['rviz'] = 'false'
    assert not merger['condition'].evaluate(context)
    assert not rviz['condition'].evaluate(context)
    context.launch_configurations['rviz'] = 'true'
    rviz_path = Path(rviz['arguments'][1])
    try:
        visual = yaml.safe_load(rviz_path.read_text())['Visualization Manager']
        assert visual['Global Options']['Fixed Frame'] == namespace+'/map'
        model = next(d for d in visual['Displays'] if d['Class']=='rviz_default_plugins/RobotModel')
        assert model['TF Prefix'] == namespace
        for display in visual['Displays']:
            if display.get('Class') == 'rviz_default_plugins/RobotModel':
                continue
            for key in ('Topic', 'Update Topic'):
                if key in display:
                    assert display[key]['Value'].startswith('/'+namespace+'/')
        for suffix in ('region', 'assigned_region', 'map'):
            display = next(d for d in visual['Displays'] if d.get('Topic', {}).get('Value') == '/'+namespace+'/area_search/'+suffix)
            assert display['Topic']['Durability Policy'] == 'Transient Local'
        goal_tool = next(t for t in visual['Tools'] if t['Class']=='rviz_default_plugins/SetGoal')
        assert goal_tool['Topic'] == '/'+namespace+'/goal_pose'
    finally:
        rviz_path.unlink()
    print(namespace, 'shared navigation parameters, clocks and RViz passed')

# Mount values must come from the active YAML, including custom parameter files.
with tempfile.TemporaryDirectory() as directory:
    custom = Path(directory)/'custom.yaml'
    data = yaml.safe_load(source.read_text())
    expected_mount = {'xyz': [0.27, -0.03, 0.61], 'rpy': [0.02, 0.31, -0.04]}
    data['lidar_mount']['ros__parameters'] = expected_mount
    custom.write_text(yaml.safe_dump(data))
    context.launch_configurations['params_file'] = str(custom)
    with patch.object(nav2, 'Node') as create:
        nav2._lidar_transform(context)
    arguments = create.call_args.kwargs['arguments']
    actual = dict(zip(arguments[::2], arguments[1::2]))
    for flag, value in zip(('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), expected_mount['xyz']+expected_mount['rpy']):
        assert float(actual[flag]) == value
    assert actual['--frame-id'] == namespace+'/base_link'
    assert actual['--child-frame-id'] == namespace+'/livox_frame'
    data['lidar_mount']['ros__parameters']['xyz'] = [0.0, 0.0]
    custom.write_text(yaml.safe_dump(data))
    try:
        nav2._lidar_transform(context)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Malformed mount accepted')
print('YAML lidar mount propagation and validation passed')
