"""Camera mount YAML parsing and actual static-TF delivery, without robot nodes."""
from pathlib import Path
import subprocess
import time
from unittest.mock import patch

import pytest
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from robot_launch.bringup import nav2


def camera_options(tmp_path, mount, namespace='robot_2', sim_time='false', rewrite=False):
    path = tmp_path/'camera.yaml'
    path.write_text(yaml.safe_dump({'camera_mount': {'ros__parameters': mount}}))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(path), robot_namespace=namespace,
                                         use_sim_time=sim_time)
    rewritten = None
    try:
        if rewrite:
            from launch.substitutions import LaunchConfiguration
            rewritten = Path(nav2._rewrite_params_with_namespace(
                context, LaunchConfiguration('params_file'), LaunchConfiguration('robot_namespace')))
            context.launch_configurations['params_file'] = str(rewritten)
        with patch.object(nav2, 'Node') as create:
            actions = nav2._camera_transform(context)
        return context, create, actions
    finally:
        if rewritten is not None:
            rewritten.unlink()


def mount():
    return dict(enabled=True, parent_frame='base_link', child_frame='front_camera_optical_frame',
                xyz=[.25, -.02, .6], rpy=[-1.5707963267948966, 0., -1.5707963267948966])


@pytest.mark.parametrize('namespace', ['', 'robot_1', 'field_robot'])
@pytest.mark.parametrize('sim_time', ['false', 'true'])
def test_active_yaml_namespace_and_clock(tmp_path, namespace, sim_time):
    data = mount()
    context, create, actions = camera_options(tmp_path, data, namespace, sim_time, rewrite=True)
    assert len(actions) == 1
    options = create.call_args.kwargs
    arguments = dict(zip(options['arguments'][::2], options['arguments'][1::2]))
    prefix = namespace+'/' if namespace else ''
    assert arguments['--frame-id'] == prefix+'base_link'
    assert arguments['--child-frame-id'] == prefix+'front_camera_optical_frame'
    for flag, value in zip(('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), data['xyz']+data['rpy']):
        assert float(arguments[flag]) == value
    assert options['parameters'][0]['use_sim_time'].evaluate(context) == (sim_time == 'true')


def test_already_prefixed_frames_are_not_prefixed_again(tmp_path):
    data = mount()
    data.update(parent_frame='robot_2/base_link', child_frame='robot_2/front_camera_optical_frame')
    _, create, _ = camera_options(tmp_path, data)
    arguments = create.call_args.kwargs['arguments']
    assert 'robot_2/front_camera_optical_frame' in arguments
    assert not any('robot_2/robot_2' in arg for arg in arguments)


@pytest.mark.parametrize('data', [{}, {'enabled': False}, {'enabled': False, 'xyz': 'not configured'}])
def test_optional_mount_does_not_start_publisher(tmp_path, data):
    _, create, actions = camera_options(tmp_path, data)
    assert actions == []
    create.assert_not_called()


@pytest.mark.parametrize('overrides', [
    {'enabled': 'true'}, {'xyz': [0., 0.]}, {'xyz': [True, 0., 0.]},
    {'xyz': [float('nan'), 0., 0.]}, {'rpy': [0., float('inf'), 0.]},
    {'rpy': '0,0,0'}, {'parent_frame': '/base_link'},
    {'child_frame': ''}, {'child_frame': 'camera frame'}, {'child_frame': 'base_link'},
])
def test_invalid_enabled_mount_is_rejected(tmp_path, overrides):
    with pytest.raises(RuntimeError):
        camera_options(tmp_path, {**mount(), **overrides})


def test_camera_optical_mount_is_received_over_tf_static(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    _, create, _ = camera_options(tmp_path, mount(), namespace='camera_mount_test')
    arguments = create.call_args.kwargs['arguments']
    binary = Path(get_package_prefix('tf2_ros'))/'lib/tf2_ros/static_transform_publisher'
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('camera_mount_test_listener', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    with (tmp_path/'static_tf.log').open('wb') as output:
        process = subprocess.Popen([str(binary), *arguments], stdout=output, stderr=subprocess.STDOUT)
        try:
            parent = 'camera_mount_test/base_link'
            child = 'camera_mount_test/front_camera_optical_frame'
            deadline = time.monotonic()+8.
            while time.monotonic() < deadline and not buffer.can_transform(parent, child, Time()):
                assert process.poll() is None, 'Static TF publisher exited'
                executor.spin_once(timeout_sec=.1)
            transform = buffer.lookup_transform(parent, child, Time()).transform
            assert (transform.translation.x, transform.translation.y, transform.translation.z) == pytest.approx(mount()['xyz'])
            q = transform.rotation
            # Forward camera: optical X -> base -Y; optical Y -> base -Z;
            # optical Z -> base +X, including the optical-axis conversion once.
            assert (q.x, q.y, q.z, q.w) == pytest.approx((-.5, .5, -.5, .5))
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.)
            executor.shutdown()
            listener.unregister()
            node.destroy_node()
            rclpy.shutdown(context=context)
