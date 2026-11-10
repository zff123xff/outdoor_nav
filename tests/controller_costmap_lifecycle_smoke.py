#!/usr/bin/env python3
"""Configure real controller/costmap nodes in an isolated ROS domain; no motion."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from robot_launch.bringup.nav2 import _rewrite_params_with_namespace


def call(node, client, request):
    assert client.wait_for_service(timeout_sec=8.), client.srv_name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
    assert future.done(), client.srv_name + ' timed out'
    return future.result()


def check(node, directory, invalid):
    namespace = 'invalid_costmap_fixture' if invalid else 'valid_costmap_fixture'
    source = Path(__file__).resolve().parents[1]/'src/robot_launch/config/go2w_nav2.yaml'
    config = yaml.safe_load(source.read_text())
    if invalid:
        config['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 2
    params = directory/(namespace+'.yaml')
    params.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(params), robot_namespace=namespace, use_sim_time='false')
    rewritten = Path(_rewrite_params_with_namespace(
        context, LaunchConfiguration('params_file'), LaunchConfiguration('robot_namespace')))
    executable = Path(get_package_prefix('nav2_controller'))/'lib/nav2_controller/controller_server'
    log_path = directory/(namespace+'.log')
    with log_path.open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/'+namespace,
                                    '--params-file', str(rewritten)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            change = node.create_client(ChangeState, '/'+namespace+'/controller_server/change_state')
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            response = call(node, change, request)
            assert response.success is (not invalid), log_path.read_text()
            state_client = node.create_client(GetState, '/'+namespace+'/controller_server/get_state')
            state = call(node, state_client, GetState.Request()).current_state.id
            assert state == (State.PRIMARY_STATE_UNCONFIGURED if invalid else State.PRIMARY_STATE_INACTIVE)
            if invalid:
                assert 'Local costmap configuration failed' in log_path.read_text()
                assert 'Created controller :' not in log_path.read_text()
            else:
                child = node.create_client(GetState, '/'+namespace+'/local_costmap/local_costmap/get_state')
                assert call(node, child, GetState.Request()).current_state.id == State.PRIMARY_STATE_INACTIVE
                parameters = node.create_client(GetParameters, '/'+namespace+'/local_costmap/local_costmap/get_parameters')
                request = GetParameters.Request(names=[
                    'obstacle_layer.min_points_per_cell', 'obstacle_layer.point_count_window_s'])
                values = call(node, parameters, request).values
                expected = config['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
                assert values[0].integer_value == expected['min_points_per_cell']
                assert values[1].double_value == expected['point_count_window_s']
                controller_parameters = node.create_client(
                    GetParameters, '/'+namespace+'/controller_server/get_parameters')
                weights = call(node, controller_parameters, GetParameters.Request(names=[
                    'FollowPath.w_traversal_cost', 'FollowPath.w_euc_cost',
                    'FollowTarget.w_traversal_cost', 'FollowTarget.w_euc_cost'])).values
                configured = config['controller_server']['ros__parameters']
                assert [v.double_value for v in weights] == [
                    configured[plugin][key] for plugin in ('FollowPath', 'FollowTarget')
                    for key in ('w_traversal_cost', 'w_euc_cost')]
            assert process.poll() is None
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            rewritten.unlink(missing_ok=True)
        assert process.returncode == 0, log_path.read_text()
    print('PASS:', namespace, 'configuration rejected' if invalid else 'controller and costmap configured')


def main():
    rclpy.init()
    node = Node('controller_costmap_lifecycle_fixture')
    try:
        with tempfile.TemporaryDirectory(prefix='controller_costmap_') as directory:
            for invalid in (True, False):
                check(node, Path(directory), invalid)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
