#!/usr/bin/env python3
"""Isolated real controller -> twist_mux -> unchanged bridge -> loopback UDP check."""
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def main():
    bridge_path = Path(os.environ.get('ROS_TO_UE_BRIDGE', '/mnt/d/shixi/MavrosBridge/ros_to_ue_bridge.py'))
    spec = importlib.util.spec_from_file_location('velocity_bridge', bridge_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.setblocking(False)
    module.UE_IP, module.UE_PORT = receiver.getsockname()
    rclpy.init()
    bridge = module.UnifiedRosToUeBridge({'dog'}, {'robot_1'})
    node = Node('velocity_transport_fixture', namespace='/robot_1')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(bridge)
    broadcaster = TransformBroadcaster(node)
    def tick():
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
    timer = node.create_timer(.02, tick)
    nav, mux, packets = [], [], []
    subs = [node.create_subscription(Twist, topic, callback, 10) for topic, callback in (
        ('cmd_vel_nav', lambda m: nav.append((m.linear.x, m.angular.z))),
        ('cmd_vel', lambda m: mux.append((m.linear.x, m.angular.z))))]
    def spin(seconds, condition=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            while True:
                try:
                    packet = json.loads(receiver.recv(65536))
                    assert packet['robdog'] == 'robot_1' and packet['topic'] == '/cmd_vel'
                    packets.append((packet['data']['linear']['x'], packet['data']['angular']['z']))
                except BlockingIOError:
                    break
            if condition and condition():
                return True
        return False
    def result(future):
        assert spin(12, future.done), 'ROS request timed out'
        return future.result()
    processes = []
    with tempfile.TemporaryDirectory(prefix='velocity_transport_') as directory:
        directory = Path(directory)
        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 'src/robot_launch/config/go2w_nav2.yaml').read_text())
        controller = config['controller_server']['ros__parameters']
        controller['progress_checker']['movement_time_allowance'] = 30.0
        # Distinct non-default values prove YAML loading, not constructor defaults.
        for name in ('FollowPath', 'FollowTarget'):
            controller[name].update(desired_linear_vel=.37, tracking_angular_vel_limit=.17,
                                    rotate_to_heading_angular_vel=.29)
        costmap = config['local_costmap']['local_costmap']['ros__parameters']
        costmap.update(plugins=['inflation_layer'], global_frame='map', robot_base_frame='base_link',
                       transform_tolerance=.5, width=20, height=20)
        params = directory/'controller.yaml'
        params.write_text(yaml.safe_dump({'/robot_1/controller_server': {'ros__parameters': controller},
            '/robot_1/local_costmap/local_costmap': {'ros__parameters': costmap}}))
        mux_params = directory/'mux.yaml'
        mux_params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'topics': {
            'nav': {'topic': 'cmd_vel_nav', 'timeout': .5, 'priority': 10},
            'remote': {'topic': 'cmd_vel_remote', 'timeout': .5, 'priority': 20}}}}}))
        log = open('/tmp/velocity_transport_smoke.log', 'w')
        try:
            for package, executable, extra in (
                ('nav2_controller', 'controller_server', ['--params-file', str(params),
                                                        '-r', 'cmd_vel:=cmd_vel_nav']),
                ('twist_mux', 'twist_mux', ['--params-file', str(mux_params),
                                           '-r', 'cmd_vel_out:=cmd_vel'])):
                path = Path(get_package_prefix(package))/'lib'/package/executable
                processes.append(subprocess.Popen([str(path), '--ros-args', '-r', '__ns:=/robot_1', *extra],
                    stdout=log, stderr=log, start_new_session=True))
            change = node.create_client(ChangeState, 'controller_server/change_state')
            assert change.wait_for_service(timeout_sec=10)
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                assert result(change.call_async(request)).success
            action = ActionClient(node, FollowPath, 'follow_path')
            assert action.wait_for_server(timeout_sec=5)
            parameter_client = node.create_client(SetParametersAtomically, 'controller_server/set_parameters_atomically')
            assert parameter_client.wait_for_service(timeout_sec=5)
            def update(plugin, key, value):
                kind = ParameterType.PARAMETER_BOOL if isinstance(value, bool) else ParameterType.PARAMETER_DOUBLE
                val = ParameterValue(type=kind)
                if kind == ParameterType.PARAMETER_BOOL:
                    val.bool_value = value
                else:
                    val.double_value = value
                response = result(parameter_client.call_async(SetParametersAtomically.Request(
                    parameters=[Parameter(name=plugin+'.'+key, value=val)])))
                assert response.result.successful, response.result.reason
            def run_case(plugin, angle, expected_v, angular_bound, require_turn=False):
                nav.clear(); mux.clear(); packets.clear()
                goal = FollowPath.Goal(controller_id=plugin, goal_checker_id='general_goal_checker')
                goal.path.header.frame_id = 'map'
                for index in range(51):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = .1*index*math.cos(angle)
                    pose.pose.position.y = .1*index*math.sin(angle)
                    pose.pose.orientation.w = 1.0
                    goal.path.poses.append(pose)
                handle = result(action.send_goal_async(goal))
                assert handle.accepted
                def matches(values):
                    return any(abs(v-expected_v) < .001 and abs(w) <= angular_bound+.001 and
                               (not require_turn or abs(w) > .01) for v,w in values)
                assert spin(3, lambda: matches(nav) and matches(mux) and matches(packets)), (nav[-5:], mux[-5:], packets[-5:])
                assert all(abs(w) <= angular_bound+.001 for v,w in packets)
                for pair in packets:
                    assert pair in nav, ('UDP values changed', pair)
                assert result(handle.cancel_goal_async()).goals_canceling
                result(handle.get_result_async())
                spin(.15)
                print('PASS:', plugin, 'angle=', angle, 'vx=', expected_v, 'angular_bound=', angular_bound,
                      'controller/mux/UDP matched')
            run_case('FollowPath', 0., .37, .17)
            run_case('FollowTarget', 0., .37, .17)
            run_case('FollowPath', math.pi/2, 0., .29, True)
            update('FollowPath', 'desired_linear_vel', .21)
            run_case('FollowPath', 0., .21, .17)
            update('FollowPath', 'use_rotate_to_heading', False)
            # A 0.4 rad carrot induces curvature > 1, limiting vx to preserve wz <= .17.
            nav.clear(); mux.clear(); packets.clear()
            # Straight angle zero separately established exact linear propagation;
            # check turning output with an intentionally high yaw demand below.
            # For tracking, predicted curvature = 2*sin(angle)/lookahead distance.
            lookahead = controller['FollowPath']['lookahead_dist']
            tracking_v = min(.21, .17/(2*math.sin(.4)/lookahead))
            run_case('FollowPath', .4, tracking_v, .17, True)
            remote = node.create_publisher(Twist, 'cmd_vel_remote', 10)
            spin(.3, lambda: remote.get_subscription_count() > 0)
            command = Twist()
            command.linear.x = -.11
            command.angular.z = -.12
            packets.clear()
            for _ in range(10):
                remote.publish(command)
                if spin(.1, lambda: (-.11, -.12) in packets and (-.11, -.12) in mux):
                    break
            assert (-.11, -.12) in mux and (-.11, -.12) in packets
            stop = Twist()
            packets.clear()
            for _ in range(10):
                remote.publish(stop)
                if spin(.1, lambda: (0., 0.) in packets):
                    break
            assert (0., 0.) in packets
            print('PASS: reverse/negative yaw and zero-stop preserved through mux/UDP')
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.close()
    executor.shutdown()
    bridge.destroy_node()
    node.destroy_node()
    receiver.close()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
