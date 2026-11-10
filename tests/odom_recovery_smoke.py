#!/usr/bin/env python3
"""Real controller outage/resume/cancel regression. Run in an isolated ROS domain."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init()
    node = Node('odom_recovery_fixture', namespace='/odom_recovery')
    broadcaster = TransformBroadcaster(node)
    flowing = True
    commands = []

    def tick():
        if flowing:
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_link'
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)

    timer = node.create_timer(0.02, tick)
    subscription = node.create_subscription(Twist, 'cmd_vel', lambda msg: commands.append(msg), 10)

    def spin(seconds, condition=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if condition and condition():
                return True
        return False

    def await_future(future):
        assert spin(12, future.done), 'service/action timed out'
        return future.result()

    process = None
    with tempfile.TemporaryDirectory(prefix='odom_recovery_') as directory:
        params = Path(directory) / 'params.yaml'
        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 'src/robot_launch/config/go2w_nav2.yaml').read_text())
        controller = config['controller_server']['ros__parameters']
        controller['progress_checker']['movement_time_allowance'] = 1.0
        costmap = config['local_costmap']['local_costmap']['ros__parameters']
        costmap.update(plugins=['inflation_layer'], global_frame='map', robot_base_frame='base_link',
                       transform_tolerance=0.15, width=20, height=20)
        params.write_text(yaml.safe_dump({'/odom_recovery/controller_server':
                                         {'ros__parameters': controller},
                                         '/odom_recovery/local_costmap/local_costmap':
                                         {'ros__parameters': costmap}}))
        logfile = Path('/tmp/odom_recovery_controller.log')
        with logfile.open('w') as log:
            executable = Path(get_package_prefix('nav2_controller')) / 'lib/nav2_controller/controller_server'
            process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/odom_recovery',
                                        '--params-file', str(params)], stdout=log, stderr=log,
                                       start_new_session=True)
            try:
                change = node.create_client(ChangeState, 'controller_server/change_state')
                assert change.wait_for_service(timeout_sec=10)
                for transition in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    assert await_future(change.call_async(request)).success, logfile.read_text()
                client = ActionClient(node, FollowPath, 'follow_path')
                assert client.wait_for_server(timeout_sec=5)
                goal = FollowPath.Goal(controller_id='FollowPath', goal_checker_id='general_goal_checker')
                goal.path.header.frame_id = 'map'
                for index in range(51):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = index * 0.1
                    pose.pose.orientation.w = 1.0
                    goal.path.poses.append(pose)
                handle = await_future(client.send_goal_async(goal))
                assert handle.accepted
                result = handle.get_result_async()
                assert spin(0.6, lambda: any(c.linear.x > 0 for c in commands)), logfile.read_text()
                flowing = False
                spin(0.4)  # Allow the last real TF to expire.
                commands.clear()
                spin(2.0)  # Longer than the 1 s progress timeout.
                assert not result.done(), logfile.read_text()
                assert commands and all(c.linear.x == 0 and c.angular.z == 0 for c in commands)
                flowing = True
                commands.clear()
                assert spin(0.6, lambda: any(c.linear.x > 0 for c in commands)), logfile.read_text()
                assert not result.done(), logfile.read_text()
                assert 'Navigation data recovered' in logfile.read_text()
                flowing = False
                spin(0.4)
                assert await_future(handle.cancel_goal_async()).goals_canceling
                assert await_future(result).status == GoalStatus.STATUS_CANCELED
                print('PASS: stops on TF outage, retains goal beyond progress timeout, resumes, cancels during outage')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
