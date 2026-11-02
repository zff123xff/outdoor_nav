#!/usr/bin/env python3
"""Isolated Action protocol test, with a mock controller and fixture-only TF.

No robot drivers or velocity publishers are created. This verifies completion
wiring and cancellation, not physical navigation or arrival.
"""
import argparse
from pathlib import Path
import os
import signal
import socket
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath, NavigateToPose
from tf2_ros import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('invalid ROS domain')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.bind(('', 0)); sock.close()
    probe = subprocess.run(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '3'],
                           capture_output=True, text=True, timeout=15, check=True)
    if set(probe.stdout.split()) - {'/rosout', '/parameter_events'}:
        raise RuntimeError('Use an empty test domain')
    output = Path(tempfile.mkdtemp(prefix='navigation_completion_'))
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    log = (output/'planner.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'nav2_straight_planner', 'straight_planner_node',
        '--ros-args', '-r', '__ns:=/completion_fixture', '-p', 'global_frame:=completion_fixture/map',
        '-p', 'robot_base_frame:=completion_fixture/base_link', '-p', 'planning_period_s:=0.05'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=['--ros-args', '-r', '__ns:=/completion_fixture'])
    node = Node('completion_test', namespace='/completion_fixture')
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    task_nodes = []
    controls = []

    async def execute(handle):
        entry = {'handle': handle, 'release': Future()}
        controls.append(entry)
        outcome = await entry['release']
        if outcome == 'success': handle.succeed()
        elif outcome == 'cancel': handle.canceled()
        else: handle.abort()
        return FollowPath.Result()

    server = ActionServer(node, FollowPath, 'follow_path', execute_callback=execute,
        cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'completion_fixture/map'; transform.child_frame_id = 'completion_fixture/base_link'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    navigation = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    lifecycle = node.create_client(ChangeState, 'straight_planner/change_state')

    def wait(check, label, timeout=12):
        deadline = time.monotonic()+timeout
        while not check():
            if process.poll() is not None:
                raise RuntimeError('Planner exited; '+str(output))
            if time.monotonic() >= deadline:
                raise TimeoutError(label+'; '+str(output))
            executor.spin_once(timeout_sec=.02)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:executor.spin_once(timeout_sec=.02)

    def start_goal():
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'completion_fixture/map'
        goal.pose.pose.position.x = .01  # Inside the old task-side arrival radius.
        goal.pose.pose.orientation.w = 1.0
        response = navigation.send_goal_async(goal)
        wait(response.done, 'navigation acceptance')
        handle = response.result()
        assert handle.accepted
        return handle, handle.get_result_async()

    try:
        wait(lifecycle.service_is_ready, 'lifecycle discovery')
        for transition in (1, 3):
            request = ChangeState.Request(); request.transition.id = transition
            future = lifecycle.call_async(request); wait(future.done, 'lifecycle transition')
            assert future.result().success
        wait(navigation.server_is_ready, 'navigation Action discovery')
        handle, result = start_goal()
        wait(lambda: len(controls)==1, 'controller acceptance')
        spin(.4)
        assert not result.done(), 'nearby pose caused premature success'
        assert len(controls)==1, 'planner repeatedly replaced an active FollowPath'
        controls[0]['release'].set_result('success')
        wait(result.done, 'navigation success')
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        print('No distance shortcut; FollowPath success completes navigation', flush=True)

        handle, result = start_goal()
        wait(lambda: len(controls)==2, 'second controller goal')
        controls[1]['release'].set_result('failure')
        wait(result.done, 'navigation failure')
        assert result.result().status == GoalStatus.STATUS_ABORTED
        print('FollowPath failure is propagated as failure', flush=True)

        handle, old_result = start_goal()
        wait(lambda: len(controls)==3, 'cancelled controller goal')
        cancel = handle.cancel_goal_async(); wait(cancel.done, 'cancel acceptance')
        wait(old_result.done, 'navigation cancellation')
        assert old_result.result().status == GoalStatus.STATUS_CANCELED
        wait(lambda: controls[2]['handle'].is_cancel_requested, 'controller cancellation request')
        handle, result = start_goal()
        wait(lambda: len(controls)==4, 'replacement goal')
        controls[2]['release'].set_result('cancel')
        spin(.3)
        assert not result.done(), 'late cancelled goal completed its replacement'
        controls[3]['release'].set_result('success')
        wait(result.done, 'replacement success')
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        print('Cancellation and stale-result isolation passed', flush=True)
        # Real task executors, without task-side odometry arrival input.
        from patrol_node.point_goal_node import PointGoalNode
        from patrol_node.patrol_node import PatrolNode
        from patrol_node.action import PointGoal, Patrol
        from multi_goal_to_gp_goal.multi_goal_to_gp_goal import MultiGoalToGpGoal
        from rclpy.parameter import Parameter
        from msg_set_msgs.msg import MultiGoal, MultiGoalPoint
        point, patrol, queue = PointGoalNode(), PatrolNode(), MultiGoalToGpGoal()
        task_nodes.extend([point, patrol, queue])
        for task_node in task_nodes:
            task_node.set_parameters([Parameter('reference_latitude', value=31.2),
                                      Parameter('reference_longitude', value=120.3),
                                      Parameter('reference_altitude', value=20.0)])
            executor.add_node(task_node)
        point_action = ActionClient(node, PointGoal, 'point_goal')
        patrol_action = ActionClient(node, Patrol, 'patrol')
        wait(point_action.server_is_ready, 'point executor')
        wait(patrol_action.server_is_ready, 'patrol executor')
        goal = PointGoal.Goal()
        goal.target.latitude = 31.2; goal.target.longitude = 120.3; goal.target.altitude = 20.0
        sent = point_action.send_goal_async(goal); wait(sent.done, 'point acceptance')
        point_result = sent.result().get_result_async()
        wait(lambda: len(controls)==5, 'point controller goal')
        spin(.2); assert not point_result.done()
        controls[4]['release'].set_result('success')
        wait(point_result.done, 'point completion')
        assert point_result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert point_result.result().result.success

        goal = Patrol.Goal()
        goal.start_point.latitude = 31.2; goal.start_point.longitude = 120.3; goal.start_point.altitude = 20.0
        goal.end_point.latitude = 31.2; goal.end_point.longitude = 120.30001; goal.end_point.altitude = 20.0
        goal.max_laps = 1
        sent = patrol_action.send_goal_async(goal); wait(sent.done, 'patrol acceptance')
        assert sent.result().accepted
        patrol_result = sent.result().get_result_async()
        for index in range(5, 8):
            wait(lambda i=index: len(controls)==i+1, 'patrol endpoint controller goal')
            assert not patrol_result.done()
            controls[index]['release'].set_result('success')
        wait(patrol_result.done, 'patrol completion')
        assert patrol_result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert patrol_result.result().result.reached_points == 3

        message = MultiGoal(); message.is_gps_aid = True
        message.multi_goal_points = [MultiGoalPoint(x_or_lat=31.2, y_or_lon=120.3, z_or_alt=20.0),
                                     MultiGoalPoint(x_or_lat=31.2, y_or_lon=120.30001, z_or_alt=20.0)]
        queue._on_multi_goal(message)
        wait(lambda: len(controls)==9, 'queue controller goal')
        controls[8]['release'].set_result('failure')
        wait(lambda: queue._current_goal_enu is None, 'queue failure')
        spin(.3)
        assert len(controls)==9 and not queue._queue
        print('Point, patrol and queue consume controller results; failure does not advance queue', flush=True)
        print('Action protocol tests passed; fixture logs: '+str(output), flush=True)
    finally:
        for task_node in task_nodes:task_node.destroy_node()
        executor.shutdown()
        server.destroy(); node.destroy_node(); rclpy.try_shutdown()
        if process.poll() is None:os.killpg(process.pid, signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
