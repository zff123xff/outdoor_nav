#!/usr/bin/env python3
"""No GPS publisher: actual point executor + ENU odom/TF + pipeline status."""
import json
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from task_interfaces.msg import Task, TaskStatus
from tf2_ros import TransformBroadcaster
from patrol_node.point_goal_node import PointGoalNode
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.examples.coordinates import enu_to_gps
from task_pipeline.task_schema import to_message


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/enu_progress_fixture'])
    fixture, point, pipeline = Node('fixture'), PointGoalNode(), PipelineDispatcher()
    for key, value in zip(('reference_latitude', 'reference_longitude', 'reference_altitude'), (31., 120., 0.)):
        point.set_parameters([Parameter(key, value=value)])
    executor = SingleThreadedExecutor()
    for node in (fixture, point, pipeline):
        executor.add_node(node)
    state = dict(x=0., sensors=True)
    broadcaster = TransformBroadcaster(fixture)
    odom_pub = fixture.create_publisher(Odometry, 'odom', 10)
    task_pub = fixture.create_publisher(Task, 'task', 10)
    statuses, events, goals = [], [], []
    fixture.create_subscription(TaskStatus, 'robot_task_status', statuses.append, 10)
    fixture.create_subscription(String, 'task_feedback', lambda msg: events.append(json.loads(msg.data)), 10)
    release = Future()
    async def navigate(handle):
        goals.append(handle.request)
        await release
        handle.abort()
        return NavigateToPose.Result()
    server = ActionServer(fixture, NavigateToPose, 'navigate_to_pose', navigate)
    def sensors():
        if not state['sensors']:
            return
        tf = TransformStamped()
        tf.header.stamp = fixture.get_clock().now().to_msg()
        tf.header.frame_id = 'enu_progress_fixture/map'
        tf.child_frame_id = 'enu_progress_fixture/base_link'
        tf.transform.translation.x = state['x']
        tf.transform.rotation.w = 1.
        broadcaster.sendTransform(tf)
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = tf.child_frame_id
        odom.pose.pose.position.x = state['x']
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
    fixture.create_timer(.1, sensors)
    def wait(predicate, label, timeout=8.):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > end:
                raise AssertionError(f'Timeout {label}: statuses={statuses}, events={events}')
            executor.spin_once(timeout_sec=.05)
    try:
        wait(lambda: task_pub.get_subscription_count() and point._robot_xy() is not None and
             point._navigation.client.server_is_ready(), 'readiness')
        latitude, longitude, altitude = enu_to_gps(10., 0., 0., (31., 120., 0.))
        task_pub.publish(to_message(dict(task_id='no_gps', task_type='point_goal', waypoints=[
            dict(latitude=latitude, longitude=longitude, altitude=altitude)])))
        wait(lambda: goals, 'ENU navigation goal')
        assert abs(goals[0].pose.pose.position.x-10.) < .01
        state['x'] = 5.
        wait(lambda: any(49 <= status.progress <= 50 for status in statuses), '50% ENU status')
        assert pipeline.fix is None
        saved = pipeline.progress
        state['sensors'] = False
        wait(lambda: point._enu_progress() == -1., 'reject stale ENU')
        assert pipeline.progress == saved and saved < 100
        release.set_result(True)
        wait(lambda: any(status.progress == 100 for status in statuses), 'terminal 100')
        wait(lambda: any(event['event'] == 'failed' for event in events), 'failed result')
        print('PASS: no GPS; ENU goal 10m, odom/TF at 5m -> 50%; stale ENU ignored; failed Action -> 100%')
    finally:
        server.destroy()
        for node in (pipeline, point, fixture):
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
