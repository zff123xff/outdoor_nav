#!/usr/bin/env python3
"""Isolated observation pipeline test: fake Nav2, never publishes cmd_vel."""
import json
import math
import time
from tempfile import TemporaryDirectory

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Point, TransformStamped
from nav2_msgs.action import NavigateToPose, FollowPath
from std_msgs.msg import String
from task_interfaces.msg import Task, TargetArray, TargetInfo, InspectionArrival, TaskStatus
from tf2_ros import TransformBroadcaster
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.inspection_executor import InspectionExecutor


def main():
    temporary = TemporaryDirectory(prefix='inspection_regression_')
    rclpy.init(args=['--ros-args','-r','__ns:=/inspection_fixture',
                    '-p',f'yolo_targets_file:={temporary.name}/targets.yaml'])
    node, dispatcher, inspector = Node('fixture'), PipelineDispatcher(), InspectionExecutor()
    executor = SingleThreadedExecutor()
    for item in (node, dispatcher, inspector):executor.add_node(item)
    state = dict(x=0.,y=0.,z=0.,yaw=0.,hold=False)
    broadcaster = TransformBroadcaster(node)
    calls, active, arrivals, events, statuses = [], [], [], [], []
    async def navigate(handle, kind):
        calls.append(kind)
        future = Future()
        active.append((time.monotonic()+.2, future, handle, kind))
        await future
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            pose = (handle.request.pose if kind=='approach' else handle.request.path.poses[-1]).pose
            state.update(x=pose.position.x,y=pose.position.y,z=pose.position.z,
                         yaw=2*math.atan2(pose.orientation.z,pose.orientation.w))
            handle.succeed()
        return NavigateToPose.Result() if kind=='approach' else FollowPath.Result()
    async def approach(handle):return await navigate(handle,'approach')
    async def align(handle):return await navigate(handle,'align')
    servers = [ActionServer(node,NavigateToPose,'navigate_to_pose',approach,
                            cancel_callback=lambda h: CancelResponse.ACCEPT),
               ActionServer(node,FollowPath,'follow_path',align,
                            cancel_callback=lambda h: CancelResponse.ACCEPT)]
    def sensors():
        transform = TransformStamped()
        transform.header.frame_id='inspection_fixture/map'
        transform.child_frame_id='inspection_fixture/base_link'
        transform.header.stamp=node.get_clock().now().to_msg()
        transform.transform.translation.x=state['x']
        transform.transform.translation.y=state['y']
        transform.transform.rotation.z=math.sin(state['yaw']/2)
        transform.transform.rotation.w=math.cos(state['yaw']/2)
        broadcaster.sendTransform(transform)
        for entry in list(active):
            deadline, future, handle, kind = entry
            if (time.monotonic()>=deadline and not state['hold']) or handle.is_cancel_requested:
                active.remove(entry)
                future.set_result(True)
    node.create_timer(.05,sensors)
    task=node.create_publisher(Task,'task',10)
    assignments=node.create_publisher(TargetArray,'targets/assigned',10)
    node.create_subscription(InspectionArrival,'targets/inspection_arrived',arrivals.append,10)
    node.create_subscription(TaskStatus,'robot_task_status',statuses.append,10)
    node.create_subscription(String,'task_feedback',lambda msg: events.append(json.loads(msg.data)),10)
    def batch(points):
        return TargetArray(targets=[TargetInfo(target_id=key,target_position=Point(x=float(x),y=0.,z=0.)) for key,x in points])
    def wait(condition, label, timeout=20.):
        deadline=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>deadline:raise AssertionError(f'{label}: phase={getattr(inspector,"phase",None)}, calls={calls}, events={events}')
            executor.spin_once(timeout_sec=.03)
    try:
        wait(lambda: task.get_subscription_count() and assignments.get_subscription_count() and
             inspector.robot_pose() is not None and inspector.nav.server_is_ready() and inspector.align.server_is_ready(), 'readiness')
        assignments.publish(batch([(1,20.),(2,5.),(3,-6.)]))
        wait(lambda: inspector.latest is not None, 'assignment receipt')
        task.publish(Task(task_id=143))
        wait(lambda: len(arrivals)==1, 'first observation')
        assert arrivals[0].target_id==2
        assignments.publish(batch([(99,0.)]))
        wait(lambda: len(arrivals)==3,'snapshot visits')
        wait(lambda: dispatcher.current is None and any(e['event']=='completed' for e in events),'completion')
        assert [event.target_id for event in arrivals]==[2,3,1]
        assert calls.count('approach')==1 and calls.count('align')==3, calls
        assert all(a.distance_m<=10. and abs(a.heading_error_rad)<=.15 for a in arrivals)
        assert any(s.task_id=='143' and s.progress==100 for s in statuses)
        # Cancel with a child running; a following FIFO task cannot start until it drains.
        state['hold']=True
        assignments.publish(batch([(4,100.)]))
        wait(lambda: 4 in inspector.latest,'next assignment')
        task.publish(Task(task_id=143))
        wait(lambda: inspector.child is not None, 'active child')
        task.publish(Task(task_id=143,cancel=True))
        wait(lambda: dispatcher.current is None and not inspector.busy,'cancel drains')
        assert len(arrivals)==3
        wait(lambda: any(e['event']=='failed' for e in events), 'canceled feedback')
        print('PASS: flat Task143 -> nearest snapshot visits -> verified arrivals -> 2.5s dwell; canceled child drained; no motion commands')
    finally:
        for server in servers:server.destroy()
        for item in (inspector,dispatcher,node):item.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        temporary.cleanup()

if __name__=='__main__':main()
