#!/usr/bin/env python3
"""Bounded isolated ROS test; fake executors never publish robot motion."""
import json
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from patrol_node.action import PointGoal, Patrol
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from task_interfaces.action import AreaSearch
from task_interfaces.msg import Task, TaskStatus
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.multi_point_executor import MultiPointExecutor
from task_pipeline.task_schema import to_message


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/pipeline_fixture'])
    node = Node('fixture')
    dispatcher = PipelineDispatcher()
    multi = MultiPointExecutor()
    executor = SingleThreadedExecutor()
    for item in (node, dispatcher, multi):
        executor.add_node(item)
    active = []
    calls = []
    statuses = []
    events = []
    point_goals = []
    async def execute(handle, kind, action_type):
        task_id = getattr(handle.request, 'mission_id', getattr(handle.request, 'task_id', ''))
        calls.append((task_id, kind))
        if kind == 'point_goal':
            point_goals.append(handle.request.target.latitude)
        waiter = Future()
        active.append((time.monotonic()+1.3, waiter, handle))
        if kind == 'point_goal':
            handle.publish_feedback(PointGoal.Feedback(distance_to_target_m=10., phase='navigating'))
        elif kind == 'patrol_line':
            handle.publish_feedback(Patrol.Feedback(reached_points=1))
        else:
            handle.publish_feedback(AreaSearch.Feedback(progress=42., phase='executing'))
        await waiter
        result = action_type.Result()
        result.success = task_id not in ('first', 'area') and not handle.is_cancel_requested
        result.message = 'fixture result'
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result
    servers = []
    for name, kind, cls in [('point_goal', 'point_goal', PointGoal), ('patrol', 'patrol_line', Patrol),
                             ('area_search', 'area_search', AreaSearch)]:
        async def callback(handle, k=kind, c=cls):
            return await execute(handle, k, c)
        servers.append(ActionServer(node, cls, name, callback,
            cancel_callback=lambda handle: CancelResponse.ACCEPT))
    def tick():
        for deadline, waiter, handle in list(active):
            if time.monotonic() >= deadline:
                active.remove((deadline, waiter, handle))
                waiter.set_result(True)
    timer = node.create_timer(.05, tick)
    node.create_subscription(TaskStatus, 'robot_task_status', lambda msg: statuses.append((time.monotonic(), msg)), 10)
    node.create_subscription(String, 'task_feedback', lambda msg: events.append(json.loads(msg.data)), 10)
    publisher = node.create_publisher(Task, 'task', 10)
    fix_pub = node.create_publisher(NavSatFix, 'mavros/global_position/global', 10)
    def wait(predicate, timeout=20.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError('Timed out; calls='+repr(calls)+' events='+repr(events))
            executor.spin_once(timeout_sec=.05)
    try:
        wait(lambda: publisher.get_subscription_count() == 1 and
             all(c.server_is_ready() for c in dispatcher.action_clients.values()) and multi.client.server_is_ready())
        p = dict(latitude=31., longitude=120., altitude=0.)
        fix_pub.publish(NavSatFix(**p))
        wait(lambda: dispatcher.fix is not None)
        for name, kind, points, area in [
            ('first', 'point_goal', [dict(p, latitude=31.0002), dict(p, latitude=32.)], {}),
            ('multi', 'multi_point', [p, dict(p, latitude=31.0003)], {}),
            ('patrol', 'patrol_line', [p, dict(p, longitude=120.0002)], {}),
            ('area', 'area_search', [], dict(center=p, length_m=10., width_m=5., yaw_deg=30.)),
        ]:
            publisher.publish(to_message(dict(task_id=name, task_type=kind, waypoints=points, area=area)))
        wait(lambda: dispatcher.current is not None)
        fix_pub.publish(NavSatFix(**dict(p, latitude=31.0001)))
        wait(lambda: len([e for e in events if e['event'] in ('completed', 'failed')]) == 4)
        wait(lambda: len([s for _, s in statuses if s.progress == 100]) == 4)
        assert [n for n, k in calls] == ['first', 'multi', 'multi', 'patrol', 'area'], calls
        assert point_goals == [31.0002, 31., 31.0003], point_goals
        terminal = [(s.task_id, s.progress) for _, s in statuses if s.progress == 100]
        assert terminal == [('first', 100), ('multi', 100), ('patrol', 100), ('area', 100)], terminal
        assert any(s.task_id == 'first' and 49 <= s.progress <= 50 for _, s in statuses), statuses
        assert all(s.failures == 0 and s.param1 == s.param2 == s.param3 == 0 for _, s in statuses)
        assert not dispatcher.queue and dispatcher.current is None
        periodic = [stamp for stamp, status in statuses if status.task_id == 'multi' and status.progress < 100]
        assert len(periodic) >= 2 and all(.75 <= b-a <= 1.25 for a, b in zip(periodic, periodic[1:])), periodic
        for task_id in ('cancel_active', 'cancel_queued'):
            publisher.publish(to_message(dict(task_id=task_id, task_type='point_goal', waypoints=[p])))
        wait(lambda: any(e['event'] == 'executing' and e['task_id'] == 'cancel_active' for e in events))
        publisher.publish(Task(task_id='cancel_queued', cancel=True))
        publisher.publish(Task(task_id='cancel_active', cancel=True))
        wait(lambda: all(any(e['task_id'] == task_id and e['event'] == 'failed' for e in events)
                         for task_id in ('cancel_active', 'cancel_queued')))
        wait(lambda: dispatcher.current is None and not dispatcher.queue)
        assert not any(task_id == 'cancel_queued' for task_id, kind in calls)
        assert not any(status.task_id == 'cancel_queued' for _, status in statuses)
        wait(lambda: any(status.task_id == 'cancel_active' and status.progress == 100 for _, status in statuses))
        print('PASS: four primitives FIFO, multi-point sequencing, first waypoint, GPS 50%, 1 Hz active status, failed/canceled Action -> 100%, queued cancellation without execution')
    finally:
        for server in servers:
            server.destroy()
        for item in (multi, dispatcher, node):
            item.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
