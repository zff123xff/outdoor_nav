#!/usr/bin/env python3
"""对外接口检查：下发 point_goal，观察 /<ns>/robot_task_status 进度 0 -> 100，20 s 内结束。

只使用公开接口：发布 task_interfaces/Task，订阅 task_interfaces/TaskStatus 与 task_feedback。
PointGoal Action server 是本文件内的假实现，只上报递增的 progress，不产生任何机器人运动。
调度器使用真实的 PipelineDispatcher，因此 TaskStatus 的发布时机、单调性、终态 100、
1 Hz 周期都按生产代码路径验证。
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from patrol_node.action import PointGoal
from std_msgs.msg import String
from task_interfaces.msg import Task, TaskStatus
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.task_schema import to_message

TASK_ID = 'point_status_demo'
DURATION_S = 20.0          # 模拟一次定点导航的总时长
FEEDBACK_PERIOD_S = 0.5    # 假执行器上报进度周期
STATUS_PERIOD_S = 1.0      # dispatcher 的 TaskStatus 周期
TIMEOUT_TOLERANCE_S = 3.0  # 传输与启动开销余量


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--domain-id', type=int, default=176, help='isolated ROS domain')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot_task_status_fixture'])
    fixture = Node('robot_task_status_fixture')
    dispatcher = PipelineDispatcher()
    executor = SingleThreadedExecutor()
    for node in (fixture, dispatcher):
        executor.add_node(node)

    goals = []
    statuses = []   # (monotonic, TaskStatus)
    events = []
    clock = {'start': None}

    async def navigate(handle):
        goals.append(handle.request)
        started = time.monotonic()
        waiter = Future()
        state = {'done': False}

        def tick():
            if state['done']:
                return
            elapsed = time.monotonic() - started
            ratio = min(1.0, elapsed / DURATION_S)
            handle.publish_feedback(PointGoal.Feedback(
                phase='navigating', distance_to_target_m=10.0 * (1.0 - ratio),
                reached=False, progress=min(99.0, 100.0 * ratio)))
            if elapsed >= DURATION_S:
                state['done'] = True
                waiter.set_result(True)

        timer = fixture.create_timer(FEEDBACK_PERIOD_S, tick)
        await waiter
        timer.cancel()
        handle.succeed()
        return PointGoal.Result(success=True, message='fake navigation reached')

    server = ActionServer(fixture, PointGoal, 'point_goal', navigate,
                          cancel_callback=lambda handle: CancelResponse.ACCEPT)

    def on_status(message):
        statuses.append((time.monotonic(), message))

    fixture.create_subscription(TaskStatus, 'robot_task_status', on_status, 10)
    fixture.create_subscription(String, 'task_feedback',
                                lambda msg: events.append(json.loads(msg.data)), 10)
    task_pub = fixture.create_publisher(Task, 'task', 10)

    def wait(predicate, label, timeout=30.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError(
                    f'Timeout {label}: progress={[s.progress for _, s in statuses]} events={events}')
            executor.spin_once(timeout_sec=0.05)

    try:
        wait(lambda: task_pub.get_subscription_count() > 0 and
             dispatcher.action_clients['point_goal'].server_is_ready(), 'readiness')
        clock['start'] = time.monotonic()
        task_pub.publish(to_message(dict(
            task_id=TASK_ID, task_type='point_goal',
            waypoints=[dict(latitude=31.0, longitude=120.0, altitude=0.0)])))
        wait(lambda: goals and any(m.progress == 100 for _, m in statuses), 'terminal 100')
        elapsed = time.monotonic() - clock['start']
        wait(lambda: any(e['event'] in ('completed', 'failed') and e['task_id'] == TASK_ID
                         for e in events), 'terminal event')

        own = [m for _, m in statuses if m.task_id == TASK_ID]
        progress = [m.progress for m in own]
        stamps = [stamp for stamp, m in statuses if m.task_id == TASK_ID and m.progress < 100]
        assert goals[0].target.latitude == 31.0 and goals[0].target.longitude == 120.0, goals[0]
        assert own, 'no TaskStatus for this task'
        assert progress[0] <= 5, f'progress must start near 0, got {progress}'
        assert progress == sorted(progress), f'progress must be monotonic, got {progress}'
        assert progress[-1] == 100, f'terminal progress must be 100, got {progress}'
        assert elapsed <= DURATION_S + TIMEOUT_TOLERANCE_S, f'finished in {elapsed:.2f}s > {DURATION_S}s budget'
        assert elapsed >= DURATION_S - TIMEOUT_TOLERANCE_S, f'finished too early ({elapsed:.2f}s), ramp not exercised'
        assert any(p <= 20 for p in progress) and any(p >= 80 for p in progress), progress
        assert len(stamps) >= 10, f'expected ~1 Hz active status, got {len(stamps)}'
        assert all(STATUS_PERIOD_S - 0.35 <= b - a <= STATUS_PERIOD_S + 0.35
                   for a, b in zip(stamps, stamps[1:])), [round(b - a, 3) for a, b in zip(stamps, stamps[1:])]
        assert all(m.failures == 0 and m.param1 == m.param2 == m.param3 == 0.0 for m in own)
        kinds = [e['event'] for e in events if e['task_id'] == TASK_ID]
        assert 'queued' in kinds and 'executing' in kinds and 'completed' in kinds, kinds
        assert dispatcher.current is None and not dispatcher.queue

        print('progress:', ' '.join(str(p) for p in progress))
        print(f'PASS: point_goal -> robot_task_status {progress[0]}..{progress[-1]}, '
              f'{len(stamps)} active frames @ ~{STATUS_PERIOD_S:g} Hz, '
              f'terminal 100 at {elapsed:.2f}s (budget {DURATION_S:g}s)')
    finally:
        server.destroy()
        for node in (dispatcher, fixture):
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
