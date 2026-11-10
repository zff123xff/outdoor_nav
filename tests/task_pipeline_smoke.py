#!/usr/bin/env python3
"""Five fixed-ID primitives with single-task admission; no robot motion."""
import json
import tempfile
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from patrol_node.action import PointGoal, Patrol
from task_interfaces.action import AreaSearch, InspectTargets
from task_interfaces.msg import Task, TaskStatus
from std_msgs.msg import String
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.multi_point_executor import MultiPointExecutor
from task_pipeline.task_schema import to_message


def main():
    target_dir = tempfile.TemporaryDirectory(prefix='task_pipeline_smoke_')
    rclpy.init(args=['--ros-args','-r','__ns:=/pipeline_fixture',
                     '-p',f'yolo_targets_file:={target_dir.name}/targets.yaml'])
    node, dispatcher, multi = Node('fixture'), PipelineDispatcher(), MultiPointExecutor()
    executor = SingleThreadedExecutor()
    for item in (node,dispatcher,multi):executor.add_node(item)
    calls, active, events, statuses = [], [], [], []
    async def execute(handle, kind, cls):
        identity=getattr(handle.request,'mission_id',getattr(handle.request,'task_id',''))
        calls.append((identity,kind))
        waiter=Future()
        active.append((time.monotonic()+1.1,waiter,handle))
        if cls==PointGoal:handle.publish_feedback(PointGoal.Feedback(progress=40.,distance_to_target_m=10.))
        elif cls==Patrol:handle.publish_feedback(Patrol.Feedback(reached_points=1))
        else:handle.publish_feedback(cls.Feedback(progress=42.,phase='fixture'))
        await waiter
        success=identity!='142' and not handle.is_cancel_requested
        if handle.is_cancel_requested:handle.canceled()
        elif success:handle.succeed()
        else:handle.abort()
        return cls.Result(success=success,message='fixture result')
    servers=[]
    for name,kind,cls in [('point_goal','point_goal',PointGoal),('patrol','patrol_line',Patrol),
                          ('area_search','area_search',AreaSearch),('target_inspection','target_inspection',InspectTargets)]:
        async def callback(handle,k=kind,c=cls):return await execute(handle,k,c)
        servers.append(ActionServer(node,cls,name,callback,cancel_callback=lambda h:CancelResponse.ACCEPT))
    def tick():
        for item in list(active):
            deadline,waiter,handle=item
            if time.monotonic()>=deadline or handle.is_cancel_requested:
                active.remove(item);waiter.set_result(True)
    node.create_timer(.03,tick)
    node.create_subscription(String,'task_feedback',lambda msg:events.append(json.loads(msg.data)),10)
    node.create_subscription(TaskStatus,'robot_task_status',statuses.append,10)
    publisher=node.create_publisher(Task,'task',10)
    def wait(predicate,label,timeout=18.):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:raise AssertionError(f'{label}: {calls} {events}')
            executor.spin_once(timeout_sec=.03)
    terminal=lambda:[e for e in events if e['event'] in ('completed','failed')]
    try:
        wait(lambda:publisher.get_subscription_count() and all(
            dispatcher.action_clients[name].server_is_ready() for name in
            ('point_goal', 'multi_point', 'patrol_line', 'area_search', 'target_inspection')),
            'readiness')
        wait(lambda:len(statuses)>=2,'continuous idle heartbeat')
        assert all(s.task_id=='0' and s.progress==0 and s.failures==0 and
                   s.param1==s.param2==s.param3==0. for s in statuses)
        p=[31.,120.,0.]
        for index,data in enumerate([dict(task_id=140,waypoints=p,gait_speed=2,air_route=7),
            dict(task_id=145,waypoints=p*2),dict(task_id=141,waypoints=p*2),
            dict(task_id=142,center=p,length_m=10.,width_m=5.),dict(task_id=143)]):
            publisher.publish(to_message(data))
            wait(lambda:len(terminal())==index+1,'sequential completion')
        assert [identity for identity,kind in calls]==['140','145','145','141','142','143'],(calls,events)
        assert [e['task_id'] for e in terminal()]==['140','145','141','142','143']
        assert next(e for e in terminal() if e['task_id']=='142')['event']=='failed'
        wait(lambda:len([s for s in statuses if s.progress==100])==5,'statuses')
        assert [s.task_id for s in statuses if s.progress==100]==['140','145','141','142','143']
        assert any(s.task_id=='142' and s.failures==1 for s in statuses)
        # A second ordinary task is rejected while the first is active.
        before=len(calls)
        publisher.publish(to_message(dict(task_id=140,waypoints=p)))
        wait(lambda:len(calls)==before+1 and dispatcher.current is not None,'active task')
        publisher.publish(to_message(dict(task_id=140,waypoints=p)))
        wait(lambda:any(e['event']=='rejected' and e['task_id']=='140' for e in events),'busy rejection')
        publisher.publish(Task(task_id=141,cancel=True))
        wait(lambda:any(e['event']=='rejected' and e['task_id']=='141' for e in events),'mismatched cancel rejection')
        assert dispatcher.current is not None
        publisher.publish(Task(task_id=140,cancel=True))
        wait(lambda:len(terminal())==6 and dispatcher.current is None,'matching cancel')
        assert len(calls)==before+1
        assert terminal()[-1]['event']=='failed'
        assert all('task_type' not in e for e in events)
        idle_start=len(statuses)
        wait(lambda:any(s.task_id=='0' for s in statuses[idle_start:]),'first idle heartbeat after cancellation')
        idle_start=len(statuses)-1
        wait(lambda:len(statuses)>=idle_start+2,'continuous idle heartbeat after cancellation')
        assert all(s.task_id=='0' and s.progress==0 and s.failures==0 and
                   s.param1==s.param2==s.param3==0. for s in statuses[idle_start:])
        print('PASS: single-task admission, busy rejection, matching cancellation and idle heartbeat')
    finally:
        for server in servers:server.destroy()
        for item in (multi,dispatcher,node):item.destroy_node()
        executor.shutdown();rclpy.try_shutdown();target_dir.cleanup()

if __name__=='__main__':main()
