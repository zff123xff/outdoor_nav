#!/usr/bin/env python3
"""New observation mode with the real visitor, simulated search/Nav2 and isolated TF.

No robot drivers or cmd_vel publishers are created.
"""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Point, TransformStamped
from nav2_msgs.action import NavigateToPose, FollowPath
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
from task_interfaces.action import AreaSearch
from task_interfaces.msg import InspectionArrival, SearchArea
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from task_pipeline.inspection_executor import InspectionExecutor
from task_pipeline.search_inspection_executor import SearchInspectionExecutor


def main():
    rclpy.init(args=['--ros-args','-r','__ns:=/search_observation_smoke',
        '-p','action_name:=search_target_inspection','-p','assigned_topic:=targets/search_assigned',
        '-p','require_fresh_assignment:=true'])
    peer, coordinator, visitor = Node('simulation_fixture'), SearchInspectionExecutor(), InspectionExecutor()
    executor = SingleThreadedExecutor()
    for node in (peer, coordinator, visitor):executor.add_node(node)
    events, arrivals, motions = [], [], []
    state = dict(x=0., y=0., z=0., yaw=0., paused=False, area_gate=None)
    broadcaster = TransformBroadcaster(peer)

    async def area(handle):
        state['area_gate'] = Future(); events.append(('search_started', time.monotonic()))
        await state['area_gate']
        handle.succeed()
        return AreaSearch.Result(success=True, message='simulated coverage complete')

    def pause(request, response):
        if not request.data:
            assert visitor.child is None and not visitor.inflight and not visitor.busy
        state['paused'] = request.data
        events.append(('paused' if request.data else 'resumed', time.monotonic()))
        response.success = True
        return response

    async def navigate(handle, kind):
        assert state['paused'], 'Visitor took navigation before search released it'
        gate = Future();motions.append((time.monotonic()+.2, gate, handle, kind))
        events.append((kind, time.monotonic()))
        await gate
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            pose = (handle.request.pose if kind == 'approach' else handle.request.path.poses[-1]).pose
            state.update(x=pose.position.x,y=pose.position.y,z=pose.position.z,
                         yaw=2*math.atan2(pose.orientation.z,pose.orientation.w))
            handle.succeed()
        return NavigateToPose.Result() if kind == 'approach' else FollowPath.Result()

    async def approach(handle):return await navigate(handle, 'approach')
    async def align(handle):return await navigate(handle, 'align')
    servers = [ActionServer(peer, AreaSearch, 'area_search', area),
        ActionServer(peer, NavigateToPose, 'navigate_to_pose', approach,
                     cancel_callback=lambda h:CancelResponse.ACCEPT),
        ActionServer(peer, FollowPath, 'follow_path', align,
                     cancel_callback=lambda h:CancelResponse.ACCEPT)]
    peer.create_service(SetBool, 'area_search/pause', pause)

    def sensors():
        transform = TransformStamped();transform.header.frame_id='search_observation_smoke/map'
        transform.child_frame_id='search_observation_smoke/base_link'
        transform.header.stamp=peer.get_clock().now().to_msg()
        transform.transform.translation.x=state['x'];transform.transform.translation.y=state['y']
        transform.transform.rotation.z=math.sin(state['yaw']/2)
        transform.transform.rotation.w=math.cos(state['yaw']/2)
        broadcaster.sendTransform(transform)
        for item in list(motions):
            deadline, future, handle, kind = item
            if handle.is_cancel_requested or time.monotonic() >= deadline:
                motions.remove(item);future.set_result(True)
    peer.create_timer(.05, sensors)
    pub = peer.create_publisher(MarkerArray, 'targets/markers', 10)
    def arrival(message):
        arrivals.append(message);events.append(('arrived', time.monotonic()))
    peer.create_subscription(InspectionArrival, 'targets/inspection_arrived', arrival, 10)
    client = ActionClient(peer, AreaSearch, 'search_inspection')
    def wait(predicate, label, timeout=20.):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        assert predicate(), (label, getattr(coordinator,'phase',None), getattr(visitor,'phase',None),events)
    def publish_marker(key=1):
        marker=Marker();marker.id=key;marker.ns='yolo_targets';marker.type=Marker.SPHERE
        marker.header.frame_id='search_observation_smoke/map'
        marker.header.stamp=peer.get_clock().now().to_msg();marker.pose.position=Point(x=20.)
        pub.publish(MarkerArray(markers=[marker]))
    try:
        wait(lambda:client.server_is_ready() and visitor.robot_pose() is not None,'ready')
        future=client.send_goal_async(AreaSearch.Goal(task_id='143',area=SearchArea(
            center=NavSatFix(latitude=31.,longitude=120.),length_m=10.,width_m=10.)))
        wait(future.done,'parent accepted');handle=future.result();assert handle.accepted
        result=handle.get_result_async()
        wait(lambda:coordinator.phase=='searching' and pub.get_subscription_count()>0,'search active')
        publish_marker()
        wait(lambda:arrivals,'verified arrival')
        event=arrivals[0]
        assert event.task_id=='143' and event.target_id==1
        assert event.distance_m<=10. and abs(event.heading_error_rad)<=.15
        assert event.robot_position.x == 11.
        wait(lambda:any(kind=='resumed' for kind,stamp in events),'search resumed after real visitor dwell')
        times=dict(events)
        assert times['paused'] < times['approach'] < times['align'] < times['arrived'] < times['resumed']
        assert times['resumed']-times['arrived']>=2.4
        publish_marker(key=99)
        until=time.monotonic()+.3
        while time.monotonic()<until:executor.spin_once(timeout_sec=.02)
        assert len(arrivals)==1 and len(coordinator.handled)==1
        state['area_gate'].set_result(True)
        wait(result.done,'search completion')
        assert result.result().status==4 and result.result().result.success
        print('PASS: marker -> acknowledged search pause -> real visitor approach/align/stop -> arrival -> 2.5s dwell -> resume retained search; spatial dedup; no robot motion commands')
    finally:
        client.destroy()
        for server in servers:server.destroy()
        coordinator.server.destroy();visitor.server.destroy()
        for node in (coordinator,visitor,peer):
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.try_shutdown()


if __name__=='__main__':main()
