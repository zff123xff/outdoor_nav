#!/usr/bin/env python3
"""Actual area Action, subprocess, live TF/cloud, map and FollowPath protocol."""
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import FollowPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from tf2_ros import TransformBroadcaster
from task_interfaces.action import AreaSearch
from co_search_nav.executor import CoSearchExecutor
from bspline.msg import Bspline


def main():
    config = str(Path(__file__).resolve().parents[1]/'src/external_pkg/co_search/co_search_nav/config/area_search.yaml')
    rclpy.init(args=['--ros-args', '-r', '__ns:=/area_fixture', '-p', 'config_file:='+config])
    fixture = Node('fixture')
    area = CoSearchExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    executor.add_node(area)
    broadcaster = TransformBroadcaster(fixture)
    cloud_pub = fixture.create_publisher(PointCloud2, 'livox/lidar', 10)
    odom_pub = fixture.create_publisher(Odometry, 'odom', 10)
    maps, paths, splines = [], [], []
    fixture.create_subscription(Bspline, 'co_search/native_bspline', splines.append, 10)
    fixture.create_subscription(OccupancyGrid, 'area_search/map', lambda msg: maps.append(msg), 10)
    # Publisher is latched; map subscription can use volatile and still receive new maps.
    def controller(request, response):
        response.current_state.id = 3
        return response
    def manager(request, response):
        response.success = True
        return response
    def origin(request, response):
        response.values = [ParameterValue(type=3, double_value=x) for x in (31., 120., 0.)]
        return response
    fixture.create_service(GetState, 'controller_server/get_state', controller)
    fixture.create_service(Trigger, 'lifecycle_manager_navigation/is_active', manager)
    fixture.create_service(GetParameters, 'nav2_input_bridge/get_parameters', origin)
    async def follow(handle):
        paths.append(handle.request.path)
        # Return success with no actual movement; cancel test still checks cleanup.
        handle.succeed()
        return FollowPath.Result()
    server = ActionServer(fixture, FollowPath, 'follow_path', follow,
                          cancel_callback=lambda h: CancelResponse.ACCEPT)
    def sensors():
        stamp = fixture.get_clock().now().to_msg()
        transforms = []
        for child, z in [('area_fixture/base_link', 0.), ('area_fixture/lidar', .5)]:
            tf = TransformStamped()
            tf.header = Header(stamp=stamp, frame_id='area_fixture/map')
            tf.child_frame_id = child
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        broadcaster.sendTransform(transforms)
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='area_fixture/map')
        odom.child_frame_id = 'area_fixture/base_link'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        points = [(3.*math.cos(a*math.pi/360), 3.*math.sin(a*math.pi/360), -.5) for a in range(720)]
        cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='area_fixture/lidar'), points))
    fixture.create_timer(.1, sensors)
    client = ActionClient(fixture, AreaSearch, 'area_search')
    def wait(predicate, label, timeout=18.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError(f'Timeout {label}: state={area.state}, engine={area.engine}')
            executor.spin_once(timeout_sec=.05)
    try:
        wait(lambda: area.navigation_ready() and area.origin is not None, 'readiness')
        goal = AreaSearch.Goal(task_id='area_protocol')
        goal.area.center.latitude, goal.area.center.longitude = 31., 120.
        goal.area.length_m, goal.area.width_m, goal.area.yaw_deg = 10., 10., 30.
        sent = client.send_goal_async(goal)
        wait(sent.done, 'acceptance')
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        wait(lambda: maps and paths and splines, 'native map, optimized B-spline and FollowPath')
        assert len(splines[0].pos_pts) > 3 and len(splines[0].knots) > 0
        assert paths[0].header.frame_id == 'area_fixture/map'
        assert any(v >= 0 for v in maps[-1].data)
        assert 0 < area.metrics['observed_ratio'] < 1
        canceled = handle.cancel_goal_async()
        wait(canceled.done, 'cancel ACK')
        wait(result.done, 'Action canceled after process/navigation release')
        assert result.result().status == 5
        assert area.process is None and not area.handles and not area.pending
        print('PASS: actual AreaSearch Action -> original co-search -> measured map/optimized B-spline -> FollowPath; cancellation releases navigation and process')
    finally:
        area.cleanup_process()
        server.destroy()
        for node in (area, fixture):
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
