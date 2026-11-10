#!/usr/bin/env python3
"""Isolated DDS test of real odom relay and GPS subscription; no robot motion."""
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mavros_odom_relay.mavros_odom_relay import OdomRelay
from multi_goal_to_gp_goal.coordinates import lla_to_enu


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/gps_relay_fixture',
                    '-p', 'publish_gps:=true', '-p', 'reference_latitude:=40.9445353',
                    '-p', 'reference_longitude:=113.172819',
                    '-p', 'reference_altitude:=1298.7345711102703'])
    relay, fixture = OdomRelay(), Node('fixture')
    executor = SingleThreadedExecutor()
    for node in (relay, fixture):
        executor.add_node(node)
    fixes, odoms = [], []
    fixture.create_subscription(NavSatFix, 'mavros/global_position/global', fixes.append, qos_profile_sensor_data)
    fixture.create_subscription(Odometry, 'odom', odoms.append, 10)
    publisher = fixture.create_publisher(Odometry, 'mavros/odom', qos_profile_sensor_data)
    msg = Odometry()
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.header.frame_id = 'gps_relay_fixture/odom'
    msg.child_frame_id = 'gps_relay_fixture/base_link'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 10., 20., 3.
    fixture.create_timer(.1, lambda: publisher.publish(msg))
    try:
        deadline = time.monotonic() + 8.
        while not (fixes and odoms):
            assert time.monotonic() < deadline, 'No real GPS and/or RELIABLE odom message received'
            executor.spin_once(timeout_sec=.1)
        fix = fixes[-1]
        enu = lla_to_enu(fix.latitude, fix.longitude, fix.altitude,
                         40.9445353, 113.172819, 1298.7345711102703)
        assert all(abs(a-b) < 1e-6 for a, b in zip(enu, (10., 20., 3.))), enu
        assert fix.header.stamp == msg.header.stamp
        assert fix.header.frame_id == msg.child_frame_id
        assert odoms[-1] == msg
        print('PASS: BEST_EFFORT mavros/odom -> RELIABLE odom + NavSatFix; ENU round trip and original timestamp verified')
    finally:
        for node in (relay, fixture):
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
