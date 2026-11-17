#!/usr/bin/env python3
"""Real parameter services and cached GPS conversion, isolated from robot control."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParametersAtomically
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

from co_search_nav.executor import CoSearchExecutor
from mavros_odom_relay.mavros_odom_relay import OdomRelay
from patrol_node.patrol_node import PatrolNode
from patrol_node.point_goal_node import PointGoalNode
from robot_launch.simulation_origin import ORIGIN_PARAMETERS
from robot_launch.simulation_origin_receiver import SimulationOriginReceiver


def main():
    config = Path(get_package_share_directory('co_search_nav'))/'config/area_search.yaml'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/origin_fixture',
                     '-p', f'config_file:={config}', '-p', 'publish_gps:=true',
                     '-p', 'reference_latitude:=40.0', '-p', 'reference_longitude:=113.0',
                     '-p', 'reference_altitude:=1200.0'])
    executor = SingleThreadedExecutor()
    probe, relay, point, patrol, area = Node('probe'), OdomRelay(), PointGoalNode(), PatrolNode(), CoSearchExecutor()
    nodes = [probe, relay, point, patrol, area]
    for node in nodes:
        executor.add_node(node)
    executable = Path(get_package_prefix('nav2_input_bridge'))/'lib/nav2_input_bridge/input_bridge_node'
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/origin_fixture',
                                '-p', 'reference_latitude:=40.0', '-p', 'reference_longitude:=113.0',
                                '-p', 'reference_altitude:=1200.0', '-p', 'goal_yaw_from_bearing:=false'],
                               stdout=log, stderr=log, start_new_session=True)
    goals = probe.create_publisher(NavSatFix, 'gp_goal', 10)
    converted = []
    probe.create_subscription(PoseStamped, 'goal_pose', converted.append, 10)
    parameters = probe.create_client(SetParametersAtomically, 'nav2_input_bridge/set_parameters_atomically')

    def wait(predicate, label, timeout=15.):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert process.poll() is None, 'input bridge exited'
            assert time.monotonic() < deadline, label
            executor.spin_once(timeout_sec=.03)

    def convert(origin):
        before = len(converted)
        request = NavSatFix(latitude=origin[0], longitude=origin[1], altitude=origin[2])
        goals.publish(request)
        wait(lambda: len(converted) > before, 'converted goal receipt')
        p = converted[-1].pose.position
        assert max(abs(p.x), abs(p.y), abs(p.z)) < 1e-6, (p.x, p.y, p.z)

    try:
        wait(lambda: parameters.service_is_ready() and goals.get_subscription_count() > 0 and
             area.origin is not None, 'startup without any UE origin')
        assert point._origin() == (40., 113., 1200.)
        convert((40., 113., 1200.))
        # Publish before constructing the receiver: retained bridge data must work.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origins = probe.create_publisher(NavSatFix, 'simulation/gps_origin', qos)
        first = (31., 120., 15.)
        origins.publish(NavSatFix(latitude=first[0], longitude=first[1], altitude=first[2]))
        receiver = SimulationOriginReceiver()
        nodes.append(receiver)
        executor.add_node(receiver)
        wait(lambda: receiver.published_origin == first and area.origin == first, 'retained UE origin synchronized')
        assert relay._origin == point._origin() == patrol._origin() == first
        convert(first)  # Verifies C++ projection cache, not merely its parameter display.
        before = len(converted)
        second = (32., 121., 25.)
        origins.publish(NavSatFix(latitude=second[0], longitude=second[1], altitude=second[2]))
        wait(lambda: receiver.published_origin == second and area.origin == second, 'runtime origin update')
        assert relay._origin == point._origin() == patrol._origin() == second
        assert len(converted) == before, 'parameter changes must not republish/reproject an active ENU goal'
        convert(second)
        invalid = SetParametersAtomically.Request(parameters=[
            Parameter('reference_latitude', value=91.).to_parameter_msg()])
        response = parameters.call_async(invalid)
        wait(response.done, 'invalid origin rejection')
        assert not response.result().result.successful
        convert(second)
        print('PASS: navigation converters start before UE origin; retained and runtime origins update '
              'C++ projection, odom relay, point/patrol and area caches; existing ENU goals unchanged')
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.seek(0)
        if process.returncode:
            print(log.read())
        log.close()


if __name__ == '__main__':
    main()
