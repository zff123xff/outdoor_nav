#!/usr/bin/env python3
"""Test the actual rate configurator against a fake FCU service and 30 Hz GPS."""
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from sensor_msgs.msg import NavSatFix
from robot_launch.mavros_global_rate import MavrosGlobalRate


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/rate_fixture'])
    fixture, rate = Node('fixture'), MavrosGlobalRate()
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    executor.add_node(rate)
    calls = []
    def interval(request, response):
        calls.append((request.message_id, request.message_rate))
        response.success = True
        return response
    fixture.create_service(MessageInterval, 'mavros/set_message_interval', interval)
    state = fixture.create_publisher(State, 'mavros/state', 10)
    fix = fixture.create_publisher(NavSatFix, 'mavros/global_position/global', 10)
    fixture.create_timer(.2, lambda: state.publish(State(connected=True)))
    fixture.create_timer(1./30., lambda: fix.publish(NavSatFix(latitude=31., longitude=120.)))
    try:
        deadline = time.monotonic()+13.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if calls and rate.configured and len(rate.samples) >= 150:
                break
        assert calls == [(33, 30.)], calls
        assert rate.configured and len(rate.samples) >= 150
        hz = (len(rate.samples)-1)/(rate.samples[-1]-rate.samples[0])
        assert 27. <= hz <= 33., hz
        rate.tick()
        print(f'PASS: GLOBAL_POSITION_INT id=33, request=30 Hz; actual fixture reception={hz:.1f} Hz')
    finally:
        for node in (rate, fixture):
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
