#!/usr/bin/env python3
"""Disabled native-engine regression; synthetic clouds stay on a private topic.

Uses real robot odometry/TF read-only. Never publishes enable, tasks or velocity.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from nav_msgs.msg import Path as NavPath
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from co_search_nav.configuration import engine_parameters, native_remappings
from co_search_nav.task_geometry import validate_task, rectangle_for_task


def exercise(node, binary, suffix, live=False, accumulate=False):
    ns = 'cloud_throttle_probe_'+suffix
    received = []
    paths = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    state_sub = node.create_subscription(String, '/'+ns+'/co_search/engine_state',
        lambda m: received.append(json.loads(m.data)), qos)
    path_sub = node.create_subscription(NavPath, '/'+ns+'/co_search/path', paths.append, 1)
    pub = node.create_publisher(PointCloud2, '/'+ns+'/cloud', qos_profile_sensor_data)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., end-time.monotonic())))

    def wait(predicate, timeout=15.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            spin(.05)
        if not predicate():
            raise AssertionError('timeout: '+json.dumps(received[-3:]))

    def send(x, ys=(0., .2)):
        header = Header(frame_id='robot_1/map', stamp=node.get_clock().now().to_msg())
        pub.publish(create_cloud_xyz32(header, [(x, y, .5) for y in ys]))

    share = get_package_share_directory('co_search_nav')
    prefix = get_package_prefix('co_search_nav')
    task = yaml.safe_load((Path(share)/'config/area_search.yaml').read_text())
    task.update(robot_namespace='robot_1', coordinate_mode='enu',
                center_enu=dict(east=0., north=0.),
                rectangle=dict(length_m=10., width_m=10., yaw_deg=0.))
    task = validate_task(task)
    region = rectangle_for_task(task, None)
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix=ns+'_') as directory:
            params = engine_parameters(task, region, 'robot_1/map', directory, share, prefix, (0., 0.))
            params.update({'adapter/cloud_topic': '/robot_1/livox/lidar' if live else '/'+ns+'/cloud',
                           'adapter/odom_topic': '/robot_1/odom',
                           'adapter/plan_period_s': .2,
                           'adapter/map_period_s': .2})
            if not live:
                params.update({'adapter/cloud_enable_x_filter': True,
                               'adapter/cloud_min_x_m': 3., 'adapter/cloud_max_x_m': 10.,
                               'adapter/cloud_accumulation_window_s': 1.,
                               'adapter/cloud_min_accumulated_points': 6 if accumulate else 1})
            config = Path(directory)/'params.yaml'
            config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            command = [binary, '--ros-args', '-r', '__ns:=/'+ns, '--params-file', str(config)]
            for a, b in native_remappings(ns):
                command += ['-r', a+':='+b]
            with (Path(directory)/'engine.log').open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                wait(lambda: received and (live or pub.get_subscription_count() > 0), 25.)
                if live:
                    spin(10.)
                    clouds = max(s['clouds'] for s in received)
                    assert clouds > 5, received[-3:]
                    assert any(s['phase'] == 'ready' for s in received), received[-3:]
                    print(json.dumps(dict(case=suffix, clouds=clouds, final_state=received[-1])), flush=True)
                    return
                if accumulate:
                    send(4.); spin(.3)
                    send(4.); spin(.3)  # Repeated voxels must not inflate validity.
                    assert max(s['clouds'] for s in received) == 0, received[-3:]
                    spin(1.1)  # The two old voxels must expire.
                    send(4., (.4, .6, .8, 1.)); spin(.3)
                    assert max(s['clouds'] for s in received) == 0, received[-3:]
                    send(4.)
                    wait(lambda: max(s['clouds'] for s in received) >= 1)
                    assert received[-1]['cloud_window_points'] == 6, received[-3:]
                    print(json.dumps(dict(case=suffix, multi_frame_valid=True,
                                          repeated_voxels_deduplicated=True, old_points_expired=True)), flush=True)
                    return
                send(4.)
                wait(lambda: received and max(s['clouds'] for s in received) >= 1)
                spin(.3)
                send(-1.9)  # Invalid frame at an available update slot.
                spin(.03)
                send(4.)    # Valid frame arrives within that same 0.2-second slot.
                spin(.7)
                count = max(s['clouds'] for s in received)
                if count != (1 if suffix == 'before' else 2):
                    print(json.dumps(dict(case=suffix, replay_states=received[-5:])), flush=True)
                # Genuine missing valid input must still expire.
                spin(3.5)
                assert any(s['phase'] == 'waiting_sensor' for s in received[-5:]), received[-5:]
                send(4.)
                wait(lambda: max(s['clouds'] for s in received) > count)
                print(json.dumps(dict(case=suffix, clouds_after_invalid_then_valid=count,
                                      genuine_timeout=True, recovered=True)), flush=True)
                return count
    finally:
        if process is not None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
        node.destroy_subscription(state_sub)
        node.destroy_subscription(path_sub)
        node.destroy_publisher(pub)
        assert not paths, 'Disabled engine unexpectedly published a path'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True)
    args = parser.parse_args()
    binary = str(Path(get_package_prefix('co_search_nav'))/'lib/co_search_nav/search_engine')
    rclpy.init()
    node = Node('readonly_cloud_throttle_regression')
    try:
        old = exercise(node, args.baseline, 'before')
        new = exercise(node, binary, 'after')
        assert old == 1 and new == 2, (old, new)
        exercise(node, binary, 'window', accumulate=True)
        exercise(node, binary, 'live', live=True)
        print('PASS: invalid frames no longer consume valid update slots; true timeout and recovery retained; no paths or motion', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
