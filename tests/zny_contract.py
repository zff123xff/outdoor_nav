#!/usr/bin/env python3
"""Isolated protocol fixture: no robot, Gazebo, /clock or external TF source.

Artificial odometry/cloud messages exist only in an initially empty test domain.
This checks navigation wiring and command output, not physical motion/arrival.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPointStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'deploy/nx/reports/zny_contract.json')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    graph = subprocess.run(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '3'],
                           capture_output=True, text=True, timeout=12, check=True)
    if any(topic not in ('/parameter_events', '/rosout') for topic in graph.stdout.split()):
        raise RuntimeError('Choose an empty test domain')
    logs = Path(tempfile.mkdtemp(prefix='zny_nav_contract_'))
    os.environ['ROS_LOG_DIR'] = str(logs/'ros')
    os.environ['HIL_SIM_WS'] = str(logs/'nonexistent_simulation_workspace')
    rclpy.init()
    node = Node('zny_protocol_fixture')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    odom = node.create_publisher(Odometry, '/zny_probe/mavros/odom', qos_profile_sensor_data)
    cloud = node.create_publisher(PointCloud2, '/zny_probe/livox/lidar', qos_profile_sensor_data)
    origin = node.create_publisher(GeoPointStamped, '/zny_probe/mavros/set_gp_origin', 10)
    goal = node.create_publisher(NavSatFix, '/zny_probe/gp_goal', 10)
    stop = node.create_publisher(Bool, '/zny_probe/target_detected', 10)
    legacy_odom = node.create_publisher(Odometry, '/legacy_probe/mavros/odom', qos_profile_sensor_data)
    values = {}
    subscriptions = [
        node.create_subscription(Twist, '/zny_probe/cmd_vel', lambda m: values.update(cmd=m), 10),
        node.create_subscription(PoseStamped, '/zny_probe/goal_pose', lambda m: values.update(goal=m), 10),
        node.create_subscription(Odometry, '/zny_probe/odom', lambda m: values.update(relayed=m), 10),
    ]
    processes, streams = [], []
    result = {'domain_id': args.domain_id, 'fixture_only': True, 'physical_motion_tested': False}
    config = yaml.safe_load((ROOT/'src/robot_launch/config/zny_hil.yaml').read_text())
    def force_clock(value):
        if isinstance(value, dict):
            return {k: True if k == 'use_sim_time' else force_clock(v) for k,v in value.items()}
        return value
    # Explicitly hostile YAML: the launch must still set every subnode to wall time.
    params = logs/'input.yaml'
    params.write_text(yaml.safe_dump(force_clock(config)))
    def start(name, command):
        stream = (logs/(name+'.log')).open('w'); streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    next_cloud = 0.
    def tick():
        nonlocal next_cloud
        stamp = node.get_clock().now().to_msg()
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'zny_probe/odom'
        message.child_frame_id = 'zny_probe/base_link'
        message.pose.pose.position.z = .42
        message.pose.pose.orientation.w = 1.
        odom.publish(message)
        message.header.frame_id = 'legacy_probe/odom'
        message.child_frame_id = 'legacy_probe/base_link'
        legacy_odom.publish(message)
        if time.monotonic() >= next_cloud:
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = 'zny_probe/livox_frame'
            message.height = message.width = 1
            message.fields = [PointField(name=k, offset=4*i, datatype=7, count=1) for i,k in enumerate('xyz')]
            message.point_step = message.row_step = 12
            message.is_dense = True
            message.data = struct.pack('<fff', 4., 4., .2)
            cloud.publish(message)
            next_cloud = time.monotonic()+.1
        rclpy.spin_once(node, timeout_sec=.02)
    def wait(predicate, label, seconds=30):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Navigation process exited; logs: '+str(logs))
            tick()
            if predicate(): return
        raise TimeoutError(label+'; logs: '+str(logs))
    def parameters(target, names):
        client=node.create_client(GetParameters, target+'/get_parameters')
        wait(client.service_is_ready, target+' parameters')
        request=GetParameters.Request();request.names=names
        future=client.call_async(request);wait(future.done,target+' parameter reply')
        return future.result().values
    try:
        start('navigation', ['bash',str(ROOT/'deploy/nx/run.sh'),'hil',
              'robot_namespace:=zny_probe','params_file:='+str(params)])
        start('legacy_defaults', ['ros2','run','nav2_input_bridge','input_bridge_node',
              '--ros-args','-r','__ns:=/legacy_probe'])
        for name in ['controller_server','straight_planner']:
            client=node.create_client(GetState,'/zny_probe/'+name+'/get_state')
            wait(client.service_is_ready,name+' lifecycle service')
            deadline=time.monotonic()+120
            while time.monotonic()<deadline:
                future=client.call_async(GetState.Request());wait(future.done,name+' state')
                if future.result().current_state.id==3:break
                tick()
            else:raise TimeoutError(name+' not active')
        active = node.create_client(Trigger, '/zny_probe/lifecycle_manager_navigation/is_active')
        wait(active.service_is_ready, 'manager active service')
        reply = active.call_async(Trigger.Request())
        wait(reply.done, 'manager bond verification')
        assert reply.result().success
        wait(lambda:buffer.can_transform('zny_probe/map','zny_probe/livox_frame',Time()),'full TF')
        transform=buffer.lookup_transform('zny_probe/odom','zny_probe/base_link',Time())
        assert abs(transform.transform.translation.z-.42)<1e-6
        transform=buffer.lookup_transform('zny_probe/base_link','zny_probe/livox_frame',Time())
        xyz=transform.transform.translation;q=transform.transform.rotation
        assert all(abs(a-b)<1e-6 for a,b in zip([xyz.x,xyz.y,xyz.z],[.18,0.,.53]))
        assert abs(2*math.atan2(q.y,q.w)-math.radians(18))<1e-4
        for target in ['nav2_input_bridge','controller_server','straight_planner','local_costmap/local_costmap']:
            assert parameters('/zny_probe/'+target,['use_sim_time'])[0].bool_value is False
        assert len(node.get_publishers_info_by_topic('/clock'))==0
        assert {e.node_name for e in node.get_publishers_info_by_topic('/tf')} <= {'nav2_input_bridge'}
        assert len(node.get_publishers_info_by_topic('/zny_probe/cmd_vel'))==1
        wait(lambda:'relayed' in values,'BEST_EFFORT -> RELIABLE odometry relay')
        # Hardware defaults retain planar TF and do not subscribe to external origins.
        wait(lambda:buffer.can_transform('legacy_probe/odom','legacy_probe/base_link',Time()),'legacy TF')
        assert abs(buffer.lookup_transform('legacy_probe/odom','legacy_probe/base_link',Time()).transform.translation.z)<1e-6
        assert not node.get_subscriptions_info_by_topic('/legacy_probe/mavros/set_gp_origin')
        wait(lambda:origin.get_subscription_count()>0,'origin subscriber')
        message=GeoPointStamped();message.position.latitude=31.25
        message.position.longitude=121.5;message.position.altitude=8.
        origin.publish(message)
        end=time.monotonic()+.5
        while time.monotonic()<end:tick()
        actual=parameters('/zny_probe/nav2_input_bridge',['reference_latitude','reference_longitude','reference_altitude'])
        assert [v.double_value for v in actual]==[31.25,121.5,8.]
        message.position.latitude=float('nan');origin.publish(message)
        for _ in range(10):tick()
        assert parameters('/zny_probe/nav2_input_bridge',['reference_latitude'])[0].double_value==31.25
        wait(lambda:goal.get_subscription_count()>0,'GPS goal subscriber')
        message=NavSatFix();message.header.stamp=node.get_clock().now().to_msg()
        message.latitude=31.25;message.longitude=121.5;message.altitude=8.
        goal.publish(message)
        wait(lambda:'goal' in values,'GPS projection after external origin')
        assert math.hypot(values['goal'].pose.position.x,values['goal'].pose.position.y)<1e-6
        message.longitude+=.00001;message.header.stamp=node.get_clock().now().to_msg();goal.publish(message)
        wait(lambda:'cmd' in values and abs(values['cmd'].linear.x)>.01,'navigation velocity output')
        stop.publish(Bool(data=True))
        wait(lambda:abs(values['cmd'].linear.x)<.01 and abs(values['cmd'].angular.z)<.01,'cancel/zero output')
        result.update(passed=True,no_clock=True,no_external_tf=True,full_odom_height=True,
                      no_simulation_workspace=True,
                      lidar_xyz=[.18,0.,.53],lidar_pitch_degrees=18,
                      wall_clock_for_costmap=True,best_effort_inputs=True,
                      external_origin_and_invalid_rejection=True,legacy_defaults_preserved=True,
                      velocity_output_and_stop=True)
    finally:
        stop.publish(Bool(data=True))
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()
        result['logs']=str(logs)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
