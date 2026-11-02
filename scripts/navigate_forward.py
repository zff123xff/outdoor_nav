#!/usr/bin/env python3
"""Send a map goal ahead of the current simulated robot heading."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=5.0)
    parser.add_argument('--namespace', default='robot')
    args = parser.parse_args()
    if not math.isfinite(args.distance) or args.distance <= 0:
        parser.error('distance must be finite and positive')
    ns = args.namespace.strip('/')
    if not ns or '/' in ns:
        parser.error('namespace must be a single robot name')
    rclpy.init()
    node = Node('navigate_forward_once', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    ready = [False]
    status = node.create_subscription(String, '/' + ns + '/ekf/status',
        lambda msg: ready.__setitem__(0, 'ready=1' in msg.data.split()), 10)
    publisher = node.create_publisher(PoseStamped, '/' + ns + '/goal_pose', 10)
    clients = [node.create_client(GetState, '/' + ns + '/' + name + '/get_state')
               for name in ('controller_server', 'straight_planner')]
    futures = [None, None]
    transform = None
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for i, client in enumerate(clients):
                if futures[i] is None and client.service_is_ready():
                    futures[i] = client.call_async(GetState.Request())
            active = all(f is not None and f.done() and f.exception() is None and
                         f.result().current_state.id == 3 for f in futures)
            if not active:
                for i, f in enumerate(futures):
                    if f is not None and f.done() and (f.exception() is not None or f.result().current_state.id != 3):
                        futures[i] = None
                continue
            if not ready[0] or node.get_clock().now().nanoseconds == 0:
                continue
            if publisher.get_subscription_count() == 0:
                continue
            try:
                transform = buffer.lookup_transform(ns + '/map', ns + '/base_link', rclpy.time.Time())
                age = (node.get_clock().now().nanoseconds -
                       rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
                if not 0 <= age <= 1.0:
                    transform = None
                    continue
            except Exception:
                continue
            break
        if transform is None:
            states = [f.result().current_state.label if f is not None and f.done() and f.exception() is None else 'no response' for f in futures]
            print(f'Readiness: EKF={ready[0]}, clock={node.get_clock().now().nanoseconds}, goal subscribers={publisher.get_subscription_count()}, lifecycle={states}', flush=True)
            print(buffer.all_frames_as_yaml(), flush=True)
            raise RuntimeError('No fresh robot TF with EKF ready and both navigation nodes active; no goal sent.')
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        goal = PoseStamped()
        goal.header.frame_id = ns + '/map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = t.x + args.distance * math.cos(yaw)
        goal.pose.position.y = t.y + args.distance * math.sin(yaw)
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        publisher.publish(goal)
        print(f'Sent {args.distance:.2f} m forward goal: current=({t.x:.3f}, {t.y:.3f}), '
              f'yaw={math.degrees(yaw):.1f} deg, goal=({goal.pose.position.x:.3f}, '
              f'{goal.pose.position.y:.3f}) in {goal.header.frame_id}', flush=True)
        # Allow DDS to deliver the one goal before this publisher exits.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
