#!/usr/bin/env python3
"""Compare ROS environments and verify real parameter/lifecycle responses."""
import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from task_pipeline.diagnostics import environment, navigation_processes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--timeout', type=float, default=5.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    ns = args.namespace.strip('/')
    if not ns:
        parser.error('--namespace must not be empty')
    report = dict(hostname=socket.gethostname(), terminal_environment=environment(),
                  navigation_processes=navigation_processes())
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('0.0.0.0', 0))
        report['udp_socket'] = 'OK'
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    rclpy.init()
    node = Node('task_ingress_diagnostic')
    try:
        # Bounded discovery without ros2 daemon; no environment changes or tasks.
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        print('visible_nodes=' + json.dumps(sorted(
            namespace.rstrip('/')+'/'+name for name, namespace in node.get_node_names_and_namespaces())))
        print(f'/{ns}/task subscribers={node.count_subscribers(f"/{ns}/task")}')
        print(f'/{ns}/task_feedback publishers={node.count_publishers(f"/{ns}/task_feedback")}')
        def call(service_type, path, request):
            client = node.create_client(service_type, path)
            try:
                if not client.wait_for_service(timeout_sec=args.timeout):
                    print(f'{path}: NOT DISCOVERED', flush=True)
                    return None
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
                if not future.done() or future.result() is None:
                    print(f'{path}: DISCOVERED BUT NO RESPONSE', flush=True)
                    return None
                return future.result()
            finally:
                node.destroy_client(client)
        response = call(GetParameters, f'/{ns}/nav2_input_bridge/get_parameters',
            GetParameters.Request(names=['reference_latitude', 'reference_longitude', 'reference_altitude']))
        if response is not None:
            print('live_origin=' + repr([parameter_value_to_python(v) for v in response.values]), flush=True)
        for name in ('controller_server', 'straight_planner'):
            response = call(GetState, f'/{ns}/{name}/get_state', GetState.Request())
            if response is not None:
                print(f'{name}: {response.current_state.label} ({response.current_state.id})', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
