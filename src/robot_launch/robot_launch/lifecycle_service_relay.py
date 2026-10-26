#!/usr/bin/env python3
"""Bound lifecycle RPCs and reconcile transitions whose replies were lost."""
import time
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState


TARGETS = {1: 2, 2: 1, 3: 3, 4: 2, 5: 4, 6: 4, 7: 4}


class LifecycleRelay(Node):
    def __init__(self):
        super().__init__('lifecycle_service_relay')
        self.timeout = float(self.declare_parameter('response_timeout', 30.0).value)
        self.query_timeout = float(self.declare_parameter('query_timeout', 2.0).value)
        self.attempts = int(self.declare_parameter('query_attempts', 3).value)
        if self.timeout <= 0 or self.query_timeout <= 0 or self.attempts < 1:
            raise ValueError('Lifecycle timeouts and query_attempts must be positive')
        names = self.declare_parameter('node_names', ['controller_server', 'straight_planner']).value
        self.group = ReentrantCallbackGroup()
        self.rpc_clients = {}
        self.servers = []
        for name in names:
            # A per-node lock prevents overlapping transition requests.
            clients = (self.create_client(ChangeState, name+'/change_state', callback_group=self.group),
                       self.create_client(GetState, name+'/get_state', callback_group=self.group),
                       threading.Lock())
            self.rpc_clients[name] = clients
            self.servers.extend([
                self.create_service(ChangeState, 'lifecycle_relay/'+name+'/change_state',
                    lambda req, res, n=name: self.change(n, req, res), callback_group=self.group),
                self.create_service(GetState, 'lifecycle_relay/'+name+'/get_state',
                    lambda req, res, n=name: self.get(n, req, res), callback_group=self.group)])

    def call(self, client, request, timeout):
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(max(0.0, deadline-time.monotonic())):
            client.remove_pending_request(future)
            return None
        try:
            return future.result()
        except Exception as error:
            self.get_logger().warning('Lifecycle RPC failed: '+str(error))
            return None

    def state(self, name):
        for _ in range(self.attempts):
            result = self.call(self.rpc_clients[name][1], GetState.Request(), self.query_timeout)
            if result is not None:
                return result.current_state
        return None

    def get(self, name, request, response):
        state = self.state(name)
        if state is not None:
            response.current_state = state
        else:
            self.get_logger().error(name+': get_state timed out; returning unknown state')
        return response

    def change(self, name, request, response):
        with self.rpc_clients[name][2]:
            target = TARGETS.get(request.transition.id)
            before = self.state(name)
            if before is None:
                self.get_logger().error(name+': initial state unavailable; refusing transition')
                return response
            if target is not None and before is not None and before.id == target:
                response.success = True
                return response
            result = self.call(self.rpc_clients[name][0], request, self.timeout)
            if result is not None:
                response.success = result.success
                return response
            # Never replay a timed-out transition: the server may already have
            # executed it, or may still be processing it. Only reads are retried.
            after = self.state(name)
            response.success = target is not None and after is not None and after.id == target
            if response.success:
                self.get_logger().warning(
                    name+': transition reply timed out, but target state '+str(target)+' verified; continuing')
            else:
                self.get_logger().error(
                    name+': transition reply timed out; target state not verified; startup fails explicitly')
            return response


def main():
    rclpy.init()
    node = LifecycleRelay()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
