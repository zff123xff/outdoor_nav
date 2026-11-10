"""Synchronize a UE origin in the background, without restarting or blocking navigation."""
import math
import re
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import NavSatFix

from .simulation_origin import ORIGIN_PARAMETERS, validate_origin


class SimulationOriginReceiver(Node):
    def __init__(self):
        super().__init__('simulation_origin_receiver')
        for key, value in (
                ('origin_topic', 'simulation/gps_origin'),
                ('target_nodes', ['nav2_input_bridge', 'mavros_odom_relay', 'point_goal_executor', 'patrol_node']),
                ('retry_period_s', .2), ('refresh_period_s', 5.), ('service_timeout_s', 3.)):
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        targets = self.get_parameter('target_nodes').value
        if not targets or any(not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', v) for v in targets):
            raise ValueError('target_nodes must be nonempty relative node names within this namespace')
        self.refresh_period = self.get_parameter('refresh_period_s').value
        self.service_timeout = self.get_parameter('service_timeout_s').value
        retry = self.get_parameter('retry_period_s').value
        if any(not math.isfinite(v) or v <= 0. for v in (retry, self.refresh_period, self.service_timeout)):
            raise ValueError('Origin retry, refresh and service timeout periods must be finite and positive')
        topic = self.get_parameter('origin_topic').value
        if not topic:
            raise ValueError('origin_topic must not be empty')
        self.origin_clients = {name: self.create_client(SetParametersAtomically, name+'/set_parameters_atomically')
                        for name in dict.fromkeys(targets)}
        self.desired_origin = None
        self.applied = {}
        self.pending = {}
        self.last_sync = {}
        self.published_origin = None
        self.live = self.create_subscription(NavSatFix, topic, self.receive, qos_profile_sensor_data)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.retained = self.create_subscription(NavSatFix, topic, self.receive, retained)
        output_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(NavSatFix, 'navigation/gps_origin', output_qos)
        self.timer = self.create_timer(retry, self.synchronize, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Listening for UE origin on {self.live.topic_name}; '
                               'navigation starts normally using YAML until an origin arrives')

    def receive(self, message):
        try:
            if message.status.status < 0:
                raise ValueError('origin NavSatFix status must not be STATUS_NO_FIX')
            origin = validate_origin(message.latitude, message.longitude, message.altitude)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.)
            return
        if origin != self.desired_origin:
            self.desired_origin = origin
            self.get_logger().info(f'UE GPS origin received: {origin}; synchronizing conversion parameters')

    def synchronize(self):
        origin = self.desired_origin
        if origin is None:
            return
        now = time.monotonic()
        for name, client in self.origin_clients.items():
            if name in self.pending:
                future, sent_origin, since = self.pending[name]
                if future.done():
                    self.pending.pop(name)
                    try:
                        result = future.result().result
                        if not result.successful:
                            raise ValueError(result.reason)
                        self.applied[name] = sent_origin
                        self.last_sync[name] = now
                    except Exception as exc:
                        self.get_logger().warning(f'Origin update to {name} failed: {exc}', throttle_duration_sec=5.)
                elif now-since > self.service_timeout:
                    future.cancel()
                    self.pending.pop(name)
                    self.get_logger().warning(f'Origin update to {name} timed out; retrying', throttle_duration_sec=5.)
                else:
                    continue
            if not client.service_is_ready():
                self.applied.pop(name, None)
                continue
            if self.applied.get(name) == origin and now-self.last_sync.get(name, 0.) < self.refresh_period:
                continue
            request = SetParametersAtomically.Request(parameters=[
                Parameter(key, value=value).to_parameter_msg() for key, value in zip(ORIGIN_PARAMETERS, origin)])
            self.pending[name] = (client.call_async(request), origin, now)
        if all(self.applied.get(name) == origin for name in self.origin_clients) and self.published_origin != origin:
            message = NavSatFix(latitude=origin[0], longitude=origin[1], altitude=origin[2])
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'wgs84'
            self.publisher.publish(message)
            self.published_origin = origin
            self.get_logger().info(f'GPS origin synchronized: {origin}; existing ENU paths and TF remain unchanged')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimulationOriginReceiver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
