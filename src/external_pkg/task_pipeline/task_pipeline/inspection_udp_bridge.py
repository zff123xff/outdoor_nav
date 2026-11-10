"""Dedicated simulation transport for assignments and observation arrivals only."""
import json
import math
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from task_interfaces.msg import TargetArray, TargetInfo, InspectionArrival
from task_pipeline.inspection_logic import validate_batch


def decode_assignment(packet, namespace):
    data = json.loads(packet)
    if not isinstance(data, dict):
        raise ValueError('assignment envelope must be an object')
    if data.get('namespace') != namespace or data.get('topic') != 'targets/assigned':
        raise ValueError('wrong namespace or topic')
    if not isinstance(data.get('data'), dict):
        raise ValueError('assignment data must be an object')
    items = data['data']['targets']
    if not isinstance(items, list) or len(items) > 1000:
        raise ValueError('targets must be an array with at most 1000 items')
    result = TargetArray()
    for item in items:
        if not isinstance(item, dict) or not isinstance(item.get('target_position'), dict):
            raise ValueError('each target must contain a position object')
        key = item['target_id']
        if isinstance(key, bool) or not isinstance(key, int) or not 0 < key < 2**64:
            raise ValueError('target_id must be a positive uint64')
        xyz = item['target_position']
        values = [xyz[k] for k in ('x', 'y', 'z')]
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values):
            raise ValueError('target_position must have finite numeric x/y/z')
        result.targets.append(TargetInfo(target_id=key,
            target_position=Point(**dict(zip(('x', 'y', 'z'), map(float, values))))))
    validate_batch(result)
    return result


def encode_arrival(message, namespace):
    point = lambda p: dict(x=p.x, y=p.y, z=p.z)
    return json.dumps(dict(namespace=namespace, topic='targets/inspection_arrived', data=dict(
        task_id=message.task_id, target_id=message.target_id,
        target_position=point(message.target_position), robot_position=point(message.robot_position),
        distance_m=message.distance_m, heading_error_rad=message.heading_error_rad)),
        allow_nan=False).encode()


class InspectionUdpBridge(Node):
    def __init__(self):
        super().__init__('inspection_udp_bridge')
        for name, value in dict(bind_address='0.0.0.0', input_port=9101,
                                ue_address='127.0.0.1', output_port=9001).items():
            self.declare_parameter(name, value)
        self.ns = self.get_namespace().strip('/')
        if not self.ns:
            raise ValueError('inspection UDP bridge requires a robot namespace')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((self.get_parameter('bind_address').value, self.get_parameter('input_port').value))
        self.destination = (self.get_parameter('ue_address').value, self.get_parameter('output_port').value)
        self.pub = self.create_publisher(TargetArray, 'targets/assigned', 10)
        self.create_subscription(InspectionArrival, 'targets/inspection_arrived', self.arrival, 10)
        self.create_timer(.02, self.receive)
        self.last_warning = 0.
        self.get_logger().info(f'Inspection UDP input={self.socket.getsockname()}, arrivals={self.destination}')

    def receive(self):
        for _ in range(32):
            try:
                packet, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                self.pub.publish(decode_assignment(packet, self.ns))
            except (ValueError, KeyError, TypeError, OverflowError, UnicodeError) as exc:
                import time
                now = time.monotonic()
                if now-self.last_warning > 5.:
                    self.get_logger().warning(f'Invalid inspection assignment from {address}: {exc}')
                    self.last_warning = now

    def arrival(self, message):
        try:
            self.socket.sendto(encode_arrival(message, self.ns), self.destination)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Cannot send inspection arrival: {exc}')

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectionUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
