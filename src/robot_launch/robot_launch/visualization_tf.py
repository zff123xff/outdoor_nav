#!/usr/bin/env python3
"""Merge navigation/model TF for RViz only; never publish navigation TF.

Gazebo joint transforms use simulation time. Copy them with wall time into a
private visualization tree. Navigation transforms retain their original stamps
and take priority for any shared child frame.
"""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage


class VisualizationTF(Node):
    def __init__(self):
        super().__init__('visualization_tf')
        self.static = ({}, {})
        self.dynamic = ({}, {})
        self.outputs = [self.create_publisher(TFMessage, 'visualization/tf', 100),
            self.create_publisher(TFMessage, 'visualization/tf_static',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        self.listeners = []
        for model, topic in ((False, '/tf'), (True, 'simulation/tf')):
            for static in (False, True):
                qos = (QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                       if static else QoSProfile(depth=100))
                self.listeners.append(self.create_subscription(TFMessage,
                    topic + ('_static' if static else ''),
                    lambda msg, m=model, s=static: self.receive(msg, m, s), qos))

    def receive(self, msg, model, static):
        cache = self.static if static else self.dynamic
        for transform in msg.transforms:
            cache[int(model)][transform.child_frame_id] = deepcopy(transform)
        merged = dict(cache[1])
        merged.update(cache[0])  # Navigation owns overlapping frames.
        output = TFMessage()
        now = self.get_clock().now().to_msg()
        for child, transform in merged.items():
            # Publish changed dynamic transforms only; avoid refreshing stale nav TF.
            if not static and child not in {t.child_frame_id for t in msg.transforms}:
                continue
            if not static and model and child in cache[0]:
                continue
            copied = deepcopy(transform)
            if child not in cache[0]:
                copied.header.stamp = now
            output.transforms.append(copied)
        if output.transforms:
            self.outputs[int(static)].publish(output)


def main():
    rclpy.init()
    node = VisualizationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
