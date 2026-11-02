#!/usr/bin/env python3
"""Adapt sensor-only FAST-LIO IMU poses to the navigation base_link frame."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rotate(q, v):
    return multiply(multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))[:3]


class LioNavOdometry(Node):
    def __init__(self):
        super().__init__('lio_nav_odometry')
        self.declare_parameter('initial_base_height', 0.30)
        self.declare_parameter('imu_xyz', [-0.02557, 0.0, 0.04232])
        self.origin = None
        self.previous = None
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, 'lio/odometry', self.receive, 10)
        prefix = self.get_namespace().strip('/')
        self.odom_frame = prefix + '/odom' if prefix else 'odom'
        self.base_frame = prefix + '/base_link' if prefix else 'base_link'

    def receive(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v*v for v in quat))
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, *quat)) or not math.isfinite(norm) or norm < 1e-9:
            return
        if self.previous is not None and stamp <= self.previous[0]:
            # Starting a new simulation epoch requires restarting the LIO as well.
            return
        quat = tuple(v / norm for v in quat)
        offset = rotate(quat, self.get_parameter('imu_xyz').value)
        position = (p.x-offset[0], p.y-offset[1], p.z-offset[2])
        if self.origin is None:
            yaw = math.atan2(2*(quat[3]*quat[2]+quat[0]*quat[1]),
                             1-2*(quat[1]**2+quat[2]**2))
            self.origin = (position, (0., 0., math.sin(-yaw/2), math.cos(-yaw/2)))
        origin, alignment = self.origin
        position = rotate(alignment, tuple(a-b for a, b in zip(position, origin)))
        position = (position[0], position[1], position[2]+self.get_parameter('initial_base_height').value)
        quat = multiply(alignment, quat)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
        q = out.pose.pose.orientation
        q.x, q.y, q.z, q.w = quat
        # Derive measured body twist from consecutive estimated poses, not cmd_vel.
        if self.previous is not None:
            prev_stamp, prev_pos, prev_quat = self.previous
            dt = stamp-prev_stamp
            if 0.001 <= dt <= 0.5:
                velocity = rotate((-quat[0], -quat[1], -quat[2], quat[3]),
                                  tuple((a-b)/dt for a, b in zip(position, prev_pos)))
                v = out.twist.twist.linear
                v.x, v.y, v.z = velocity
                dq = multiply((-prev_quat[0], -prev_quat[1], -prev_quat[2], prev_quat[3]), quat)
                sign = 1.0 if dq[3] >= 0 else -1.0
                angular = out.twist.twist.angular
                angular.x, angular.y, angular.z = [2*sign*x/dt for x in dq[:3]]
        self.previous = (stamp, position, quat)
        self.publisher.publish(out)
        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = position
        tf.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = LioNavOdometry()
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
