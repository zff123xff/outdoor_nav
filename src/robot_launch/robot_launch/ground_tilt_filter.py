"""A shared low-pass terrain reference; localization TF remains untouched."""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from .ground_plane import rotation_matrix, fit_ground_plane


class TiltLowPass:
    def __init__(self, time_constant, reset_gap):
        if not math.isfinite(time_constant) or time_constant < 0:
            raise ValueError('time_constant_s must be finite and nonnegative')
        if not math.isfinite(reset_gap) or reset_gap <= 0:
            raise ValueError('reset_gap_s must be finite and positive')
        self.tau = time_constant
        self.reset_gap = reset_gap
        self.stamp = None
        self.roll = self.pitch = 0.0

    def update(self, roll, pitch, stamp):
        if self.stamp is None or stamp < self.stamp or stamp-self.stamp > self.reset_gap:
            self.roll, self.pitch = roll, pitch
        elif stamp > self.stamp:
            alpha = -math.expm1(-(stamp-self.stamp)/self.tau) if self.tau else 1.0
            self.roll += alpha*math.atan2(math.sin(roll-self.roll), math.cos(roll-self.roll))
            self.pitch += alpha*math.atan2(math.sin(pitch-self.pitch), math.cos(pitch-self.pitch))
        self.stamp = stamp
        return self.roll, self.pitch


class GroundTiltFilter(Node):
    def __init__(self):
        super().__init__('ground_tilt_filter')
        for key, value in [('global_frame', 'map'), ('robot_base_frame', 'base_link'),
                           ('height_reference_frame', 'ground_height_reference'),
                           ('time_constant_s', 0.3), ('reset_gap_s', 1.0),
                           ('estimate_ground_plane', False), ('cloud_topic', 'livox/lidar'),
                           ('ground_max_offset_m', .35), ('ground_max_tilt_deg', 20.0),
                           ('ground_fit_residual_m', .035)]:
            self.declare_parameter(key, value)
        self.world = self.get_parameter('global_frame').value
        self.base = self.get_parameter('robot_base_frame').value
        self.output = self.get_parameter('height_reference_frame').value
        if self.output in (self.world, self.base):
            raise ValueError('height reference must be a separate TF frame')
        self.filter = TiltLowPass(self.get_parameter('time_constant_s').value,
                                  self.get_parameter('reset_gap_s').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.last_stamp = None
        self.ground_plane = None; self.pending_cloud = None; self.ground_stamp = 0.0
        self.ground_offset = None
        if self.get_parameter('estimate_ground_plane').value:
            self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                     self.receive_cloud, qos_profile_sensor_data)
            self.create_timer(.2, self.update_ground)
        self.create_timer(0.02, self.publish_reference)

    def receive_cloud(self, msg):
        if self.pending_cloud is None:
            self.pending_cloud = msg

    def update_ground(self):
        msg = self.pending_cloud
        if msg is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        try:
            sensor = self.buffer.lookup_transform(self.world, msg.header.frame_id, stamp)
            body = self.buffer.lookup_transform(self.world, self.base, stamp)
        except TransformException:
            if (self.get_clock().now()-stamp).nanoseconds > 2e9:
                self.pending_cloud = None
            return
        self.pending_cloud = None
        raw = np.array([tuple(p) for p in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
        if not len(raw):
            return
        t = sensor.transform.translation; p = body.transform.translation
        points = raw @ rotation_matrix(sensor.transform.rotation).T + [t.x, t.y, t.z]
        plane = fit_ground_plane(points, [p.x, p.y, p.z],
                                 rotation_matrix(body.transform.rotation)[:, 2],
                                 max_offset=self.get_parameter('ground_max_offset_m').value,
                                 max_tilt_deg=self.get_parameter('ground_max_tilt_deg').value,
                                 residual=self.get_parameter('ground_fit_residual_m').value)
        if plane is not None:
            self.ground_plane = plane
            self.ground_stamp = stamp.nanoseconds/1e9

    def publish_reference(self):
        try:
            transform = self.buffer.lookup_transform(self.world, self.base, Time())
        except TransformException:
            return
        stamp = Time.from_msg(transform.header.stamp).nanoseconds
        if stamp == self.last_stamp:
            return
        q = transform.transform.rotation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-9:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        if getattr(self, 'ground_plane', None) is not None:
            a, b, c = self.ground_plane
            normal = np.array([-a, -b, 1.0]); normal /= np.linalg.norm(normal)
            nx = math.cos(yaw)*normal[0]+math.sin(yaw)*normal[1]
            ny = -math.sin(yaw)*normal[0]+math.cos(yaw)*normal[1]
            roll = -math.asin(max(-1.0, min(1.0, ny)))
            pitch = math.atan2(nx, normal[2])
            p = transform.transform.translation
            height = a*p.x+b*p.y+c
            dt = (stamp-self.last_stamp)/1e9 if self.last_stamp is not None and stamp > self.last_stamp else .02
            alpha = (-math.expm1(-dt/self.filter.tau)) if self.filter.tau else 1.0
            self.ground_offset = (height if self.ground_offset is None else
                                  self.ground_offset+alpha*(height-self.ground_offset))
            p.z = self.ground_offset
        roll, pitch = self.filter.update(roll, pitch, stamp/1e9)
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        q.x, q.y = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy
        q.z, q.w = cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
        transform.child_frame_id = self.output
        self.broadcaster.sendTransform(transform)
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GroundTiltFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
