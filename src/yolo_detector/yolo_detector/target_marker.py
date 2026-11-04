"""Area-search simulation observations and RViz marks; no actuator outputs."""
from collections import deque
import math
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, PointCloud2, NavSatFix, NavSatStatus
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from task_interfaces.msg import TargetEvent
from visualization_msgs.msg import Marker, MarkerArray
from yolo_interfaces.msg import DetectionArray
from tf2_ros import Buffer, TransformListener
from .projection import transform_matrix, apply_transform, project, box_position, enu_to_gps, Tracks
from .targets import targets_path, read_targets, matches


class SimulationTargetMarker(Node):
    def __init__(self):
        super().__init__('simulation_target_marker')
        defaults = dict(detection_topic='yolo/detections', camera_info_topic='front_camera/camera_info',
            cloud_topic='livox/lidar', found_topic='targets/found', request_topic='targets/request',
            marker_topic='targets/markers', map_frame='map', sensor_timeout_s=1.5,
            sync_tolerance_s=.15, min_lidar_points=3, max_depth_spread_m=.5,
            track_distance_m=1., targets_file='')
        self.settings = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        for key in ('sensor_timeout_s','sync_tolerance_s','max_depth_spread_m','track_distance_m'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if self.settings['min_lidar_points'] < 1:
            raise ValueError('min_lidar_points must be positive')
        self.ns = self.get_namespace().strip('/')
        self.frame = self.settings['map_frame'].strip('/')
        if self.ns and not self.frame.startswith(self.ns+'/'):
            self.frame = self.ns+'/'+self.frame
        self.path = targets_path(self.ns, self.settings['targets_file'])
        self.targets = []
        self.active = False
        self.active_since = 0.
        self.camera = None
        self.clouds = deque(maxlen=30)
        self.origin = None
        self.origin_call = None
        self.origin_since = 0.
        self.tracks = Tracks(self.settings['track_distance_m'])
        self.last_published = {}
        self.class_ids = {}
        self.marked = set()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.origin_client = self.create_client(GetParameters, 'nav2_input_bridge/get_parameters')
        self.found = self.create_publisher(TargetEvent, self.settings['found_topic'], 10)
        self.markers = self.create_publisher(MarkerArray, self.settings['marker_topic'], 10)
        self.create_subscription(DetectionArray, self.settings['detection_topic'], self.detected, 10)
        self.create_subscription(CameraInfo, self.settings['camera_info_topic'],
                                 lambda msg:setattr(self,'camera',msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.settings['cloud_topic'], self.cloud, qos_profile_sensor_data)
        self.create_subscription(TargetEvent, self.settings['request_topic'], self.request, 10)
        self.create_subscription(Bool, 'area_search/active', self.activation,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1., self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.poll()

    def activation(self, msg):
        if msg.data and not self.active:
            self.active_since = self.get_clock().now().nanoseconds/1e9
            self.poll()
        if not msg.data:
            self.clouds.clear()
        self.active = msg.data

    def cloud(self, msg):
        if self.active:
            self.clouds.append(msg)

    def poll(self):
        try:
            self.targets = read_targets(self.path)
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f'Target classes unchanged: {exc}', throttle_duration_sec=10.)
        if self.origin is not None:
            return
        if self.origin_call is not None:
            if time.monotonic()-self.origin_since > 3.:
                self.origin_call.cancel()
                self.origin_call = None
            return
        if not self.origin_client.service_is_ready():
            return
        future = self.origin_client.call_async(GetParameters.Request(names=[
            'reference_latitude','reference_longitude','reference_altitude']))
        self.origin_call = future
        self.origin_since = time.monotonic()
        def done(result):
            if self.origin_call is not result:
                return
            self.origin_call = None
            try:
                values = tuple(float(parameter_value_to_python(p)) for p in result.result().values)
                if len(values) == 3 and all(math.isfinite(v) for v in values) and abs(values[0])<=90 and abs(values[1])<=180:
                    self.origin = values
            except Exception as exc:
                self.get_logger().warning(f'Navigation origin unavailable: {exc}', throttle_duration_sec=10.)
        future.add_done_callback(done)

    def detected(self, msg):
        if not self.active or self.camera is None or self.origin is None or not self.clouds:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds/1e9
        now = self.get_clock().now().nanoseconds/1e9
        if abs(now-stamp)>self.settings['sensor_timeout_s'] or stamp<self.active_since-.05:
            return
        if ((msg.image_width,msg.image_height)!=(self.camera.width,self.camera.height) or
                not msg.header.frame_id or msg.header.frame_id!=self.camera.header.frame_id):
            self.get_logger().warning('Detection image and CameraInfo size/optical frame mismatch',throttle_duration_sec=5.)
            return
        cloud = min(self.clouds, key=lambda c:abs(Time.from_msg(c.header.stamp).nanoseconds/1e9-stamp))
        cloud_stamp = Time.from_msg(cloud.header.stamp).nanoseconds/1e9
        if abs(stamp-cloud_stamp)>self.settings['sync_tolerance_s']:
            return
        boxes = [box for box in msg.detections if matches(self.targets,box.class_name,box.class_id)]
        if not boxes:
            return
        try:
            raw = list(point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True))
            if not raw:
                return
            points = np.array([(float(p[0]),float(p[1]),float(p[2])) for p in raw])
            camera_tf = self.tf.lookup_transform_full(msg.header.frame_id,Time.from_msg(msg.header.stamp),
                cloud.header.frame_id,Time.from_msg(cloud.header.stamp),self.frame)
            map_tf = self.tf.lookup_transform(self.frame,cloud.header.frame_id,Time.from_msg(cloud.header.stamp))
            camera_points = apply_transform(points,transform_matrix(camera_tf.transform))
            valid = np.isfinite(camera_points).all(axis=1) & (camera_points[:,2]>.05)
            points, camera_points = points[valid], camera_points[valid]
            if not len(points):
                return
            uv = project(camera_points,self.camera)
            map_points = apply_transform(points,transform_matrix(map_tf.transform))
            observations = []
            class_ids = []
            for box in boxes:
                position = box_position(box,uv,camera_points,map_points,
                    self.settings['min_lidar_points'],self.settings['max_depth_spread_m'])
                if position is not None:
                    observations.append((box.class_name,position))
                    class_ids.append(box.class_id)
            ids = self.tracks.update(observations,now)
            self.class_ids.update(zip(ids, class_ids))
            for target_id,(kind,point) in zip(ids,observations):
                if now-self.last_published.get(target_id,-math.inf)<1.:
                    continue
                self.last_published[target_id]=now
                latitude,longitude,altitude=enu_to_gps(point,self.origin)
                event=TargetEvent(target_id=target_id,target_type=kind,command='target found')
                event.header.stamp=msg.header.stamp
                event.header.frame_id=self.frame
                event.position=NavSatFix(header=event.header,status=NavSatStatus(status=0),
                    latitude=latitude,longitude=longitude,altitude=altitude)
                self.found.publish(event)
                self.publish_marker(target_id,point,kind,False)
        except Exception as exc:
            self.get_logger().warning(f'No 3D observation: {exc}',throttle_duration_sec=5.)

    def request(self, event):
        # This protocol only changes visualization; it has no actuator clients.
        if not self.active or event.command!='request shoot':
            return
        track=self.tracks.items.get(event.target_id)
        now=self.get_clock().now().nanoseconds/1e9
        if (track is None or now-track['stamp']>self.settings['sensor_timeout_s'] or
                not matches(self.targets,track['kind'],self.class_ids.get(event.target_id))):
            self.get_logger().warning('Ignoring unknown/stale simulation marker request',throttle_duration_sec=5.)
            return
        self.marked.add(event.target_id)
        self.publish_marker(event.target_id,track['point'],track['kind'],True)
        self.get_logger().info(f'Simulation mark only: target {event.target_id}')

    def publish_marker(self,target_id,point,kind,requested):
        requested = requested or target_id in self.marked
        marker=Marker()
        marker.header.frame_id=self.frame
        marker.header.stamp=self.get_clock().now().to_msg()
        marker.ns='simulation_targets'
        marker.id=int(target_id)
        marker.type=Marker.SPHERE
        marker.action=Marker.ADD
        marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,point)
        marker.pose.orientation.w=1.
        marker.scale.x=marker.scale.y=marker.scale.z=.3
        marker.color.a=1.
        marker.color.r=1. if requested else 0.
        marker.color.g=0. if requested else 1.
        marker.text=f'SIMULATION {kind} #{target_id}'
        marker.lifetime.sec=3
        self.markers.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node=SimulationTargetMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
