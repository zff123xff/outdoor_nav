"""Publish RTMP video on the detector's existing image topic.

Decoding and publishing are independent: the decoder thread only ever leaves the newest
frame in a single-slot buffer, and the publisher timer takes at most one frame per tick.
A frame is never published twice, so the reported publish rate cannot be inflated.

The optional small image is written before the full-resolution one, because the primary
write can block for seconds and would otherwise consume the small frame's age budget.
"""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from .stream import Settings, StreamWorker


def _frame_is_stale(settings, received, stamp):
    """True when a decoded frame waited too long locally or carries no clock."""
    return (time.monotonic()-received > settings.max_frame_age_s or
            (stamp.sec == 0 and stamp.nanosec == 0))


class RtmpImageBridge(Node):
    def __init__(self):
        super().__init__('rtmp_image_bridge')
        defaults = dict(rtmp_url='', mode='pull', image_topic='front_camera/image_raw',
                        frame_id='', width=640, height=480, encoding='rgb8', publish_fps=30.,
                        read_timeout_s=5., reconnect_delay_s=2., max_frame_age_s=.5,
                        ffmpeg_path='ffmpeg', gst_launch_path='gst-launch-1.0',
                        decoder='auto', stats_period_s=5., secondary_topic='',
                        secondary_width=0, secondary_height=0)
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        values = {key: self.get_parameter(key).value for key in defaults}
        image_topic = values.pop('image_topic')
        secondary_topic = values.pop('secondary_topic', '')
        frame_id = values.pop('frame_id')
        if not image_topic:
            raise ValueError('image_topic must not be empty')
        prefix = self.get_namespace().strip('/')
        self.frame_id = frame_id or '/'.join(filter(None, (prefix, 'front_camera_optical_frame')))
        self.settings = Settings(**values)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Image, image_topic, qos)
        self.secondary_publisher = (self.create_publisher(Image, secondary_topic, qos)
                                    if secondary_topic and self.settings.secondary_enabled else None)
        self.worker = StreamWorker(self.settings,
            stamp=lambda: self.get_clock().now().to_msg(),
            report=self.report)
        self.produced_total = 0
        self.published_total = 0
        self.publish_count = 0
        self.publish_latency_ms = 0.
        self.stats_time = None
        self.last_publish_time = 0.
        # Token bucket: a fixed minimum interval near the source period beats against the
        # frame clock and drops ~13% of a 30 fps stream. Tokens enforce the cap on average
        # (burst 2) while letting every frame through when source rate <= cap.
        self.tokens = 1.
        self.tokens_time = time.monotonic()
        self.secondary_tokens = 1.
        self.secondary_tokens_time = time.monotonic()
        self.secondary_publish_count = 0
        # Video reception/publication must not wait on a missing simulation clock.
        # Image stamps still use this node's selected ROS clock.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        # Tick 4x faster than the publish cap and rate-limit inside. A timer running at or
        # near the source rate beats against it and silently loses frames (measured: a 2x
        # tick lost ~26% of a 30 fps stream; 4x tracks it within 1%).
        self.timer = self.create_timer(0.25/self.settings.publish_fps, self.publish_frame, clock=steady)
        self.stats_timer = self.create_timer(self.settings.stats_period_s, self.report_stats,
                                             clock=steady)
        self.worker.start()
        self.get_logger().info(f'RTMP {self.settings.mode} -> {self.publisher.topic_name}, '
                               f'{self.settings.width}x{self.settings.height} '
                               f'{self.settings.encoding} publish_fps={self.settings.publish_fps:g} '
                               f'backend={self.worker.backend}')

    def report(self, level, text):
        # Humble caches severity per Python call site; keep these on separate lines.
        if level == 'info':
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)

    def publish_frame(self):
        now = time.monotonic()
        rate = getattr(self.settings, 'publish_fps', 30.)
        elapsed = now-getattr(self, 'tokens_time', now)
        self.tokens_time = now
        self.tokens = min(2., getattr(self, 'tokens', 1.)+elapsed*rate)
        if self.tokens < 1.:
            return
        frame = self.worker.frames.take()
        if frame is None:
            return
        # Take the secondary frame before publishing the primary: publishing 6 MB holds
        # the interpreter long enough for the decoder to replace an unread small frame.
        secondary_frame = None
        frames_secondary = getattr(self.worker, 'frames_secondary', None)
        if frames_secondary is not None:
            secondary_frame = frames_secondary.take()
        payload, stamp, received = frame
        if _frame_is_stale(self.settings, received, stamp):
            return
        # The small image goes out first: a full-resolution publish can block for seconds
        # under load (measured 1.69 s for 1920x1080), and a secondary frame checked after
        # that write is already older than max_frame_age_s, so every one was dropped
        # (measured secondary_fps=0.0 with secondary_frames=3).
        secondary = getattr(self, '_publish_secondary', None)
        if secondary is not None:  # SimpleNamespace test doubles do not carry the method
            secondary(secondary_frame)
        message = Image(width=self.settings.width, height=self.settings.height,
                        encoding=self.settings.encoding, is_bigendian=0,
                        step=self.settings.width*3, data=payload)
        message.header.frame_id = self.frame_id
        message.header.stamp = stamp
        started = time.monotonic()
        self.publisher.publish(message)
        self.last_publish_time = time.monotonic()
        self.tokens -= 1.
        self.publish_latency_ms = (self.last_publish_time-started)*1000.
        self.publish_count = getattr(self, 'publish_count', 0)+1
        self.published_total = getattr(self, 'published_total', 0)+1

    def _publish_secondary(self, frame):
        """Small-image output for light consumers (Python YOLO); same decoded frame."""
        publisher = getattr(self, 'secondary_publisher', None)
        frames = getattr(self.worker, 'frames_secondary', None)
        if publisher is None or frames is None:
            return
        now = time.monotonic()
        rate = getattr(self.settings, 'publish_fps', 30.)
        elapsed = now-getattr(self, 'secondary_tokens_time', now)
        self.secondary_tokens_time = now
        self.secondary_tokens = min(2., getattr(self, 'secondary_tokens', 1.)+elapsed*rate)
        if self.secondary_tokens < 1.:
            return
        if frame is None:
            return
        payload, stamp, received = frame
        if _frame_is_stale(self.settings, received, stamp):
            return
        message = Image(width=self.settings.secondary_width, height=self.settings.secondary_height,
                        encoding=self.settings.encoding, is_bigendian=0,
                        step=self.settings.secondary_width*3, data=payload)
        message.header.frame_id = self.frame_id
        message.header.stamp = stamp
        publisher.publish(message)
        self.secondary_tokens -= 1.
        self.secondary_publish_count = getattr(self, 'secondary_publish_count', 0)+1

    def report_stats(self):
        snap = self.worker.snapshot()
        now = time.monotonic()
        span = (now-self.stats_time) if self.stats_time is not None else self.settings.stats_period_s
        self.stats_time = now
        self.produced_total = getattr(self, 'produced_total', 0)+snap['frames']
        published = getattr(self, 'publish_count', 0)
        self.publish_count = 0
        secondary_published = getattr(self, 'secondary_publish_count', 0)
        self.secondary_publish_count = 0
        fps_in = snap['frames']/span if span > 1e-3 else 0.
        fps_pub = published/span if span > 1e-3 else 0.
        dropped = max(0, snap['frames']-published)
        extra = ''
        if self.secondary_publisher is not None:
            extra = (' secondary_fps=%.1f secondary_frames=%d secondary_dropped=%d'
                     ' secondary_size=%dx%d'
                     % (secondary_published/span if span > 1e-3 else 0., snap['secondary'],
                        max(0, snap['secondary']-secondary_published),
                        self.settings.secondary_width, self.settings.secondary_height))
        self.get_logger().info(
            '[RTMP统计] input_fps=%.1f decode_fps=%.1f publish_fps=%.1f dropped_frames=%d '
            'overwritten_frames=%d width=%d height=%d decode_latency_ms=%.1f '
            'publish_latency_ms=%.1f backend=%s%s'
            % (fps_in, fps_in, fps_pub, dropped, snap['replaced'],
               self.settings.width, self.settings.height, snap['decode_latency_ms'],
               getattr(self, 'publish_latency_ms', 0.), snap['backend'], extra))

    def destroy_node(self):
        self.timer.cancel()
        self.stats_timer.cancel()
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RtmpImageBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, RuntimeError) as exc:
        rclpy.logging.get_logger('rtmp_image_bridge').fatal(str(exc))
        raise SystemExit(2)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
