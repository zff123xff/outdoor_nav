"""One pending frame, one inference worker, original-image coordinates."""

from copy import deepcopy
import math
from pathlib import Path
import threading

import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from rclpy.clock import Clock, ClockType
from .targets import targets_path, read_targets, write_targets, matches
from yolo_interfaces.msg import Detection, DetectionArray


class LatestFrame:
    """A single replaceable slot; stop wakes an idle consumer."""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None
        self._stopped = False

    def put(self, frame):
        with self._condition:
            if not self._stopped:
                self._frame = frame
                self._condition.notify()

    def take(self):
        with self._condition:
            self._condition.wait_for(lambda: self._stopped or self._frame is not None)
            if self._stopped:
                return None
            frame, self._frame = self._frame, None
            return frame

    def stop(self):
        with self._condition:
            self._stopped = True
            self._frame = None
            self._condition.notify_all()


def detection_array(result, header, width, height, targets=()):
    message = DetectionArray(header=header, image_width=width, image_height=height)
    if result.boxes is None:
        raise ValueError('Model did not return detection boxes')
    # One GPU-to-CPU transfer per array, rather than synchronizing for each box.
    coordinates = result.boxes.xyxy.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy()
    scores = result.boxes.conf.cpu().numpy()
    for xyxy, class_value, score in zip(coordinates, classes, scores):
        if not all(math.isfinite(float(v)) for v in (*xyxy, class_value, score)):
            continue
        x1, y1, x2, y2 = [int(v) for v in xyxy]
        x1, x2 = [max(0, min(v, width - 1)) for v in (x1, x2)]
        y1, y2 = [max(0, min(v, height - 1)) for v in (y1, y2)]
        if x2 <= x1 or y2 <= y1:
            continue
        class_id = int(class_value)
        names = result.names
        name = names.get(class_id, str(class_id)) if isinstance(names, dict) else names[class_id]
        if not matches(targets, str(name), class_id):
            continue
        message.detections.append(Detection(
            class_id=class_id, class_name=str(name), confidence=float(score),
            x_min=x1, y_min=y1, x_max=x2, y_max=y2,
            center_x=(x1 + x2) / 2.0, center_y=(y1 + y2) / 2.0,
            width=float(x2 - x1), height=float(y2 - y1)))
    return message


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        defaults = {
            'image_topic': 'front_camera/image_raw',
            'detection_topic': 'yolo/detections',
            'debug_image_topic': 'yolo/debug_image',
            'expected_width': 640, 'expected_height': 480,
            'expected_encoding': 'rgb8', 'strict_image_size': True,
            'model_path': '/home/nhy/models/best.engine',
            'model_input_width': 640, 'model_input_height': 480,
            'confidence_threshold': 0.4, 'iou_threshold': 0.45,
            'max_detections': 100, 'publish_debug_image': False,
            'targets_file': '',
        }
        self.settings = {
            name: self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
            for name, default in defaults.items()
        }
        self._validate_settings()
        self.targets_file = targets_path(self.get_namespace(), self.settings['targets_file'])
        if not self.targets_file.exists():
            write_targets(self.targets_file, [])
        self._targets_lock = threading.Lock()
        self._targets = tuple(read_targets(self.targets_file))
        self._target_timer = self.create_timer(1., self._reload_targets,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.bridge = CvBridge()
        self.frames = LatestFrame()
        self._stopping = threading.Event()
        self._failure = None
        self._failure_guard = self.create_guard_condition(self._raise_worker_failure)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.publisher = self.create_publisher(
            DetectionArray, self.settings['detection_topic'], 10)
        self.debug_publisher = (
            self.create_publisher(Image, self.settings['debug_image_topic'], qos)
            if self.settings['publish_debug_image'] else None)
        self.subscription = self.create_subscription(
            Image, self.settings['image_topic'], self.image_callback, qos)
        self.worker = threading.Thread(target=self._run_worker, name='yolo-inference')
        self.worker.start()

    def _reload_targets(self):
        try:
            values = tuple(read_targets(self.targets_file))
        except Exception as exc:
            self.get_logger().error(f'Target file not updated: {exc}', throttle_duration_sec=5.)
            return
        with self._targets_lock:
            self._targets = values

    def _validate_settings(self):
        p = self.settings
        for name in ('image_topic', 'detection_topic', 'debug_image_topic'):
            topic = p[name]
            if not topic or topic.startswith(('/', '~')):
                raise ValueError(f'{name} must be a relative topic: {topic!r}')
        if len({p[k] for k in ('image_topic', 'detection_topic', 'debug_image_topic')}) != 3:
            raise ValueError('Image, detection and debug topics must be distinct')
        for name in ('expected_width', 'expected_height', 'model_input_width',
                     'model_input_height', 'max_detections'):
            if p[name] <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('model_input_width', 'model_input_height'):
            if p[name] % 32:
                raise ValueError(f'{name} must be a multiple of 32')
        for name in ('confidence_threshold', 'iou_threshold'):
            if not math.isfinite(p[name]) or not 0 <= p[name] <= 1:
                raise ValueError(f'{name} must be between 0 and 1')
        path = Path(p['model_path']).expanduser()
        if path.suffix != '.engine' or not path.is_file():
            raise ValueError(f'model_path must point to an existing .engine file: {path}')
        p['model_path'] = str(path)

    def image_callback(self, msg):
        p = self.settings
        if msg.width <= 0 or msg.height <= 0:
            self.get_logger().error('Invalid zero-sized image', throttle_duration_sec=5.0)
            return
        if p['strict_image_size'] and (msg.width, msg.height) != (
                p['expected_width'], p['expected_height']):
            self.get_logger().error(
                f'Invalid image size {msg.width}x{msg.height}; expected '
                f'{p["expected_width"]}x{p["expected_height"]}', throttle_duration_sec=5.0)
            return
        if msg.encoding != p['expected_encoding']:
            self.get_logger().warning(
                f'Encoding {msg.encoding!r}; expected {p["expected_encoding"]!r}; '
                'attempting bgr8 conversion', throttle_duration_sec=5.0)
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if image.shape != (msg.height, msg.width, 3) or image.dtype != np.uint8:
                raise ValueError('Converted image must be HxWx3 uint8')
        except Exception as error:
            self.get_logger().error(f'Image conversion failed: {error}', throttle_duration_sec=5.0)
            return
        self.frames.put((image, deepcopy(msg.header)))

    def _load_model(self):
        # All TensorRT/CUDA work, including warmup, stays on this worker.
        from ultralytics import YOLO
        model = YOLO(self.settings['model_path'], task='detect')
        warmup = np.zeros((self.settings['model_input_height'],
                           self.settings['model_input_width'], 3), dtype=np.uint8)
        self._predict(model, warmup)
        backend = model.predictor.model
        shape = tuple(backend.bindings['images'].shape)
        expected = (1, 3, *warmup.shape[:2])
        if backend.dynamic or shape != expected:
            raise ValueError(f'Expected static engine {expected}; got {shape}, '
                             f'dynamic={backend.dynamic}')
        self.get_logger().info(
            f'TensorRT ready: input={shape}, fp16_io={backend.fp16}; waiting for images')
        return model

    def _predict(self, model, image):
        p = self.settings
        return model.predict(
            source=image, imgsz=(p['model_input_height'], p['model_input_width']),
            rect=False, batch=1, device=0, conf=p['confidence_threshold'],
            iou=p['iou_threshold'], max_det=p['max_detections'], verbose=False)

    def _run_worker(self):
        try:
            model = self._load_model()
            while not self._stopping.is_set():
                frame = self.frames.take()
                if frame is None:
                    break
                image, header = frame
                result = self._predict(model, image)[0]
                with self._targets_lock:
                    targets = self._targets
                message = detection_array(result, header, image.shape[1], image.shape[0], targets)
                if self._stopping.is_set() or not self.context.ok():
                    break
                self.publisher.publish(message)
                if self.debug_publisher is not None:
                    import cv2
                    annotated = image.copy()
                    for box in message.detections:
                        cv2.rectangle(annotated, (box.x_min, box.y_min), (box.x_max, box.y_max), (0, 255, 0), 2)
                        cv2.putText(annotated, box.class_name, (box.x_min, max(15, box.y_min)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
                    debug = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    debug.header = header
                    self.debug_publisher.publish(debug)
        except Exception as error:
            if not self._stopping.is_set() and self.context.ok():
                self._failure = error
                self._failure_guard.trigger()

    def _raise_worker_failure(self):
        raise RuntimeError(f'YOLO worker failed: {self._failure}') from self._failure

    def destroy_node(self):
        self._stopping.set()
        self.frames.stop()
        # Never destroy publishers while a worker can still access them.
        self.worker.join()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, ROSInterruptException):
        pass
    except Exception as error:
        rclpy.logging.get_logger('yolo_detector').fatal(str(error))
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
