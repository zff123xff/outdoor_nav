"""CPU-only tests; integration uses a fake model but real DDS messages."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from yolo_interfaces.msg import DetectionArray

from yolo_detector.yolo_node import LatestFrame, YoloDetector, detection_array


class Tensor:
    def __init__(self, data):
        self.data = np.asarray(data)

    def cpu(self):
        return self

    def numpy(self):
        return self.data


def result(boxes=(), classes=(), scores=()):
    return SimpleNamespace(
        boxes=SimpleNamespace(xyxy=Tensor(boxes), cls=Tensor(classes), conf=Tensor(scores)),
        names={0: 'person', 2: 'car'})


def test_latest_frame_replaces_pending_and_stop_wakes_waiter():
    frames = LatestFrame()
    frames.put('old')
    frames.put('new')
    assert frames.take() == 'new'
    taken = []
    waiter = threading.Thread(target=lambda: taken.append(frames.take()))
    waiter.start()
    frames.stop()
    waiter.join(timeout=1)
    assert not waiter.is_alive()
    assert taken == [None]
    frames.put('ignored')
    assert frames.take() is None


def test_clipping_header_and_invalid_boxes():
    header = Header(frame_id='front_camera')
    header.stamp.sec = 42
    output = detection_array(result(
        [(-10, -5, 700, 500), (30, 20, 10, 5), (float('nan'), 0, 5, 5)],
        [0, 0, 2], [.93, .8, .5]), header, 640, 480)
    assert output.header == header
    assert (output.image_width, output.image_height) == (640, 480)
    assert len(output.detections) == 1
    box = output.detections[0]
    assert (box.x_min, box.y_min, box.x_max, box.y_max) == (0, 0, 639, 479)
    assert (box.center_x, box.center_y, box.width, box.height) == (319.5, 239.5, 639., 479.)
    assert box.class_name == 'person'


@pytest.fixture
def detector(tmp_path, monkeypatch):
    engine = tmp_path / 'test.engine'
    engine.touch()
    monkeypatch.setattr(YoloDetector, '_load_model', lambda self: object())
    monkeypatch.setattr(YoloDetector, '_predict', lambda self, model, image: [result()])
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot_2',
                     '-p', f'model_path:={engine}', '-p', f'targets_file:={tmp_path / "targets.yaml"}'])
    node = YoloDetector()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def image_message(encoding='rgb8'):
    image = Image(width=640, height=480, encoding=encoding, step=640 * 3)
    image.header.frame_id = 'front_camera'
    image.header.stamp.sec = 123
    image.data = np.tile(np.array([255, 0, 0], dtype=np.uint8), 640 * 480).tobytes()
    return image


def test_input_validation_and_rgb_conversion(detector):
    detector.frames.stop()
    detector.worker.join(timeout=2)
    detector.frames = LatestFrame()
    invalid = image_message()
    invalid.width = 0
    detector.image_callback(invalid)
    invalid.width = 320
    detector.image_callback(invalid)
    invalid = image_message('unsupported')
    detector.image_callback(invalid)
    assert detector.frames._frame is None
    image = image_message()
    detector.image_callback(image)
    pixels, header = detector.frames.take()
    assert pixels[0, 0].tolist() == [0, 0, 255]
    assert header == image.header
    image.header.stamp.sec = 999
    assert header.stamp.sec == 123
    # A convertible encoding mismatch must still be accepted.
    detector.image_callback(image_message('bgr8'))
    pixels, _ = detector.frames.take()
    assert pixels[0, 0].tolist() == [255, 0, 0]


def test_real_dds_empty_frame_and_namespace(detector):
    peer = Node('yolo_test_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(peer)
    received = []
    subscription = peer.create_subscription(
        DetectionArray, '/robot_2/yolo/detections', received.append, 10)
    publisher = peer.create_publisher(
        Image, '/robot_2/front_camera/image_raw', qos_profile_sensor_data)
    assert detector.get_fully_qualified_name() == '/robot_2/yolo_detector'
    assert detector.subscription.qos_profile.depth == 1
    assert detector.publisher.qos_profile.depth == 10
    message = image_message()
    try:
        deadline = time.monotonic() + 10
        while not received and time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=.1)
        assert received, 'No DetectionArray received through DDS within 10 seconds'
        output = received[0]
        assert output.header == message.header
        assert (output.image_width, output.image_height) == (640, 480)
        assert list(output.detections) == []
    finally:
        executor.remove_node(peer)
        executor.remove_node(detector)
        executor.shutdown()
        peer.destroy_node()


def test_worker_failure_is_visible(detector, monkeypatch):
    def fail(model, image):
        raise RuntimeError('simulated TensorRT failure')
    monkeypatch.setattr(detector, '_predict', fail)
    detector.image_callback(image_message())
    deadline = time.monotonic() + 3
    while detector._failure is None and time.monotonic() < deadline:
        time.sleep(.01)
    with pytest.raises(RuntimeError, match='simulated TensorRT failure'):
        detector._raise_worker_failure()


@pytest.mark.parametrize('setting,value', [
    ('image_topic', '/absolute'), ('debug_image_topic', '~private'),
    ('confidence_threshold', float('nan')), ('iou_threshold', 1.1),
    ('model_input_width', 641), ('max_detections', 0),
])
def test_invalid_settings(detector, setting, value):
    detector.settings[setting] = value
    with pytest.raises(ValueError):
        detector._validate_settings()


def test_target_file_reload_filters_without_restarting(detector):
    from yolo_detector.targets import write_targets
    write_targets(detector.targets_file,['car'])
    detector._reload_targets()
    output=detection_array(result([(1,1,20,20),(25,25,40,40)],[0,2],[.9,.9]),
        Header(),640,480,detector._targets)
    assert [box.class_name for box in output.detections]==['car']
    detector.targets_file.write_text('targets: broken')
    detector._reload_targets()
    assert detector._targets==('car',)
