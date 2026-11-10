"""CPU-only tests; integration uses a fake model but real DDS messages."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
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
        [0, 0, 2], [.93, .8, .5]), header, 640, 480, ['person'])
    assert output.header == header
    assert (output.image_width, output.image_height) == (640, 480)
    assert len(output.detections) == 1
    box = output.detections[0]
    assert (box.x_min, box.y_min, box.x_max, box.y_max) == (0, 0, 639, 479)
    assert (box.center_x, box.center_y, box.width, box.height) == (319.5, 239.5, 639., 479.)
    assert box.class_name == 'person'


@pytest.fixture
def detector(tmp_path, monkeypatch, request):
    engine = tmp_path / 'test.engine'
    engine.touch()
    monkeypatch.setattr(YoloDetector, '_load_model', lambda self: object())
    monkeypatch.setattr(YoloDetector, '_predict', lambda self, model, image: [result()])
    import json
    overrides = {'camera_info_source': 'topic', **getattr(request, 'param', {})}
    args = ['--ros-args', '-r', '__ns:=/robot_2',
            '-p', f'model_path:={engine}', '-p', f'targets_file:={tmp_path / "targets.yaml"}']
    for name, value in overrides.items():
        args.extend(['-p', f'{name}:={json.dumps(value)}'])
    rclpy.init(args=args)
    node = YoloDetector()
    from yolo_detector.targets import write_targets
    write_targets(node.targets_file, ['person', 'car'])
    node._reload_targets()
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
    executor = MultiThreadedExecutor(num_threads=2)
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


def test_empty_targets_skip_inference_and_resume_without_restart(detector, monkeypatch):
    from unittest.mock import Mock
    from yolo_detector.targets import write_targets
    prediction = Mock(return_value=[result()])
    monkeypatch.setattr(detector, '_predict', prediction)
    write_targets(detector.targets_file, [])
    detector._reload_targets()
    detector.image_callback(image_message())
    assert detector.frames._frame is None
    prediction.assert_not_called()
    assert detector.worker.is_alive()
    assert not detection_array(result([(1,1,20,20)],[0],[.9]),
                               Header(),640,480,[]).detections
    write_targets(detector.targets_file, ['person'])
    detector._reload_targets()
    detector.image_callback(image_message())
    deadline = time.monotonic() + 2
    while not prediction.called and time.monotonic() < deadline:
        time.sleep(.01)
    prediction.assert_called_once()


# 3D projection uses a fake inference model and measured cloud messages on an
# isolated test domain; no robot navigation/action clients are created.
def projection_input(detector, points=None):
    from copy import deepcopy
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs_py import point_cloud2
    from geometry_msgs.msg import TransformStamped
    header = Header(frame_id='robot_2/camera_optical')
    header.stamp = detector.get_clock().now().to_msg()
    detector._camera_callback(CameraInfo(header=header, width=640, height=480,
        k=[100., 0., 320., 0., 100., 240., 0., 0., 1.], distortion_model='plumb_bob'))
    if points is None:
        points = [[0., 0., 5.], [.1, 0., 5.], [-.1, 0., 5.], [4., 0., 5.]]
    detector._clouds.append(point_cloud2.create_cloud_xyz32(deepcopy(header), points))
    transform = TransformStamped(header=Header(frame_id='robot_2/map'), child_frame_id=header.frame_id)
    transform.transform.rotation.w = 1.
    transform.transform.translation.x = 10.
    transform.transform.translation.y = 20.
    detector._tf.set_transform_static(transform, 'test')
    return detection_array(result([(310, 230, 330, 250)], [0], [.9]), header, 640, 480, ['person'])


@pytest.mark.parametrize('detector', [
    {'camera_info_source': 'topic'},
    {'camera_info_source': 'yaml', 'camera_fx': 100.0, 'camera_fy': 100.0,
     'camera_cx': 320.0, 'camera_cy': 240.0},
], indirect=True)
def test_3d_marker_map_coordinates_and_dds(detector, monkeypatch):
    from visualization_msgs.msg import MarkerArray
    monkeypatch.setattr(detector, '_predict', lambda self_model, image: [
        result([(310, 230, 330, 250)], [0], [.9])])
    peer = Node('marker_test_peer')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(detector)
    executor.add_node(peer)
    received = []
    subscription = peer.create_subscription(MarkerArray, '/robot_2/targets/markers', received.append, 10)
    publisher = peer.create_publisher(Image, '/robot_2/front_camera/image_raw', qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 8.
        while not received and time.monotonic() < deadline:
            message = projection_input(detector)
            image = image_message()
            image.header = message.header
            publisher.publish(image)
            executor.spin_once(timeout_sec=.05)
        assert received, 'No 3D MarkerArray delivered over DDS'
        if detector.settings['camera_info_source'] == 'yaml':
            assert not hasattr(detector, '_camera_subscription')
            assert detector._camera_info is None  # Callback cannot overwrite fixed calibration.
        marker = received[0].markers[0]
        assert marker.header.frame_id == 'robot_2/map'
        assert marker.pose.position.x == pytest.approx(10.)
        assert marker.pose.position.y == pytest.approx(20.)
        assert marker.pose.position.z == pytest.approx(5.)
        assert marker.pose.orientation.w == 1.
        assert marker.text == 'person #1'
        assert marker.lifetime.sec == 3
        assert all(m.markers[0].id == marker.id for m in received)
    finally:
        executor.remove_node(peer)
        executor.remove_node(detector)
        executor.shutdown()
        peer.destroy_node()


@pytest.mark.parametrize('failure', ['missing_camera', 'missing_tf', 'bad_frame', 'bad_size',
                                     'unsynced_cloud', 'stale_detection', 'sparse_cloud', 'empty_targets'])
def test_3d_missing_inputs_do_not_publish_or_crash(detector, failure):
    from tf2_ros import Buffer
    message = projection_input(detector, points=[[0., 0., 5.]] if failure == 'sparse_cloud' else None)
    published = []
    detector.marker_publisher = SimpleNamespace(publish=published.append)
    if failure == 'missing_camera':
        detector._camera_info = None
    elif failure == 'missing_tf':
        detector._tf = Buffer()
    elif failure == 'bad_frame':
        message.header.frame_id = 'wrong_camera'
    elif failure == 'bad_size':
        message.image_width = 320
    elif failure == 'unsynced_cloud':
        detector._clouds[0].header.stamp.sec -= 1
    elif failure == 'stale_detection':
        message.header.stamp.sec -= 10
    elif failure == 'empty_targets':
        detector._targets = ()
    detector._pending_3d.append(message)
    detector._publish_target_markers()
    assert not published
    assert detector._failure is None


def test_debug_health_severity_transition_does_not_crash(detector):
    detector.debug_publisher = SimpleNamespace()
    detector._log_debug_health()  # warning: no camera input
    detector._record_diagnostic(last_input=time.monotonic(), model_ready=False)
    detector._log_debug_health()  # info: model warmup
    detector.debug_publisher = None


@pytest.mark.parametrize('overrides', [
    {'camera_width': 0}, {'camera_height': 0}, {'camera_fx': 0.0},
    {'camera_fy': float('nan')}, {'camera_cx': float('inf')},
    {'camera_distortion_model': 'unknown'},
    {'camera_distortion_coefficients': [float('nan')] * 5},
    {'camera_distortion_model': 'equidistant', 'camera_distortion_coefficients': [0.0] * 5},
])
def test_yaml_intrinsics_reject_invalid_geometry(detector, overrides):
    from yolo_detector.yolo_node import camera_info_from_settings
    settings = {**detector.settings, 'camera_fx': 100.0, 'camera_fy': 100.0,
                'camera_cx': 320.0, 'camera_cy': 240.0, **overrides}
    with pytest.raises(ValueError):
        camera_info_from_settings(settings)


@pytest.mark.parametrize('detector', [{'camera_info_source': 'yaml'}], indirect=True)
def test_invalid_yaml_intrinsics_keep_detector_alive(detector):
    message = projection_input(detector)
    published = []
    detector.marker_publisher = SimpleNamespace(publish=published.append)
    detector._pending_3d.append(message)
    detector._publish_target_markers()
    assert not published
    assert detector._failure is None
    assert 'positive fx and fy' in detector._calibration_error


@pytest.mark.parametrize('detector', [{
    'camera_info_source': 'yaml', 'camera_fx': 100.0, 'camera_fy': 100.0,
    'camera_cx': 320.0, 'camera_cy': 240.0, 'camera_optical_frame': 'wrong_camera',
}], indirect=True)
def test_yaml_explicit_optical_frame_must_match_image(detector):
    message = projection_input(detector)
    published = []
    detector.marker_publisher = SimpleNamespace(publish=published.append)
    detector._pending_3d.append(message)
    detector._publish_target_markers()
    assert not published


@pytest.mark.parametrize('detector', [{
    'camera_info_source': 'yaml', 'camera_scale_to_image': True,
    'camera_width': 1920, 'camera_height': 1080,
    'camera_fx': 960, 'camera_fy': 960, 'camera_cx': 960, 'camera_cy': 540,
}], indirect=True)
def test_integer_yaml_intrinsics_resize_to_decoded_image(detector):
    from yolo_detector.projection import project
    header = Header(frame_id='robot_2/camera_optical')
    camera = detector._projection_camera(header, 640, 480)
    assert (camera.width, camera.height) == (640, 480)
    pixels = project(np.array([[0., 0., 5.], [1., 1., 5.]]), camera)
    assert pixels[0] == pytest.approx([320., 240.])
    assert pixels[1] == pytest.approx([384., 325.3333333333333])
    # Scaling is per frame; it never mutates the source calibration.
    original = detector._projection_camera(header, 1920, 1080)
    assert original.k[0] == 960. and original.k[4] == 960.
    assert detector._yaml_camera_info.width == 1920


def test_missing_selection_file_pauses_then_recovers(detector):
    from yolo_detector.targets import write_targets
    detector.targets_file.unlink()
    detector._reload_targets()
    assert detector._targets == ()
    write_targets(detector.targets_file, ['person'])
    detector._reload_targets()
    assert detector._targets == ('person',)


def test_tf_progresses_during_busy_image_callback(detector, monkeypatch):
    """A slow 1080p callback must not leave TF queued behind image processing."""
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from rclpy.time import Time

    entered, release = threading.Event(), threading.Event()

    def busy_image(message):
        entered.set()
        release.wait(timeout=8.)

    monkeypatch.setattr(detector, '_receive_image', busy_image)
    peer = Node('tf_backlog_test_peer')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(detector)
    executor.add_node(peer)
    images = peer.create_publisher(Image, '/robot_2/front_camera/image_raw', qos_profile_sensor_data)
    transforms = peer.create_publisher(TFMessage, '/tf', 10)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        deadline = time.monotonic()+5.
        while not entered.is_set() and time.monotonic() < deadline:
            images.publish(image_message())
            entered.wait(timeout=.05)
        assert entered.is_set(), 'Image callback did not start over DDS'
        transform = TransformStamped()
        transform.header.frame_id = 'robot_2/map'
        transform.child_frame_id = 'robot_2/backlog_probe'
        transform.header.stamp = detector.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        deadline = time.monotonic()+4.
        while time.monotonic() < deadline:
            transforms.publish(TFMessage(transforms=[transform]))
            if detector._tf.can_transform(transform.header.frame_id,
                                          transform.child_frame_id, Time()):
                break
            time.sleep(.02)
        assert not release.is_set()
        assert detector._tf.can_transform(transform.header.frame_id,
                                         transform.child_frame_id, Time()), (
            'TF stalled behind the busy image callback')
    finally:
        release.set()
        executor.shutdown(timeout_sec=4.)
        spinner.join(timeout=4.)
        peer.destroy_node()
    assert not spinner.is_alive()


def motion_clouds(detector, offsets=(-.4, -.2, 0.)):
    """One measured point per scan, with translating and rotating lidar poses."""
    from copy import deepcopy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs_py import point_cloud2
    from rclpy.time import Time
    now = detector.get_clock().now().nanoseconds
    header = Header(frame_id='robot_2/motion_camera')
    header.stamp = Time(nanoseconds=now).to_msg()
    detector._camera_callback(CameraInfo(header=header, width=640, height=480,
        k=[100., 0., 320., 0., 100., 240., 0., 0., 1.], distortion_model='plumb_bob'))
    camera = TransformStamped(header=Header(frame_id='robot_2/map'), child_frame_id=header.frame_id)
    camera.transform.translation.x = 10.
    camera.transform.translation.y = 20.
    camera.transform.rotation.w = 1.
    detector._tf.set_transform_static(camera, 'test')
    target = np.array([10., 20., 5.])
    for index, offset in enumerate(offsets):
        stamp = Time(nanoseconds=now+round(offset*1e9)).to_msg()
        angle = index*np.pi/2.
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                             [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        translation = np.array([float(index), 0., 0.])
        pose = TransformStamped(header=Header(frame_id='robot_2/map', stamp=stamp),
                                child_frame_id='robot_2/motion_lidar')
        pose.transform.translation.x = float(index)
        pose.transform.rotation.z = float(np.sin(angle/2.))
        pose.transform.rotation.w = float(np.cos(angle/2.))
        detector._tf.set_transform(pose, 'test')
        points = [(target-translation) @ rotation]
        cloud = point_cloud2.create_cloud_xyz32(
            Header(frame_id=pose.child_frame_id, stamp=stamp), points)
        detector._cloud_callback(cloud)
    return detection_array(result([(310, 230, 330, 250)], [0], [.9]),
                           deepcopy(header), 640, 480, ['person'])


def test_accumulation_recovers_sparse_scans_with_motion_compensation(detector):
    message = motion_clouds(detector)
    published = []
    detector.marker_publisher = SimpleNamespace(publish=published.append)
    detector._pending_3d.append(message)
    detector._publish_target_markers()
    assert len(published) == 1, detector._projection_stats
    position = published[0].markers[0].pose.position
    assert [position.x, position.y, position.z] == pytest.approx([10., 20., 5.], abs=1e-5)
    assert detector._projection_stats['accumulated_clouds'] == 3
    assert detector._projection_stats['accumulated_points'] == 3
    assert published[0].markers[0].header.stamp == message.header.stamp


def test_zero_accumulation_restores_single_sparse_scan_rejection(detector):
    detector.settings['cloud_accumulation_s'] = 0.
    message = motion_clouds(detector)
    published = []
    detector.marker_publisher = SimpleNamespace(publish=published.append)
    detector._pending_3d.append(message)
    detector._publish_target_markers()
    assert not published
    assert detector._projection_stats['accumulated_clouds'] == 1


def test_accumulation_excludes_old_future_and_replayed_packets(detector):
    from copy import deepcopy
    message = motion_clouds(detector, offsets=(-1., -.2, 0., .3))
    # Anchor is the scan at image time; the old and future scans are excluded.
    detector._cloud_callback(deepcopy(detector._clouds[2]))
    published = []
    detector.marker_publisher = SimpleNamespace(publish=published.append)
    detector._pending_3d.append(message)
    detector._publish_target_markers()
    assert not published  # Only two unique in-window measurements.
    assert detector._projection_stats['accumulated_clouds'] == 2


def test_accumulation_skips_frames_without_tf(detector):
    from copy import deepcopy
    message = motion_clouds(detector, offsets=(-.4, -.2, 0.))
    cloud = deepcopy(detector._clouds[0])
    cloud.header.frame_id = 'robot_2/no_tf'
    detector._cloud_callback(cloud)
    points = detector._accumulated_map_points(message.header)
    assert points.shape == (3, 3)
    assert points == pytest.approx(np.tile([10., 20., 5.], (3, 1)), abs=1e-5)
    assert detector._projection_stats['tf_skipped_clouds'] == 1


def test_old_dense_history_does_not_replace_fresh_lidar(detector):
    message = motion_clouds(detector, offsets=(-.4, -.3, -.2))
    with pytest.raises(ValueError, match='no time-matched lidar cloud'):
        detector._accumulated_map_points(message.header)


def test_cloud_history_prunes_and_resets_on_clock_jump(detector):
    from sensor_msgs_py import point_cloud2
    from rclpy.time import Time
    stamp = detector.get_clock().now().nanoseconds
    def send(offset):
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='lidar',
            stamp=Time(nanoseconds=stamp+round(offset*1e9)).to_msg()), [[0., 0., 5.]])
        detector._cloud_callback(cloud)
    for offset in [-3., -2., -1., 0.]:send(offset)
    assert len(detector._clouds) == 3
    send(-10.)
    assert len(detector._clouds) == 1


@pytest.mark.parametrize('value', [-.1, float('nan'), float('inf'), 6., True])
def test_invalid_accumulation_window_rejected(detector, value):
    detector.settings['cloud_accumulation_s'] = value
    with pytest.raises(ValueError, match='cloud_accumulation_s'):
        detector._validate_settings()
