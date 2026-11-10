#!/usr/bin/env python3
"""Real DDS image/debug transport with fake predictions, no GPU required."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from yolo_detector.targets import write_targets
from yolo_detector.yolo_node import YoloDetector
from yolo_interfaces.msg import DetectionArray


class Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)
    def cpu(self):
        return self
    def numpy(self):
        return self.values


def main():
    with TemporaryDirectory(prefix='yolo_debug_') as directory:
        engine, targets = Path(directory)/'model.engine', Path(directory)/'targets.yaml'
        engine.touch()
        write_targets(targets, ['person'])
        rclpy.init(args=['--ros-args', '-r', '__ns:=/debug_fixture',
                        '-p', 'publish_debug_image:=true', '-p', f'model_path:={engine}',
                        '-p', f'targets_file:={targets}'])
        prediction = SimpleNamespace(names={0: 'person'}, boxes=SimpleNamespace(
            xyxy=Tensor([[10, 20, 50, 70]]), cls=Tensor([0]), conf=Tensor([.9])))
        with patch.object(YoloDetector, '_load_model', return_value=object()), \
                patch.object(YoloDetector, '_predict', side_effect=lambda *args: [prediction]):
            node, detector = Node('debug_image_fixture'), YoloDetector()
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(detector)
            images, detections = [], []
            node.create_subscription(Image, 'yolo/debug_image', images.append, qos_profile_sensor_data)
            node.create_subscription(DetectionArray, 'yolo/detections', detections.append, 10)
            publisher = node.create_publisher(Image, 'front_camera/image_raw', qos_profile_sensor_data)
            pixels = np.empty((480, 640, 3), dtype=np.uint8)
            pixels[:] = [255, 0, 0]  # RGB red; expected BGR red [0,0,255].
            image = Image(width=640, height=480, encoding='rgb8', step=640*3, data=pixels.tobytes())
            image.header = Header(frame_id='camera_optical')
            def wait(predicate, label, timeout=8.):
                end = time.monotonic()+timeout
                while not predicate():
                    assert time.monotonic() < end, label
                    executor.spin_once(timeout_sec=.02)
            def send_until(predicate, label):
                end = time.monotonic()+8.
                while not predicate():
                    assert time.monotonic() < end, label
                    image.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(image)
                    for _ in range(5):
                        executor.spin_once(timeout_sec=.02)
            try:
                wait(lambda: publisher.get_subscription_count() and
                     detector.debug_publisher.get_subscription_count(), 'DDS matching')
                send_until(lambda: images and detections, 'annotated image and detection receipt')
                output = images[-1]
                data = np.asarray(output.data, dtype=np.uint8).reshape((480, 640, 3))
                assert output.encoding == 'bgr8' and output.step == 640*3
                assert output.header.frame_id == 'camera_optical'
                assert output.header == detections[-1].header
                assert data[20, 10].tolist() == [0, 255, 0], 'bounding box not drawn'
                assert data[200, 200].tolist() == [0, 0, 255], 'RGB/BGR conversion wrong'
                print('PASS: actual DDS debug Image, header, dimensions, encoding, colour conversion and green box')
                prediction = SimpleNamespace(names={0: 'person'}, boxes=SimpleNamespace(
                    xyxy=Tensor(np.empty((0, 4))), cls=Tensor([]), conf=Tensor([])))
                images.clear(); detections.clear()
                send_until(lambda: detections and not detections[-1].detections and images and
                           images[-1].header == detections[-1].header, 'no detections still publish image')
                assert np.asarray(images[-1].data, dtype=np.uint8).reshape((480, 640, 3))[20, 10].tolist() == [0,0,255]
                print('PASS: active classes with zero detections still publish unannotated image')
                write_targets(targets, [])
                detector._reload_targets()
                end = time.monotonic()+.3
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.02)
                images.clear(); detections.clear()
                for _ in range(15):
                    publisher.publish(image)
                    executor.spin_once(timeout_sec=.02)
                assert not images and not detections
                print('PASS: empty target classes enter standby; neither detections nor debug images are published')
            finally:
                detector.destroy_node()
                node.destroy_node()
                executor.shutdown()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
