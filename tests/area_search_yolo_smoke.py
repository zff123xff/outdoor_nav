#!/usr/bin/env python3
"""Real DDS Task -> class file -> YOLO reload/output, with a fake model and no motion."""
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import Image
from task_interfaces.action import AreaSearch
from task_interfaces.msg import Task
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.task_schema import to_message
from yolo_detector.targets import read_targets
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
    with TemporaryDirectory(prefix='area_yolo_') as directory:
        classes = Path(directory)/'classes.yaml'
        engine = Path(directory)/'fixture.engine'
        engine.touch()
        rclpy.init(args=['--ros-args', '-r', '__ns:=/area_yolo_fixture',
                         '-p', f'yolo_targets_file:={classes}', '-p', f'targets_file:={classes}',
                         '-p', f'model_path:={engine}'])
        result = SimpleNamespace(
            names={0: 'person', 2: 'car'}, boxes=SimpleNamespace(
                xyxy=Tensor([[10, 10, 60, 60], [80, 80, 140, 140]]),
                cls=Tensor([0, 2]), conf=Tensor([.9, .9])))
        with patch.object(YoloDetector, '_load_model', return_value=object()), \
                patch.object(YoloDetector, '_predict', return_value=[result]):
            node, dispatcher, detector = Node('fixture'), PipelineDispatcher(), YoloDetector()
            executor = SingleThreadedExecutor()
            for item in (node, dispatcher, detector):
                executor.add_node(item)
            active, detections = [], []

            async def execute(handle):
                waiter = Future()
                active.append(waiter)
                await waiter
                handle.succeed()
                return AreaSearch.Result(success=True, message='fixture completed')

            server = ActionServer(node, AreaSearch, 'area_search', execute)
            publisher = node.create_publisher(Task, 'task', 10)
            images = node.create_publisher(Image, 'front_camera/image_raw', qos_profile_sensor_data)
            node.create_subscription(DetectionArray, 'yolo/detections', detections.append, 10)
            image = Image(width=640, height=480, encoding='rgb8', step=640*3,
                          data=bytes(640*480*3))

            def publish_image():
                image.header.stamp = node.get_clock().now().to_msg()
                image.header.frame_id = 'area_yolo_fixture/front_camera'
                images.publish(image)

            node.create_timer(.1, publish_image)

            def wait(predicate, label):
                deadline = time.monotonic()+15.
                while not predicate():
                    assert time.monotonic() < deadline, label
                    executor.spin_once(timeout_sec=.03)

            def task(target):
                return to_message(dict(task_id='142', targets=[target],
                                       center=[31., 120., 0.], length_m=10., width_m=10.))

            try:
                wait(lambda: publisher.get_subscription_count() > 0 and
                     dispatcher.action_clients['area_search'].server_is_ready(), 'readiness')
                publisher.publish(task('person'))
                publisher.publish(task('car'))
                wait(lambda: len(active) == 1 and detections and
                     [d.class_name for d in detections[-1].detections] == ['person'],
                     'Task person selection reaches detector output')
                assert read_targets(classes) == ['person']
                active[0].set_result(True)
                wait(lambda: len(active) == 2 and detections and
                     [d.class_name for d in detections[-1].detections] == ['car'],
                     'queued Task car selection reaches detector output')
                assert read_targets(classes) == ['car']
                active[1].set_result(True)
                wait(lambda: dispatcher.current is None and detector._targets == (),
                     'terminal cleanup reaches detector standby')
                assert read_targets(classes) == []
                print('PASS: DDS Task 142 -> atomic class file -> YOLO reload/filter; FIFO and terminal standby')
            finally:
                server.destroy()
                for item in (detector, dispatcher, node):
                    item.destroy_node()
                executor.shutdown()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
