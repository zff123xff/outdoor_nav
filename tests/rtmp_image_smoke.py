#!/usr/bin/env python3
"""Actual H264 RTMP -> FFmpeg -> ROS Image -> CPU fake YOLO, isolated domain."""
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from types import SimpleNamespace as NS
import numpy as np
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from yolo_interfaces.msg import DetectionArray
from rtmp_image_bridge.node import RtmpImageBridge
from yolo_detector.yolo_node import YoloDetector
from yolo_detector.targets import write_targets


def main():
    probe=socket.socket();probe.bind(('127.0.0.1',0));port=probe.getsockname()[1];probe.close()
    url=f'rtmp://127.0.0.1:{port}/live/robot_2'
    publishers=[]
    with tempfile.TemporaryDirectory(prefix='rtmp_ros_smoke_') as directory:
        root=Path(directory);engine=root/'dummy.engine';engine.touch()
        targets=root/'targets.yaml';write_targets(targets,['person','car'])
        rclpy.init(args=['--ros-args','-r','__ns:=/robot_2','-p',f'rtmp_url:={url}',
            '-p','width:=1920','-p','height:=1080',
            '-p','expected_width:=1920','-p','expected_height:=1080','-p','mode:=listen','-p','read_timeout_s:=8.0','-p','reconnect_delay_s:=0.3',
            '-p',f'model_path:={engine}','-p',f'targets_file:={targets}'])
        executor=SingleThreadedExecutor()
        bridge=RtmpImageBridge()
        empty=NS(cpu=lambda:NS(numpy=lambda:np.array([])))
        prediction=NS(boxes=NS(xyxy=empty,cls=empty,conf=empty),names={})
        loader=patch.object(YoloDetector,'_load_model',lambda self:object())
        predictor=patch.object(YoloDetector,'_predict',lambda *args:[prediction])
        loader.start();predictor.start()
        detector=YoloDetector()
        node=Node('rtmp_test_peer')
        images,detections=[],[]
        node.create_subscription(Image,'front_camera/image_raw',images.append,qos_profile_sensor_data)
        node.create_subscription(DetectionArray,'yolo/detections',detections.append,10)
        for item in (bridge,detector,node):executor.add_node(item)
        def wait(predicate,label,timeout=15.):
            deadline=time.monotonic()+timeout
            while not predicate():
                if time.monotonic()>deadline:raise AssertionError(label)
                executor.spin_once(timeout_sec=.03)
        def push(color):
            # Retry connection while the bridge reopens its single-client RTMP listener.
            for attempt in range(10):
                with (root/f'push_{len(publishers)}.log').open('wb') as log:
                    process=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-nostdin',
                        '-re','-f','lavfi','-i',f'color=c={color}:s=1920x1080:r=10',
                        '-c:v','libx264','-threads','1','-preset','ultrafast','-tune','zerolatency',
                        '-g','10','-pix_fmt','yuv420p','-f','flv',url],stdout=subprocess.DEVNULL,stderr=log)
                publishers.append(process)
                until=time.monotonic()+.3
                while time.monotonic()<until:executor.spin_once(timeout_sec=.03)
                if process.poll() is None:return process
            raise AssertionError('RTMP source could not connect')
        def stop(process):
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=2.)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2.)
        try:
            wait(lambda:bridge.publisher.get_subscription_count()>=2,'DDS image subscriptions')
            first=push('red')
            wait(lambda:images and detections,'RTMP frames and YOLO receipt')
            image=images[-1]
            assert (image.width,image.height,image.encoding,image.step)==(1920,1080,'rgb8',5760)
            assert len(image.data)==1920*1080*3
            assert image.data[0]>240 and image.data[1]<15 and image.data[2]<15
            assert image.header.frame_id=='robot_2/front_camera_optical_frame'
            assert image.header.stamp.sec>0
            assert detections[-1].header.frame_id==image.header.frame_id
            stop(first)
            # Drain in-flight DDS, then verify that no frozen images are replayed.
            until=time.monotonic()+1.
            while time.monotonic()<until:executor.spin_once(timeout_sec=.03)
            count=len(images)
            until=time.monotonic()+.5
            while time.monotonic()<until:executor.spin_once(timeout_sec=.03)
            assert len(images)==count,'Disconnected stream replayed an old frame'
            second=push('blue')
            wait(lambda:len(images)>count and images[-1].data[2]>240,'new stream after reconnect')
            assert images[-1].data[0]<15
            stop(second)
            print('PASS: actual H264 RTMP -> 1920x1080 rgb8 ROS Image -> fake YOLO DDS; disconnect silence, reconnect new frames; no GPU or robot motion')
        finally:
            for process in publishers:stop(process)
            for item in (detector,bridge,node):item.destroy_node()
            executor.shutdown();rclpy.try_shutdown()
            predictor.stop();loader.stop()

if __name__=='__main__':main()
