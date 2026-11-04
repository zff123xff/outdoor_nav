#!/usr/bin/env python3
"""CPU-only model fixture; real task/file reload, camera/cloud TF and markers."""
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Bool
from std_msgs.msg import String
import json
from task_interfaces.msg import Task, TargetEvent
from task_interfaces.action import AreaSearch
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.task_schema import to_message
from yolo_detector.yolo_node import YoloDetector
from yolo_detector.targets import read_targets, write_targets
from yolo_detector.projection import enu_to_gps


class Tensor:
    def __init__(self, values):self.values=np.asarray(values)
    def cpu(self):return self
    def numpy(self):return self.values


def main():
    with tempfile.TemporaryDirectory(prefix='simulation_targets_') as directory:
        path=Path(directory)/'targets.yaml'
        engine=Path(directory)/'fixture.engine';engine.touch()
        rclpy.init(args=['--ros-args','-r','__ns:=/targets_fixture',
            '-p',f'model_path:={engine}','-p',f'targets_file:={path}'])
        original_load,original_predict=YoloDetector._load_model,YoloDetector._predict
        YoloDetector._load_model=lambda self:object()
        YoloDetector._predict=lambda self,model,image:[NS(
            boxes=NS(xyxy=Tensor([[310,230,330,250],[390,230,410,250]]),
                     cls=Tensor([39,56]),conf=Tensor([.9,.9])),names={39:'bottle',56:'chair'})]
        fixture=Node('fixture')
        pipeline,detector=PipelineDispatcher(),YoloDetector()
        pipeline.set_parameters([Parameter('simulation_targets',value=True)])
        executor=SingleThreadedExecutor()
        for node in (fixture,pipeline,detector):executor.add_node(node)
        image_pub=fixture.create_publisher(Image,'front_camera/image_raw',10)
        camera_pub=fixture.create_publisher(CameraInfo,'front_camera/camera_info',10)
        cloud_pub=fixture.create_publisher(__import__('sensor_msgs.msg',fromlist=['PointCloud2']).PointCloud2,'livox/lidar',10)
        task_pub=fixture.create_publisher(Task,'task',10)
        active_pub=fixture.create_publisher(Bool,'area_search/active',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        request_pub=fixture.create_publisher(TargetEvent,'targets/request',10)
        found,visuals,events=[],[],[]
        fixture.create_subscription(String,'task_feedback',lambda msg:events.append(json.loads(msg.data)),10)
        fixture.create_subscription(TargetEvent,'targets/found',found.append,10)
        fixture.create_subscription(MarkerArray,'targets/markers',lambda msg:visuals.extend(msg.markers),10)
        broadcaster=TransformBroadcaster(fixture)
        release=Future()
        state=dict(active=False)
        async def area(handle):
            state['active']=True
            active_pub.publish(Bool(data=True))
            await release
            state['active']=False
            active_pub.publish(Bool(data=False))
            if handle.is_cancel_requested:handle.canceled()
            else:handle.succeed()
            return AreaSearch.Result(success=not handle.is_cancel_requested,message='fixture finished')
        def cancel(handle):
            if not release.done():release.set_result(False)
            return CancelResponse.ACCEPT
        server=ActionServer(fixture,AreaSearch,'area_search',area,cancel_callback=cancel)
        def origin(request,response):
            response.values=[ParameterValue(type=3,double_value=v) for v in (31.,120.,100.)]
            return response
        fixture.create_service(GetParameters,'nav2_input_bridge/get_parameters',origin)
        def sensors():
            stamp=fixture.get_clock().now().to_msg()
            transforms=[]
            for frame in ('camera_optical','lidar'):
                tf=TransformStamped();tf.header=Header(stamp=stamp,frame_id='targets_fixture/map')
                tf.child_frame_id='targets_fixture/'+frame
                tf.transform.translation.z=1.
                tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5
                tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
                transforms.append(tf)
            broadcaster.sendTransform(transforms)
            header=Header(stamp=stamp,frame_id='targets_fixture/camera_optical')
            camera_pub.publish(CameraInfo(header=header,width=640,height=480,
                k=[100.,0.,320.,0.,100.,240.,0.,0.,1.],distortion_model='plumb_bob'))
            points=[(x,y,5.) for center in (0.,4.) for x,y in
                    [(center-.1,0.),(center,0.),(center+.1,0.),(center,.1),(center,-.1)]]
            cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='targets_fixture/lidar'),points))
            image=Image(header=header,width=640,height=480,encoding='rgb8',step=640*3)
            image.data=bytes(640*480*3)
            image_pub.publish(image)
            if state['active']:active_pub.publish(Bool(data=True))
        fixture.create_timer(.1,sensors)
        def wait(predicate,label,timeout=8.):
            deadline=time.monotonic()+timeout
            while not predicate():
                if time.monotonic()>deadline:raise AssertionError(f'Timeout {label}: found={found}')
                executor.spin_once(timeout_sec=.03)
        try:
            wait(lambda:task_pub.get_subscription_count() and pipeline.action_clients['area_search'].server_is_ready(),'readiness')
            task_pub.publish(to_message(dict(task_id='observe',task_type='area_search',targets=['bottle'],
                area=dict(center=dict(latitude=31.,longitude=120.,altitude=100.),length_m=10.,width_m=10.,yaw_deg=0.))))
            wait(lambda:found and detector._targets==('bottle',),'selected bottle observation')
            assert {event.target_type for event in found}=={'bottle'}
            first=found[0]
            assert first.target_id==1 and first.command=='target found'
            expected=enu_to_gps([5.,0.,1.],(31.,120.,100.))
            assert np.allclose([first.position.latitude,first.position.longitude,first.position.altitude],expected,rtol=0,atol=1e-6)
            request_pub.publish(TargetEvent(target_id=1,command='request shoot'))
            wait(lambda:any(m.id==1 and m.color.r==1. and m.color.g==0. for m in visuals),'simulation mark')
            task_pub.publish(to_message(dict(task_id='queued',task_type='point_goal',targets=['chair'],
                waypoints=[dict(latitude=31.,longitude=120.,altitude=100.)])))
            wait(lambda:len(pipeline.queue)==1,'queued task')
            assert read_targets(path)==['bottle'],'queued task changed active detector classes'
            task_pub.publish(Task(task_id='queued',cancel=True))
            wait(lambda:not pipeline.queue,'queued cancellation')
            write_targets(path,['chair'],'external_update')
            wait(lambda:any(event.target_type=='chair' for event in found) and detector._targets==('chair',),'1s class reload')
            assert {event.target_id for event in found if event.target_type=='chair'}=={2}
            assert {event.target_id for event in found if event.target_type=='bottle'}=={1}
            request_pub.publish(TargetEvent(target_id=999,command='request shoot'))
            task_pub.publish(Task(task_id='observe',cancel=True))
            wait(lambda:pipeline.current is None and pipeline._target_marker_process is None,'area cancellation')
            assert not any(m.id==999 for m in visuals)
            # A successful next task gets a fresh child; completion cleans it up.
            release=Future()
            area_task=dict(task_id='success',task_type='area_search',targets=['39'],
                area=dict(center=dict(latitude=31.,longitude=120.,altitude=100.),length_m=10.,width_m=10.,yaw_deg=0.))
            task_pub.publish(to_message(area_task))
            previous=len(found)
            wait(lambda:len(found)>previous and detector._targets==('39',),'next task and numeric category')
            assert found[-1].target_id==1,'new task must use a fresh marker node'
            visuals.clear()
            request_pub.publish(TargetEvent(target_id=1,command='request shoot'))
            wait(lambda:any(m.id==1 and m.color.r==1. for m in visuals),'numeric category mark')
            release.set_result(True)
            wait(lambda:pipeline.current is None and pipeline._target_marker_process is None,'successful cleanup')
            wait(lambda:any(e['event']=='completed' and e['task_id']=='success' for e in events),'success feedback')
            # Unexpected child termination must cancel the Action and fail the task.
            release=Future()
            area_task['task_id']='child_failure'
            task_pub.publish(to_message(area_task))
            wait(lambda:pipeline.goal_handle is not None and pipeline._target_marker_process is not None,'child failure readiness')
            pipeline._target_marker_process.terminate()
            wait(lambda:pipeline.current is None and pipeline._target_marker_process is None,'child failure cleanup')
            wait(lambda:any(e['event']=='failed' and e['task_id']=='child_failure' and 'marker exited' in e['message'] for e in events),'failure feedback')
            print('PASS: Task/file/1s YOLO reload; lidar/TF/GPS; stable IDs; simulation marks; queued isolation; dispatcher starts/stops child on cancel, success and crash')
        finally:
            server.destroy()
            for node in (detector,pipeline,fixture):node.destroy_node()
            executor.shutdown();rclpy.try_shutdown()
            YoloDetector._load_model,YoloDetector._predict=original_load,original_predict


if __name__=='__main__':main()
