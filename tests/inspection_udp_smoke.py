#!/usr/bin/env python3
"""Actual loopback datagram -> ROS assignment; ROS observation -> datagram."""
import json
import socket
import time
import rclpy
from rclpy.node import Node
from task_interfaces.msg import TargetArray, InspectionArrival
from task_pipeline.inspection_udp_bridge import InspectionUdpBridge


def main():
    receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1',0));receiver.setblocking(False)
    rclpy.init(args=['--ros-args','-r','__ns:=/udp_fixture','-p','input_port:=0',
                     '-p',f'output_port:={receiver.getsockname()[1]}'])
    bridge=InspectionUdpBridge();node=Node('fixture');received=[]
    from rclpy.executors import SingleThreadedExecutor
    executor=SingleThreadedExecutor()
    executor.add_node(bridge);executor.add_node(node)
    node.create_subscription(TargetArray,'targets/assigned',received.append,10)
    pub=node.create_publisher(InspectionArrival,'targets/inspection_arrived',10)
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    def wait(predicate,label):
        deadline=time.monotonic()+7.
        while not predicate():
            if time.monotonic()>deadline:raise AssertionError(label)
            executor.spin_once(timeout_sec=.03)
    try:
        wait(lambda:pub.get_subscription_count() and bridge.pub.get_subscription_count(),'DDS discovery')
        packet=dict(namespace='udp_fixture',topic='targets/assigned',data=dict(targets=[dict(
            target_id=7,target_position=dict(x=10.,y=2.,z=0.))]))
        sender.sendto(json.dumps(packet).encode(),('127.0.0.1',bridge.socket.getsockname()[1]))
        wait(lambda:received,'actual ROS assignment receipt')
        assert received[0].targets[0].target_id==7
        pub.publish(InspectionArrival(task_id='143',target_id=7,distance_m=9.))
        output=[]
        def datagram():
            try:output.append(json.loads(receiver.recvfrom(65535)[0]));return True
            except BlockingIOError:return False
        wait(datagram,'actual UDP arrival receipt')
        assert output[0]['topic']=='targets/inspection_arrived'
        assert output[0]['data']['target_id']==7
        print('PASS: real loopback UDP <-> typed ROS inspection transport')
    finally:
        sender.close();receiver.close();bridge.destroy_node();node.destroy_node()
        executor.shutdown();rclpy.try_shutdown()

if __name__=='__main__':main()
