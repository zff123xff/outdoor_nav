#!/usr/bin/env python3
"""Real MQTT wire -> gateway -> task topic -> dispatcher -> mock PointGoal."""
import json
import subprocess
from pathlib import Path
import socket
import struct
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from patrol_node.action import PointGoal
from task_pipeline.dispatcher import PipelineDispatcher
from task_pipeline.mqtt_task_gateway import MqttTaskGateway


def packet(kind, data):
    length = len(data)
    prefix = bytearray([kind])
    while True:
        digit = length % 128
        length //= 128
        prefix.append(digit | (128 if length else 0))
        if not length:
            return bytes(prefix)+data


def main():
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen(1)
    listener.settimeout(5.)
    port = listener.getsockname()[1]
    stop = threading.Event()
    received, errors = [], []
    data = dict(task_id='140', waypoints=[31.,120.,0.])
    def broker():
        try:
            conn, _ = listener.accept()
            conn.settimeout(.2)
            with conn:
                def read(n):
                    output = b''
                    while len(output) < n:
                        try:
                            block = conn.recv(n-len(output))
                        except socket.timeout:
                            if stop.is_set():
                                raise EOFError()
                            continue
                        if not block:
                            raise EOFError()
                        output += block
                    return output
                while not stop.is_set():
                    header = read(1)[0]
                    size, shift = 0, 0
                    while True:
                        b = read(1)[0]
                        size += (b & 127) << shift
                        if not b & 128:
                            break
                        shift += 7
                    body = read(size)
                    kind = header >> 4
                    if kind == 1:
                        conn.sendall(b'\x20\x02\x00\x00')
                    elif kind == 8:
                        conn.sendall(packet(0x90, body[:2]+b'\x00'))
                        topic = b'robots/mqtt_fixture/task'
                        conn.sendall(packet(0x30, struct.pack('!H', len(topic))+topic+json.dumps(data).encode()))
                    elif kind == 3:
                        length = struct.unpack('!H', body[:2])[0]
                        topic = body[2:2+length].decode()
                        rest = body[2+length:]
                        if (header >> 1) & 3:
                            conn.sendall(packet(0x40, rest[:2]))
                            rest = rest[2:]
                        received.append((topic, json.loads(rest)))
                    elif kind == 12:
                        conn.sendall(b'\xd0\x00')
                    elif kind == 14:
                        break
        except EOFError:
            pass
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=broker, daemon=True)
    thread.start()
    rclpy.init(args=['--ros-args', '-r', '__ns:=/mqtt_fixture',
                     '-p', 'robot_namespace:=mqtt_fixture', '-p', f'address:=tcp://127.0.0.1:{port}'])
    fixture = Node('fixture')
    dispatcher, gateway = PipelineDispatcher(), MqttTaskGateway()
    seen = []
    def execute(handle):
        seen.append(handle.request)
        handle.succeed()
        return PointGoal.Result(success=True, message='fixture reached')
    server = ActionServer(fixture, PointGoal, 'point_goal', execute)
    executor = SingleThreadedExecutor()
    for node in (fixture, dispatcher, gateway):
        executor.add_node(node)
    try:
        deadline = time.monotonic()+12.
        while not any(topic.endswith('/taskstatus') and payload['progress'] == 100 for topic, payload in received):
            if time.monotonic() > deadline:
                raise AssertionError(f'MQTT timeout: seen={seen}, received={received}, errors={errors}')
            executor.spin_once(timeout_sec=.03)
        assert not errors, errors
        assert len(seen) == 1 and seen[0].mission_id == '140'
        assert seen[0].target.latitude == 31.
        example = Path(__file__).resolve().parents[1]/'src/external_pkg/task_pipeline/config/examples/gps/point_goal.yaml'
        sender = subprocess.Popen(['ros2', 'run', 'task_pipeline', 'send_point_goal',
            '--file', str(example), '--namespace', 'mqtt_fixture'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic()+12.
        try:
            while sender.poll() is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.03)
            if sender.poll() is None:
                sender.terminate()
            output, _ = sender.communicate(timeout=3.)
            assert sender.returncode == 0, output
            assert len(seen) == 2 and seen[1].mission_id == '140'
        finally:
            if sender.poll() is None:
                sender.kill()
                sender.wait(timeout=3.)
        print('PASS: MQTT TCP JSON -> typed Task -> FIFO -> PointGoal Action -> MQTT taskstatus=100; actual example sender topic submission and result')
    finally:
        gateway.destroy_node()
        server.destroy()
        for node in (dispatcher, fixture):
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        stop.set()
        listener.close()
        thread.join(timeout=2.)


if __name__ == '__main__':
    main()
