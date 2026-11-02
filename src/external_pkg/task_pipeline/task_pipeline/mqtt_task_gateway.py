"""Navigation-owned MQTT primitive ingress; delegates execution to the dispatcher."""
import json
import queue
import time
import uuid
from urllib.parse import urlparse

import paho.mqtt.client as mqtt
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters

from execute_task_server.srv import ExecuteTask
from task_pipeline.coordinates import enu_to_gps


def normalize_waypoints(task, origin):
    """Normalize ENU in navigation, using its live authoritative GPS origin."""
    params = task.get('params', {})
    if params.get('waypoint_frame') == 'enu':
        points = []
        for point in params['waypoints']:
            lat, lon, alt = enu_to_gps(point['x'], point['y'], point.get('z', 0), origin)
            points.append({'lat': lat, 'lon': lon, 'alt': alt})
        params['waypoints'], params['waypoint_frame'] = points, 'gps'
    return task


class MqttTaskGateway(Node):
    def __init__(self):
        super().__init__('mqtt_task_gateway')
        for name, default in [('robot_namespace', 'robot_1'), ('vehicle_id', 1),
                              ('address', 'tcp://127.0.0.1:1883'),
                              ('mqtt_username', ''), ('mqtt_passwd', ''), ('request_timeout_s', 10.0)]:
            self.declare_parameter(name, default)
        self.ns = str(self.get_parameter('robot_namespace').value).strip('/')
        vehicle = int(self.get_parameter('vehicle_id').value)
        address = urlparse(self.get_parameter('address').value)
        if address.scheme not in ('tcp', 'mqtt', 'ssl', 'mqtts') or not address.hostname:
            raise ValueError('invalid MQTT broker address')
        self.timeout = float(self.get_parameter('request_timeout_s').value)
        self.pending = queue.Queue(maxsize=64)
        self.calls = {}
        self.request_topic = f'/sim/command/{vehicle}'
        self.response_topic = f'/sim/response/{vehicle}'
        self.service = self.create_client(ExecuteTask, f'/agent/{self.ns}/execute_task')
        self.origin = self.create_client(
            GetParameters, f'/{self.ns}/nav2_input_bridge/get_parameters')
        self.mqtt = mqtt.Client(client_id='nav_primitive_'+uuid.uuid4().hex[:10], protocol=mqtt.MQTTv5)
        username = self.get_parameter('mqtt_username').value
        if username:
            self.mqtt.username_pw_set(username, self.get_parameter('mqtt_passwd').value)
        tls = address.scheme in ('ssl', 'mqtts')
        if tls:
            self.mqtt.tls_set()
        self.mqtt.on_connect, self.mqtt.on_message = self._connected, self._message
        self.mqtt.connect_async(address.hostname, address.port or (8883 if tls else 1883), 30)
        self.mqtt.loop_start()
        self.create_timer(0.05, self._dispatch)

    def _connected(self, client, userdata, flags, reason, properties=None):
        if getattr(reason, 'value', reason) == 0:
            client.subscribe(self.request_topic, qos=1)
            self.get_logger().info(f'MQTT primitive ingress: {self.request_topic}')
        else:
            self.get_logger().error(f'MQTT connection rejected: {reason}')

    def _message(self, client, userdata, message):
        correlation = getattr(message.properties, 'CorrelationData', None)
        try:
            self.pending.put_nowait((message.payload, correlation))
        except queue.Full:
            self._reply({'status': 'rejected', 'message': 'ingress queue is full'}, correlation)

    def _track(self, future, correlation, callback):
        self.calls[future] = (time.monotonic()+self.timeout, correlation)

        def done(result):
            if result not in self.calls:
                return
            self.calls.pop(result)
            try:
                callback(result.result())
            except Exception as exc:
                self._reply({'status': 'rejected', 'message': str(exc)}, correlation)
        future.add_done_callback(done)

    def _dispatch(self):
        for future, (deadline, correlation) in list(self.calls.items()):
            if time.monotonic() >= deadline:
                self.calls.pop(future)
                future.cancel()
                self._reply({'status': 'rejected', 'message': 'navigation request timed out'}, correlation)
        try:
            payload, correlation = self.pending.get_nowait()
        except queue.Empty:
            return
        try:
            task = json.loads(payload)
            if not isinstance(task, dict) or task.get('agent_id') != self.ns:
                raise ValueError('task agent_id must match robot_namespace')
            if not all(isinstance(task.get(key), str) and task[key] for key in ('mission_id', 'primitive_id')):
                raise ValueError('mission_id and primitive_id are required')
            if not isinstance(task.get('params'), dict):
                raise ValueError('task params must be an object')
            if task['params'].get('waypoint_frame') == 'enu':
                if not self.origin.service_is_ready():
                    raise RuntimeError('navigation origin parameter service is unavailable')
                future = self.origin.call_async(GetParameters.Request(names=[
                    'reference_latitude', 'reference_longitude', 'reference_altitude']))

                def with_origin(result):
                    values = tuple(float(parameter_value_to_python(p)) for p in result.values)
                    self._forward(normalize_waypoints(task, values), correlation)
                self._track(future, correlation, with_origin)
            else:
                self._forward(task, correlation)
        except Exception as exc:
            self._reply({'status': 'rejected', 'message': str(exc)}, correlation)

    def _forward(self, task, correlation):
        if not self.service.service_is_ready():
            raise RuntimeError('ExecuteTask dispatcher is unavailable')
        request = ExecuteTask.Request()
        request.request_json = json.dumps(task, allow_nan=False)

        def result(response):
            payload = json.loads(response.response_json)
            accepted = payload.get('status') in ('success', 'accepted')
            self._reply({**payload, 'status': 'accepted' if accepted else 'rejected'}, correlation)
        self._track(self.service.call_async(request), correlation, result)

    def _reply(self, payload, correlation):
        if correlation is None:
            return
        props = Properties(PacketTypes.PUBLISH)
        props.CorrelationData = correlation
        self.mqtt.publish(self.response_topic, json.dumps(payload, ensure_ascii=False),
                          qos=1, retain=False, properties=props)

    def destroy_node(self):
        self.mqtt.disconnect()
        self.mqtt.loop_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttTaskGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
