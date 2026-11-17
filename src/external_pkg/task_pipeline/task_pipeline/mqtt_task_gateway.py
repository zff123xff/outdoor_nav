"""MQTT JSON tasks -> the same typed topic used by example senders."""
import json
import threading
import uuid
from urllib.parse import urlparse

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from task_interfaces.msg import Task, TaskStatus
from task_pipeline.task_schema import to_message


class MqttTaskGateway(Node):
    def __init__(self):
        super().__init__('mqtt_task_gateway')
        for name, default in [('robot_namespace', 'robot_1'),
                              ('address', 'tcp://127.0.0.1:1883'),
                              ('mqtt_username', ''), ('mqtt_passwd', '')]:
            self.declare_parameter(name, default)
        ns = str(self.get_parameter('robot_namespace').value).strip('/')
        self.request_topic = f'robots/{ns}/task'
        self.status_topic = f'robots/{ns}/taskstatus'
        self.events_topic = f'robots/{ns}/task_feedback'
        # MQTT callbacks run in another thread; retain at most one message
        # until the ROS timer can hand it to the single-task dispatcher.
        self.pending = None
        self.pending_lock = threading.Lock()
        self.pub = self.create_publisher(Task, 'task', 10)
        self.create_subscription(TaskStatus, 'robot_task_status', self.status, 10)
        from std_msgs.msg import String
        self.create_subscription(String, 'task_feedback',
            lambda msg: self.mqtt.publish(self.events_topic, msg.data, qos=1, retain=False), 10)
        address = urlparse(self.get_parameter('address').value)
        if address.scheme not in ('tcp', 'mqtt', 'ssl', 'mqtts') or not address.hostname:
            raise ValueError('invalid MQTT broker address')
        self.mqtt = mqtt.Client(client_id='dog_task_'+uuid.uuid4().hex[:10], protocol=mqtt.MQTTv311)
        username = self.get_parameter('mqtt_username').value
        if username:
            self.mqtt.username_pw_set(username, self.get_parameter('mqtt_passwd').value)
        if address.scheme in ('ssl', 'mqtts'):
            self.mqtt.tls_set()
        self.mqtt.on_connect = self.connected
        self.mqtt.on_message = self.message
        self.mqtt.connect_async(address.hostname, address.port or (8883 if address.scheme in ('ssl', 'mqtts') else 1883), 30)
        self.mqtt.loop_start()
        self.create_timer(.05, self.dispatch)

    def connected(self, client, userdata, flags, reason, properties=None):
        if getattr(reason, 'value', reason) == 0:
            client.subscribe(self.request_topic, qos=1)

    def message(self, client, userdata, message):
        if message.retain:
            self.get_logger().warning('Ignoring retained motion task')
            return
        with self.pending_lock:
            if self.pending is None:
                self.pending = message.payload
                return
        try:
            request = json.loads(message.payload)
            task_id = request.get('task_id') if isinstance(request, dict) else None
        except (ValueError, TypeError):
            task_id = None
        self.mqtt.publish(self.events_topic, json.dumps(dict(
            event='rejected', task_id=task_id,
            message='another MQTT task is awaiting dispatch')), qos=1)

    def dispatch(self):
        if not self.pub.get_subscription_count():
            return
        with self.pending_lock:
            payload = self.pending
            self.pending = None
        if payload is None:
            return
        try:
            self.pub.publish(to_message(json.loads(payload)))
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().error(f'MQTT task rejected: {exc}')
            self.mqtt.publish(self.events_topic, json.dumps(dict(event='rejected', message=str(exc))), qos=1)

    def status(self, message):
        data = {key: getattr(message, key) for key in
                ('task_id', 'progress', 'failures', 'param1', 'param2', 'param3')}
        self.mqtt.publish(self.status_topic, json.dumps(data), qos=1, retain=False)

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
