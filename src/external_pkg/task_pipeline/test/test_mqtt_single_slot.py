"""MQTT-to-ROS handoff does not accumulate an execution queue."""
import json
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

from task_pipeline.mqtt_task_gateway import MqttTaskGateway


def test_second_mqtt_message_is_rejected_while_handoff_slot_is_full():
    state = NS(pending=None, pending_lock=threading.Lock(), mqtt=NS(publish=Mock()),
               events_topic='robots/robot_1/task_feedback', pub=NS(
                   get_subscription_count=lambda: 1, publish=Mock()))
    first = json.dumps(dict(task_id=140, waypoints=[31., 120., 0.])).encode()
    second = json.dumps(dict(task_id=141, waypoints=[31., 120., 0.]*2)).encode()
    MqttTaskGateway.message(state, None, None, NS(retain=False, payload=first))
    MqttTaskGateway.message(state, None, None, NS(retain=False, payload=second))
    assert state.pending == first
    rejection = json.loads(state.mqtt.publish.call_args.args[1])
    assert (rejection['event'], rejection['task_id']) == ('rejected', 141)
    MqttTaskGateway.dispatch(state)
    assert state.pending is None
    assert state.pub.publish.call_args.args[0].task_id == 140
