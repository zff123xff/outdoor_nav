from types import SimpleNamespace as NS
from unittest.mock import Mock
from task_pipeline.dispatcher import PipelineDispatcher


def test_idle_status_is_all_zero_including_string_id():
    node = NS(current=None, status_pub=Mock(), progress=100, failures=7)
    PipelineDispatcher.publish_status(node)
    status = node.status_pub.publish.call_args.args[0]
    assert status.task_id == '0'
    assert status.progress == status.failures == status.param1 == status.param2 == status.param3 == 0


def test_active_status_uses_mission_sequence_id_and_terminal_then_idle_are_preserved():
    node = NS(current=NS(task_id=140, mission_seq_id=7), status_pub=Mock(), progress=100, failures=1)
    PipelineDispatcher.publish_status(node)
    status = node.status_pub.publish.call_args.args[0]
    assert (status.task_id, status.progress, status.failures) == ('7', 100, 1)
    node.current = None
    PipelineDispatcher.publish_status(node)
    assert node.status_pub.publish.call_args.args[0].task_id == '0'
