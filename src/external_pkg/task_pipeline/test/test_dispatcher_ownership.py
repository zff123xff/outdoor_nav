from types import SimpleNamespace as NS
from unittest.mock import Mock
from rclpy.task import Future
from task_pipeline.dispatcher import PipelineDispatcher


def test_unknown_primitive_result_cannot_release_active_channel():
    future=Future();future.set_exception(RuntimeError('result unavailable'))
    node=NS(finish=Mock(),ownership_error=Mock())
    PipelineDispatcher.result(node,future)
    node.finish.assert_not_called()
    node.ownership_error.assert_called_once()


def test_ownership_error_cancels_known_handle_and_reports_diagnostic():
    handle=NS(accepted=True,cancel_goal_async=Mock())
    node=NS(goal_handle=handle,current=NS(task_id='143'),emit=Mock(),get_logger=Mock())
    PipelineDispatcher.ownership_error(node,RuntimeError('transport failed'))
    assert node._ownership_error=='transport failed'
    handle.cancel_goal_async.assert_called_once()
    assert node.emit.call_args.args[0]=='ownership_error'
