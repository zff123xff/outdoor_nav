from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from rclpy.task import Future
from task_pipeline.inspection_executor import InspectionExecutor


def test_cancel_during_goal_acceptance_is_drained_before_releasing_parent():
    result=Future()
    handle=NS(accepted=True,cancel_goal_async=Mock(),get_result_async=lambda:result)
    accepted=Future();accepted.set_result(handle)
    parent=NS(inflight=True,child=None,canceling=True,phase='approaching',
              completed=Mock(),get_logger=Mock())
    InspectionExecutor.accepted(parent,accepted)
    assert parent.child is handle
    assert not parent.inflight
    handle.cancel_goal_async.assert_called_once()
    parent.completed.assert_not_called()


def test_timeout_draining_never_advances_until_result():
    parent=NS(child=object(),canceling=False,phase='draining',fail_target=Mock())
    future=Future();future.set_result(NS(status=5))
    InspectionExecutor.completed(parent,future)
    assert parent.child is None
    parent.fail_target.assert_called_once_with('target timed out')


def test_navigation_failure_does_not_enter_verification():
    future=Future();future.set_result(NS(status=6))
    parent=NS(child=object(),canceling=False,phase='approaching',fail_target=Mock())
    InspectionExecutor.completed(parent,future)
    parent.fail_target.assert_called_once()
    assert parent.phase=='approaching'


def test_transport_error_with_uncertain_goal_keeps_ownership():
    future=Future();future.set_exception(RuntimeError('lost response'))
    parent=NS(inflight=True,canceling=False,phase='approaching',get_logger=Mock())
    InspectionExecutor.accepted(parent,future)
    assert parent.canceling and parent.phase=='transport_error'


def test_stopped_requires_advancing_pose_samples_for_a_continuous_window():
    parent=NS(previous_sample=None,stopped_since=None,
              cfg=dict(pose_max_age_s=.5,linear_stop_mps=.05,angular_stop_rps=.05,settle_s=.5))
    sample=lambda stamp,x=0.:(x,0.,0.,0.,stamp)
    assert not InspectionExecutor.stopped(parent,sample(1),0.)
    assert not InspectionExecutor.stopped(parent,sample(2),.2)
    assert not InspectionExecutor.stopped(parent,sample(2),.8)
    assert not InspectionExecutor.stopped(parent,sample(3),1.) # A gap resets the window.
    assert not InspectionExecutor.stopped(parent,sample(4),1.2)
    assert not InspectionExecutor.stopped(parent,sample(5),1.4)
    assert InspectionExecutor.stopped(parent,sample(6),1.8)
    assert not InspectionExecutor.stopped(parent,sample(7,x=1.),2.)


def test_missing_result_never_releases_a_potentially_running_child():
    future=Future();future.set_exception(RuntimeError('result transport lost'))
    child=NS(cancel_goal_async=Mock())
    parent=NS(child=child,canceling=False,phase='approaching',get_logger=Mock())
    InspectionExecutor.completed(parent,future)
    assert parent.child is child and parent.phase=='transport_error' and parent.canceling
    child.cancel_goal_async.assert_called_once()
