"""Task topic admits one task and cancels only the matching active task."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

from task_interfaces.msg import Task
from task_pipeline.dispatcher import PipelineDispatcher


def node(current=None):
    return NS(current=current, _pending_finish=None, _ownership_error='',
              emit=Mock(), publish_status=Mock(), cancel=Mock(),
              target_interaction=NS(begin=Mock()), get_parameter=lambda _: NS(value=30.),
              fix=None, goal_handle=None, sending=False)


def test_idle_task_is_validated_and_accepted_immediately():
    state = node()
    task = Task(task_id=140, waypoints=[31., 120., 0.])
    PipelineDispatcher.receive(state, task)
    assert state.current is task
    assert state.progress == 1
    assert state.emit.call_args.args == ('accepted', task)
    state.publish_status.assert_called_once_with()


def test_busy_task_is_rejected_before_its_parameters_are_checked():
    active = Task(task_id=140, waypoints=[31., 120., 0.])
    state = node(active)
    invalid = Task(task_id=141)  # Missing both required waypoints.
    PipelineDispatcher.receive(state, invalid)
    assert state.current is active
    assert state.emit.call_args.args == ('rejected', invalid)
    assert state.emit.call_args.kwargs['message'] == 'another task is still active'


def test_invalid_idle_task_is_rejected_without_becoming_current():
    state = node()
    task = Task(task_id=140)
    PipelineDispatcher.receive(state, task)
    assert state.current is None
    assert state.emit.call_args.args == ('rejected', task)
    state.publish_status.assert_not_called()


def test_cancel_only_matches_current_task_id():
    active = Task(task_id=140, waypoints=[31., 120., 0.])
    state = node(active)
    state.finish = Mock()
    PipelineDispatcher.cancel(state, Task(task_id=141, cancel=True))
    assert state.current is active
    state.finish.assert_not_called()
    assert state.emit.call_args.args[0] == 'rejected'
    PipelineDispatcher.cancel(state, Task(task_id=140, cancel=True))
    assert state.cancel_pending == 140
    state.finish.assert_called_once_with(False, 'canceled before dispatch')
