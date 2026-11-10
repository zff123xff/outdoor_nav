"""Inspection orchestration rejects early confirmations and retires canceled goals."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from std_srvs.srv import SetBool
from task_interfaces.msg import TargetEvent
from task_pipeline.target_interaction import TargetInteraction


def interaction():
    node = Mock()
    parameters = {'simulation_targets': True, 'target_interaction_enabled': True}
    node.get_parameter.side_effect = lambda name: NS(value=parameters[name])
    node.current = NS(task_id='area', task_type='area_search')
    node.goal_handle = Mock(accepted=True)
    services = {}
    node.create_client.side_effect = lambda kind, name: services.setdefault(name, Mock())
    with patch('task_pipeline.target_interaction.ActionClient'):
        state = TargetInteraction(node)
    state.begin(node.current)
    return node, state


def found(target_id=1):
    event = TargetEvent(target_id=target_id, command=TargetEvent.TARGET_FOUND)
    event.position.status.status = 0
    return event


def test_suspension_response_precedes_approach_and_confirmation_precedes_resume():
    node, state = interaction()
    paused = Future()
    state.pause.call_async.return_value = paused
    state.observed(found())
    state.shoot(TargetEvent(target_id=1))
    state.request_pub.publish.assert_not_called()
    state.tick()
    state.approach.send_goal_async.assert_not_called()
    paused.set_result(SetBool.Response(success=True))
    accepted = Future()
    state.approach.send_goal_async.return_value = accepted
    state.tick()
    assert state.phase == 'approaching_target'
    handle = Mock(accepted=True)
    completed = Future()
    handle.get_result_async.return_value = completed
    accepted.set_result(handle)
    state.feedback(NS(feedback=NS(phase='awaiting_confirmation', distance_m=4.5, heading_error_rad=0.)),
                   state.generation)
    state.shoot(TargetEvent(target_id=2))
    state.request_pub.publish.assert_not_called()
    state.shoot(TargetEvent(target_id=1))
    assert state.request_pub.publish.call_args.args[0].command == TargetEvent.REQUEST_SHOOT
    state.pause.call_async.assert_called_once()
    completed.set_result(NS(status=4, result=NS(success=True, message='marked')))
    assert state.phase == 'resuming_search' and state.handled == {1}
    resumed = Future()
    state.pause.call_async.return_value = resumed
    state.tick()
    assert state.pause.call_async.call_args.args[0].data is False
    resumed.set_result(SetBool.Response(success=True))
    state.observed(found(1))
    assert state.phase == 'searching'
    state.observed(found(2))
    assert state.phase == 'pausing_search' and state.target_id == 2


def test_cancellation_during_acceptance_waits_for_target_action_to_retire():
    node, state = interaction()
    state.phase = 'starting_approach'
    state.target_id = 1
    state.deadline = float('inf')
    accepted = Future()
    state.approach.send_goal_async.return_value = accepted
    state.tick()
    state.stop()
    assert not state.drained()
    handle = Mock(accepted=True)
    result = Future()
    handle.get_result_async.return_value = result
    accepted.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert not state.drained()
    result.set_result(NS(status=5))
    assert state.drained()
    assert state.handled == set()


def test_old_feedback_does_not_modify_a_new_task():
    node, state = interaction()
    token = state.generation
    state.begin(NS(task_id='next'))
    state.feedback(NS(feedback=NS(phase='awaiting_confirmation')), token)
    assert state.phase == 'searching'


def test_rejected_pause_cancels_area_task_without_starting_navigation():
    node, state = interaction()
    state.observed(found())
    future = Future()
    state.pause.call_async.return_value = future
    state.tick()
    future.set_result(SetBool.Response(success=False, message='not active'))
    assert state.stopping and 'not active' in node._interaction_error
    node.goal_handle.cancel_goal_async.assert_called_once()
    state.approach.send_goal_async.assert_not_called()


def test_cancel_does_not_release_task_with_pause_request_still_in_transit():
    node, state = interaction()
    paused = Future()
    state.pause.call_async.return_value = paused
    state.observed(found())
    state.tick()
    state.stop()
    assert not state.drained()  # A late pause must not suspend the next queued area task.
    paused.set_result(SetBool.Response(success=False, message='area ended'))
    assert state.drained()
    state.approach.send_goal_async.assert_not_called()
