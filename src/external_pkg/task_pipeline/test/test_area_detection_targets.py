"""142/143 select persistent classes only for accepted tasks; no robot motion."""
from unittest.mock import Mock

import pytest
import rclpy
from task_interfaces.msg import Task
from task_pipeline import dispatcher
from task_pipeline.task_schema import to_message
from yolo_detector.targets import DEFAULT_TARGETS_FILE, read_targets, write_targets


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=['--ros-args', '-r', '__ns:=/detection_targets_fixture',
                     '-p', f'yolo_targets_file:={tmp_path / "classes.yaml"}'])
    item = None
    try:
        item = dispatcher.PipelineDispatcher()
        item.emit = Mock()
        for client in item.action_clients.values():
            client.server_is_ready = Mock(return_value=True)
            client.send_goal_async = Mock(return_value=Mock())
        write_targets(item.yolo_targets_path, ['truck'])
        yield item
    finally:
        if item is not None:
            before = item.yolo_targets_path.read_bytes()
            item.destroy_node()
            assert item.yolo_targets_path.read_bytes() == before, 'Shutdown changed selected classes'
        rclpy.try_shutdown()


def detection_task(task_id, classes):
    fields = dict(task_id=task_id, targets=classes)
    if task_id == 142:
        fields.update(center=[31., 120., 0.], length_m=10., width_m=10.)
    return to_message(fields)


def kind(task_id):
    return 'area_search' if task_id == 142 else 'target_inspection'


@pytest.mark.parametrize('task_id', [142, 143])
def test_busy_task_is_rejected_without_overwriting_classes(node, task_id):
    node.receive(detection_task(task_id, ['person', 'car']))
    node.receive(detection_task(task_id, ['truck']))
    assert node.emit.call_args.args[0] == 'rejected'
    assert read_targets(node.yolo_targets_path) == ['truck']
    node.pump()
    assert read_targets(node.yolo_targets_path) == ['person', 'car']
    node.pump()
    assert read_targets(node.yolo_targets_path) == ['person', 'car']
    node.finish(True, 'finished')
    assert read_targets(node.yolo_targets_path) == ['person', 'car']
    node.pump()
    assert read_targets(node.yolo_targets_path) == ['person', 'car']
    assert node.action_clients[kind(task_id)].send_goal_async.call_count == 1


@pytest.mark.parametrize('task_id', [142, 143])
@pytest.mark.parametrize('success', [True, False])
def test_terminal_result_retains_classes(node, task_id, success):
    node.receive(detection_task(task_id, ['person']))
    node.pump()
    node.finish(success, 'terminal')
    assert read_targets(node.yolo_targets_path) == ['person']


@pytest.mark.parametrize('task_id,waypoints', [(140, [31., 120., 0.]),
    (141, [31., 120., 0.]*2), (145, [31., 120., 0.]*3)])
def test_navigation_only_tasks_do_not_touch_classes(node, task_id, waypoints):
    before = node.yolo_targets_path.read_bytes()
    node.receive(to_message(dict(task_id=task_id, targets=['person'], waypoints=waypoints)))
    node.pump()
    node.finish(True, 'finished')
    assert node.yolo_targets_path.read_bytes() == before


@pytest.mark.parametrize('task_id', [142, 143])
def test_cancellation_before_action_dispatch_does_not_change_manual_classes(node, task_id):
    node.receive(detection_task(task_id, ['person']))
    node.receive(Task(task_id=task_id, cancel=True))
    node.pump()
    assert read_targets(node.yolo_targets_path) == ['truck']
    node.action_clients[kind(task_id)].send_goal_async.assert_not_called()


@pytest.mark.parametrize('task_id', [142, 143])
def test_mismatched_cancel_does_not_stop_active_task(node, task_id):
    node.receive(detection_task(task_id, ['person']))
    node.receive(Task(task_id=140, cancel=True))
    assert node.current is not None and node.current.task_id == task_id
    assert node.cancel_pending is None
    assert node.emit.call_args.args[0] == 'rejected'


@pytest.mark.parametrize('task_id', [142, 143])
def test_busy_rejection_precedes_parameter_validation(node, task_id):
    node.receive(detection_task(task_id, ['person']))
    invalid = Task(task_id=140)  # Missing the required waypoint.
    node.receive(invalid)
    assert node.emit.call_args.args[0] == 'rejected'
    assert node.emit.call_args.kwargs['message'] == 'another task is still active'


@pytest.mark.parametrize('task_id', [142, 143])
def test_active_cancellation_retains_classes(node, task_id):
    node.receive(detection_task(task_id, ['person']))
    node.pump()
    node.cancel(Task(task_id=task_id, cancel=True))
    node.finish(False, 'cancel confirmed')
    assert read_targets(node.yolo_targets_path) == ['person']


@pytest.mark.parametrize('task_id', [142, 143])
def test_waiting_for_action_server_does_not_change_classes(node, task_id):
    node.action_clients[kind(task_id)].server_is_ready.return_value = False
    node.receive(detection_task(task_id, ['person']))
    node.pump()
    assert read_targets(node.yolo_targets_path) == ['truck']
    node.cancel(Task(task_id=task_id, cancel=True))
    assert read_targets(node.yolo_targets_path) == ['truck']


@pytest.mark.parametrize('task_id', [142, 143])
def test_file_write_failure_fails_before_sending_action(node, monkeypatch, task_id):
    monkeypatch.setattr(dispatcher, 'write_targets', Mock(side_effect=OSError('disk failure')))
    node.receive(detection_task(task_id, ['person']))
    node.pump()
    node.action_clients[kind(task_id)].send_goal_async.assert_not_called()
    assert node.current is None
    assert node.emit.call_args.args[0] == 'failed'
    assert 'target file update failed' in node.emit.call_args.kwargs['message']


@pytest.mark.parametrize('task_id', [142, 143])
def test_empty_selection_pauses_detector_without_blocking_task(node, task_id):
    node.receive(detection_task(task_id, []))
    node.pump()
    assert read_targets(node.yolo_targets_path) == []
    node.action_clients[kind(task_id)].send_goal_async.assert_called_once()


def test_default_file_path_is_shared_independent_of_namespace():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/separate_robot'])
    item = dispatcher.PipelineDispatcher()
    try:
        assert str(item.yolo_targets_path) == DEFAULT_TARGETS_FILE
    finally:
        item.destroy_node()
        rclpy.shutdown()


def test_143_search_mode_uses_area_action_and_writes_classes(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('inspection_search_mode', value=True)])
    task = detection_task(143, ['person'])
    task.center = [31., 120., 0.]; task.length_m = 10.; task.width_m = 8.
    node.receive(task); node.pump()
    node.action_clients['target_inspection'].send_goal_async.assert_not_called()
    goal = node.action_clients['search_inspection'].send_goal_async.call_args.args[0]
    assert goal.task_id == '143' and goal.area.length_m == 10.
    assert goal.area.width_m == 8. and goal.area.center.latitude == 31.
    assert read_targets(node.yolo_targets_path) == ['person']


def test_143_search_mode_rejects_missing_region_without_changing_classes(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('inspection_search_mode', value=True)])
    node.receive(detection_task(143, ['person'])); node.pump()
    assert node.current is None
    node.action_clients['search_inspection'].send_goal_async.assert_not_called()
    assert read_targets(node.yolo_targets_path) == ['truck']
