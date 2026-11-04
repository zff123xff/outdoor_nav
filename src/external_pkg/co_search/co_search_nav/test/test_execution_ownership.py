"""Native backend preemption and stop preserve shared FollowPath ownership."""
from types import SimpleNamespace, MethodType
from co_search_nav.executor import CoSearchExecutor


class Future:
    def __init__(self, result=None):
        self.value = result; self.callbacks = []
    def result(self): return self.value
    def add_done_callback(self, callback): self.callbacks.append(callback)
    def complete(self, result):
        self.value = result
        for callback in self.callbacks: callback(self)


class Handle:
    accepted = True
    def __init__(self): self.cancels = 0; self.future = Future()
    def get_result_async(self): return self.future
    def cancel_goal_async(self): self.cancels += 1


def executor():
    instance = SimpleNamespace(task={'mission_id': 'test'}, state='executing', generation=1, sequence=2,
        handles={}, pending={}, finishing=None, enabled=True, reports=[], active=[], enable=[], cleaned=False)
    def report(state, reason=''):
        instance.state = state; instance.reports.append((state, reason))
    instance.report = report
    instance.active_pub = SimpleNamespace(publish=lambda msg: instance.active.append(msg.data))
    instance.enable_pub = SimpleNamespace(publish=lambda msg: instance.enable.append(msg.data))
    instance.cleanup_process = lambda: setattr(instance, 'cleaned', True)
    for name in ('stop', 'finish_if_stopped', 'accepted', 'result'):
        setattr(instance, name, MethodType(getattr(CoSearchExecutor, name), instance))
    return instance


def test_superseded_abort_does_not_fail_the_current_path():
    node = executor(); old, current = Handle(), Handle()
    node.handles = {id(old): old, id(current): current}
    node.result(Future(SimpleNamespace(status=6)), old, 1, 1)
    assert node.state == 'executing' and not node.finishing
    assert id(current) in node.handles and not node.active
    node.result(Future(SimpleNamespace(status=4)), current, 1, 2)
    assert node.state == 'planning' and not node.active  # mission retains navigation


def test_cancel_waits_for_old_goal_and_inflight_replacement_to_reach_terminal():
    node = executor(); old = Handle(); node.handles[id(old)] = old
    pending = Future(); node.pending[2] = pending
    node.stop('cancelled', 'cancel task')
    assert old.cancels == 1 and node.state == 'stopping' and node.active == []
    node.result(Future(SimpleNamespace(status=5)), old, 1, 1)
    assert not node.cleaned and node.active == []
    replacement = Handle(); pending.value = replacement
    node.accepted(pending, 1, 2)
    assert replacement.cancels == 1 and not node.cleaned
    replacement.future.complete(SimpleNamespace(status=5))
    assert node.state == 'cancelled' and node.cleaned and node.active == [False]


def test_old_generation_result_cannot_advance_a_new_task():
    node = executor(); old = Handle(); node.handles[id(old)] = old
    node.result(Future(SimpleNamespace(status=4)), old, 0, 2)
    assert node.state == 'executing' and node.reports == []


def test_rejected_latest_path_fails_task_and_cancels_remaining_owned_goals():
    node = executor(); old = Handle(); node.handles[id(old)] = old
    future = Future(SimpleNamespace(accepted=False)); node.pending[2] = future
    node.accepted(future, 1, 2)
    assert node.finishing[0] == 'failed' and old.cancels == 1
    node.result(Future(SimpleNamespace(status=5)), old, 1, 1)
    assert node.state == 'failed' and node.active == [False]
