"""Lost lifecycle replies must never cause a transition to be replayed."""
import threading
from types import SimpleNamespace
import unittest

from lifecycle_msgs.srv import ChangeState
from robot_launch.lifecycle_service_relay import LifecycleRelay


class LifecycleReplyTests(unittest.TestCase):
    def fixture(self, states):
        iterator = iter(states)
        calls = []
        relay = SimpleNamespace(
            rpc_clients={'controller': (object(), object(), threading.Lock())},
            timeout=.1, state=lambda name: next(iterator),
            call=lambda *args: calls.append(args),
            get_logger=lambda: SimpleNamespace(warning=lambda text: None, error=lambda text: None))
        request = ChangeState.Request()
        request.transition.id = 1  # configure: target inactive (2)
        return relay, calls, request

    def test_lost_reply_is_accepted_only_after_target_is_verified(self):
        relay, calls, request = self.fixture([SimpleNamespace(id=1), SimpleNamespace(id=2)])
        response = LifecycleRelay.change(relay, 'controller', request, ChangeState.Response())
        self.assertTrue(response.success)
        self.assertEqual(len(calls), 1)

    def test_unknown_target_fails_without_replaying_transition(self):
        relay, calls, request = self.fixture([SimpleNamespace(id=1), None])
        response = LifecycleRelay.change(relay, 'controller', request, ChangeState.Response())
        self.assertFalse(response.success)
        self.assertEqual(len(calls), 1)

    def test_missing_initial_state_sends_no_transition(self):
        relay, calls, request = self.fixture([None])
        response = LifecycleRelay.change(relay, 'controller', request, ChangeState.Response())
        self.assertFalse(response.success)
        self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
