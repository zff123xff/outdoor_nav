"""Completion needs fresh native exhaustion, never a coverage shortcut."""
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock
from co_search_nav.executor import CoSearchExecutor


def node():
    return NS(task={'co_search': {'exhaustion_confirmations': 3}}, state='planning',
              finishing=None, enabled=True, metrics={'observed_ratio': .65},
              exhaustion_count=0, exhaustion_cloud=None, navigation_ready=lambda: True,
              stop=Mock())


def update(n, cloud, phase='exhausted', reachable=0):
    CoSearchExecutor.engine_state(n, NS(data=json.dumps(dict(
        phase=phase, clouds=cloud, reachable_frontiers=reachable))))


def test_native_exhaustion_returns_completed_below_full_coverage():
    n = node()
    for cloud in (10, 11): update(n, cloud)
    n.stop.assert_not_called()
    update(n, 12)
    assert n.stop.call_args.args[0] == 'completed'
    assert n.metrics['observed_ratio'] == .65


def test_same_cloud_and_reachable_viewpoint_do_not_confirm_exhaustion():
    n = node()
    for _ in range(5): update(n, 10)
    assert n.exhaustion_count == 1
    update(n, 11, reachable=1)
    assert n.exhaustion_count == 0
    n.stop.assert_not_called()


def test_planning_failure_breaks_exhaustion_confirmation_sequence():
    n = node(); update(n, 10); update(n, 11)
    update(n, 12, phase='planning')
    assert n.exhaustion_count == 0
    update(n, 13)
    n.stop.assert_not_called()
