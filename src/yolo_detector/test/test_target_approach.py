"""Stand-off geometry and stopped-state evidence for simulation inspection."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from yolo_detector.target_approach import TargetApproach, approach_pose, angle_error


@pytest.mark.parametrize('robot,target,expected', [
    ((0., 0.), (10., 0.), (5.5, 0., 0.)),
    ((0., 0.), (0., 10.), (0., 5.5, math.pi/2)),
    ((2., 0.), (5., 0.), (2., 0., 0.)),
])
def test_standoff_and_heading(robot, target, expected):
    assert approach_pose(robot, target, 4.5) == pytest.approx(expected)


def test_coincident_target_does_not_invent_heading():
    with pytest.raises(ValueError):
        approach_pose((1., 1.), (1., 1.), 4.5)
    assert angle_error(-math.pi+.01, math.pi-.01) == pytest.approx(.02)


def test_standoff_accounts_for_target_height():
    x, y, yaw = approach_pose((0., 0.), (10., 0.), 4.5, height_difference=2.)
    assert math.sqrt((10.-x)**2 + 2.**2) == pytest.approx(4.5)
    with pytest.raises(ValueError, match='height'):
        approach_pose((0., 0.), (10., 0.), 4.5, height_difference=6.)


@pytest.mark.parametrize('phase,requested_id,position,yaw,stopped,expected', [
    ('approaching', 1, 0., 0., True, False),
    ('awaiting_confirmation', 2, 0., 0., True, False),
    ('awaiting_confirmation', 1, -1., 0., True, False),
    ('awaiting_confirmation', 1, 0., 1., True, False),
    ('awaiting_confirmation', 1, 0., 0., False, False),
    ('awaiting_confirmation', 1, 0., 0., True, True),
])
def test_confirmation_requires_stopped_aligned_matching_target_within_3d_range(
        phase, requested_id, position, yaw, stopped, expected):
    approach = SimpleNamespace(busy=True, phase=phase, goal=SimpleNamespace(request=SimpleNamespace(target_id=1)),
        suspended=lambda: True, target=lambda target_id: (4.5, 0., 1.),
        robot_pose=lambda: (position, 0., yaw, 1, 0.), sample_stopped=lambda pose, now: stopped,
        settings=dict(approach_max_distance_m=5., approach_yaw_tolerance_rad=.15), stop=Mock())
    assert TargetApproach.confirm(approach, requested_id) is expected
    assert approach.stop.called is expected


def test_stopped_requires_continuous_fresh_pose_updates_and_rejects_rotation():
    node = SimpleNamespace(tf=Mock())
    node.tf.robot_update_age.return_value = .01
    approach = SimpleNamespace(previous_pose=None, stationary_since=None, node=node,
        settings=dict(approach_stop_speed_mps=.05, approach_stop_yaw_rate_rps=.05,
                      approach_stop_hold_s=.5))
    sample = lambda pose, now: TargetApproach.sample_stopped(approach, pose, now)
    assert not sample((0., 0., 0., 1), 0.)
    assert not sample((0., 0., 0., 2), .1)
    assert sample((0., 0., 0., 3), .7)
    assert not sample((0., 0., .2, 4), .8)
    assert not sample((0., 0., .2, 5), .9)
    node.tf.robot_update_age.return_value = .6
    assert not sample((0., 0., .2, 5), 1.5)
