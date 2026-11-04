"""Point progress can use fresh ENU pose without any GPS fix."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus
from patrol_node.point_goal_node import PointGoalNode
from task_pipeline.dispatcher import PipelineDispatcher


def point_node():
    node = NS(_target_enu=(10., 0., 0.), _progress_start_xy=None,
              _progress_total_m=None, xy=(0., 0.))
    node._robot_xy = lambda: node.xy
    return node


def test_enu_progress_uses_start_displacement_and_target_in_same_frame():
    node = point_node()
    assert PointGoalNode._enu_progress(node) == 0.
    node.xy = (5., 0.)
    assert PointGoalNode._enu_progress(node) == 50.
    node.xy = (10., 0.)
    assert PointGoalNode._enu_progress(node) == 99.  # Await Action result for 100.


def test_stale_or_missing_pose_does_not_manufacture_progress():
    node = point_node()
    node.xy = None
    assert PointGoalNode._enu_progress(node) == -1.
    assert node._progress_start_xy is None


@pytest.mark.parametrize('gps', [None, (NavSatFix(), time.monotonic()-3.)])
def test_dispatcher_uses_enu_when_gps_absent_or_stale(gps):
    node = NS(current=NS(task_type='point_goal'), fix=gps, progress=0)
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=50.)))
    assert node.progress == 50


def test_valid_gps_has_priority_and_progress_never_regresses():
    node = NS(current=NS(task_type='point_goal'), fix=(NavSatFix(), time.monotonic()), progress=60)
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=80.)))
    assert node.progress == 60
    node.fix = None
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=30.)))
    assert node.progress == 60
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=-1.)))
    assert node.progress == 60


def test_invalid_fix_immediately_allows_enu_feedback():
    node = NS(current=NS(task_type='point_goal'), fix=(NavSatFix(), time.monotonic()), progress=0)
    PipelineDispatcher.on_fix(node, NavSatFix(status=NavSatStatus(status=-1)))
    assert node.fix is None
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=25.)))
    assert node.progress == 25
