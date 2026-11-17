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


def test_enu_progress_uses_remaining_distance_to_target_in_same_frame():
    node = point_node()
    assert PointGoalNode._enu_progress(node) == 0.
    node.xy = (5., 0.)
    assert PointGoalNode._enu_progress(node) == 50.
    node.xy = (10., 0.)
    assert PointGoalNode._enu_progress(node) == 99.  # Await Action result for 100.


@pytest.mark.parametrize('xy', [(-5., 0.), (0., 5.), (0., 10.), (25., 0.)])
def test_enu_motion_without_approaching_target_does_not_increase_progress(xy):
    node = point_node()
    PointGoalNode._enu_progress(node)
    node.xy = xy
    assert PointGoalNode._enu_progress(node) == 0.


def test_enu_detour_counts_only_remaining_distance_reduction():
    node = point_node()
    PointGoalNode._enu_progress(node)
    node.xy = (7., 4.)  # Five meters remaining, despite a longer displacement.
    assert PointGoalNode._enu_progress(node) == 50.


def test_enu_zero_initial_distance_waits_for_action_result():
    node = point_node()
    node.xy = (10., 0.)
    assert PointGoalNode._enu_progress(node) == 0.
    node.xy = (9., 0.)
    assert PointGoalNode._enu_progress(node) == 0.


def gps_fix(latitude=0., longitude=0.):
    return NavSatFix(latitude=latitude, longitude=longitude)


def gps_node():
    return NS(current=NS(task_id='140', waypoints=[0., .001, 0.]),
              fix=None, start=None, initial_distance=None, progress=0)


@pytest.mark.parametrize('latitude,longitude', [(0., -.0005), (.0005, 0.),
                                              (.001, 0.), (0., .0025)])
def test_gps_motion_without_approaching_target_does_not_increase_progress(latitude, longitude):
    node = gps_node()
    PipelineDispatcher.on_fix(node, gps_fix())
    PipelineDispatcher.on_fix(node, gps_fix(latitude, longitude))
    assert node.progress == 0


def test_gps_progress_tracks_approach_and_holds_during_detour():
    node = gps_node()
    PipelineDispatcher.on_fix(node, gps_fix())
    PipelineDispatcher.on_fix(node, gps_fix(longitude=.0005))
    assert node.progress == 50
    PipelineDispatcher.on_fix(node, gps_fix(latitude=.001, longitude=.0005))
    assert node.progress == 50
    PipelineDispatcher.on_fix(node, gps_fix(longitude=.001))
    assert node.progress == 99


def test_gps_zero_initial_distance_waits_for_action_result():
    node = gps_node()
    PipelineDispatcher.on_fix(node, gps_fix(longitude=.001))
    PipelineDispatcher.on_fix(node, gps_fix())
    assert node.progress == 0


def test_stale_or_missing_pose_does_not_manufacture_progress():
    node = point_node()
    node.xy = None
    assert PointGoalNode._enu_progress(node) == -1.
    assert node._progress_start_xy is None


@pytest.mark.parametrize('gps', [None, (NavSatFix(), time.monotonic()-3.)])
def test_dispatcher_uses_enu_when_gps_absent_or_stale(gps):
    node = NS(current=NS(task_id='140'), fix=gps, progress=0)
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=50.)))
    assert node.progress == 50


def test_valid_gps_has_priority_and_progress_never_regresses():
    node = NS(current=NS(task_id='140'), fix=(NavSatFix(), time.monotonic()), progress=60)
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=80.)))
    assert node.progress == 60
    node.fix = None
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=30.)))
    assert node.progress == 60
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=-1.)))
    assert node.progress == 60


def test_invalid_fix_immediately_allows_enu_feedback():
    node = NS(current=NS(task_id='140'), fix=(NavSatFix(), time.monotonic()), progress=0)
    PipelineDispatcher.on_fix(node, NavSatFix(status=NavSatStatus(status=-1)))
    assert node.fix is None
    PipelineDispatcher.feedback(node, NS(feedback=NS(progress=25.)))
    assert node.progress == 25
