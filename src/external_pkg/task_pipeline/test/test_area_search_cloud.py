"""Exact-time point cloud TF may arrive after the sensor callback."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformException
from task_pipeline.area_search import AreaSearch


def cloud_node(monkeypatch):
    monkeypatch.setattr('task_pipeline.area_search.point_cloud2.read_points', lambda *a, **k: [])
    return NS(
        grid=NS(updates=0, seed_footprint=Mock()), state='entering',
        pending_cloud=None, last_cloud=0, last_map_update=0, last_cloud_error='',
        task={'options': {'map_update_period_s': 0.8, 'sensor_timeout_s': 3.0}},
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=101_000_000_000)),
        tf=NS(lookup_transform=Mock(return_value=object())), frame='robot_1/map', base='robot_1/base_link',
        height_frame='robot_1/ground_height_reference',
        pose=lambda: (1, 2, 0), get_logger=lambda: NS(debug=Mock()),
    )


def scan(seconds=100):
    msg = PointCloud2()
    msg.header.stamp.sec = seconds
    msg.header.frame_id = 'robot_1/livox_frame'
    return msg


def test_tf_lag_retries_same_scan_without_claiming_freshness(monkeypatch):
    node = cloud_node(monkeypatch)
    node.tf.lookup_transform.side_effect = [TransformException('future extrapolation'), object(), object()]
    first = scan()
    AreaSearch.cloud(node, first)
    AreaSearch.process_cloud(node)
    assert node.pending_cloud is first
    assert node.last_cloud == 0
    assert node.grid.updates == 0
    AreaSearch.cloud(node, scan(101))
    assert node.pending_cloud is first  # newer scans cannot starve pending TF
    AreaSearch.process_cloud(node)
    assert node.pending_cloud is None
    assert node.last_cloud > 0
    assert node.grid.updates == 1
    assert node.last_cloud_error == ''
    assert node.tf.lookup_transform.call_args_list[0] == node.tf.lookup_transform.call_args_list[1]


@pytest.mark.parametrize('stamp', [97, 102])
def test_expired_or_future_scan_is_discarded_without_refreshing_watchdog(monkeypatch, stamp):
    node = cloud_node(monkeypatch)
    AreaSearch.cloud(node, scan(stamp))
    AreaSearch.process_cloud(node)
    assert node.pending_cloud is None
    assert node.last_cloud == 0
    assert node.grid.updates == 0
    node.tf.lookup_transform.assert_not_called()
    assert 'timestamp out of range' in node.last_cloud_error


def test_pause_discards_pending_scan(monkeypatch):
    node = cloud_node(monkeypatch)
    node.pending_cloud = scan()
    node.state = 'paused'
    AreaSearch.process_cloud(node)
    assert node.pending_cloud is None
    assert node.last_cloud == 0


def test_terminal_cancellation_feedback_preserves_original_timeout_reason():
    node = NS(state='paused', interruption_reason='sensor data timeout: waiting for TF', report=Mock())
    AreaSearch._finish_interrupted_action(node, 'FollowPath cancellation reached terminal state')
    assert 'sensor data timeout: waiting for TF' in node.report.call_args.args[1]
