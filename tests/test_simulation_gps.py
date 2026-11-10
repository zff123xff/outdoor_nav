"""Simulation GPS uses the navigation projection and the selected YAML origin."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
import yaml
from launch import LaunchContext
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from multi_goal_to_gp_goal.coordinates import enu_to_lla, lla_to_enu
from mavros_odom_relay.mavros_odom_relay import OdomRelay
from robot_launch.bringup import external


@pytest.mark.parametrize('origin', [(40.9445353, 113.172819, 1298.7345711102703),
                                   (0., 179.999, 10.), (-31., -120., -5.)])
@pytest.mark.parametrize('enu', [(0., 0., 0.), (10., 20., 3.), (-30., -50., -4.), (5000., 0., 0.)])
def test_projection_round_trip_matches_navigation(origin, enu):
    lla = enu_to_lla(*enu, *origin)
    assert lla_to_enu(*lla, *origin) == pytest.approx(enu, abs=1e-6)
    assert -180. <= lla[1] <= 180.


def relay(enabled=True):
    return NS(_gps_pub=Mock() if enabled else None, _pub=Mock(),
              _origin=(40., 113., 1200.), get_namespace=lambda: '/robot_2',
              get_parameter=lambda key: NS(value=''), get_logger=lambda: Mock())


def test_fix_preserves_timestamp_height_and_sensor_frame():
    node = relay()
    msg = Odometry()
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.header.frame_id = 'robot_2/odom'
    msg.child_frame_id = 'robot_2/base_link'
    msg.pose.pose.position.z = 3.
    OdomRelay._on_odom(node, msg)
    node._pub.publish.assert_called_once_with(msg)
    fix = node._gps_pub.publish.call_args.args[0]
    assert (fix.latitude, fix.longitude, fix.altitude) == (40., 113., 1203.)
    assert fix.header.stamp == msg.header.stamp
    assert fix.header.frame_id == msg.child_frame_id
    assert fix.status.status == NavSatStatus.STATUS_FIX
    assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_invalid_position_still_relays_odom_but_does_not_publish_fix(value):
    node = relay()
    msg = Odometry()
    msg.pose.pose.position.x = value
    OdomRelay._on_odom(node, msg)
    node._pub.publish.assert_called_once_with(msg)
    node._gps_pub.publish.assert_not_called()


def test_real_mode_only_relays_odom():
    node = relay(enabled=False)
    msg = Odometry()
    OdomRelay._on_odom(node, msg)
    node._pub.publish.assert_called_once_with(msg)


@pytest.mark.parametrize('namespaced', [False, True])
def test_launch_reads_custom_yaml_origin(tmp_path, namespaced):
    path = tmp_path/'custom.yaml'
    origin = dict(reference_latitude=31.25, reference_longitude=120.5, reference_altitude=42.)
    key = '/robot_2/nav2_input_bridge' if namespaced else 'nav2_input_bridge'
    path.write_text(yaml.safe_dump({key: {'ros__parameters': origin}}))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(path), robot_namespace='robot_2', use_sim_time='true')
    with patch.object(external, 'Node') as node:
        external._odom_relay(context)
    options = node.call_args.kwargs
    assert options['namespace'] == 'robot_2'
    params = options['parameters'][0]
    assert params['publish_gps'] is True
    assert all(params[name] == value for name, value in origin.items())


def test_launch_does_not_fall_back_to_zero_origin(tmp_path):
    path = tmp_path/'missing.yaml'
    path.write_text('{}')
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(path), robot_namespace='robot_2', use_sim_time='false')
    with pytest.raises(RuntimeError, match='reference_latitude'):
        external._odom_relay(context)
