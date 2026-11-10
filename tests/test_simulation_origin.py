"""Background origin synchronization and GPS conversion caches; no movement."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import NavSatFix, NavSatStatus

from robot_launch.simulation_origin import ORIGIN_PARAMETERS, validate_origin
from robot_launch.simulation_origin_receiver import SimulationOriginReceiver
from mavros_odom_relay.mavros_odom_relay import OdomRelay
from co_search_nav.executor import CoSearchExecutor


@pytest.mark.parametrize('values', [(91., 0., 0.), (0., -181., 0.),
                                   (float('nan'), 0., 0.), (0., 0., float('inf')),
                                   (True, 0., 0.), ('31', 120., 0.)])
def test_invalid_origin_is_rejected(values):
    with pytest.raises(ValueError):
        validate_origin(*values)


@pytest.fixture
def receiver():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/origin_fixture', '-p', 'target_nodes:=[sink]'])
    node = None
    try:
        node = SimulationOriginReceiver()
        client = node.origin_clients['sink']
        client.service_is_ready = Mock(return_value=True)
        client.call_async = Mock(side_effect=lambda request: Future())
        node.publisher.publish = Mock()
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def message(values=(31., 120., 15.)):
    return NavSatFix(latitude=values[0], longitude=values[1], altitude=values[2])


def ack(node, successful=True):
    node.pending['sink'][0].set_result(SimpleNamespace(
        result=SetParametersResult(successful=successful, reason='fixture')))
    node.synchronize()


def test_no_origin_never_blocks_startup_or_writes_parameters(receiver):
    receiver.synchronize()
    receiver.origin_clients['sink'].call_async.assert_not_called()
    assert receiver.desired_origin is None


def test_three_coordinates_are_atomic_and_output_waits_for_ack(receiver):
    receiver.receive(message())
    receiver.synchronize()
    request = receiver.origin_clients['sink'].call_async.call_args.args[0]
    assert [p.name for p in request.parameters] == list(ORIGIN_PARAMETERS)
    assert [p.value.double_value for p in request.parameters] == [31., 120., 15.]
    receiver.publisher.publish.assert_not_called()
    ack(receiver)
    assert receiver.published_origin == (31., 120., 15.)
    receiver.publisher.publish.assert_called_once()


def test_latest_origin_wins_over_an_older_inflight_response(receiver):
    receiver.receive(message())
    receiver.synchronize()
    receiver.receive(message((32., 121., 16.)))
    ack(receiver)
    receiver.publisher.publish.assert_not_called()
    assert receiver.pending['sink'][1] == (32., 121., 16.)
    ack(receiver)
    assert receiver.published_origin == (32., 121., 16.)


def test_failed_update_retries_without_announcing_success(receiver):
    receiver.receive(message())
    receiver.synchronize()
    ack(receiver, False)
    receiver.publisher.publish.assert_not_called()
    assert 'sink' in receiver.pending
    ack(receiver)
    assert receiver.published_origin == (31., 120., 15.)


def test_late_starting_target_and_restart_are_resynchronized(receiver):
    client = receiver.origin_clients['sink']
    client.service_is_ready.return_value = False
    receiver.receive(message())
    receiver.synchronize()
    client.call_async.assert_not_called()
    client.service_is_ready.return_value = True
    receiver.synchronize()
    ack(receiver)
    receiver.last_sync['sink'] = 0.
    receiver.synchronize()
    assert 'sink' in receiver.pending


def test_invalid_or_repeated_messages_do_not_replace_valid_origin(receiver):
    receiver.receive(message())
    invalid = message((32., 121., 16.))
    invalid.status.status = NavSatStatus.STATUS_NO_FIX
    receiver.receive(invalid)
    receiver.receive(message((float('nan'), 121., 16.)))
    receiver.receive(message())
    assert receiver.desired_origin == (31., 120., 15.)


def test_relay_updates_its_actual_projection_cache_atomically():
    node = SimpleNamespace(_origin=(40., 113., 1200.))
    result = OdomRelay._on_origin_parameters(node, [
        Parameter(key, value=value) for key, value in zip(ORIGIN_PARAMETERS, (31., 120., 15.))])
    assert result.successful and node._origin == (31., 120., 15.)
    result = OdomRelay._on_origin_parameters(node, [Parameter('reference_latitude', value=91.)])
    assert not result.successful and node._origin == (31., 120., 15.)


def test_area_origin_updates_without_reprojecting_current_region():
    region = object()
    node = SimpleNamespace(origin=(40., 113., 1200.), region=region)
    CoSearchExecutor.update_origin(node, message())
    assert node.origin == (31., 120., 15.) and node.region is region
    CoSearchExecutor.update_origin(node, message((91., 120., 15.)))
    assert node.origin == (31., 120., 15.)
