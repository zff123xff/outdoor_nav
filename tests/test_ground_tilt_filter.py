import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped
from robot_launch.ground_tilt_filter import TiltLowPass, GroundTiltFilter


def test_first_sample_initializes_without_flat_ground_ramp():
    f = TiltLowPass(.3, 1.0)
    assert f.update(.2, -.3, 10) == (.2, -.3)


@pytest.mark.parametrize('steps', [3, 30])
def test_step_response_uses_sample_time_not_callback_frequency(steps):
    f = TiltLowPass(.3, 1.0)
    f.update(0, 0, 10)
    for i in range(steps):
        result = f.update(.4, -.2, 10+.3*(i+1)/steps)
    assert result == pytest.approx((.4*(1-math.exp(-1)), -.2*(1-math.exp(-1))))


def test_duplicate_backward_and_long_gap_samples():
    f = TiltLowPass(.3, 1.0)
    f.update(.2, -.1, 10)
    assert f.update(1, 1, 10) == (.2, -.1)
    assert f.update(.4, .3, 9) == (.4, .3)
    assert f.update(.6, .5, 12) == (.6, .5)


def test_filter_uses_shortest_angle_across_wrap():
    f = TiltLowPass(.3, 1.0)
    f.update(math.pi-.01, 0, 1)
    roll, _ = f.update(-math.pi+.01, 0, 1.01)
    assert abs(roll-math.pi) < .02


def test_reference_keeps_position_timestamp_and_current_yaw():
    transform = TransformStamped()
    transform.header.frame_id = 'robot_1/map'
    transform.child_frame_id = 'robot_1/base_link'
    transform.header.stamp.sec = 10
    transform.transform.translation.x = 3.0
    transform.transform.translation.z = .2
    yaw = 1.2
    transform.transform.rotation.z = math.sin(yaw/2)
    transform.transform.rotation.w = math.cos(yaw/2)
    node = NS(world='robot_1/map', base='robot_1/base_link',
              output='robot_1/ground_height_reference', last_stamp=None,
              buffer=NS(lookup_transform=Mock(return_value=transform)),
              broadcaster=NS(sendTransform=Mock()), filter=TiltLowPass(.3, 1))
    node.filter.update(.4, -.2, 9.9)
    GroundTiltFilter.publish_reference(node)
    result = node.broadcaster.sendTransform.call_args.args[0]
    q = result.transform.rotation
    actual_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    assert actual_yaw == pytest.approx(yaw)
    assert result.transform.translation.x == 3
    assert result.transform.translation.z == .2
    assert result.header.stamp.sec == 10
    assert result.child_frame_id == node.output
    GroundTiltFilter.publish_reference(node)
    assert node.broadcaster.sendTransform.call_count == 1
