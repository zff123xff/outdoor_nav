# Copyright 2026 outdoor_nav contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the ping-pong waypoint queue."""

import pytest

from patrol_node.waypoints import GeoPoint, PingPongQueue, nearest_endpoint


def _points():
    a = GeoPoint(30.0000000, 120.0000000, 12.0)
    b = GeoPoint(30.0010000, 120.0000000, 12.0)
    return a, b


def test_queue_starts_with_both_endpoints():
    a, b = _points()
    queue = PingPongQueue(a, b)
    assert len(queue) == 2


def test_targets_alternate_between_the_two_endpoints():
    a, b = _points()
    queue = PingPongQueue(a, b)

    targets = []
    for _ in range(6):
        target = queue.take_next()
        targets.append(target)
        queue.on_reached(target)

    assert targets == [a, b, a, b, a, b]


def test_queue_never_grows():
    """The queue stays bounded: a full take/push cycle leaves it unchanged."""
    a, b = _points()
    queue = PingPongQueue(a, b)
    initial = len(queue)

    for _ in range(10):
        target = queue.take_next()
        queue.on_reached(target)
        assert len(queue) == initial


def test_clear_drops_pending_targets():
    a, b = _points()
    queue = PingPongQueue(a, b)
    queue.take_next()
    queue.clear()
    assert len(queue) == 0


@pytest.mark.parametrize('robot_xy,expected', [((0, 1), 0), ((0, 9), 1),
                                             ((0, 5), 0), ((0, 10), 1)])
def test_nearest_endpoint_uses_current_horizontal_position(robot_xy, expected):
    assert nearest_endpoint(robot_xy, (0, 0), (0, 10)) == expected


def test_round_trip_from_b_returns_to_b():
    a, b = _points()
    queue = PingPongQueue(a, b, start_at_b=True)
    targets = []
    for _ in range(3):
        target = queue.take_next()
        targets.append(target)
        queue.on_reached(target)
    assert targets == [b, a, b]


def test_invalid_position_is_not_used_to_choose_an_endpoint():
    with pytest.raises(ValueError, match='finite'):
        nearest_endpoint((float('nan'), 0), (0, 0), (0, 10))
