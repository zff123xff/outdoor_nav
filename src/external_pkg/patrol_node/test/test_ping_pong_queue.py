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

from patrol_node.waypoints import GeoPoint, PingPongQueue


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
