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

"""
Pure waypoint helpers of the ping-pong patrol.

Kept free of ROS imports (and of the generated ``patrol_node/action``
package) so that the queue logic can be unit tested straight from the
source tree, without building the package first.
"""

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class GeoPoint:
    """A GPS waypoint."""

    latitude: float
    longitude: float
    altitude: float


class PingPongQueue:
    """
    Waypoint queue that reproduces a two-point patrol.

    The point that was just reached is pushed back onto the tail of the
    queue.  Starting from ``[A, B]`` that makes the queue alternate between
    holding ``[B]`` and ``[A]``, so the dispatched targets are
    ``A, B, A, B ...`` and the queue never grows.
    """

    def __init__(self, point_a: GeoPoint, point_b: GeoPoint) -> None:
        self._queue: deque = deque((point_a, point_b))

    def __len__(self) -> int:
        return len(self._queue)

    def take_next(self) -> GeoPoint:
        """Pop the next target off the head of the queue."""
        return self._queue.popleft()

    def on_reached(self, point: GeoPoint) -> None:
        """Push the point that was just reached back onto the tail."""
        self._queue.append(point)

    def clear(self) -> None:
        """Drop every pending target."""
        self._queue.clear()
