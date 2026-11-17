"""Monitor localization updates without comparing clocks on different hosts."""

import time

from rclpy.time import Time
from tf2_ros import Buffer, TransformException


class LiveRobotTransformBuffer(Buffer):
    """Expire a robot transform when its latest common timestamp stops advancing."""

    def __init__(self, global_frame, base_frame):
        super().__init__()
        self._global_frame = global_frame
        self._base_frame = base_frame
        self._robot_stamp_ns = None
        self._robot_update_monotonic = None

    def set_transform(self, transform, authority):
        """Record progress only when a dynamic TF message reaches this buffer."""
        super().set_transform(transform, authority)
        try:
            robot = self.lookup_transform(self._global_frame, self._base_frame, Time())
        except TransformException:
            return
        stamp_ns = Time.from_msg(robot.header.stamp).nanoseconds
        if self._robot_stamp_ns is None or stamp_ns > self._robot_stamp_ns:
            self._robot_stamp_ns = stamp_ns
            self._robot_update_monotonic = time.monotonic()

    def robot_update_age(self):
        """Return seconds since the latest common robot TF timestamp advanced."""
        if self._robot_update_monotonic is None:
            return None
        return time.monotonic() - self._robot_update_monotonic
