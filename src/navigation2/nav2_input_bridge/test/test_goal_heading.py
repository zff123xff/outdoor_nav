"""Exercise the real bridge on unique topics, with TF isolated from live robots.

No controller is launched and no velocity or real robot TF is published.
"""
import math
import os
import subprocess
import tempfile
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('bearing', [True, False])
def test_heading_at_dispatch(bearing):
    executable = os.environ['INPUT_BRIDGE_EXECUTABLE']
    prefix = '/heading_test_' + uuid.uuid4().hex
    rclpy.init()
    node = rclpy.create_node('probe', namespace=prefix)
    received = []
    node.create_subscription(
        PoseStamped, prefix + '/goal_pose', received.append, 10)
    goals = node.create_publisher(NavSatFix, prefix + '/gp_goal', 10)
    transforms = node.create_publisher(TFMessage, prefix + '/tf', 10)
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([
        executable, '--ros-args', '-r', '__ns:=' + prefix,
        '-r', '/tf:=' + prefix + '/tf',
        '-r', '/tf_static:=' + prefix + '/tf_static',
        '-p', 'goal_output_frame:=' + prefix[1:] + '/map',
        '-p', 'goal_yaw_from_bearing:=' + str(bearing).lower(),
    ], stdout=log, stderr=log)

    def spin_until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, 'bridge exited unexpectedly'
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def send_goal(expect_output=True):
        received.clear()
        goal = NavSatFix()
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.status.status = NavSatStatus.STATUS_FIX
        # Reference and goal are both (0, 0, 0): target ENU is exactly zero.
        goals.publish(goal)
        if expect_output:
            assert spin_until(lambda: bool(received)), 'no converted goal received'
            return received[-1]
        assert not spin_until(lambda: bool(received), timeout=0.35)

    def set_robot(x, y, yaw=0.0, stale=False):
        transform = TransformStamped()
        transform.header.frame_id = prefix[1:] + '/map'
        transform.child_frame_id = prefix[1:] + '/base_link'
        stamp = node.get_clock().now()
        if stale:
            stamp = stamp - Duration(seconds=10)
        transform.header.stamp = stamp.to_msg()
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        transforms.publish(TFMessage(transforms=[transform]))
        # Allow the separate bridge executor to ingest the transform first.
        spin_until(lambda: False, timeout=0.15)

    def assert_yaw(pose, expected):
        q = pose.pose.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        error = math.atan2(math.sin(yaw - expected), math.cos(yaw - expected))
        assert abs(error) < 1e-6

    try:
        assert spin_until(lambda: goals.get_subscription_count() > 0)
        assert spin_until(lambda: node.count_publishers(prefix + '/goal_pose') > 0)
        if not bearing:
            # Existing fixed-yaw behavior stays available without TF.
            assert_yaw(send_goal(), 0.0)
            return
        assert spin_until(lambda: transforms.get_subscription_count() > 0)
        send_goal(expect_output=False)  # Never invent a bearing without pose.
        set_robot(-10, 0, stale=True)
        send_goal(expect_output=False)
        for x, y, yaw in [
            (-10, 0, 0), (0, -10, math.pi / 2),
            (10, 0, math.pi), (0, 10, -math.pi / 2),
            (-10, -10, math.pi / 4),
        ]:
            set_robot(x, y)
            assert_yaw(send_goal(), yaw)
        set_robot(0, 0, yaw=1.2)
        assert_yaw(send_goal(), 1.2)  # Coincident goal keeps robot heading.
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.seek(0)
        print(log.read())
        log.close()
