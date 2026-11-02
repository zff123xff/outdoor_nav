"""Synthetic coordinate/timestamp checks; these do not measure Go2 accuracy."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from nav_msgs.msg import Odometry

spec = importlib.util.spec_from_file_location('lio_nav_odometry', Path(__file__).with_name('lio_nav_odometry.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.output = []
        self.transforms = []
        params = {'imu_xyz': [-0.02557, 0., 0.04232], 'initial_base_height': .30}
        self.node = SimpleNamespace(origin=None, previous=None,
            publisher=SimpleNamespace(publish=self.output.append),
            tf=SimpleNamespace(sendTransform=self.transforms.append),
            odom_frame='robot/odom', base_frame='robot/base_link',
            get_parameter=lambda name: SimpleNamespace(value=params[name]))

    def feed(self, seconds, position, yaw=0.):
        m = Odometry()
        m.header.stamp.sec = seconds
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = position
        m.pose.pose.orientation.z = math.sin(yaw/2)
        m.pose.pose.orientation.w = math.cos(yaw/2)
        module.LioNavOdometry.receive(self.node, m)

    def test_initial_frame_and_imu_lever_arm(self):
        self.feed(1, (8., 4., 2.), math.pi/2)
        out = self.output[-1]
        self.assertEqual(out.header.frame_id, 'robot/odom')
        self.assertEqual(out.child_frame_id, 'robot/base_link')
        self.assertAlmostEqual(out.pose.pose.position.x, 0.)
        self.assertAlmostEqual(out.pose.pose.position.z, .30)
        self.assertAlmostEqual(out.pose.pose.orientation.w, 1.)
        # North in the initial estimate becomes forward in the aligned frame.
        self.feed(2, (8., 5., 2.), math.pi/2)
        self.assertAlmostEqual(self.output[-1].pose.pose.position.x, 1.)
        self.assertAlmostEqual(self.output[-1].pose.pose.position.y, 0.)

    def test_twist_is_derived_from_estimated_pose(self):
        self.feed(1, (0., 0., 0.))
        m = Odometry()
        m.header.stamp.sec = 1
        m.header.stamp.nanosec = 100000000
        m.pose.pose.position.x = .02
        m.pose.pose.orientation.w = 1.
        module.LioNavOdometry.receive(self.node, m)
        self.assertAlmostEqual(self.output[-1].twist.twist.linear.x, .2)
        self.assertEqual(self.output[-1].header.stamp.nanosec, 100000000)

    def test_rejects_nonfinite_quaternion_and_old_timestamps(self):
        self.feed(2, (0., 0., 0.))
        self.feed(1, (10., 0., 0.))
        self.feed(2, (20., 0., 0.))
        self.feed(3, (float('nan'), 0., 0.))
        m = Odometry()
        m.header.stamp.sec = 3
        m.pose.pose.orientation.w = 0.
        module.LioNavOdometry.receive(self.node, m)
        self.assertEqual(len(self.output), 1)


if __name__ == '__main__':
    unittest.main()
