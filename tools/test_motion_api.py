#!/usr/bin/env python3
"""运动协议回归：不依赖 rclpy/grpc。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from messages import Motion, Velocity  # noqa: E402
from motion.api import MotionCancelled, MotionProgress  # noqa: E402
from motion.dry_run import DryRunRobot  # noqa: E402


class MotionApiTest(unittest.TestCase):
    def test_dry_run_reports_progress(self):
        robot = DryRunRobot()
        call = robot.execute_stream(Velocity(0, 0, 0, 0.01))
        progress = list(call)
        self.assertEqual(len(progress), 1)
        self.assertIsInstance(progress[0], MotionProgress)
        self.assertTrue(progress[0].is_final)
        self.assertTrue(progress[0].success)

    def test_distance_motion_updates_position_in_current_heading(self):
        robot = DryRunRobot()
        robot._yaw = 1.5707963267948966
        # 不等待真实时长；仍通过模拟后端的完成回调更新位姿。
        cmd = Motion("line_walk", (("direction", 0), ("distance", 0.4)), duration=0)
        list(robot.execute_stream(cmd))
        self.assertAlmostEqual(robot.get_pose().x, 0, places=6)
        self.assertAlmostEqual(robot.get_pose().y, 0.4, places=6)

    def test_cancel_raises_motion_cancelled(self):
        robot = DryRunRobot()
        call = robot.execute_stream(Velocity(0, 0, 0, 10.0))
        call.cancel()
        with self.assertRaises(MotionCancelled):
            list(call)

    def test_dry_run_rotation_updates_yaw(self):
        robot = DryRunRobot()
        list(robot.execute_stream(Motion("rotation", (("direction", 0), ("angle", 90.0)))))
        self.assertAlmostEqual(robot.get_pose().yaw_rad, 1.5708, places=3)


if __name__ == "__main__":
    unittest.main()
