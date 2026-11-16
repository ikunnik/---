#!/usr/bin/env python3
"""ROS2 后端纯逻辑测试：不依赖 rclpy/机器人。"""
import math
import os
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from actions import Actions  # noqa: E402
from config import CFG  # noqa: E402
from ros2_robot_link import _clamp_vx, _quat_yaw, _wrap_pi  # noqa: E402


class Ros2HelpersTest(unittest.TestCase):
    def test_wrap_pi(self):
        self.assertAlmostEqual(_wrap_pi(0.0), 0.0)
        self.assertAlmostEqual(abs(_wrap_pi(math.pi)), math.pi, places=6)
        self.assertAlmostEqual(abs(_wrap_pi(-math.pi)), math.pi, places=6)
        self.assertAlmostEqual(_wrap_pi(3 * math.pi / 2), -math.pi / 2)

    def test_quat_yaw(self):
        self.assertAlmostEqual(_quat_yaw(0, 0, 0, 1), 0.0)
        half = math.radians(90) / 2
        self.assertAlmostEqual(
            _quat_yaw(0, 0, math.sin(half), math.cos(half)),
            math.radians(90),
        )

    def test_clamp_vx(self):
        self.assertEqual(_clamp_vx(0.0, 0.3), 0.0)
        self.assertAlmostEqual(_clamp_vx(0.1, 0.3), 0.3)
        self.assertAlmostEqual(_clamp_vx(-0.1, 0.3), -0.3)
        self.assertAlmostEqual(_clamp_vx(0.5, 0.3), 0.5)

    def test_ros2_distance_move_gets_longer_timeout(self):
        cfg = replace(CFG, motion_backend="ros2", ros2_line_vx=0.4,
                      step_timeout_s=3.0)
        request = Actions(cfg).move("forward", 1.0)
        self.assertGreaterEqual(request.timeout_s, 4.4)


if __name__ == "__main__":
    unittest.main()
