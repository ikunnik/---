#!/usr/bin/env python3
"""绕障后回原路线：只测纯决策，不需要机器人。"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CFG                                      # noqa: E402
from decisions.avoidance import AvoidStateMachine           # noqa: E402
from decisions.types import StepContext                     # noqa: E402
from messages import Perception, Pose                       # noqa: E402


def context(x=0.0, y=0.0, yaw=0.0, min_m=2.0) -> StepContext:
    return StepContext(
        now=100.0,
        perception=Perception(2.0, True, 100.0, left_m=2, right_m=2, left_valid=True, right_valid=True, min_m=min_m),
        perception_ts=100.0,
        pose=Pose(yaw, 100.0, x=x, y=y),
        result=None,
        executor_busy=False,
        fresh=True,
        valid=True,
        voice_done=False,
    )


class PathReturnTest(unittest.TestCase):
    def make_machine(self):
        machine = AvoidStateMachine(CFG)
        machine.entry_yaw_rad = 0.0
        machine.entry_x = 0.0
        machine.entry_y = 0.0
        machine.bypass_active = True
        machine.path_return_attempts = 0
        machine.return_clear_origin = (-CFG.path_return_clearance_m, 0.0)
        return machine

    def test_lateral_sign(self):
        machine = self.make_machine()
        self.assertAlmostEqual(machine._path_lateral(Pose(0, 100, x=0, y=0.3)), 0.3)
        self.assertAlmostEqual(machine._path_lateral(Pose(0, 100, x=0, y=-0.3)), -0.3)

    def test_offset_starts_return_turn(self):
        machine = self.make_machine()
        step = machine._maybe_start_path_return(context(y=0.4))
        self.assertIsNotNone(step)
        self.assertEqual(step.state, "RETURN")
        self.assertEqual(step.request.kind, "turn")
        self.assertEqual(dict(step.request.cmd.params)["direction"], 1)  # 偏左 -> 右转

    def test_within_tolerance_does_nothing(self):
        machine = self.make_machine()
        self.assertIsNone(machine._maybe_start_path_return(context(y=0.05)))

    def test_no_position_stops_instead_of_claiming_return(self):
        from dataclasses import replace
        machine = self.make_machine()
        ctx = replace(context(), pose=Pose(0, 100))
        step = machine._maybe_start_path_return(ctx)
        self.assertEqual(step.state, "BLOCKED")
        self.assertEqual(step.request.kind, "stop")

    def test_step_grows_with_offset(self):
        machine = self.make_machine()
        machine._maybe_start_path_return(context(y=0.2))
        self.assertGreater(machine.path_return_step_m, 0.1)
        self.assertLessEqual(machine.path_return_step_m, float(
            CFG.path_return_max_step_m))


if __name__ == "__main__":
    unittest.main()
