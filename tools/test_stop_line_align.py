#!/usr/bin/env python3
"""停止线倾角检测 + 到线对齐的纯逻辑测试。"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CFG                                      # noqa: E402
from decisions.avoidance import AvoidStateMachine           # noqa: E402
from decisions.traffic import TrafficSequence               # noqa: E402
from messages import CommandResult, StopLineState          # noqa: E402
from workers.stop_line import detect_stop_line, load_profile  # noqa: E402


def make_yellow_line_frame(angle_deg=0.0):
    frame = np.zeros((480, 640, 3), dtype=np.float32)
    profile = load_profile("assets/stop_line_profile.json")
    x0 = 120
    y0 = 330
    length = 420
    slope = np.tan(np.radians(angle_deg))
    for x in range(x0, min(640, x0 + length)):
        y = int(round(y0 + slope * (x - x0)))
        if 0 <= y < 480:
            frame[max(0, y - 5):y + 5, x] = (1.0, 0.65, 0.0)
    return frame, profile


class StopLineAlignTest(unittest.TestCase):
    def test_horizontal_line_angle_near_zero(self):
        frame, profile = make_yellow_line_frame(0.0)
        state = detect_stop_line(frame, profile, now=100)
        self.assertTrue(state.visible)
        self.assertLess(abs(state.angle_deg), 1.5)

    def test_tilted_line_reports_angle(self):
        frame, profile = make_yellow_line_frame(10.0)
        state = detect_stop_line(frame, profile, now=100)
        self.assertTrue(state.visible)
        self.assertGreater(abs(state.angle_deg), 2.0)

    def test_traffic_sequence_aligns_when_tilted(self):
        seq = TrafficSequence(CFG, None, AvoidStateMachine(CFG))
        line = StopLineState(True, 0.9, 100.0, 0.02, (0, 0, 10, 10), 6.0)
        self.assertTrue(seq._needs_align(100.0, line))
        step = seq._start_align(100.0, line)
        self.assertEqual(step.state, "ALIGN")
        self.assertEqual(step.request.kind, "turn")
        self.assertEqual(dict(step.request.cmd.params)["direction"], 1)
        self.assertAlmostEqual(
            dict(step.request.cmd.params)["angle"],
            6.0 * CFG.stop_line_align_gain,
            places=4,
        )

    def test_small_angle_does_not_align(self):
        seq = TrafficSequence(CFG, None, AvoidStateMachine(CFG))
        line = StopLineState(True, 0.9, 100.0, 0.02, (0, 0, 10, 10), 0.5)
        self.assertFalse(seq._needs_align(100.0, line))

    def test_align_action_completes_back_to_at_line(self):
        seq = TrafficSequence(CFG, None, AvoidStateMachine(CFG))
        line = StopLineState(True, 0.9, 100.0, 0.02, (0, 0, 10, 10), 6.0)
        step = seq._start_align(100.0, line)
        self.assertIsNotNone(seq._action)
        result = CommandResult(
            1, step.request.cmd.tag, "ok", ts=100.1
        )
        seq._service_action(100.1, result)
        self.assertIsNone(seq._action)
        self.assertEqual(seq.phase, "AT_LINE")
        self.assertAlmostEqual(seq.stop_ts, 100.1)


if __name__ == "__main__":
    unittest.main()
