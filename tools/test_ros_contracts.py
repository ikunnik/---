"""Offline task identity, capture provenance and receive-time regression tests."""
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rover_x1.contracts import (FrameContext, FrameFreshness, TaskKey,
                                CLOCK_SYSTEM, CLOCK_DEVICE)


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.stream = str(uuid4())
        self.frame = FrameContext(self.stream, 1, 'camera2_optical_frame',
                                  1_234_567_890, CLOCK_SYSTEM)

    def test_missing_stamp_is_explicit_and_zero_is_valid(self):
        self.assertIsNone(FrameContext(self.stream, 0).source_stamp_ns)
        self.assertEqual(replace(self.frame, source_stamp_ns=0).source_stamp_ns, 0)
        for changes in ({'source_stamp_ns': None}, {'clock_domain': 0},
                        {'source_stamp_ns': -1}, {'sequence': -1},
                        {'source_stamp_ns': (1 << 31)*10**9}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(self.frame, **changes)

    def test_repeated_or_reordered_frames_do_not_refresh_health(self):
        gate = FrameFreshness(self.stream)
        self.assertTrue(gate.observe(self.frame, now_monotonic=10))
        for frame in (self.frame, replace(self.frame, sequence=0),
                      replace(self.frame, sequence=2),
                      replace(self.frame, sequence=3, source_stamp_ns=1)):
            self.assertFalse(gate.observe(frame, now_monotonic=10.4))
        self.assertFalse(gate.fresh(now_monotonic=10.5, max_age_s=.5))

    def test_source_clock_changes_and_old_sessions_are_rejected(self):
        gate = FrameFreshness(self.stream)
        gate.observe(self.frame, now_monotonic=10)
        for frame in (replace(self.frame, stream_id=str(uuid4())),
                      replace(self.frame, sequence=2, clock_domain=CLOCK_DEVICE),
                      FrameContext(self.stream, 2)):
            self.assertFalse(gate.observe(frame, now_monotonic=10.1))

    def test_detector_failure_cannot_be_reported_healthy(self):
        gate = FrameFreshness(self.stream)
        bad = replace(self.frame, detector_ok=False)
        self.assertTrue(gate.observe(bad, now_monotonic=10))
        self.assertFalse(gate.fresh(now_monotonic=10.1, max_age_s=.5))
        good = replace(self.frame, sequence=2, source_stamp_ns=2_000_000_000)
        self.assertTrue(gate.observe(good, now_monotonic=10.2))
        self.assertTrue(gate.fresh(now_monotonic=10.3, max_age_s=.5))
        self.assertFalse(gate.fresh(now_monotonic=9, max_age_s=.5))

    def test_missing_capture_stamp_uses_sequence_without_inventing_time(self):
        gate = FrameFreshness(self.stream)
        self.assertTrue(gate.observe(FrameContext(self.stream, 1), now_monotonic=10))
        self.assertFalse(gate.observe(FrameContext(self.stream, 1), now_monotonic=11))
        self.assertIsNone(gate.last_source_ns)

    def test_task_identity_requires_session_generation_and_goal(self):
        task = TaskKey.new(str(uuid4()), 1)
        for changes in ({'session_id': str(uuid4())}, {'generation': 2},
                        {'goal_id': str(uuid4())}):
            self.assertNotEqual(task, replace(task, **changes))
        for changes in ({'session_id': ''}, {'generation': -1}, {'generation': True},
                        {'generation': 1 << 64}, {'goal_id': str(uuid4()).upper()}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(task, **changes)


if __name__ == '__main__':
    unittest.main(verbosity=2)
