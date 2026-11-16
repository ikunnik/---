"""无盲道直行回归：纯控制器模拟，不连接机器人。"""
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from messages import CommandResult, LaneState, Perception
from decisions.lane import LaneFollower


class NoLineTests(unittest.TestCase):
    def setUp(self):
        self.cfg = replace(CFG,
                           lane_loop_update_threshold=.12, lane_loop_min_update_s=2.5,
                           lane_control_duration_s=1.2, lane_batch_steps=2,
                           lane_trial_seconds=6.0)
        self.f = LaneFollower(self.cfg)
        self.t = 100.0

    def tick(self, t, visible=False, center=.7, result=None, **kwargs):
        lane = LaneState(visible, center, center, 0, 0, ts=t,
                         confidence=.9 if visible else 0)
        lane = kwargs.pop('lane', lane)
        per = kwargs.pop('per', Perception(2, True, t))
        return self.f.step(t, lane, per, kwargs.pop('per_ts', t), None,
                           result, self.f.has_pending)

    def assert_straight(self, out, interrupt):
        self.assertEqual(out.state, 'LANE_NO_LINE_STRAIGHT')
        self.assertEqual(out.interrupt, interrupt)
        self.assertEqual(out.request.kind, 'lane_velocity')
        for step in out.request.cmd.steps:
            self.assertEqual((step.vx, step.vy, step.vyaw), (.4, 0, 0))
        self.assertTrue(out.request.cmd.loop)

    def assert_stop(self, out):
        self.assertEqual(out.state, 'LANE_STOP')
        self.assertTrue(out.interrupt)
        self.assertEqual(out.request.cmd.vx, 0)

    def test_start_without_line_and_keep_loop_without_resend(self):
        first = self.tick(self.t)
        self.assert_straight(first, False)
        for i in range(1, 51):
            out = self.tick(self.t+i*.1)
            self.assertIsNone(out.request)
            self.assertFalse(out.interrupt)
        self.assertEqual(self.f._pending.tag, first.request.cmd.tag)

    def test_line_disappears_immediately_clears_turn(self):
        self.f = LaneFollower(replace(self.cfg, lane_reacquire_enabled=False))
        turning = self.tick(self.t, visible=True)
        self.assertNotEqual(turning.request.cmd.steps[0].vyaw, 0)
        straight = self.tick(self.t+.1)
        self.assert_straight(straight, True)
        self.assertNotEqual(straight.request.cmd.tag, turning.request.cmd.tag)
        self.assertIsNone(self.f._center)
        self.assertEqual(self.f._active_vyaw, 0)
        # 旧转向的取消回执不能终止新直行序列。
        old = CommandResult(1, turning.request.cmd.tag, 'cancelled', ts=self.t+.11)
        self.assertIsNone(self.tick(self.t+.12, result=old).request)

    def test_line_lost_turns_toward_last_side(self):
        self.tick(self.t, visible=True, center=.75)
        out = self.tick(self.t + .1)
        self.assertEqual(out.state, 'LANE_NO_LINE_STRAIGHT')
        self.assertLess(out.request.cmd.steps[0].vyaw, 0)

    def test_line_returns_without_waiting_for_normal_loop_threshold(self):
        self.tick(self.t)
        out = self.tick(self.t+.1, visible=True, center=.55)
        self.assertTrue(out.interrupt)
        self.assertEqual(out.state, 'LANE_VELOCITY')
        self.assertNotEqual(out.request.cmd.steps[0].vyaw, 0)
        self.assertFalse(self.f._no_line_mode)
        self.assertEqual(self.f._trial_started, self.t)

    def test_missing_stale_and_future_frames_stop(self):
        for lane in (None, LaneState(False, ts=self.t-1), LaneState(False, ts=self.t+1),
                     LaneState(False, ts=float('nan'))):
            self.f = LaneFollower(self.cfg)
            self.tick(self.t)
            self.assert_stop(self.tick(self.t+.1, lane=lane))

    def test_depth_loss_and_obstacle_stop(self):
        cases = [(None, self.t+.1), (Perception(2, False, self.t+.1), self.t+.1),
                 (Perception(2, True, self.t-1), self.t-1),
                 (Perception(.3, True, self.t+.1), self.t+.1)]
        for per, stamp in cases:
            self.f = LaneFollower(self.cfg)
            self.tick(self.t)
            self.assert_stop(self.tick(self.t+.1, per=per, per_ts=stamp))

    def test_visible_but_uncertain_or_invalid_still_stops(self):
        for lane in (LaneState(True, ts=self.t+.1, confidence=.01),
                     LaneState(True, center=float('nan'), ts=self.t+.1, confidence=.9)):
            self.f = LaneFollower(self.cfg)
            self.tick(self.t)
            self.assert_stop(self.tick(self.t+.1, lane=lane))

    def test_trial_deadline_survives_line_mode_changes_and_reset(self):
        self.tick(self.t)
        self.tick(self.t+1, visible=True)
        self.tick(self.t+2)
        self.assert_stop(self.tick(self.t+6.01))
        self.f.reset()
        after = self.tick(self.t+6.1)
        self.assertEqual(after.state, 'LANE_STOP')
        self.assertIsNone(after.request)

    def test_failure_not_hidden_by_switching_modes(self):
        for started_visible in (True, False):
            for status in ('failed', 'cancelled'):
                self.f = LaneFollower(self.cfg)
                first = self.tick(self.t, visible=started_visible)
                result = CommandResult(1, first.request.cmd.tag, status, ts=self.t+.1)
                self.assert_stop(self.tick(self.t+.1, visible=not started_visible, result=result))

    def test_original_stop_on_loss_remains_selectable(self):
        self.f = LaneFollower(replace(self.cfg, lane_no_line_straight=False))
        self.tick(self.t, visible=True)
        self.assert_stop(self.tick(self.t+.1))



if __name__ == '__main__':
    unittest.main(verbosity=2)
