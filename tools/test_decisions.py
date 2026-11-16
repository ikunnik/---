"""统一决策入口回归：直接输入快照，不接机器人和相机。"""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from decisions.engine import DecisionEngine, DecisionInput
from decisions.safety import depth_valid, lane_fresh, stop_line_alive
from decisions.types import BLOCKED
from messages import (
    CommandResult, LaneState, Perception, Pose, StopLineState,
    TrafficLightState, VoiceCue,
)


class DecisionTests(unittest.TestCase):
    def engine(self, **kwargs):
        engine = DecisionEngine(CFG, **kwargs)
        engine.start(100)
        return engine

    def snapshot(self, now=100, **changes):
        defaults = dict(
            now=now, perception=Perception(2, True, now, left_m=2, right_m=2,
                left_valid=True, right_valid=True, min_m=2),
            perception_ts=now, pose=Pose(0, now, x=0, y=0), lane=LaneState(False, ts=now),
            line=StopLineState(False, ts=now), line_ts=now,
        )
        defaults.update(changes)
        return DecisionInput(**defaults)

    def assert_stop(self, output):
        self.assertIsNotNone(output.step.request)
        self.assertEqual(output.step.request.kind, 'stop')
        cmd = output.step.request.cmd
        self.assertEqual((cmd.vx, cmd.vy, cmd.vyaw), (0, 0, 0))

    def test_default_mode_only_uses_avoidance(self):
        engine = self.engine()
        outputs = [engine.evaluate(self.snapshot(100+i*.1)) for i in range(10)]
        self.assertTrue(all(out.category == 'avoidance' for out in outputs))
        self.assertTrue(any(out.step.request and out.step.request.kind == 'forward' for out in outputs))
        blocked = engine.evaluate(self.snapshot(101, perception=Perception(.3, True, 101)))
        self.assert_stop(blocked)
        self.assertTrue(blocked.step.interrupt)
        self.assertIsNone(engine.lane)
        self.assertIsNone(engine.traffic)

    def test_lane_mode_does_not_require_stop_line_or_signal(self):
        engine = self.engine(lane_follow=True)
        out = engine.evaluate(self.snapshot(line=None, line_ts=0, traffic=None))
        self.assertEqual(out.category, 'lane')
        self.assertEqual(out.step.state, 'LANE_NO_LINE_STRAIGHT')
        self.assertTrue(out.track_result)
        self.assertTrue(out.step.request.cmd.loop)
        self.assertTrue(engine.has_pending_action)
        self.assertTrue(all(step.vyaw == 0 for step in out.step.request.cmd.steps))
        self.assertIsNone(engine.traffic)

    def test_lane_obstacle_preempts_then_returns_after_path_clears(self):
        engine = self.engine(lane_follow=True)
        first = engine.evaluate(self.snapshot())
        old_result = CommandResult(1, first.step.request.cmd.tag, 'cancelled', ts=100.1)
        out = engine.evaluate(self.snapshot(100.1, perception=Perception(.3, True, 100.1),
                                             executor_busy=True, result=old_result))
        self.assertEqual(out.category, 'avoidance')
        self.assertTrue(out.clear_pending)
        self.assertTrue(out.step.interrupt)
        self.assert_stop(out)
        self.assertFalse(engine.lane.has_pending)
        outputs = [engine.evaluate(self.snapshot(100.2+i*.1)) for i in range(15)]
        self.assertTrue(any(out.category == 'avoidance' for out in outputs))
        self.assertTrue(any(out.step.state == 'LANE_NO_LINE_STRAIGHT' for out in outputs))
        self.assertNotEqual(engine.avoidance.phase, BLOCKED)

    def test_lane_failure_and_stale_camera_still_stop(self):
        for failure in ('failed', 'stale'):
            with self.subTest(failure=failure):
                engine = self.engine(lane_follow=True)
                first = engine.evaluate(self.snapshot())
                changes = dict(executor_busy=True)
                if failure == 'failed':
                    changes['result'] = CommandResult(1, first.step.request.cmd.tag, 'failed', ts=100.1)
                else:
                    changes['lane'] = None
                out = engine.evaluate(self.snapshot(100.1, **changes))
                self.assert_stop(out)
                self.assertTrue(out.step.interrupt)
                self.assertFalse(engine.has_pending_action)

    def test_lane_notification_is_emitted_once_and_trial_limit_survives(self):
        engine = DecisionEngine(
            replace(CFG, lane_trial_seconds=1.0), lane_follow=True)
        engine.start(100)
        line = LaneState(True, center=.5, ts=100, confidence=.9)
        out = engine.evaluate(self.snapshot(lane=line))
        self.assertEqual(len(out.notices), 1)
        self.assertEqual(out.notices[0].kind, 'lane')
        again = engine.evaluate(self.snapshot(100.1, lane=replace(line, ts=100.1)))
        self.assertEqual(again.notices, ())
        expired = engine.evaluate(self.snapshot(101.2))
        self.assert_stop(expired)
        self.assertTrue(expired.step.interrupt)
        self.assertFalse(engine.has_pending_action)

    def test_lane_trial_zero_means_unlimited(self):
        engine = self.engine(lane_follow=True)  # 默认 lane_trial_seconds=0
        line = LaneState(True, center=.5, ts=100, confidence=.9)
        engine.evaluate(self.snapshot(lane=line))
        for t in (110.0, 200.0, 1000.0):
            engine.evaluate(self.snapshot(t, lane=replace(line, ts=t)))
        self.assertFalse(engine.lane.trial_finished)
        self.assertIsNone(engine.terminal_status)

    def test_missing_stop_line_blocks_normal_forward_then_recovers_when_healthy(self):
        engine = self.engine(traffic_sequence=True)
        out = engine.evaluate(self.snapshot(line=None, line_ts=0, executor_busy=True))
        self.assertEqual(out.category, 'safety')
        self.assertTrue(out.clear_pending)
        self.assertTrue(out.step.interrupt)
        self.assert_stop(out)
        outputs = [engine.evaluate(self.snapshot(100.1+i*.1)) for i in range(10)]
        self.assertTrue(any(out.step.request and out.step.request.kind == 'forward' for out in outputs))
        self.assertFalse(engine.traffic.active)

    def test_traffic_takeover_interrupts_even_when_no_new_request(self):
        engine = self.engine(traffic_sequence=True)
        line = StopLineState(True, .6, 100)
        out = engine.evaluate(self.snapshot(line=line, executor_busy=True))
        self.assertEqual(out.category, 'traffic')
        self.assertTrue(out.clear_pending)
        self.assertTrue(out.step.interrupt)
        self.assertIsNone(out.step.request)
        # 取消旧前进完成后，下一轮从统一入口生成停止线前进。
        out = engine.evaluate(self.snapshot(100.1, line=replace(line, ts=100.1)))
        self.assertEqual(out.step.request.cmd.tag, 'traffic_forward')
        self.assertTrue(out.track_result)
        engine.note_sent(out.step.request, 100.1)
        self.assertTrue(engine.has_pending_action)
        # 到线判断优先于等待上一条前进结束。
        reached = engine.evaluate(self.snapshot(100.2,
            line=StopLineState(True, .9, 100.2), executor_busy=True))
        self.assert_stop(reached)
        self.assertTrue(reached.step.interrupt)
        self.assertTrue(reached.track_result)

    def test_traffic_failure_is_consumed_by_its_owner_and_returns_to_avoidance_stopped(self):
        engine = self.engine(traffic_sequence=True)
        line = StopLineState(True, .6, 100)
        out = engine.evaluate(self.snapshot(line=line))
        engine.note_sent(out.step.request, 100)
        result = CommandResult(1, out.step.request.cmd.tag, 'failed', ts=100.1)
        failed = engine.evaluate(self.snapshot(100.1, line=replace(line, ts=100.1), result=result))
        self.assert_stop(failed)
        self.assertTrue(failed.step.interrupt)
        self.assertFalse(engine.traffic.active)
        self.assertFalse(engine.has_pending_action)

    def test_voice_wait_prevents_traffic_takeover(self):
        engine = self.engine(traffic_sequence=True, wait_for_voice=True)
        engine.avoidance.voice.announce(VoiceCue('wait', reason='test'), 100)
        out = engine.evaluate(self.snapshot(line=StopLineState(True, .6, 100)))
        self.assertFalse(engine.traffic.active)
        self.assertEqual(out.category, 'avoidance')
        self.assert_stop(out)
        engine.evaluate(self.snapshot(100.1, line=StopLineState(True, .6, 100.1), voice_done=True))
        self.assertFalse(engine.avoidance.voice_pending)

    def _uturn_snapshot(self, now, yaw_deg, result=None):
        return self.snapshot(
            now=now,
            pose=Pose(math.radians(yaw_deg), now),
            perception=Perception(2, True, now, left_m=2, right_m=2,
                                  left_valid=True, right_valid=True, min_m=2),
            result=result,
        )

    def test_uturn_retries_small_yaw_shortfall_once(self):
        engine = self.engine(uturn=True)
        first = engine.evaluate(self._uturn_snapshot(100, 0))
        self.assertEqual(first.step.state, 'UTURN_LEFT_1')
        self.assertEqual(first.step.request.kind, 'turn')

        result = CommandResult(1, first.step.request.cmd.tag, 'ok', ts=100.1)
        wait = engine.evaluate(self._uturn_snapshot(100.2, 75, result))
        self.assertFalse(engine.uturn.failed)
        self.assertEqual(wait.step.state, 'UTURN_RETRY')
        self.assertIsNone(wait.step.request)

        retry = engine.evaluate(self._uturn_snapshot(100.3, 75)).step
        self.assertEqual(retry.state, 'UTURN_RETRY')
        self.assertIsNotNone(retry.request)
        self.assertAlmostEqual(retry.request.expected_yaw_delta_deg, 15.0, places=3)

        result2 = CommandResult(2, retry.request.cmd.tag, 'ok', ts=100.4)
        done = engine.evaluate(self._uturn_snapshot(100.5, 90, result2))
        self.assertFalse(engine.uturn.failed)
        self.assertEqual(done.step.state, 'UTURN_SETTLE')

        forward = engine.evaluate(self._uturn_snapshot(100.9, 90))
        self.assertEqual(forward.step.state, 'UTURN_FORWARD')
        self.assertEqual(forward.step.request.kind, 'advance')

    def test_uturn_large_yaw_error_locks(self):
        engine = self.engine(uturn=True)
        first = engine.evaluate(self._uturn_snapshot(100, 0))
        result = CommandResult(1, first.step.request.cmd.tag, 'ok', ts=100.1)
        out = engine.evaluate(self._uturn_snapshot(100.2, 55, result))
        self.assertTrue(engine.uturn.failed)
        self.assertEqual(out.step.state, 'UTURN_FAILED')
        self.assertIn('55.0', engine.uturn.reason)

    def test_uturn_ignore_obstacles_skips_depth_checks(self):
        engine = DecisionEngine(
            replace(CFG, uturn_ignore_obstacles=True), uturn=True)
        engine.start(100)
        out = engine.evaluate(self.snapshot(
            now=100, perception=None, perception_ts=0.0, pose=Pose(0, 100)))
        self.assertEqual(out.step.state, 'UTURN_LEFT_1')
        self.assertIsNotNone(out.step.request)

    def test_uturn_without_ignore_still_blocks_close_obstacle(self):
        engine = self.engine(uturn=True)
        close = CFG.rotate_clear_m * 0.5
        out = engine.evaluate(self.snapshot(
            now=100, pose=Pose(0, 100),
            perception=Perception(close, True, 100, min_m=close)))
        self.assertTrue(engine.uturn.failed)
        self.assertEqual(out.step.state, 'UTURN_FAILED')

    def test_traffic_green_passes_without_waiting_for_voice(self):
        cfg = replace(CFG, stop_settle_s=0.2, traffic_pass_m=2.0)
        engine = DecisionEngine(cfg, traffic_sequence=True, wait_for_voice=True)
        engine.start(100)
        line = StopLineState(True, 0.6, 100)
        out = engine.evaluate(self.snapshot(100, line=line, line_ts=100))
        forward = out.step.request
        engine.note_sent(forward, 100)

        out = engine.evaluate(self.snapshot(
            100.1, line=StopLineState(True, 0.9, 100.1), line_ts=100.1,
            result=CommandResult(1, forward.cmd.tag, 'ok', ts=100.1)))
        stop = out.step.request
        engine.note_sent(stop, 100.1)
        engine.evaluate(self.snapshot(
            100.2, line=StopLineState(True, 0.9, 100.2), line_ts=100.2,
            result=CommandResult(2, stop.cmd.tag, 'ok', ts=100.2)))
        engine.evaluate(self.snapshot(
            100.3, line=StopLineState(True, 0.9, 100.3), line_ts=100.3))
        engine.evaluate(self.snapshot(
            100.4, line=StopLineState(True, 0.9, 100.4), line_ts=100.4))
        self.assertEqual(engine.traffic.phase, 'SIGNAL')

        green = TrafficLightState('green', True, 100.5)
        out = engine.evaluate(self.snapshot(
            100.5, line=StopLineState(True, 0.9, 100.5), line_ts=100.5,
            traffic=green))
        # 绿灯不阻塞在语音回执上：同一拍就进入 PASS 并前进
        self.assertEqual(engine.traffic.phase, 'PASS')
        self.assertEqual(out.step.request.kind, 'forward')
        self.assertFalse(engine.avoidance.voice_pending)

    def _drive_uturn_first_turn(self, engine):
        first = engine.evaluate(self.snapshot(
            now=100, pose=Pose(0, 100),
            perception=Perception(1.3, True, 100, min_m=2.0)))
        result = CommandResult(1, first.step.request.cmd.tag, 'ok', ts=100.1)
        engine.evaluate(self.snapshot(
            now=100.2, pose=Pose(math.radians(90), 100.2),
            result=result,
            perception=Perception(1.3, True, 100.2, min_m=2.0)))

    def test_uturn_skips_forward_space_check_by_default(self):
        engine = self.engine(uturn=True)
        self._drive_uturn_first_turn(engine)
        out = engine.evaluate(self.snapshot(
            now=100.6, pose=Pose(math.radians(90), 100.6),
            perception=Perception(1.3, True, 100.6, min_m=2.0)))
        self.assertFalse(engine.uturn.failed)
        self.assertEqual(out.step.state, 'UTURN_FORWARD')
        self.assertEqual(out.step.request.kind, 'advance')

    def test_uturn_forward_space_check_can_be_restored(self):
        cfg = replace(CFG, uturn_check_forward_space=True)
        engine = DecisionEngine(cfg, uturn=True)
        engine.start(100)
        self._drive_uturn_first_turn(engine)
        out = engine.evaluate(self.snapshot(
            now=100.6, pose=Pose(math.radians(90), 100.6),
            perception=Perception(1.3, True, 100.6, min_m=2.0)))
        self.assertTrue(engine.uturn.failed)
        self.assertIn('空间不足', engine.uturn.reason)

    def test_common_sensor_boundaries_preserve_each_mode(self):
        per = Perception(2, True, 100)
        self.assertFalse(depth_valid(CFG, per, 100, 100+CFG.stale_s))
        self.assertFalse(depth_valid(CFG, None, 100, 100))
        self.assertFalse(depth_valid(CFG, replace(per, valid=False), 100, 100))
        # 用二进制可精确表示的阈值验证 <= 与 < 的区别。
        cfg = replace(CFG, lane_stale_s=.5, stop_line_stale_s=.5)
        lane = LaneState(False, ts=100)
        self.assertTrue(lane_fresh(cfg, lane, 100.5))
        self.assertFalse(lane_fresh(cfg, replace(lane, ts=101), 100))
        self.assertTrue(stop_line_alive(cfg, StopLineState(False, ts=100), 100, 100.5))

    def test_worker_honors_takeover_without_request(self):
        from comm import Latest
        from workers.decision import DecisionWorker
        from unittest.mock import patch
        depth, line, traffic = Latest(), Latest(), Latest()
        with patch('comm.time.time', return_value=100):
            depth.set(Perception(2, True, 100))
            line.set(StopLineState(True, .6, 100))
        executor = MagicMock()
        executor.busy = True
        executor.result.return_value = (None, 0)
        robot = MagicMock()
        robot.get_pose.return_value = Pose(0, 100)
        worker = DecisionWorker(CFG, depth, executor, robot, stop_line=line,
                                traffic=traffic, traffic_sequence=True)
        # 一轮真实 Worker → Engine → interrupt，完全使用模拟执行器。
        with patch('workers.decision.time.time', return_value=100), \
             patch('workers.decision.time.sleep', side_effect=lambda _: worker.stop()):
            worker.run()
        executor.interrupt.assert_called_once()
        executor.submit.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
