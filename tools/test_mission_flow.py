"""整段任务串联回归：红绿灯通过 → 掉头 → 盲道 → 停止。

不接机器人：直接驱动 DecisionEngine 产生动作意图，把每一步的
CommandResult 喂回去，并用 ControllerSupervisor 推进控制器。
验证的是“阶段完成 → 调度器推进”的接线，而不是真实感知/运动。
"""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from decisions.engine import DecisionEngine, DecisionInput
from messages import (
    CommandResult, LaneState, Perception, Pose, StopLineState, TrafficLightState,
)
from workers.supervisor import ControllerStatus, ControllerSupervisor

LANE_TRIAL_S = 1.0
PASS_M = 2.0


def snapshot(now, *, line=None, line_ts=0.0, traffic=None, result=None,
             lane=None, pose=None, busy=False, perception=None):
    perception = perception or Perception(
        2, True, now, left_m=2, right_m=2,
        left_valid=True, right_valid=True, min_m=2)
    return DecisionInput(
        now=now,
        perception=perception,
        perception_ts=perception.ts,
        pose=pose or Pose(0, now),
        result=result,
        executor_busy=busy,
        lane=lane,
        line=line,
        line_ts=line_ts,
        traffic=traffic,
    )


class MissionFlowTests(unittest.TestCase):
    def setUp(self):
        self.cfg = replace(CFG, lane_trial_seconds=LANE_TRIAL_S,
                           traffic_pass_m=PASS_M)
        self.sup = ControllerSupervisor(
            self.cfg, 'traffic', mission=True,
            available_modes={'traffic', 'uturn', 'lane', 'avoidance'},
        )

    def advance(self, engine):
        command, _ = self.sup.snapshot()
        self.sup.status.set(ControllerStatus(
            command.revision, command.mode,
            engine.terminal_status or 'running'))
        self.sup.tick()
        return self.sup.snapshot()[0].mode

    def drive_traffic_to_completed(self):
        engine = DecisionEngine(self.cfg, traffic_sequence=True)
        engine.start(100)
        line = StopLineState(True, 0.60, 100)
        out = engine.evaluate(snapshot(100, line=line, line_ts=100))
        self.assertEqual(out.category, 'traffic')
        forward = out.step.request
        engine.note_sent(forward, 100)

        # 到停止线：立即打断前进并停车
        out = engine.evaluate(snapshot(
            100.1, line=StopLineState(True, 0.90, 100.1), line_ts=100.1,
            result=CommandResult(1, forward.cmd.tag, 'ok', ts=100.1)))
        self.assertEqual(out.step.request.kind, 'stop')
        stop = out.step.request
        engine.note_sent(stop, 100.1)

        # 停稳后开启红绿灯检测
        engine.evaluate(snapshot(
            100.4, line=StopLineState(True, 0.90, 100.4), line_ts=100.4,
            result=CommandResult(2, stop.cmd.tag, 'ok', ts=100.3)))
        # 停车结果消费后还要再走一拍，才进入 AT_LINE 的停稳计时
        engine.evaluate(snapshot(
            100.45, line=StopLineState(True, 0.90, 100.45), line_ts=100.45))
        self.assertEqual(engine.traffic.phase, 'SIGNAL')

        # 绿灯 -> 通过
        green = TrafficLightState('green', True, 100.5)
        out = engine.evaluate(snapshot(
            100.5, line=StopLineState(True, 0.90, 100.5), line_ts=100.5,
            traffic=green))
        self.assertEqual(engine.traffic.phase, 'PASS')
        self.assertEqual(out.step.request.kind, 'forward')
        self.assertEqual(out.step.request.cmd.motion_id, 'line_walk')
        self.assertEqual(
            dict(out.step.request.cmd.params)['distance'], PASS_M)

        # 2m 距离动作成功 -> 序列完成，随后掉头
        pass_req = out.step.request
        engine.note_sent(pass_req, 100.5)
        engine.evaluate(snapshot(
            100.6, line=StopLineState(True, 0.90, 100.6), line_ts=100.6,
            traffic=green,
            result=CommandResult(3, pass_req.cmd.tag, 'ok', ts=100.6)))
        self.assertEqual(engine.terminal_status, 'completed')
        return engine

    def drive_uturn_to_completed(self):
        engine = DecisionEngine(self.cfg, uturn=True)
        engine.start(200)
        now, yaw = 200.0, 0.0
        pending = None
        finish_ts = 0.0
        result = None
        generation = 0
        for _ in range(200):
            if pending is not None and now >= finish_ts:
                generation += 1
                result = CommandResult(generation, pending.cmd.tag, 'ok', ts=now)
                pending = None
            out = engine.evaluate(snapshot(
                now, pose=Pose(math.radians(yaw), now), result=result))
            request = out.step.request
            if request is not None and request.kind != 'stop':
                pending = request
                finish_ts = now + 0.2
                if request.kind == 'turn':
                    yaw += request.expected_yaw_delta_deg
            result = None
            if engine.uturn.completed or engine.uturn.failed:
                break
            now += 0.1
        self.assertTrue(engine.uturn.completed, engine.uturn.reason)
        self.assertAlmostEqual(yaw, 180.0, places=3)
        self.assertEqual(engine.terminal_status, 'completed')
        return engine

    def drive_lane_to_completed(self):
        engine = DecisionEngine(self.cfg, lane_follow=True)
        engine.start(300)
        lane = LaneState(True, center=0.5, ts=300, confidence=0.9)
        out = engine.evaluate(snapshot(300, lane=lane))
        self.assertEqual(out.step.state, 'LANE_VELOCITY')
        engine.evaluate(snapshot(300 + LANE_TRIAL_S + 0.1,
                                 lane=replace(lane, ts=300 + LANE_TRIAL_S + 0.1)))
        self.assertTrue(engine.lane.trial_finished)
        self.assertEqual(engine.terminal_status, 'completed')
        return engine

    def test_traffic_then_uturn_then_lane_then_stopped(self):
        self.assertEqual(self.sup.snapshot()[0].mode, 'traffic')

        traffic = self.drive_traffic_to_completed()
        self.assertEqual(self.advance(traffic), 'uturn')

        uturn = self.drive_uturn_to_completed()
        self.assertEqual(self.advance(uturn), 'lane')

        lane = self.drive_lane_to_completed()
        self.assertEqual(self.advance(lane), 'stopped')


if __name__ == '__main__':
    unittest.main(verbosity=2)
