"""绕障侧向偏移回归：横向位移必须覆盖机身、障碍与净空，并按实测校验。

旧实现把斜向长度固定成 0.7m，45° 下横向只有 0.495m，而且斜向航段本身就是
朝障碍前角切过去的——横向加长也不改善前角余量，只能贴着角擦过。
本文件锁住三件事：
  1. 横向目标由 config 的机身/障碍/净空三项算出，并受该侧实测空间约束；
  2. 斜向长度 = 横向目标 / sin(转角)，转角越陡、同样横向的路程越短；
  3. 实测横向不足时会补一段斜向，不会带着不足的偏移进入并肩前进。
"""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from decisions.avoidance import AvoidStateMachine
from decisions.types import StepContext
from messages import CommandResult, Perception, Pose
from tools.test_complex_avoidance import WorldSim


def context(center=3.0, left=3.0, right=3.0, x=0.0, y=0.0, yaw=0.0,
            now=100.0, near=0.1):
    perception = Perception(center, True, now, left, right, True, True,
                            min(center, left, right), near)
    return StepContext(now, perception, now, Pose(yaw, now, x=x, y=y),
                       None, False, True, True, False)


def lateral_needed(cfg=CFG):
    return (cfg.body_half_width_m + cfg.obstacle_half_width_m
            + cfg.bypass_side_clearance_m)


class LateralTargetTests(unittest.TestCase):
    def test_initial_choice_logged_depths_and_side_limits(self):
        for left, right, expected in [(1.25, 1.35, 0), (1.25, 1.60, 1),
                                      (0.80, 0.90, 1), (1.25, 0.70, 0)]:
            with self.subTest(left=left, right=right):
                sm = AvoidStateMachine(CFG)
                out = sm._step_plan(context(center=0.71, left=left, right=right, near=0.86))
                self.assertEqual(dict(out.request.cmd.params)["direction"], expected)

    def test_short_right_diagonal_escalates_then_checks_left_from_original_position(self):
        sm = AvoidStateMachine(CFG)
        sm._step_plan(context(center=0.71, left=1.25, right=1.60, near=0.86))
        sm.heading_dev_deg = -CFG.bypass_angle_deg
        out = None
        for _ in range(10):
            # 0.66m 处有一张宽牌：任何转角下斜向都塞不进去，追加到上限才放弃
            turned = context(center=0.66, yaw=math.radians(sm.heading_dev_deg))
            out = sm._check_after_turn(turned)
            if out.request is None or out.request.kind != "turn":
                break
            sm.pending = None        # 追加转角已下发；模拟旋转达成
            sm.heading_dev_deg -= CFG.bypass_angle_step_deg
            self.assertEqual(sm.tried_angles, set())
        self.assertEqual(out.request.kind, "realign")
        self.assertEqual(sm.tried_angles, {-1})
        aligned = context(center=0.71, left=1.25, right=1.60)
        left = sm._step_plan(aligned)
        self.assertEqual(dict(left.request.cmd.params)["direction"], 0)
        self.assertEqual(sm._check_after_turn(context()).request.kind, "advance")

    def test_probe_after_right_failure_with_logged_left_reading(self):
        for center, expected in [(3.0, "advance"), (1.12, "turn")]:
            with self.subTest(center=center):
                sm = AvoidStateMachine(CFG, wait_for_voice=True)
                sm.tried_angles.add(-1)
                start = sm._step_plan(context(center=0.70, left=0.70, right=1.43, near=0.82))
                self.assertEqual(start.request.kind, "turn")
                self.assertEqual(dict(start.request.cmd.params)["direction"], 0)
                self.assertIsNone(start.voice)
                sm.heading_dev_deg = CFG.bypass_angle_deg
                out = sm._check_after_turn(context(center=center, yaw=math.radians(CFG.bypass_angle_deg)))
                self.assertEqual(out.request.kind, expected)
                if expected == "advance":
                    self.assertEqual(out.voice.side, "left")
                else:
                    # 探测方向塞不下整段斜向：先同侧追加转角，不立刻放弃这一侧
                    self.assertAlmostEqual(dict(out.request.cmd.params)["angle"],
                                           CFG.bypass_angle_step_deg)
                    self.assertEqual(sm.tried_angles, {-1})

    def test_probe_excluded_after_translation_or_with_tight_rotation_space(self):
        for moved, left in [(0.37, 0.70), (0.0, 0.18), (0.0, 0.45)]:
            sm = AvoidStateMachine(CFG)
            sm.tried_angles.add(-1)
            sm.bypass_distance_m = moved
            out = sm._step_plan(context(center=0.70, left=left, right=1.43, near=0.82))
            self.assertEqual(out.request.kind, "stop")

    def test_both_sides_below_threshold_stop(self):
        sm = AvoidStateMachine(CFG)
        out = sm._step_plan(context(center=0.71, left=0.65, right=0.68))
        self.assertEqual(out.state, "BLOCKED")
        self.assertEqual(out.request.kind, "stop")

    def test_target_covers_body_obstacle_and_clearance(self):
        sm = AvoidStateMachine(CFG)
        self.assertAlmostEqual(
            sm._bypass_lateral_target(context(left=4.0).perception, "left"),
            lateral_needed(),
        )

    def test_wall_side_requires_extra_lateral(self):
        sm = AvoidStateMachine(CFG)
        ctx = context(left=4.0, near=CFG.wall_near_ratio)
        self.assertAlmostEqual(
            sm._bypass_lateral_target(ctx.perception, "left"),
            lateral_needed() + CFG.wall_extra_m,
        )

    def test_target_clamped_by_measured_side_space(self):
        sm = AvoidStateMachine(CFG)
        usable = CFG.min_passage_m - CFG.body_half_width_m - CFG.bypass_side_clearance_m
        self.assertAlmostEqual(
            sm._bypass_lateral_target(context(left=CFG.min_passage_m).perception, "left"),
            usable,
        )

    def test_diagonal_length_follows_target_and_angle(self):
        sm = AvoidStateMachine(CFG)
        wanted = 0.60
        expected = wanted / math.sin(math.radians(CFG.bypass_angle_deg))
        self.assertAlmostEqual(sm._diagonal_from_lateral(wanted), expected)
        # 更陡的转角用更短的路程拿到同样的横向偏移
        steep = AvoidStateMachine(replace(CFG, bypass_angle_deg=75.0))
        self.assertLess(steep._diagonal_from_lateral(wanted),
                        sm._diagonal_from_lateral(wanted))

    def test_plan_sets_diagonal_from_lateral_target(self):
        sm = AvoidStateMachine(CFG)
        sm.entry_yaw_rad = 0.0
        sm.entry_x = 0.0
        sm.entry_y = 0.0
        sm.bypass_active = True
        # 右侧只有 0.2m，只能往左绕
        out = sm._step_plan(context(center=0.6, left=4.0, right=0.2))
        self.assertEqual(out.request.kind, "turn")
        self.assertAlmostEqual(dict(out.request.cmd.params)["angle"],
                               CFG.bypass_angle_deg)
        self.assertAlmostEqual(sm.diagonal_remaining,
                               sm._diagonal_from_lateral(lateral_needed()))
        self.assertAlmostEqual(sm.bypass_lateral_target, lateral_needed())


class LateralFeedbackTests(unittest.TestCase):
    def anchored(self):
        sm = AvoidStateMachine(CFG)
        sm.entry_x = sm.entry_y = sm.entry_yaw_rad = 0.0
        sm.bypass_active = True
        sm.bypass_started_ts = 100.0
        sm._bypass_stage = "diagonal"
        sm.diagonal_remaining = 0.0
        sm.bypass_lateral_target = lateral_needed()
        sm.bypass_lateral_sign = 1.0
        return sm

    def test_short_lateral_adds_another_diagonal(self):
        sm = self.anchored()
        out = sm._check_after_advance(
            context(x=0.2, y=0.20, yaw=math.radians(CFG.bypass_angle_deg))
        )
        self.assertEqual(out.request.kind, "advance")
        self.assertGreater(dict(out.request.cmd.params)["distance"],
                           sm.cfg.bypass_step_min_m)

    def test_enough_lateral_goes_straight_to_realign(self):
        sm = self.anchored()
        out = sm._check_after_advance(
            context(x=0.5, y=lateral_needed(), yaw=math.radians(CFG.bypass_angle_deg))
        )
        self.assertEqual(out.request.kind, "realign")

    def test_retry_is_bounded(self):
        sm = self.anchored()
        calls = 0
        for _ in range(int(CFG.bypass_lateral_max_retry) + 2):
            sm.pending = None
            out = sm._check_after_advance(
                context(x=0.2, y=0.10, yaw=math.radians(CFG.bypass_angle_deg))
            )
            if out.request is not None and out.request.kind == "advance":
                calls += 1
                continue
            break
        self.assertEqual(calls, int(CFG.bypass_lateral_max_retry))


class BodyClearanceTests(unittest.TestCase):
    """沿整条绕行路径检查与障碍的最小间隔，而不是只看终点。"""

    def run_scene(self, boxes):
        sim = WorldSim([list(b) for b in boxes])
        min_clear = 9e9
        max_lateral = 0.0
        for _ in range(2500):
            sim.tick()
            for x0, x1, y0, y1 in boxes:
                min_clear = min(min_clear, math.hypot(
                    max(x0 - sim.x, 0, sim.x - x1),
                    max(y0 - sim.y, 0, sim.y - y1)))
            max_lateral = max(max_lateral, abs(sim.y))
            if str(sim.sm.phase) == "GO":
                break
        return sim, min_clear, max_lateral

    def test_single_obstacle_keeps_body_clearance(self):
        boxes = [(0.6, 1.2, -0.18, 0.18)]
        sim, min_clear, max_lateral = self.run_scene(boxes)
        self.assertEqual(str(sim.sm.phase), "GO")
        self.assertGreaterEqual(max_lateral, lateral_needed() - 0.02)
        # 0.495m 的旧横向在斜向航段只剩 0.297m 的间隔，扣掉机身半宽不足 0.12m
        self.assertGreaterEqual(min_clear,
                                CFG.body_half_width_m + CFG.bypass_side_clearance_m)

    def test_second_obstacle_on_bypass_side_keeps_clearance(self):
        boxes = [(0.6, 1.1, -0.18, 0.18), (1.8, 2.25, 0.35, 0.8),
                 (-1, 6, -1.0, -0.9)]
        sim, min_clear, _ = self.run_scene(boxes)
        self.assertEqual(str(sim.sm.phase), "GO")
        self.assertGreaterEqual(min_clear,
                                CFG.body_half_width_m + CFG.bypass_side_clearance_m)


if __name__ == "__main__":
    unittest.main(verbosity=2)
