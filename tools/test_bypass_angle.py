"""复现 pp1.mp4：黄牌在正前方 0.66m、左侧 1.3m 是空的，狗左右看完却停车。

日志证据（21:20:33~21:20:41）：
  固定左转 40°（左 1.15 / 右 1.22）→ 回正；原地探测右 40°（左 0.66 / 右 1.22）
  → 回正 → 锁定"无可用绕行方向（左 1.27m / 右 0.69m，已试方向 [-1, 1]）"

根因：0.45m 宽的 A 字牌在 0.66~0.69m 处张角约 35°，固定 40° 转完后它仍被中心
锥体的边缘切到；距离取锥体内最近 5% 像素，所以 center 始终是 0.66 → _safe_step
永远算出 0 → 旧逻辑把这侧记成"已试"，两侧都试完就锁定停车。
本文件锁住新行为：同侧追加转角再看，而不是立刻放弃。
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from decisions.avoidance import AvoidStateMachine
from tools.test_bypass_lateral import context


def scene(center=0.66, left=1.27, right=1.29, near=0.85, dev=0.0):
    return context(center=center, left=left, right=right, near=near,
                   yaw=math.radians(dev))


class WideNearObstacleTests(unittest.TestCase):
    def test_turns_further_instead_of_stopping(self):
        sm = AvoidStateMachine(CFG)
        out = sm._step_plan(scene())
        self.assertEqual(out.request.kind, "turn")
        self.assertEqual(dict(out.request.cmd.params)["direction"], 0)  # 优先左绕
        self.assertEqual(sm.tried_angles, set())

        # 转完 40°：牌子仍在中心锥体边上（日志里 center 一直是 0.66）
        sm.heading_dev_deg = CFG.bypass_angle_deg
        first_len = sm.diagonal_remaining
        out = sm._check_after_turn(scene(left=1.48, right=0.64, near=0.32,
                                        dev=CFG.bypass_angle_deg))
        self.assertEqual(out.request.kind, "turn", out.log)
        self.assertEqual(dict(out.request.cmd.params)["direction"], 0)
        self.assertAlmostEqual(dict(out.request.cmd.params)["angle"],
                               CFG.bypass_angle_step_deg)
        self.assertEqual(sm.tried_angles, set())      # 左侧仍可用，没被写成"已试"
        self.assertLess(sm.diagonal_remaining, first_len)  # 更陡 → 前进量更短

        # 追加到 55°：牌子离开中心锥体 → 正常斜向，不再 BLOCKED
        sm.pending = None
        sm.heading_dev_deg = CFG.bypass_angle_deg + CFG.bypass_angle_step_deg
        out = sm._check_after_turn(scene(center=2.60, left=1.50, right=0.90, near=0.10,
                                        dev=sm.heading_dev_deg))
        self.assertEqual(out.request.kind, "advance", out.log)
        self.assertNotEqual(out.state, "BLOCKED")

    def test_escalation_stops_at_max_angle_then_gives_side_up(self):
        sm = AvoidStateMachine(CFG)
        sm._step_plan(scene())
        sm.heading_dev_deg = CFG.bypass_angle_deg
        out = None
        for _ in range(10):
            out = sm._check_after_turn(scene(left=1.48, right=0.64, near=0.32,
                                            dev=sm.heading_dev_deg))
            if out.request is None or out.request.kind != "turn":
                break
            sm.pending = None
            sm.heading_dev_deg += CFG.bypass_angle_step_deg
            self.assertLessEqual(abs(sm.heading_dev_deg), CFG.bypass_angle_max_deg)
        self.assertEqual(out.request.kind, "realign", out.log)
        self.assertEqual(sm.tried_angles, {1})
        self.assertLessEqual(abs(sm.heading_dev_deg), CFG.bypass_angle_max_deg)

    def test_no_escalation_once_it_has_actually_moved(self):
        sm = AvoidStateMachine(CFG)
        sm._step_plan(scene())
        sm.heading_dev_deg = CFG.bypass_angle_deg
        sm.bypass_distance_m = 0.35      # 已经斜着走了一段，说明不是转角不够
        out = sm._check_after_turn(scene(left=1.48, right=0.64, near=0.32,
                                        dev=CFG.bypass_angle_deg))
        self.assertEqual(out.request.kind, "realign", out.log)
        self.assertEqual(sm.tried_angles, {1})

    def test_clear_view_still_starts_diagonal_immediately(self):
        sm = AvoidStateMachine(CFG)
        sm._step_plan(scene())
        sm.heading_dev_deg = CFG.bypass_angle_deg
        out = sm._check_after_turn(scene(center=2.60, left=1.50, right=1.40, near=0.05,
                                        dev=CFG.bypass_angle_deg))
        self.assertEqual(out.request.kind, "advance", out.log)


if __name__ == "__main__":
    unittest.main(verbosity=2)
