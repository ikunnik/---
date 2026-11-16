"""绕障各阶段整段距离及左右回线回归。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from actions import MAX_DISTANCE
from decisions.avoidance.constants import (
    BYPASS_ANGLE_DEG, BYPASS_DIAGONAL_M, BYPASS_PARALLEL_M,
)
from tools.selftest import PureSim, fast_cfg, scene, GO, BLOCKED


class BypassTests(unittest.TestCase):
    def test_full_target_left_right_bypass(self):
        cfg = fast_cfg()
        for side in ('left', 'right'):
            with self.subTest(side=side):
                sim = PureSim(cfg, scene_=scene(
                    center=.5,
                    left=2 if side == 'left' else .45,
                    right=2 if side == 'right' else .45,
                    near=.1,
                ))
                advances = []
                turns = []
                for _ in range(600):
                    before = len(sim.requests)
                    sim.step(.1)
                    if len(sim.requests) > before:
                        req = sim.requests[-1]
                        if req.kind == 'turn':
                            params = dict(req.cmd.params)
                            turns.append(params)
                            if len(turns) == 1:
                                self.assertEqual(
                                    params['angle'], BYPASS_ANGLE_DEG
                                )
                                sim.scene = scene(
                                    center=2.0, left=2, right=2, near=.1
                                )
                        if req.kind == 'advance':
                            limit = MAX_DISTANCE
                            self.assertGreater(dict(req.cmd.params)['distance'], 0)
                            self.assertLessEqual(dict(req.cmd.params)['distance'], limit + 1e-8)
                            advances.append(
                                float(dict(req.cmd.params)['distance'])
                            )
                        if req.kind in ('turn', 'advance', 'realign'):
                            sim.resolve()
                    if sim.sm.phase in (GO, BLOCKED):
                        break
                self.assertEqual(sim.sm.phase, GO)
                self.assertGreater(len(advances), 2)
                self.assertAlmostEqual(advances[0], BYPASS_DIAGONAL_M)
                self.assertTrue(any(abs(d - cfg.min_bypass_advance_m) < 1e-8 for d in advances))
                self.assertGreater(sum(advances), BYPASS_DIAGONAL_M + cfg.min_bypass_advance_m)
                self.assertLessEqual(abs(sim.y), cfg.path_return_tolerance_m)
                self.assertLess(abs(math.degrees(sim.yaw)), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
