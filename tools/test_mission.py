"""调度器串联回归：traffic → uturn → lane → stopped；失败锁停、人工切换退出串联。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from workers.supervisor import ControllerStatus, ControllerSupervisor


class MissionTests(unittest.TestCase):
    def make(self, mission=True, modes=None):
        return ControllerSupervisor(
            CFG, 'traffic', mission=mission,
            available_modes=modes or {'traffic', 'uturn', 'lane', 'avoidance'},
        )

    def report(self, sup, state):
        command, _ = sup.snapshot()
        sup.status.set(ControllerStatus(command.revision, command.mode, state))
        sup.tick()
        return sup.snapshot()[0]

    def test_mission_advances_traffic_uturn_lane_stopped(self):
        sup = self.make()
        self.assertEqual(sup.snapshot()[0].mode, 'traffic')
        self.assertEqual(self.report(sup, 'completed').mode, 'uturn')
        self.assertEqual(self.report(sup, 'completed').mode, 'lane')
        self.assertEqual(self.report(sup, 'completed').mode, 'stopped')
        self.assertEqual(self.report(sup, 'completed').mode, 'stopped')

    def test_failure_stops_and_disables_mission(self):
        sup = self.make()
        self.report(sup, 'completed')          # traffic -> uturn
        self.assertEqual(self.report(sup, 'failed').mode, 'stopped')
        sup.enable('uturn')                     # 人工恢复
        self.assertEqual(self.report(sup, 'completed').mode, 'stopped')

    def test_manual_enable_leaves_mission(self):
        sup = self.make()
        sup.enable('avoidance')
        self.assertEqual(sup.snapshot()[0].mode, 'avoidance')
        self.assertEqual(self.report(sup, 'completed').mode, 'stopped')

    def test_enable_rejects_unavailable_mode(self):
        sup = self.make(mission=False, modes={'traffic'})
        with self.assertRaises(ValueError):
            sup.enable('lane')

    def test_mission_requires_all_stages(self):
        with self.assertRaises(ValueError):
            ControllerSupervisor(CFG, 'traffic', mission=True,
                                 available_modes={'traffic', 'uturn'})


if __name__ == '__main__':
    unittest.main(verbosity=2)
