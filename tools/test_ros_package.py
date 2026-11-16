"""ROS package startup settings, without DDS or robot connections."""
import math
import sys
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import yaml
from app.cli import parse_args
from config import CFG
from rover_x1.settings import build_settings
from rover_x1.defaults import default_parameters, render_defaults

ROOT = Path(__file__).resolve().parents[1]


class RosPackageSettingsTests(unittest.TestCase):
    def build(self, *, mode='avoidance', values=None, **options):
        return build_settings(values or {}, mode=mode, dry_run=False,
                              traffic_light_now=False, share_dir=ROOT, **options)

    def test_modes_select_required_workers(self):
        for mode in ('mission', 'avoidance', 'traffic', 'lane'):
            with self.subTest(mode=mode):
                cfg, args = self.build(mode=mode)
                traffic = mode in ('mission', 'traffic')
                self.assertEqual(cfg.mission_enabled, mode == 'mission')
                self.assertEqual(cfg.lane_follow_enabled, mode == 'lane')
                self.assertEqual(cfg.traffic_sequence_enabled, traffic)
                self.assertEqual(cfg.stop_line_enabled, traffic)
                self.assertEqual(cfg.traffic_light_enabled, traffic)
                self.assertFalse(args.dry_run)

    def test_mode_overrides_conflicting_feature_flags(self):
        cfg, _ = self.build(mode='lane', values={
            'mission_enabled': True, 'traffic_sequence_enabled': True,
            'traffic_light_enabled': True, 'stop_line_enabled': True,
        })
        self.assertTrue(cfg.lane_follow_enabled)
        self.assertFalse(cfg.mission_enabled or cfg.traffic_sequence_enabled or cfg.stop_line_enabled)

    def test_generated_defaults_have_no_drift(self):
        self.assertEqual((ROOT/"config/rover.yaml").read_text(), render_defaults())
        values = yaml.safe_load(render_defaults())["/**"]["ros__parameters"]
        self.assertEqual(values, default_parameters())

    def test_yaml_preserves_current_tuning(self):
        values = yaml.safe_load((ROOT/'config/rover.yaml').read_text())['/**']['ros__parameters']
        cfg, _ = self.build(values=values)
        for name, value in values.items():
            if name not in ('voice_pack_dir', 'lane_profile', 'traffic_light_profile',
                            'stop_line_profile', 'dds_uri_file'):
                self.assertEqual(getattr(cfg, name), value, name)
        self.assertEqual(cfg.min_bypass_advance_m, CFG.min_bypass_advance_m)

    def test_assets_are_independent_of_working_directory(self):
        with patch('os.getcwd', return_value='/tmp'):
            cfg, _ = self.build()
        self.assertEqual(Path(cfg.voice_pack_dir), ROOT/'assets/voice')
        self.assertTrue(Path(cfg.stop_line_profile).is_file())
        self.assertTrue(Path(cfg.lane_profile).is_file())
        self.assertTrue(Path(cfg.traffic_light_profile).is_file())

    def test_parameter_overrides_reach_config(self):
        cfg, _ = self.build(values={'min_bypass_advance_m': 1.2, 'voice_enabled': False})
        self.assertEqual(cfg.min_bypass_advance_m, 1.2)
        self.assertFalse(cfg.voice_enabled)

    def test_bad_parameters_fail_before_runtime(self):
        for values in ({'typo': 1}, {'danger_m': math.nan}, {'danger_m': 1.1},
                       {'forward_vx': 9.0}, {'voice_enabled': 'false'},
                       {'ros2_rate_hz': 0.0}, {'motion_backend': 'unknown'},
                       {'lane_profile': '/nonexistent/rover-profile.json'}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.build(values=values)
        with self.assertRaises(ValueError):
            self.build(mode='unknown')

    def test_cli_can_parse_explicit_args_without_consuming_ros_args(self):
        with patch.object(sys, 'argv', ['controller', '--ros-args', '-p', 'check_config:=true']):
            args = parse_args(CFG, argv=['--no-voice'])
        self.assertTrue(args.no_voice)


if __name__ == '__main__':
    unittest.main(verbosity=2)
