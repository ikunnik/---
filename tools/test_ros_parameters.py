"""Real rclpy parameter contract, isolated ROS domain; no robot or SDK calls."""
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rover_x1.controller import declare_settings

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
except ImportError:
    rclpy = None

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(rclpy is None, 'ROS 2 environment is not sourced')
class RosParameterTests(unittest.TestCase):
    def setUp(self):
        self.env = patch.dict(os.environ, {'ROS_DOMAIN_ID': '231', 'ROS_LOCALHOST_ONLY': '1',
                                         'ROS_LOG_DIR': '/tmp/rover_ros_test_logs'})
        self.env.start()
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = None

    def tearDown(self):
        if self.node:
            self.node.destroy_node()
        self.context.shutdown()
        self.env.stop()

    def make_node(self, **overrides):
        self.node = Node('rover_test', context=self.context, use_global_arguments=False,
                         automatically_declare_parameters_from_overrides=True,
                         parameter_overrides=[Parameter(name, value=value) for name, value in overrides.items()])
        return self.node

    def test_effective_parameters_and_read_only_contract(self):
        node = self.make_node(mode='lane', min_bypass_advance_m=1.1, check_config=True)
        cfg, args, mode, check = declare_settings(node, ROOT)
        self.assertEqual(mode, 'lane')
        self.assertTrue(check)
        self.assertEqual(cfg.min_bypass_advance_m, 1.1)
        self.assertEqual(node.get_parameter('lane_follow_enabled').value, True)
        self.assertEqual(node.get_parameter('traffic_sequence_enabled').value, False)
        self.assertEqual(node.get_parameter('voice_pack_dir').value, str(ROOT/'assets/voice'))
        result = node.set_parameters_atomically([Parameter('min_bypass_advance_m', value=2.0)])
        self.assertFalse(result.successful)
        self.assertEqual(node.get_parameter('min_bypass_advance_m').value, 1.1)

    def test_motion_context_and_name_are_isolated_from_launch(self):
        from dataclasses import replace
        from geometry_msgs.msg import Twist
        from rclpy.qos import qos_profile_sensor_data
        from config import CFG
        from motion.ros2_backend import Ros2MotionBackend
        node = self.make_node()
        node.create_subscription(Twist, '/rover_packaging_test/velocity', lambda msg: None, qos_profile_sensor_data)
        cfg = replace(CFG, ros2_vel_topic='/rover_packaging_test/velocity',
                      ros2_odom_topic='/rover_packaging_test/odom', ros2_startup_timeout_s=3.0)
        backend = Ros2MotionBackend(cfg.robot_ip, cfg=cfg)
        try:
            with patch.object(sys, 'argv', ['controller', '--ros-args', '-r', '__node:=wrong_name']):
                backend._init_ros()  # Only isolated ROS topics; never SDK connect/stand/walk.
            self.assertEqual(backend._node.get_name(), 'rover_x1_motion')
            self.assertEqual(node.get_name(), 'rover_test')
        finally:
            backend.close()
        self.assertTrue(self.context.ok())

    def test_typo_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Unknown ROS parameters'):
            declare_settings(self.make_node(min_bypas_advance_m=1.0), ROOT)

    def test_wrong_type_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'must be float'):
            declare_settings(self.make_node(danger_m=1), ROOT)

    def test_sim_time_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'use_sim_time'):
            declare_settings(self.make_node(use_sim_time=True), ROOT)


if __name__ == '__main__':
    unittest.main(verbosity=2)
