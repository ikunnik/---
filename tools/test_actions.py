"""统一动作接口的离线回归：SDK 参数、取消、失败和循环看门狗。"""
import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from actions import Actions
from comm import Mailbox
from config import CFG
from messages import ActionRequest
from robot_link import RobotLink
from workers.executor import MotionExecutor
from selftest import RecordingRobot


class ActionsTests(unittest.TestCase):
    def setUp(self):
        self.cfg = replace(CFG, motion_backend="sdk")
        self.actions = Actions(self.cfg)
        # build_request 只编码请求，不连接机器人。
        self.link = RobotLink(self.cfg.robot_ip)

    def velocity_payload(self, request):
        wire = self.link.build_request(request.cmd)
        self.assertTrue(wire.immediate_start)
        self.assertEqual(len(wire.sequence.motions), 1)
        motion = wire.sequence.motions[0]
        self.assertEqual(motion.motion_id, 'walk_velocity_seq')
        self.assertEqual(motion.parameters[0].key, 'velocity_sequence')
        return wire.sequence.loop, motion.parameters[0].string_value

    def test_mode_presets_keep_original_velocity_and_loop_semantics(self):
        a = self.actions
        cases = [
            (a.forward(), False, f'{CFG.forward_vx:g},0,0,{CFG.forward_s:g}'),
            (a.stop(), False, f'0,0,0,{CFG.stop_s:g}'),
            (a.traffic_forward('通过'), False,
             f'{CFG.traffic_vx:g},0,0,{CFG.traffic_forward_s:g}'),
            (a.lane(.4, .12, 9), True, '0.4,0,0.12,1.2;0.4,0,0.12,1.2'),
            (a.lane(.4, 0, 10, purpose='无盲道直行'), True,
             '0.4,0,0,1.2;0.4,0,0,1.2'),
            (a.velocity(0, .2, -.1, duration=.5), False, '0,0.2,-0.1,0.5'),
        ]
        for request, loop, payload in cases:
            with self.subTest(kind=request.kind, purpose=request.purpose):
                self.assertIsInstance(request, ActionRequest)
                self.assertEqual(self.velocity_payload(request), (loop, payload))
        self.assertEqual(a.traffic_forward('通过').timeout_s, CFG.traffic_action_timeout_s)
        self.assertEqual(a.lane(.4, 0, 1).timeout_s, 0)
        self.assertEqual(a.lane(.4, 0, 1).kind, 'lane_velocity')

    def test_distance_and_rotation_preserve_sdk_units_and_direction(self):
        for direction, code in [('forward', 0), ('backward', 1), ('left', 2), ('right', 3)]:
            with self.subTest(direction=direction):
                request = self.actions.move(direction, .8)
                wire = self.link.build_request(request.cmd)
                motion = wire.sequence.motions[0]
                self.assertFalse(wire.sequence.loop)
                self.assertEqual(motion.motion_id, 'line_walk')
                params = {p.key:p for p in motion.parameters}
                self.assertEqual(params['direction'].int_value, code)
                self.assertAlmostEqual(params['distance'].float_value, .8, places=6)
                self.assertEqual(request.timeout_s, CFG.step_timeout_s)
        for delta, code in [(30, 0), (-60, 1)]:
            with self.subTest(delta=delta):
                request = self.actions.turn(delta)
                motion = self.link.build_request(request.cmd).sequence.motions[0]
                params = {p.key:p for p in motion.parameters}
                self.assertEqual(motion.motion_id, 'rotation')
                self.assertEqual(params['direction'].int_value, code)
                self.assertEqual(params['angle'].float_value, abs(delta))
                self.assertEqual(request.expected_yaw_delta_deg, delta)
                realign = self.actions.realign(delta)
                self.assertEqual(realign.cmd, request.cmd)
                self.assertEqual(realign.kind, 'realign')
                self.assertEqual(realign.timeout_s, CFG.realign_timeout_s)
        circle = self.actions.circle('right', 2)
        self.assertEqual(dict(circle.cmd.params), {'direction': 1, 'angle': 720.0})
        self.assertEqual(circle.expected_yaw_delta_deg, -720)

    def test_shutdown_brake_keeps_zero_speed_and_no_stand_down(self):
        self.link._client = MagicMock()
        self.link.brake()
        self.link._client.velocity_sequence.assert_called_once_with(
            '0,0,0,0.5', stand_down_after=False, show_progress=False)

    def wait_for(self, predicate, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.005)
        self.fail('等待模拟执行器超时')

    def executor(self, robot, cfg=CFG):
        executor = MotionExecutor(cfg, robot, Mailbox())
        executor.start()
        def cleanup():
            executor.stop()
            executor.join(2)
        self.addCleanup(cleanup)
        return executor

    def finished(self, executor, tag, status):
        result, _ = executor.result()
        return result is not None and result.tag == tag and result.status == status

    def test_submit_replaces_running_command_and_invalidates_queued_commands(self):
        robot = RecordingRobot(cap_duration=1)
        executor = self.executor(robot)
        move = self.actions.turn(60)
        old_generation = executor.submit(move)
        self.wait_for(lambda: robot.count(move.cmd.tag) == 1)
        generation = executor.submit(self.actions.stop(), interrupt=True)
        self.wait_for(lambda: self.finished(executor, 'stop', 'ok'))
        self.assertGreater(generation, old_generation)
        self.assertGreaterEqual(robot.cancelled, 1)
        self.assertEqual(robot.rotate_count, 0)
        stale = self.actions.move('forward', .2)
        executor.mailbox.put((old_generation, stale.cmd))
        self.wait_for(lambda: self.finished(executor, stale.cmd.tag, 'cancelled'))
        self.assertEqual(robot.count(stale.cmd.tag), 0)

    def test_submit_failure_is_not_reported_as_success(self):
        robot = RecordingRobot(fail_rotations=True, cap_duration=.01)
        executor = self.executor(robot)
        request = self.actions.turn(-30)
        executor.submit(request)
        self.wait_for(lambda: self.finished(executor, request.cmd.tag, 'failed'))
        self.assertEqual(robot.rotate_count, 0)

    def test_any_loop_tag_gets_watchdog_stop_and_latches_all_motion_types(self):
        cfg = replace(CFG, lane_loop_heartbeat_timeout_s=.08)
        robot = RecordingRobot(cap_duration=5)
        executor = self.executor(robot, cfg)
        request = self.actions.velocity(.4, duration=1.2, loop=True, tag='custom_loop')
        executor.submit(request)
        self.wait_for(lambda: robot.count('custom_loop') == 1)
        self.wait_for(lambda: self.finished(executor, 'watchdog_stop', 'ok'))
        self.assertGreaterEqual(robot.cancelled, 1)
        for request in [self.actions.forward(), self.actions.turn(30), self.actions.lane(.4, 0, 1)]:
            before = robot.count('watchdog_latched_stop')
            executor.submit(request)
            self.wait_for(lambda: robot.count('watchdog_latched_stop') > before)
            self.wait_for(lambda: self.finished(executor, 'watchdog_latched_stop', 'ok'))
            self.assertEqual(robot.count(request.cmd.tag), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
