#!/usr/bin/env python3
"""离线几何场景与故障回归：模拟位置、深度和动作回执，不连接机器人。"""
import math
import sys
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from actions import MAX_DISTANCE
from decisions.avoidance import AvoidStateMachine
from decisions.engine import DecisionEngine, DecisionInput
from decisions.types import BLOCKED, GO, HOLD, RETURN, PendingAction, StepContext
from messages import CommandResult, Motion, Perception, Pose
from motion.ros2_backend import Ros2MotionBackend


def context(now=100.0, x=0.0, y=0.5, yaw=0.0, center=3.0, left=3.0, right=3.0):
    p = Perception(center, True, now, left, right, True, True, min(center, left, right))
    return StepContext(now, p, now, Pose(yaw, now, x=x, y=y), None, False, True, True, False)


def anchored(cfg=CFG):
    sm = AvoidStateMachine(cfg)
    sm.entry_x = sm.entry_y = sm.entry_yaw_rad = 0.0
    sm.bypass_active = True
    sm.bypass_started_ts = 100.0
    sm._bypass_stage = 'parallel'
    sm.return_clear_origin = (-cfg.path_return_clearance_m, 0.0)
    return sm


def ray_box(x, y, angle, box):
    lo, hi = 0.0, 4.0
    for pos, direction, near, far in ((x, math.cos(angle), box[0], box[1]),
                                       (y, math.sin(angle), box[2], box[3])):
        if abs(direction) < 1e-9:
            if not near <= pos <= far:
                return 4.0
            continue
        a, b = (near-pos)/direction, (far-pos)/direction
        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
    return lo if hi >= lo else 4.0


class WorldSim:
    """矩形障碍 + 前向三个射线扇区；逐步积分，每一拍重新感知。"""
    def __init__(self, boxes, *, cfg=CFG, heading=0.0):
        self.cfg = cfg
        self.boxes = boxes
        self.sm = AvoidStateMachine(cfg)
        self.now = 100.0
        self.x = self.y = self.yaw = 0.0
        self.heading = heading
        self.result = None
        self.active = None
        self.requests = []
        self.trace = []
        self.origin = None
        self.collided = False
        self.sm.reset(self.now)

    def pose(self):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return Pose(self.yaw + self.heading, self.now,
                    x=10 + self.x*c-self.y*s, y=20+self.x*s+self.y*c)

    def sense(self):
        def sector(angles):
            return min(ray_box(self.x, self.y, self.yaw+math.radians(a), box)
                       for a in angles for box in self.boxes)
        center, left, right = sector((-8, 0, 8)), sector((20, 35, 50)), sector((-20, -35, -50))
        return Perception(center, True, self.now, left, right, True, True,
                          min(center, left, right), 0.1)

    def tick(self, dt=0.05):
        self.now += dt
        if self.active:
            request, remaining = self.active
            kind = request.kind
            if kind in ('advance', 'turn', 'realign'):
                params = dict(request.cmd.params)
                amount = min(remaining, (self.cfg.ros2_line_vx if kind == 'advance' else 60.0) * dt)
                if kind == 'advance':
                    self.x += amount * math.cos(self.yaw)
                    self.y += amount * math.sin(self.yaw)
                else:
                    self.yaw += math.radians(amount) * (1 if params['direction'] == 0 else -1)
                remaining -= amount
            else:
                remaining -= dt
            if remaining <= 1e-8:
                self.result = CommandResult(1, request.cmd.tag, 'ok', ts=self.now)
                self.active = None
            else:
                self.active = (request, remaining)
        for x0, x1, y0, y1 in self.boxes:
            # 验证路径至少保留 0.2m 的几何间隔；并非真实机器人完整碰撞模型。
            dist = math.hypot(max(x0-self.x, 0, self.x-x1), max(y0-self.y, 0, self.y-y1))
            self.collided |= dist < 0.2
        p = self.sense()
        out = self.sm.step(self.now, p, self.now, self.pose(), self.result, self.active is not None)
        self.result = None
        if self.origin is None and self.sm.entry_yaw_rad is not None:
            self.origin = (self.sm.entry_x, self.sm.entry_y, self.sm.entry_yaw_rad)
        if self.sm.entry_yaw_rad is not None and self.origin is not None:
            assert self.origin == (self.sm.entry_x, self.sm.entry_y, self.sm.entry_yaw_rad), '原路线被重置'
        if out.request:
            request = out.request
            self.requests.append((self.now, request, self.sm.pending.kind if self.sm.pending else None))
            if request.kind == 'advance':
                remaining = dict(request.cmd.params)['distance']
            elif request.kind in ('turn', 'realign'):
                remaining = dict(request.cmd.params)['angle']
            else:
                remaining = request.cmd.duration
            if request.kind != 'forward':
                self.active = (request, remaining)
        self.trace.append((round(self.now, 2), self.sm.phase, round(self.x, 2), round(self.y, 2), out.log))
        return out

    def run(self, limit=2500):
        for _ in range(limit):
            self.tick()
            if self.sm.phase == BLOCKED or (self.origin is not None and self.sm.phase == GO):
                break
        return self.sm.phase


class ComplexAvoidanceTests(unittest.TestCase):
    def assert_route(self, sim):
        state = sim.run()
        self.assertEqual(state, GO, str(sim.trace[-12:]))
        self.assertFalse(sim.collided)
        self.assertLessEqual(abs(sim.y), sim.cfg.path_return_tolerance_m)
        self.assertLessEqual(abs(math.degrees(sim.yaw)), sim.cfg.yaw_tol_deg)
        self.assertTrue(any(kind == 'return_advance' for _, _, kind in sim.requests))
        for _, request, _ in sim.requests:
            if request.kind == 'advance':
                limit = MAX_DISTANCE
                self.assertGreater(dict(request.cmd.params)['distance'], 0)
                self.assertLessEqual(dict(request.cmd.params)['distance'], limit + 1e-8)

    def test_left_right_and_rotated_original_route(self):
        for side in (1, -1):
            for heading in (0, math.radians(35)):
                with self.subTest(side=side, heading=heading):
                    wall = (-1, 6, -1.0, -0.9) if side == 1 else (-1, 6, 0.9, 1.0)
                    sim = WorldSim([(0.6, 1.2, -0.18, 0.18), wall], heading=heading)
                    self.assert_route(sim)
                    first = next(r for _, r, _ in sim.requests if r.kind == 'turn')
                    self.assertEqual(dict(first.cmd.params)['direction'], 0 if side == 1 else 1)

    def test_compact_route_reduces_longitudinal_space_without_widening(self):
        scenes = [
            ([(0.6, 1.2, -0.18, 0.18)], 0.70),
            ([(0.6, 2.8, -0.18, 0.18)], 0.95),
            ([(0.6, 1.1, -0.18, 0.18), (1.8, 2.25, 0.35, 0.8),
              (-1, 6, -1.0, -0.9)], 0.85),
        ]
        previous = replace(CFG, min_bypass_advance_m=1.0,
                           path_return_angle_deg=30, path_return_clearance_m=0)
        for boxes, ratio in scenes:
            with self.subTest(boxes=boxes):
                # 保留早期0.4m紧凑配置的空间对比；当前默认并肩距离为0.9m。
                compact = replace(CFG, min_bypass_advance_m=0.4)
                before, after = WorldSim(boxes, cfg=previous), WorldSim(boxes, cfg=compact)
                self.assert_route(before)
                self.assert_route(after)
                self.assertLess(after.x, before.x * ratio)
                self.assertLessEqual(max(abs(row[3]) for row in after.trace),
                                     max(abs(row[3]) for row in before.trace) + 0.01)

    def test_diagonal_full_target_ignores_old_segment_limit(self):
        cfg = replace(CFG, bypass_step_max_m=0.1)
        sm = anchored(cfg)
        sm.diagonal_remaining = 0.8
        out = sm._check_after_turn(context())
        self.assertAlmostEqual(dict(out.request.cmd.params)['distance'], 0.8)
        self.assertAlmostEqual(sm.pending.distance_m, 0.8)
        self.assertEqual(sm.pending.timeout_s, out.request.timeout_s)

    def test_obstructed_return_continues_full_parallel_target(self):
        sm = anchored()
        out = sm._maybe_start_path_return(context(right=0.5))
        self.assertEqual(out.request.kind, 'advance')
        self.assertAlmostEqual(dict(out.request.cmd.params)['distance'], CFG.min_bypass_advance_m)

    def test_parallel_target_runs_as_one_action_and_finishes_by_measured_progress(self):
        cfg = replace(CFG, min_bypass_advance_m=0.9, bypass_step_max_m=0.4)
        sm = anchored(cfg)
        sm._bypass_stage = 'diagonal'
        out = sm._check_after_realign(context())
        self.assertAlmostEqual(dict(out.request.cmd.params)['distance'], 0.9)
        self.assertAlmostEqual(sm.pending.distance_m, 0.9)
        self.assertEqual(sm.pending.timeout_s, out.request.timeout_s)
        self.assertGreater(sm.pending.timeout_s, 0.9 / cfg.ros2_line_vx)
        # 走过旧分段边界0.4m时继续原动作，不停下或另发动作。
        mid = context(now=101, x=0.4)
        continuing = sm.step(mid.now, mid.perception, mid.now, mid.pose, None, True)
        self.assertIsNone(continuing.request)
        self.assertFalse(continuing.interrupt)
        self.assertEqual(sm.pending.tag, out.request.cmd.tag)
        receipt = CommandResult(1, out.request.cmd.tag, 'ok', ts=102)
        end = context(now=102, x=0.9)
        sm.step(end.now, end.perception, end.now, end.pose, receipt, False)
        self.assertIsNone(sm.pending)
        self.assertAlmostEqual(sm.parallel_remaining, 0)
        self.assertAlmostEqual(sm.bypass_distance_m, 0.9)
        self.assertIsNone(sm._check_after_advance(context(now=102.3, x=0.9)).request)

    def test_parallel_still_respects_clearance_and_sdk_distance_limit(self):
        cfg = replace(CFG, danger_m=0.7, emergency_m=0.2, safety_buffer_m=0.1)
        for wanted, center, expected in ((0.9, 1.4, 0.6), (4.0, 6.0, MAX_DISTANCE)):
            with self.subTest(wanted=wanted):
                sm = anchored(cfg)
                out = sm._start_segment(context(center=center), wanted, 'parallel')
                self.assertAlmostEqual(dict(out.request.cmd.params)['distance'], expected)
                self.assertAlmostEqual(sm.pending.distance_m, expected)
        sm = anchored(cfg)
        out = sm._start_segment(context(center=0.8), 0.9, 'parallel')
        self.assertEqual(out.request.kind, 'stop')
        self.assertIsNone(sm.pending)

    def test_full_actions_keep_live_safety_and_timeout_checks(self):
        cfg = replace(CFG, danger_m=0.7, emergency_m=0.2)
        for stage in ('diagonal', 'parallel', 'return'):
            for fault in ('obstacle', 'emergency', 'depth', 'pose', 'timeout'):
                with self.subTest(stage=stage, fault=fault):
                    sm = anchored(cfg)
                    sm.parallel_remaining = sm.diagonal_remaining = 0.9
                    sm._start_segment(context(), 0.9, stage, returning=stage == 'return')
                    now = 100 + sm.pending.timeout_s + 0.01 if fault == 'timeout' else 100.2
                    center = 0.6 if fault == 'obstacle' else 0.1 if fault == 'emergency' else 3
                    ctx = context(now=now, x=0.1, center=center)
                    if fault == 'depth':
                        ctx = replace(ctx, perception=replace(ctx.perception, valid=False))
                    if fault == 'pose':
                        ctx = replace(ctx, pose=replace(ctx.pose, ts=98))
                    out = sm.step(ctx.now, ctx.perception, ctx.now, ctx.pose, None, True)
                    self.assertTrue(out.interrupt)
                    self.assertEqual(out.request.kind, 'stop')
                    if fault == 'obstacle':
                        self.assertEqual(sm.pending.kind, 'avoid_stop')
                        self.assertAlmostEqual(sm.bypass_distance_m, 0.1)
                    else:
                        self.assertEqual(out.state, BLOCKED)

    def test_return_clearance_requires_measured_forward_progress(self):
        sm = anchored()
        sm.return_clear_origin = None
        first = sm._maybe_start_path_return(context())
        self.assertEqual(first.request.kind, 'advance')
        self.assertEqual(sm.path_return_attempts, 0)
        sm.pending = None
        # 下发过0.4m不代表走够；实测只前进0.2m时仍不能回切。
        halfway = sm._maybe_start_path_return(context(now=101, x=0.2))
        self.assertEqual(halfway.request.kind, 'advance')
        self.assertAlmostEqual(dict(halfway.request.cmd.params)['distance'], 0.2)
        sm.pending = None
        ready = sm._maybe_start_path_return(context(now=102, x=0.4))
        self.assertEqual(ready.request.kind, 'turn')
        self.assertAlmostEqual(dict(ready.request.cmd.params)['angle'], 60)

    def test_obstructed_return_side_resets_clearance_confirmation(self):
        sm = anchored()
        out = sm._maybe_start_path_return(context(right=0.5))
        self.assertEqual(out.request.kind, 'advance')
        self.assertIsNone(sm.return_clear_origin)
        sm.pending = None
        out = sm._maybe_start_path_return(context(now=101, x=0.4))
        self.assertEqual(out.request.kind, 'advance')
        self.assertEqual(sm.return_clear_origin, (0.4, 0.5))

    def test_long_obstacle_defers_return_until_clear(self):
        sim = WorldSim([(0.6, 2.8, -0.18, 0.18)])
        self.assert_route(sim)
        self.assertGreater(sim.x, 2.8)
        self.assertGreater(sum(dict(r.cmd.params)['distance'] for _, r, _ in sim.requests if r.kind == 'advance'), 3.0)

    def test_second_obstacle_replans_and_keeps_original_route(self):
        sim = WorldSim([(0.6, 1.1, -0.18, 0.18), (1.8, 2.25, 0.35, 0.8), (-1, 6, -1.0, -0.9)])
        self.assert_route(sim)
        turns = [r for _, r, kind in sim.requests if kind == 'turn']
        self.assertGreaterEqual(len(turns), 2)

    def test_obstacle_appears_during_return_then_clears(self):
        class TransientSim(WorldSim):
            obstruction_until = None

            def sense(self):
                p = super().sense()
                pending = self.sm.pending
                if (self.obstruction_until is None and pending is not None
                        and pending.kind == 'return_advance'
                        and self.now - pending.started_ts > 0.2):
                    self.obstruction_until = self.now + 1.0
                if self.obstruction_until is not None and self.now < self.obstruction_until:
                    return replace(p, distance_m=0.6, min_m=min(p.min_m, 0.6))
                return p

        sim = TransientSim([(0.6, 1.2, -0.18, 0.18)])
        self.assert_route(sim)
        self.assertTrue(any(kind == 'avoid_stop' for _, _, kind in sim.requests))

    def test_failed_stop_and_stop_timeout_never_replan(self):
        for failure in ('failed', 'timeout'):
            sm = anchored()
            sm.heading_dev_deg = -30
            sm._check_after_return_turn(context(y=0.5, yaw=-math.pi/6))
            ctx = context(now=100.1, x=0.05, y=0.48, yaw=-math.pi/6, center=0.6)
            out = sm._service_pending(ctx)
            when = 100.2 if failure == 'failed' else 105
            receipt = CommandResult(1, out.request.cmd.tag, 'failed', ts=when) if failure == 'failed' else None
            out = sm._service_pending(replace(context(now=when), result=receipt))
            self.assertEqual(out.state, BLOCKED)
            self.assertTrue(sm.blocked_latched)
            self.assertIsNone(sm.pending)

    def test_return_failure_receipt_never_completes(self):
        sm = anchored()
        sm.heading_dev_deg = -30
        out = sm._check_after_return_turn(context(y=0.5, yaw=-math.pi/6))
        receipt = CommandResult(1, out.request.cmd.tag, 'failed', ts=100.2)
        out = sm._service_pending(replace(context(now=100.2), result=receipt))
        self.assertEqual(out.state, BLOCKED)
        self.assertEqual(out.voice.reason, 'advance_failed')

    def test_rotation_checks_new_close_side_obstacle(self):
        sm = anchored()
        out = sm._maybe_start_path_return(context())
        ctx = context(now=100.1, center=3, left=CFG.rotate_clear_m / 2)
        out = sm._service_pending(ctx)
        self.assertEqual(out.state, BLOCKED)
        self.assertTrue(out.interrupt)

    def test_success_requires_heading_and_not_only_lateral_position(self):
        sm = anchored()
        ctx = context(y=0.0, yaw=math.radians(25))
        out = sm._complete_bypass(ctx, RETURN, 'done')
        self.assertEqual(out.request.kind, 'realign')
        self.assertNotEqual(out.state, GO)

    def test_return_checks_depth_after_turn_before_advancing(self):
        sm = anchored()
        sm.heading_dev_deg = -30
        out = sm._check_after_return_turn(context(y=0.5, yaw=-math.pi/6, center=0.6))
        self.assertEqual(out.request.kind, 'realign')
        self.assertTrue(sm.return_blocked)

    def test_dead_end_stops_without_claiming_success(self):
        sim = WorldSim([(0.6, 3, -2, 2)])
        self.assertEqual(sim.run(), BLOCKED)
        self.assertFalse(sim.collided)
        self.assertFalse(any(r.kind == 'forward' for _, r, _ in sim.requests))

    def test_return_uses_actual_heading_and_full_target(self):
        sm = anchored()
        ctx = context(y=0.5, yaw=math.radians(-25))
        sm.heading_dev_deg = -25
        out = sm._check_after_return_turn(ctx)
        self.assertEqual(sm.pending.kind, 'return_advance')
        self.assertAlmostEqual(dict(out.request.cmd.params)['distance'], 0.5 / math.sin(math.radians(25)))
        self.assertEqual(sm.pending.timeout_s, out.request.timeout_s)

    def test_long_configured_return_reuses_distance_timeout(self):
        cfg = replace(CFG, path_return_max_step_m=1.5)
        sm = anchored(cfg)
        sm.heading_dev_deg = -30
        out = sm._check_after_return_turn(context(y=1.0, yaw=-math.pi/6))
        self.assertAlmostEqual(dict(out.request.cmd.params)['distance'], 1.5)
        self.assertGreater(sm.pending.timeout_s, 1.5 / cfg.ros2_line_vx)
        self.assertEqual(sm.pending.timeout_s, out.request.timeout_s)

    def test_success_waits_for_position_newer_than_receipt(self):
        sm = anchored()
        out = sm._start_segment(context(), 0.4, 'parallel')
        receipt = CommandResult(1, out.request.cmd.tag, 'ok', ts=100.2)
        stale_pose = context(now=100.1).pose
        ctx = replace(context(now=100.2), result=receipt, pose=stale_pose)
        self.assertIsNone(sm._service_pending(ctx).request)
        self.assertIsNotNone(sm.pending)
        ctx = replace(context(now=100.3, x=0.4), result=receipt)
        sm._service_pending(ctx)
        self.assertIsNone(sm.pending)
        self.assertNotEqual(sm.phase, BLOCKED)

    def test_return_emergency_and_invalid_depth_stop(self):
        for distance in (CFG.emergency_m / 2, 3):
            sm = anchored()
            ctx = context(y=0.5, yaw=-0.5)
            sm.heading_dev_deg = math.degrees(ctx.pose.yaw_rad)
            sm._check_after_return_turn(ctx)
            bad = context(now=100.1, center=distance, yaw=-0.5)
            if distance == 3:
                bad = replace(bad, fresh=False, valid=False)
            out = sm._service_pending(bad)
            self.assertEqual(out.state, BLOCKED)
            self.assertTrue(out.interrupt)
            self.assertEqual(out.request.kind, 'stop')

    def test_new_obstacle_waits_for_stop_receipt(self):
        sm = anchored()
        sm.heading_dev_deg = -30
        sm._check_after_return_turn(context(y=0.5, yaw=-math.pi/6))
        ctx = context(now=100.1, x=0.05, y=0.48, yaw=-math.pi/6, center=0.6)
        out = sm._service_pending(ctx)
        self.assertTrue(out.interrupt)
        self.assertEqual(sm.pending.kind, 'avoid_stop')
        self.assertEqual(out.request.kind, 'stop')
        waiting = sm.step(100.2, ctx.perception, 100.1, replace(ctx.pose, ts=100.2), None, True)
        self.assertIsNone(waiting.request)
        engine = DecisionEngine(CFG)
        engine.avoidance = sm
        receipt = CommandResult(1, out.request.cmd.tag, 'ok', ts=100.3)
        done = engine.evaluate(DecisionInput(100.3, replace(ctx.perception, ts=100.3), 100.3,
                                            replace(ctx.pose, ts=100.3), receipt, False))
        self.assertIsNone(sm.pending)
        self.assertNotEqual(done.step.state, GO)

    def test_stop_request_is_tracked_by_engine(self):
        engine = DecisionEngine(CFG)
        engine.avoidance = anchored()
        sm = engine.avoidance
        sm.heading_dev_deg = -30
        sm._check_after_return_turn(context(y=0.5, yaw=-math.pi/6))
        ctx = context(now=100.1, x=0.05, y=0.48, yaw=-math.pi/6, center=0.6)
        out = engine.evaluate(DecisionInput(ctx.now, ctx.perception, ctx.now, ctx.pose, None, True))
        self.assertTrue(out.track_result)
        self.assertEqual(out.step.request.kind, 'stop')

    def test_missing_position_and_exhausted_returns_never_complete(self):
        for fault in ('missing', 'stale', 'attempts'):
            sm = anchored()
            ctx = context()
            if fault == 'missing': ctx = replace(ctx, pose=Pose(0, 100))
            if fault == 'stale': ctx = replace(ctx, pose=replace(ctx.pose, ts=98))
            if fault == 'attempts': sm.path_return_attempts = CFG.path_return_max_attempts
            out = sm._complete_bypass(ctx, RETURN, 'done')
            self.assertEqual(out.state, BLOCKED, fault)
            self.assertTrue(sm.bypass_active)
            self.assertIsNotNone(sm.entry_yaw_rad)

    def test_wrong_way_success_or_no_progress_is_failure(self):
        for x in (0.0, -0.3):
            sm = anchored()
            ctx = context(y=0.5)
            out = sm._start_segment(ctx, 0.4, 'parallel')
            receipt = CommandResult(1, out.request.cmd.tag, 'ok', ts=100.2)
            after = replace(context(now=100.2, x=x), result=receipt)
            self.assertEqual(sm._service_pending(after).state, BLOCKED)

    def test_budget_includes_return_and_latches(self):
        sm = anchored(replace(CFG, max_bypass_distance_m=0.3))
        sm.heading_dev_deg = -30
        out = sm._check_after_return_turn(context(y=0.5, yaw=-math.pi/6))
        self.assertEqual(out.state, BLOCKED)
        for i in range(100):
            ctx = context(now=100.1+i*0.1)
            sm.step(ctx.now, ctx.perception, ctx.now, ctx.pose, None, False)
        self.assertEqual(sm.phase, BLOCKED)

    def test_total_timeout_stops_even_while_waiting(self):
        sm = anchored()
        ctx = context(now=100+CFG.bypass_timeout_s+1)
        out = sm.step(ctx.now, ctx.perception, ctx.now, ctx.pose, None, False)
        self.assertEqual(out.state, BLOCKED)
        self.assertEqual(out.request.kind, 'stop')

    def test_voice_resume_rechecks_new_obstacle_and_depth(self):
        for missing in (True, False):
            sm = AvoidStateMachine(CFG, wait_for_voice=True)
            ctx = context(y=0, center=0.6)
            sm._step_plan(ctx)
            self.assertTrue(sm.voice.pending)
            now = 103.0
            p = None if missing else replace(ctx.perception, ts=now, left_m=0.3, right_m=0.3, min_m=0.3)
            out = sm.step(now, p, now, replace(ctx.pose, ts=now), None, False, voice_done=True)
            self.assertEqual(out.request.kind, 'stop')
            self.assertIsNone(sm.pending)

    def test_repeated_frame_does_not_confirm_stable_obstacle(self):
        sm = AvoidStateMachine(CFG)
        p = context(center=0.6).perception
        for i in range(30): sm._history_append(100+i*.02, p)
        self.assertFalse(sm._stable_obstacle(100.58))

    def test_stale_source_depth_is_not_refreshed_by_mailbox(self):
        sm = AvoidStateMachine(CFG)
        ctx = context()
        out = sm.step(102, ctx.perception, 102, replace(ctx.pose, ts=102), None, False)
        self.assertEqual(out.request.kind, 'stop')

    def test_odom_loss_during_distance_move_clears_target(self):
        backend = Ros2MotionBackend('unused', cfg=CFG)
        with patch.object(backend, '_odom_fresh', side_effect=[(0, 0, 0, 100), None]):
            with self.assertRaisesRegex(RuntimeError, '里程计失效'):
                backend._run_line(Motion('line_walk', (('direction', 0), ('distance', 1))), threading.Event())
        self.assertEqual(backend._target, (0, 0, 0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
