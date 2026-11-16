"""RETURN：横向偏差、切角回原线、回正。"""
import math

from decisions.types import CHECK, HOLD, RETURN, PendingAction, StepContext, StepResult
from messages import Pose


class PathReturnMixin:
    def _path_lateral(self, pose: Pose | None) -> float | None:
        """当前点到“绕障前航向直线”的横向偏差；左正右负。"""
        if pose is None or self.entry_yaw_rad is None:
            return None
        ex, ey = self.entry_x, self.entry_y
        if not (math.isfinite(ex) and math.isfinite(ey)
                and math.isfinite(pose.x) and math.isfinite(pose.y)):
            return None
        dx = float(pose.x) - ex
        dy = float(pose.y) - ey
        return -dx * math.sin(self.entry_yaw_rad) + dy * math.cos(self.entry_yaw_rad)


    def _maybe_start_path_return(self, ctx: StepContext) -> StepResult | None:
        if not self.cfg.path_return_enabled or not self.bypass_active:
            return None
        if self.pending is not None:
            return StepResult(self.phase)
        lateral = self._path_lateral(ctx.pose)
        if lateral is None or not self._pose_valid(ctx.pose, ctx.now):
            return self._block(ctx.now, "缺少原路线或当前位置，不能确认回线", True, "no_pose")
        if abs(lateral) <= self.cfg.path_return_tolerance_m:
            return None
        if self.path_return_attempts >= self.cfg.path_return_max_attempts:
            return self._block(ctx.now, "回线尝试耗尽，仍偏离原路线", True, "realign_limit")
        if not self._rotation_clear(ctx.perception):
            return self._block(ctx.now, "回线转身净空不足", True, "realign_space")
        side = "right" if lateral > 0 else "left"
        if not self._side_ok(ctx.perception, side):
            return self._defer_return(ctx)
        confirmation = self._confirm_return_clearance(ctx)
        if confirmation is not None:
            return confirmation
        phi = max(5.0, min(80.0, float(self.cfg.path_return_angle_deg)))
        # 目标是相对原路线的绝对切入角，不累加当前残留偏航。
        target = -math.copysign(phi, lateral)
        delta = target - self.heading_dev_deg
        request = self.actions.turn(delta, purpose="绕障后切回原路线")
        self.path_return_step_m = min(self.cfg.path_return_max_step_m,
                                     abs(lateral) / math.sin(math.radians(phi)))
        self.path_return_attempts += 1
        self.pending = PendingAction(
            kind="return_turn", tag=request.cmd.tag, started_ts=ctx.now,
            timeout_s=request.timeout_s, expected_yaw_delta_deg=delta,
            start_yaw_rad=ctx.pose.yaw_rad,
        )
        self.phase = RETURN
        return StepResult(RETURN, request,
                          log=f"横向偏差 {lateral:+.2f}m，转向原路线 {delta:+.1f}°")

    def _confirm_return_clearance(self, ctx: StepContext) -> StepResult | None:
        """侧区清空不等于已经超过障碍尾部；用实测纵向进展确认后再陡角回切。"""
        clearance = max(0.0, float(self.cfg.path_return_clearance_m))
        if clearance == 0:
            return None
        if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        if self.return_clear_origin is None:
            self.return_clear_origin = (ctx.pose.x, ctx.pose.y)
        dx = ctx.pose.x - self.return_clear_origin[0]
        dy = ctx.pose.y - self.return_clear_origin[1]
        progress = dx * math.cos(self.entry_yaw_rad) + dy * math.sin(self.entry_yaw_rad)
        remaining = clearance - progress
        if remaining <= 1e-6:
            return None
        self.parallel_remaining = max(self.cfg.bypass_step_min_m, remaining)
        return self._start_segment(ctx, self.parallel_remaining, "parallel")

    def _check_after_return_turn(self, ctx: StepContext) -> StepResult:
        lateral = self._path_lateral(ctx.pose)
        if lateral is None or not self._pose_valid(ctx.pose, ctx.now):
            return self._block(ctx.now, "回线途中位置失效", True, "no_pose")
        if abs(lateral) <= self.cfg.path_return_tolerance_m:
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        inward = math.sin(math.radians(self.heading_dev_deg))
        if lateral * inward >= 0 or abs(inward) < 0.05:
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        step = min(self.cfg.path_return_max_step_m, abs(lateral / inward))
        return self._start_segment(ctx, step, "return", returning=True)

    def _check_after_return_advance(self, ctx: StepContext) -> StepResult:
        # 动作完成后按实测偏差检查是否需要补足剩余路程。
        return self._check_after_return_turn(ctx)

    def _check_after_realign(self, ctx: StepContext) -> StepResult:
        if self.replan_after_realign:
            self.replan_after_realign = False
            self.phase = HOLD
            self.history.clear()
            self.last_history_frame_ts = None
            return self._stop_result(HOLD, ctx.now, log="已恢复原航向，重新确认障碍")
        if self.return_blocked:
            self.return_blocked = False
            self.parallel_remaining = self.cfg.min_bypass_advance_m
            self._bypass_stage = "parallel"
        elif self._bypass_stage == "diagonal":
            self.parallel_remaining = self.cfg.min_bypass_advance_m
            self._bypass_stage = "parallel"
        if self._bypass_stage == "parallel" and self.parallel_remaining >= self.cfg.bypass_step_min_m - 1e-6:
            return self._start_segment(ctx, self.parallel_remaining, "parallel")
        path_step = self._maybe_start_path_return(ctx)
        if path_step is not None:
            return path_step
        if ctx.perception.distance_m >= self.cfg.clear_m:
            return self._complete_bypass(ctx, CHECK, "已绕回原路线并回正，恢复前进")
        return self._replan(ctx, "原路前方仍有障碍，保留原路线重新规划")
