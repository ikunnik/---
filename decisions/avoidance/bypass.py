"""TURN/ADVANCE/CHECK：按阶段目标执行，动作完成后重新观察。"""
from dataclasses import replace

from messages import VoiceCue
from decisions.types import CHECK, HOLD, StepContext, StepResult


class BypassMixin:
    def _step_check(self, ctx: StepContext) -> StepResult:
        if ctx.now < self.settle_until:
            return StepResult(CHECK)

        if not ctx.fresh:
            return self._enter_hold(
                ctx.now, "动作后感知失效", interrupt=False, keep_entry=True
            )

        if ctx.perception.ts < self.action_finished_ts + self.cfg.settle_s:
            return StepResult(CHECK)

        if not ctx.valid:
            return self._enter_hold(
                ctx.now, "动作后深度无效", interrupt=False, keep_entry=True
            )

        if self.entry_yaw_rad is not None:
            if ctx.pose is None:
                return self._block(
                    ctx.now, "没有姿态数据，无法校验航向",
                    interrupt=True, reason="no_pose",
                )
            self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)

        mode = self.check_mode
        if mode == "avoid_stop":
            self.phase = HOLD
            self.history.clear()
            self.last_history_frame_ts = None
            return self._stop_result(HOLD, ctx.now, log="停车已确认，等待场景稳定")
        if mode == "turn":
            return self._check_after_turn(ctx)
        if mode == "advance":
            return self._check_after_advance(ctx)
        if mode == "realign":
            return self._check_after_realign(ctx)
        if mode == "return_turn":
            return self._check_after_return_turn(ctx)
        if mode == "return_advance":
            return self._check_after_return_advance(ctx)

        return self._block(
            ctx.now, f"未知检查模式 {mode}", interrupt=True, reason="internal"
        )

    def _check_after_turn(self, ctx: StepContext) -> StepResult:
        if self.probing_side:
            self.probing_side = False
            p = ctx.perception
            required = self.cfg.body_half_width_m + self.cfg.bypass_side_clearance_m
            if (not self._rotation_clear(p) or min(p.left_m, p.right_m) < required
                    or self._safe_step(p, self.diagonal_remaining) < self.diagonal_remaining):
                return self._replan(ctx, "探测方向无法容纳完整斜向段")
            step = self._start_segment(ctx, self.diagonal_remaining, "diagonal")
            if step.request is not None and step.request.kind == "advance":
                cue = self.voice.announce(VoiceCue("bypass", side=self.bypass_side,
                                          angle_deg=self.cfg.bypass_angle_deg), ctx.now, wait=False)
                return replace(step, voice=cue)
            return step
        # 初次试向只够走半段都不到时，先留在原观察点，给另一侧一次机会。
        # 回正后仍由 PLAN 用新帧复核另一侧的净空。
        if (self.alternate_side_available
                and self._safe_step(ctx.perception, self.diagonal_remaining)
                < self.diagonal_remaining * 0.5):
            return self._replan(ctx, "斜向净空不足半段，回正复核另一侧")
        return self._start_segment(ctx, self.diagonal_remaining, "diagonal")

    def _check_after_advance(self, ctx: StepContext) -> StepResult:
        if self._bypass_stage == "diagonal":
            extra = self._extend_diagonal_if_short(ctx)
            if extra is not None:
                return extra
        remaining = (self.diagonal_remaining if self._bypass_stage == "diagonal"
                     else self.parallel_remaining)
        if remaining >= self.cfg.bypass_step_min_m - 1e-6:
            return self._start_segment(ctx, remaining, self._bypass_stage)
        return self._start_realign(ctx.now, ctx.perception, ctx.pose)

    def _extend_diagonal_if_short(self, ctx: StepContext) -> StepResult | None:
        """斜向段走完但实测横向偏移不足时补一段（打滑、转角不到位、目标被截断）。

        旧实现只按“走了多少路”判断斜向结束，从不校验横向是否真的让开了障碍，
        所以转角偏小或里程计漂移时会贴着障碍前进。
        """
        shortfall = self._lateral_shortfall(ctx.pose)
        if shortfall is None or shortfall <= float(self.cfg.bypass_lateral_verify_m):
            return None
        if self.lateral_retries >= int(self.cfg.bypass_lateral_max_retry):
            return None
        extra = self._diagonal_from_lateral(shortfall)
        if extra < self.cfg.bypass_step_min_m - 1e-6:
            return None
        self.lateral_retries += 1
        step = self._start_segment(ctx, extra, "diagonal")
        if step.request is not None and step.request.kind == "advance":
            step = StepResult(
                step.state, step.request, interrupt=step.interrupt,
                log=f"横向偏移不足 {shortfall:.2f}m，追加斜向 {extra:.2f}m",
                voice=step.voice,
            )
        return step
