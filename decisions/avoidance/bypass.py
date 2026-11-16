"""TURN/ADVANCE/CHECK：按阶段目标执行，动作完成后重新观察。"""
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
        return self._start_segment(ctx, self.diagonal_remaining, "diagonal")

    def _check_after_advance(self, ctx: StepContext) -> StepResult:
        remaining = (self.diagonal_remaining if self._bypass_stage == "diagonal"
                     else self.parallel_remaining)
        if remaining >= self.cfg.bypass_step_min_m - 1e-6:
            return self._start_segment(ctx, remaining, self._bypass_stage)
        return self._start_realign(ctx.now, ctx.perception, ctx.pose)
