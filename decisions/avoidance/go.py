"""GO：安全前进；带未回正航向时先回正。"""
import math

from decisions.types import (
    ADVANCE, ANNOUNCE, BLOCKED, CHECK, GO, HOLD, PLAN, REALIGN, RETURN, TURN,
    DeferredAction, PendingAction, StepContext, StepResult,
)
from decisions.rules import wrap_pi
from messages import ActionRequest, CommandResult, Perception, Pose, VoiceCue


class GoMixin:
    def _step_go(self, ctx: StepContext) -> StepResult:
        if not ctx.valid:
            why = "还没有感知数据" if ctx.perception is None else "感知失效/超时"
            return self._enter_hold(
                ctx.now, why, interrupt=True, keep_entry=False
            )

        if ctx.perception.distance_m < self.cfg.danger_m:
            return self._enter_hold(
                ctx.now,
                f"障碍物太近 {ctx.perception.distance_m:.2f}m",
                interrupt=True,
                keep_entry=False,
            )

        # 万一在 GO 里还带着未回正的航向（例如从 HOLD 恢复），先回正
        if self.bypass_active and abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            if ctx.pose is None:
                return self._enter_hold(
                    ctx.now, "需要回正但没有姿态数据",
                    interrupt=True, keep_entry=True,
                    voice=self.voice.announce(
                        VoiceCue("wait", reason="no_pose_wait"), ctx.now
                    ),
                )
            if ctx.perception.min_m >= self.cfg.rotate_clear_m:
                return self._start_realign(ctx.now, ctx.perception, ctx.pose)
            return self._enter_hold(
                ctx.now, "需要回正但净空不足",
                interrupt=True, keep_entry=True,
                voice=self.voice.announce(
                    VoiceCue("wait", reason="realign_space_wait"), ctx.now
                ),
            )

        if ctx.executor_busy:
            return StepResult(GO)

        return StepResult(GO, self.actions.forward())

