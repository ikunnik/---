"""PLAN：固定角度 ±45°，用左右空间决定往哪边，不再扫描候选角。"""
from decisions.avoidance.constants import BYPASS_ANGLE_DEG, BYPASS_DIAGONAL_M
from decisions.types import ANNOUNCE, TURN, DeferredAction, PendingAction, StepContext, StepResult
from messages import VoiceCue


class PlanMixin:
    def _step_plan(self, ctx: StepContext) -> StepResult:
        if not self._pose_valid(ctx.pose, ctx.now):
            return self._block(
                ctx.now, "没有姿态数据，无法规划绕行",
                interrupt=True, reason="no_pose",
            )
        if self.entry_yaw_rad is None:
            self.entry_yaw_rad = ctx.pose.yaw_rad
            self.entry_x = float(getattr(ctx.pose, "x", float("nan")))
            self.entry_y = float(getattr(ctx.pose, "y", float("nan")))
            self.path_return_attempts = 0
            self.bypass_active = True
            self._bypass_stage = "diagonal"
        self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)

        if self.bypass_started_ts is None:
            self.bypass_started_ts = ctx.now
        if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            self.replan_after_realign = True
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        if not self._rotation_clear(ctx.perception):
            return self._block(
                ctx.now,
                f"净空不足，不能转身 min={ctx.perception.min_m:.2f}m",
                interrupt=True,
                reason="rotate_space",
            )

        return self._plan_turn(ctx)

    def _side_ok(self, perception, side: str) -> bool:
        if side == "left":
            distance, valid = perception.left_m, perception.left_valid
        else:
            distance, valid = perception.right_m, perception.right_valid
        if not valid or distance != distance or distance == float("inf"):
            return False
        required = float(self.cfg.min_passage_m)
        if perception.near_ratio >= self.cfg.wall_near_ratio:
            required += float(self.cfg.wall_extra_m)
        return distance >= required

    def _plan_turn(self, ctx: StepContext) -> StepResult:
        left_ok = self._side_ok(ctx.perception, "left")
        right_ok = self._side_ok(ctx.perception, "right")

        # 固定角度，但保留左右空间判断；两侧都可走时选更空的一侧。
        options = []
        if left_ok and 1 not in self.tried_angles:
            options.append(("left", 1.0, ctx.perception.left_m))
        if right_ok and -1 not in self.tried_angles:
            options.append(("right", -1.0, ctx.perception.right_m))
        if not options:
            return self._block(
                ctx.now, "左右都没有足够空间，停车锁定",
                interrupt=True, reason="no_space",
            )
        # 同一次绕障优先保持绕行侧，避免相近读数导致来回换边。
        options.sort(key=lambda item: (item[0] == self.bypass_side, item[2]), reverse=True)
        side, sign, _ = options[0]
        delta = sign * BYPASS_ANGLE_DEG

        if abs(self.heading_dev_deg + delta) > self.cfg.max_heading_dev:
            if len(options) < 2:
                return self._block(
                    ctx.now, "固定绕障角度超出航向预算",
                    interrupt=True, reason="no_space",
                )
            side, sign, _ = options[1]
            delta = sign * BYPASS_ANGLE_DEG
            if abs(self.heading_dev_deg + delta) > self.cfg.max_heading_dev:
                return self._block(
                    ctx.now, "固定绕障角度超出航向预算",
                    interrupt=True, reason="no_space",
                )

        return self._start_fixed_turn(ctx, side, delta)

    def _start_fixed_turn(self, ctx: StepContext, side: str,
                          delta: float) -> StepResult:
        request = self.actions.turn(
            delta, purpose=f"绕障固定 {abs(delta):.0f}°"
        )
        pending = PendingAction(
            kind="turn",
            tag=request.cmd.tag,
            started_ts=ctx.now,
            timeout_s=self.cfg.turn_timeout_s,
            expected_yaw_delta_deg=delta,
            start_yaw_rad=ctx.pose.yaw_rad,
        )
        log = (
            f"固定{'左' if delta > 0 else '右'}转 {abs(delta):.0f}°"
            f"（左 {ctx.perception.left_m:.2f}m / "
            f"右 {ctx.perception.right_m:.2f}m）"
        )
        cue = self.voice.announce(
            VoiceCue("bypass", side=side, angle_deg=abs(delta)),
            ctx.now,
            DeferredAction("turn", request=request, pending=pending, log=log),
        )
        if cue is not None and self.voice.pending:
            return self._stop_result(
                ANNOUNCE, ctx.now,
                log="播报绕行提示，停车等待",
                voice=cue,
            )
        self.return_clear_origin = None
        self.bypass_side = side
        self.diagonal_remaining = BYPASS_DIAGONAL_M
        self._bypass_stage = "diagonal"
        self.pending = pending
        self.phase = TURN
        return StepResult(TURN, request, log=log, voice=cue)
