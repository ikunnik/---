"""PLAN：固定角度（默认 60°），用左右空间决定往哪边，不再扫描候选角。"""
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
        # 已试的一侧失败，另一侧接近通道门槛时，可原地转向观察一次。
        # 此处只授权旋转；正对该方向取得新深度后才决定能否迈步。
        probe = len(self.tried_angles) == 1 and self.bypass_distance_m == 0
        if not options and probe:
            sign = -next(iter(self.tried_angles))
            side = "left" if sign > 0 else "right"
            distance = ctx.perception.left_m if sign > 0 else ctx.perception.right_m
            if distance >= self.cfg.min_passage_m - self.cfg.safety_buffer_m:
                options.append((side, sign, distance))
        if not options:
            return self._block(
                ctx.now, f"无可用绕行方向（左 {ctx.perception.left_m:.2f}m / "
                f"右 {ctx.perception.right_m:.2f}m，已试方向 {sorted(self.tried_angles)}）",
                interrupt=True, reason="no_space",
            )
        # 同一次绕障优先保持绕行侧，避免相近读数导致来回换边。
        close_first_choice = (self.bypass_side is None and len(options) == 2
                              and abs(ctx.perception.left_m - ctx.perception.right_m)
                              <= self.cfg.bypass_side_tie_m)
        if not close_first_choice:
            options.sort(key=lambda item: (item[0] == self.bypass_side, item[2]), reverse=True)
        side, sign, _ = options[0]
        delta = sign * float(self.cfg.bypass_angle_deg)

        if abs(self.heading_dev_deg + delta) > self.cfg.max_heading_dev:
            if len(options) < 2:
                return self._block(
                    ctx.now, "固定绕障角度超出航向预算",
                    interrupt=True, reason="no_space",
                )
            side, sign, _ = options[1]
            delta = sign * float(self.cfg.bypass_angle_deg)
            if abs(self.heading_dev_deg + delta) > self.cfg.max_heading_dev:
                return self._block(
                    ctx.now, "固定绕障角度超出航向预算",
                    interrupt=True, reason="no_space",
                )

        self.alternate_side_available = len(options) == 2
        return self._start_fixed_turn(ctx, side, delta, probe=probe)

    def _start_fixed_turn(self, ctx: StepContext, side: str,
                          delta: float, *, probe: bool = False) -> StepResult:
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
        # 斜向长度由横向偏移目标反算，不再固定 0.7m：目标 = 机身半宽 +
        # 障碍半宽 + 净空，且不超过该侧实测空间。已经偏出 entry 线时按累计量算。
        sign = 1.0 if delta > 0 else -1.0
        needed = self._bypass_lateral_target(ctx.perception, side)
        if probe:
            needed = self._bypass_lateral_target(None, side)
            if ctx.perception.near_ratio >= self.cfg.wall_near_ratio:
                needed += self.cfg.wall_extra_m
        current = self._path_lateral(ctx.pose)
        base = abs(current) if (current is not None and current * sign >= 0) else 0.0
        self.bypass_lateral_sign = sign
        self.bypass_lateral_target = base + needed
        self.diagonal_remaining = self._diagonal_from_lateral(needed)
        self.lateral_retries = 0
        log = (
            f"固定{'左' if delta > 0 else '右'}转 {abs(delta):.0f}°，"
            f"横向目标 {self.bypass_lateral_target:.2f}m"
            f"（左 {ctx.perception.left_m:.2f}m / "
            f"右 {ctx.perception.right_m:.2f}m）"
        )
        cue = None
        if probe:
            log = "原地探测另一侧，等待转后深度；" + log
        else:
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
        self.probing_side = probe
        self.bypass_side = side
        self._bypass_stage = "diagonal"
        self.pending = pending
        self.phase = TURN
        return StepResult(TURN, request, log=log, voice=cue)
