"""动作结果、取消、超时、航向校验和 pending 生命周期。"""
import math

from decisions.types import CHECK, REALIGN, RETURN, PendingAction, StepContext, StepResult
from decisions.rules import wrap_pi
from messages import Perception, Pose


class PendingMixin:
    def _service_pending(self, ctx: StepContext) -> StepResult:
        p = self.pending

        veto = self._pending_safety_veto(ctx, p)
        if veto is not None:
            return veto

        if (
            ctx.result is None
            or ctx.result.status == "running"
            or ctx.result.tag != p.tag
        ):
            return self._pending_wait_or_timeout(ctx, p)

        if ctx.result.status != "ok":
            return self._fail_pending(
                ctx.now,
                p,
                f"{p.kind} {ctx.result.status}: {ctx.result.message}",
                f"{p.kind}_failed",
            )

        # 执行器完成与里程计发布不同步：先等一帧不早于回执的位姿，
        # 避免用动作结束前的位置误报无进展或错误航向。
        if ctx.pose is not None and ctx.pose.ts < ctx.result.ts:
            return self._pending_wait_or_timeout(ctx, p)

        actual_delta, failed = self._validate_action_result(ctx, p)
        if failed is not None:
            return failed

        if p.kind in ("advance", "return_advance"):
            drift = self._apply_advance_progress(ctx, p)
            if drift is not None:
                return drift

        if p.kind == "avoid_stop":
            failed = self._record_progress(ctx, p, completed=False)
            if failed is not None:
                return failed
        return self._finish_pending(ctx.now, p, actual_delta)


    def _pending_safety_veto(self, ctx: StepContext,
                             p: PendingAction) -> StepResult | None:
        """动作执行期间也必须盯住感知，危险时立即打断。"""
        if p.kind == "avoid_stop":
            return None
        if not ctx.fresh or not ctx.valid:
            return self._fail_pending(
                ctx.now, p, f"{p.kind} 途中感知失效", "perception_lost"
            )

        if p.kind in ("advance", "return_advance"):
            if ctx.perception.distance_m < self.cfg.emergency_m:
                return self._fail_pending(ctx.now, p, "前进/回切中触发紧急距离", "emergency")
            if ctx.perception.distance_m < self.cfg.danger_m:
                return self._stop_for_replan(ctx, p)
        elif p.kind in ("turn", "realign", "return_turn"):
            if not self._rotation_clear(ctx.perception):
                return self._fail_pending(ctx.now, p, "转向期间净空不足", "rotate_space")
        return None

    def _apply_advance_progress(self, ctx, p):
        failed = self._record_progress(ctx, p, completed=True)
        if failed is not None:
            return failed
        self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)
        if abs(self.heading_dev_deg) > self.cfg.max_heading_dev + self.cfg.yaw_tol_deg:
            return self._fail_pending(ctx.now, p, "前进航向超出预算", "heading_drift")
        return None

    def _pending_wait_or_timeout(self, ctx: StepContext,
                                 p: PendingAction) -> StepResult:
        if ctx.now - p.started_ts >= p.timeout_s:
            return self._fail_pending(
                ctx.now,
                p,
                f"{p.kind} 超时（{p.timeout_s:.1f}s）",
                f"{p.kind}_timeout",
            )
        return StepResult(self.phase)


    def _validate_action_result(
        self, ctx: StepContext, p: PendingAction
    ) -> tuple[float | None, StepResult | None]:
        """校验转向/回正的 yaw；返回 (实际角度, 失败结果或 None)。"""
        actual_delta = None
        if p.kind not in ("turn", "realign", "return_turn"):
            return actual_delta, None

        if ctx.pose is None or p.start_yaw_rad is None:
            return actual_delta, self._fail_pending(
                ctx.now,
                p,
                "动作完成但无法读取姿态，航向不可信",
                "pose_missing",
            )

        actual_delta = math.degrees(
            wrap_pi(ctx.pose.yaw_rad - p.start_yaw_rad)
        )
        expected = p.expected_yaw_delta_deg or 0.0
        tolerance = max(self.cfg.yaw_tol_deg, abs(expected) * 0.10)
        if abs(actual_delta - expected) > tolerance:
            return actual_delta, self._fail_pending(
                ctx.now,
                p,
                f"航向校验失败：期望 {expected:+.1f}°，实际 {actual_delta:+.1f}°",
                "yaw_mismatch",
            )

        self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)
        return actual_delta, None


    def _finish_pending(self, now: float, p: PendingAction,
                        actual_delta: float | None) -> StepResult:
        self.pending = None
        self.action_finished_ts = now
        self.settle_until = now + self.cfg.settle_s
        if p.kind == "avoid_stop":
            self.check_mode = "avoid_stop"
            self.phase = CHECK
            return StepResult(CHECK, log="已确认停车，等待新帧")
        if p.kind in ("return_turn", "return_advance"):
            self.check_mode = p.kind
            self.phase = RETURN
            if p.kind == "return_turn":
                log = f"绕回切入完成（实际 {actual_delta:+.1f}°），准备横移"
            else:
                log = "绕回横移完成，准备回正到原航向"
            return StepResult(RETURN, log=log)

        self.check_mode = (
            "turn" if p.kind == "turn"
            else "advance" if p.kind == "advance"
            else "realign"
        )
        self.phase = CHECK

        if p.kind == "turn":
            log = f"转向完成（实际 {actual_delta:+.1f}°），等待新帧"
        elif p.kind == "advance":
            log = f"前进完成，累计 {self.bypass_distance_m:.2f}m"
        else:
            log = f"回正完成（实际 {actual_delta:+.1f}°），等待新帧"
        return StepResult(CHECK, log=log)


    def _fail_pending(self, now: float, p: PendingAction,
                      why: str, reason: str) -> StepResult:
        self.pending = None
        if reason.startswith("return_advance_"):
            reason = reason.replace("return_advance_", "advance_", 1)
        elif reason.startswith("return_turn_"):
            reason = reason.replace("return_turn_", "turn_", 1)
        elif reason.startswith("avoid_stop_"):
            reason = "internal"
        return self._block(now, why, interrupt=True, reason=reason)


    def _start_realign(self, now: float, perception: Perception,
                       pose: Pose | None) -> StepResult:
        if not self._pose_valid(pose, now):
            return self._block(
                now, "没有姿态数据，无法回正",
                interrupt=True, reason="no_pose",
            )
        if self.entry_yaw_rad is None:
            return self._block(
                now, "没有避障前航向，无法回正",
                interrupt=True, reason="no_entry_yaw",
            )

        self.heading_dev_deg = self._heading_dev(pose.yaw_rad)
        if abs(self.heading_dev_deg) <= self.cfg.yaw_tol_deg:
            self.action_finished_ts = now
            self.settle_until = now + self.cfg.settle_s
            self.check_mode = "realign"
            self.phase = CHECK
            return StepResult(CHECK, log="航向已经对齐")

        if not self._rotation_clear(perception):
            return self._block(
                now,
                f"回正净空不足 min={perception.min_m:.2f}m",
                interrupt=True,
                reason="realign_space",
            )

        self.realign_attempts += 1
        if self.realign_attempts > self.cfg.max_realign_attempts:
            return self._block(
                now, "回正次数达到上限", interrupt=True, reason="realign_limit"
            )

        delta = -self.heading_dev_deg
        request = self.actions.realign(delta)
        cmd = request.cmd
        self.pending = PendingAction(
            kind="realign",
            tag=cmd.tag,
            started_ts=now,
            timeout_s=self.cfg.realign_timeout_s,
            expected_yaw_delta_deg=delta,
            start_yaw_rad=pose.yaw_rad,
        )
        self.phase = REALIGN
        return StepResult(
            REALIGN,
            request,
            log=(
                f"回正 {delta:+.0f}°"
                f"（目标 {math.degrees(self.entry_yaw_rad):.1f}°）"
            ),
        )

