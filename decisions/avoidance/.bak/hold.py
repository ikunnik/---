"""HOLD：障碍稳定、动态等待、恢复检查。"""
import math

from decisions.types import GO, HOLD, PLAN, DeferredAction, StepContext, StepResult
from messages import VoiceCue



class HoldMixin:
    def _step_hold(self, ctx: StepContext) -> StepResult:
        if not ctx.valid:
            self.history.clear()
            return self._stop_result(HOLD, ctx.now)

        self._history_append(ctx.now, ctx.perception)

        captured = self._hold_capture_entry_yaw(ctx)
        if captured is not None:
            return captured

        if ctx.perception.distance_m >= self.cfg.clear_m:
            return self._hold_when_clear(ctx)
        if ctx.perception.distance_m < self.cfg.danger_m:
            return self._hold_when_dangerous(ctx)
        return self._hold_in_buffer_zone(ctx)


    def _hold_capture_entry_yaw(self, ctx: StepContext) -> StepResult | None:
        """启动时 0.8~1.0m 就挡路：先记录 entry_yaw，不要永远停着等。"""
        if not (
            ctx.perception.distance_m < self.cfg.clear_m
            and self.entry_yaw_rad is None
        ):
            return None

        if not self._pose_valid(ctx.pose, ctx.now):
            return self._stop_result(
                HOLD, ctx.now, self._warn(ctx.now, "没有新鲜位置/姿态数据，暂不绕障"),
                voice=self.voice.announce(
                    VoiceCue("wait", reason="no_pose_wait"), ctx.now
                ),
            )

        self.entry_yaw_rad = ctx.pose.yaw_rad
        self.entry_x = float(getattr(ctx.pose, "x", float("nan")))
        self.entry_y = float(getattr(ctx.pose, "y", float("nan")))
        self.path_return_attempts = 0
        self.heading_dev_deg = 0.0
        self.bypass_active = True
        self._bypass_stage = "diagonal"
        self.bypass_distance_m = 0.0
        self.avoid_cycles = 0
        self.realign_attempts = 0
        self.tried_angles.clear()
        return self._stop_result(
            HOLD,
            ctx.now,
            f"记录避障前航向 {math.degrees(ctx.pose.yaw_rad):.1f}°，等待障碍稳定",
        )


    def _hold_when_clear(self, ctx: StepContext) -> StepResult:
        """前方 >= clear_m：可能需要回正，安全后恢复 GO。"""
        if (
            self.entry_yaw_rad is not None
            and abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg
        ):
            if (
                self._stable_clear(ctx.now)
                and ctx.perception.min_m >= self.cfg.rotate_clear_m
            ):
                if ctx.pose is None:
                    return self._stop_result(
                        HOLD, ctx.now,
                        self._warn(ctx.now, "没有姿态数据，无法回正"),
                        voice=self.voice.announce(
                            VoiceCue("wait", reason="no_pose_wait"), ctx.now
                        ),
                    )
                return self._start_realign(
                    ctx.now, ctx.perception, ctx.pose
                )

            if self._stable_clear(ctx.now):
                return self._stop_result(
                    HOLD, ctx.now,
                    voice=self.voice.announce(
                        VoiceCue("wait", reason="realign_space_wait"), ctx.now
                    ),
                )
            return self._stop_result(HOLD, ctx.now)

        if self._stable_clear(ctx.now):
            if self.bypass_started_ts is not None and self.bypass_distance_m > 0:
                return self._check_after_realign(ctx)
            path_step = self._maybe_start_path_return(ctx)
            if path_step is not None:
                return path_step
            return self._complete_bypass(
                ctx, stop_state=HOLD, log="路径安全，恢复前进"
            )
        return self._stop_result(HOLD, ctx.now)


    def _hold_when_dangerous(self, ctx: StepContext) -> StepResult:
        """中心 < danger_m：动态等待，稳定且安全后进入 PLAN。"""
        if self._is_approaching(ctx.now):
            return self._dynamic_wait(ctx)

        if not self._stable_obstacle(ctx.now):
            return self._stop_result(HOLD, ctx.now)

        if self.entry_yaw_rad is None:
            return self._stop_result(
                HOLD, ctx.now, self._warn(ctx.now, "没有姿态数据，暂不绕障"),
                voice=self.voice.announce(
                    VoiceCue("wait", reason="no_pose_wait"), ctx.now
                ),
            )

        if ctx.perception.min_m < self.cfg.rotate_clear_m:
            return self._block(
                ctx.now,
                f"离障碍太近，无法原地转身 min={ctx.perception.min_m:.2f}m",
                interrupt=True,
                reason="too_close",
            )

        if self.avoid_cycles >= self.cfg.max_avoid_cycles:
            return self._block(
                ctx.now, "绕障次数达到上限", interrupt=True, reason="max_cycles"
            )

        if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            if self.return_blocked:
                return self._start_realign(ctx.now, ctx.perception, ctx.pose)
            return self._replan(ctx, "障碍稳定，重新规划")
        self.phase = PLAN
        return StepResult(PLAN, log="障碍稳定，开始规划绕行")


    def _hold_in_buffer_zone(self, ctx: StepContext) -> StepResult:
        """danger_m <= 中心 < clear_m：避障未结束时继续规划，否则等待。"""
        if not (self.bypass_active and self.entry_yaw_rad is not None):
            return self._stop_result(HOLD, ctx.now)

        if self._is_approaching(ctx.now):
            return self._dynamic_wait(ctx)

        if self._stable_obstacle(ctx.now):
            if self.avoid_cycles >= self.cfg.max_avoid_cycles:
                return self._block(
                    ctx.now, "绕障次数达到上限", interrupt=True,
                    reason="max_cycles",
                )
            if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
                if self.return_blocked:
                    return self._start_realign(ctx.now, ctx.perception, ctx.pose)
                return self._replan(ctx, "仍在障碍附近，继续规划")
            self.phase = PLAN
            return StepResult(PLAN, log="仍在障碍附近，继续规划")

        return self._stop_result(HOLD, ctx.now)


    def _dynamic_wait(self, ctx: StepContext) -> StepResult:
        return self._stop_result(
            HOLD, ctx.now,
            self._warn(ctx.now, "障碍物正在接近，停车等待"),
            voice=self.voice.announce(VoiceCue("dynamic"), ctx.now),
        )


    def _complete_bypass(self, ctx: StepContext, stop_state: str,
                         log: str) -> StepResult:
        """绕行完成：先播完成提示并停车，收到回执后再恢复 GO。"""
        if not ctx.valid or ctx.perception.distance_m < self.cfg.clear_m:
            return self._enter_hold(ctx.now, "完成前路径仍受阻", True, True)
        if self.bypass_active:
            if not self._pose_valid(ctx.pose, ctx.now):
                return self._block(ctx.now, "不能确认回线位置", True, "no_pose")
            self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)
            if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
                return self._start_realign(ctx.now, ctx.perception, ctx.pose)
            path_step = self._maybe_start_path_return(ctx)
            if path_step is not None:
                return path_step
        done_voice = None
        if (
            self.bypass_active
            and self.bypass_distance_m >= self.cfg.min_bypass_advance_m
        ):
            done_voice = self.voice.announce(
                VoiceCue("bypass_done"),
                ctx.now,
                DeferredAction("go", log=log),
            )

        if done_voice is not None and self.voice.pending:
            return self._stop_result(
                stop_state, ctx.now,
                log="播报完成提示，停车等待",
                voice=done_voice,
            )

        self.phase = GO
        self._reset_bypass(keep_entry=False)
        return StepResult(GO, log=log, voice=done_voice)

