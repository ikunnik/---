"""状态切换、停车/锁定、历史和稳定窗口。"""

import math

from decisions.types import BLOCKED, HOLD, StepContext, StepResult
from decisions.rules import wrap_pi
from messages import ActionRequest, Perception, VoiceCue


class TransitionMixin:
    def _heading_dev(self, yaw_rad: float) -> float:
        if self.entry_yaw_rad is None:
            return 0.0
        return math.degrees(wrap_pi(yaw_rad - self.entry_yaw_rad))


    def _reset_bypass(self, keep_entry: bool):
        self.bypass_active = False
        self._reset_navigation()
        self._bypass_stage = "diagonal"
        self.bypass_distance_m = 0.0
        self.avoid_cycles = 0
        self.realign_attempts = 0
        self.tried_angles.clear()
        self.settle_until = 0.0
        self.action_finished_ts = 0.0
        self.check_mode = ""
        self.voice.clear_keys()
        if not keep_entry:
            self.entry_yaw_rad = None
            self.entry_x = float("nan")
            self.entry_y = float("nan")
            self.path_return_attempts = 0
            self.heading_dev_deg = 0.0


    def _enter_hold(self, now: float, why: str, interrupt: bool,
                    keep_entry: bool, voice: VoiceCue | None = None) -> StepResult:
        was = self.phase
        self.phase = HOLD
        self.pending = None
        if not keep_entry:
            self._reset_navigation()
            self.entry_yaw_rad = None
            self.entry_x = float("nan")
            self.entry_y = float("nan")
            self.path_return_attempts = 0
            self.bypass_active = False
            self.bypass_distance_m = 0.0
            self.avoid_cycles = 0
            self.realign_attempts = 0
            self.tried_angles.clear()
            self.heading_dev_deg = 0.0
            self.voice.clear_keys()
        self.history.clear()

        if interrupt:
            self.last_stop_cmd_ts = now
            return StepResult(
                HOLD,
                self._stop_request(),
                interrupt=True,
                log=(f"停车：{why}" if was != HOLD else None),
                voice=voice,
            )

        return self._stop_result(
            HOLD, now, (f"停车：{why}" if was != HOLD else None), voice=voice
        )


    def _block(self, now: float, why: str, interrupt: bool,
               reason: str = "internal") -> StepResult:
        was = self.phase
        self.phase = BLOCKED
        self.pending = None
        self.blocked_latched = self.blocked_latched or reason not in (
            "no_space", "too_close", "rotate_space", "realign_space")
        self.voice.reset()
        voice = None
        if was != BLOCKED:
            self.blocked_since = now
            self.history.clear()
            voice = self.voice.announce(
                VoiceCue("blocked", reason=reason), now
            )
        self.last_stop_cmd_ts = now
        return StepResult(
            BLOCKED,
            self._stop_request(),
            interrupt=interrupt,
            log=(f"锁定：{why}" if was != BLOCKED else None),
            voice=voice,
        )


    def _stop_result(self, state: str, now: float, log: str | None = None,
                     voice: VoiceCue | None = None) -> StepResult:
        if now - self.last_stop_cmd_ts >= self.cfg.stop_refresh_s:
            self.last_stop_cmd_ts = now
            return StepResult(state, self._stop_request(), log=log, voice=voice)
        return StepResult(state, None, log=log, voice=voice)


    def _stop_request(self) -> ActionRequest:
        return self.actions.stop()


    def _history_append(self, now: float, perception: Perception):
        # 只统计不同相机帧，不把 50Hz 重复读取同一帧算成稳定证据。
        if self.last_history_frame_ts is not None and perception.ts <= self.last_history_frame_ts:
            return
        self.last_history_frame_ts = perception.ts
        self.history.append(
            (now, perception.distance_m, perception.near_ratio, perception.valid)
        )
        cutoff = now - max(self.cfg.confirm_s, self.cfg.blocked_recheck_s) * 2.0 - 1.0
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()


    def _window(self, now: float, duration: float):
        # 多留一点历史再过滤：如果只取 <= duration，采样点会永远差一个周期，
        # 导致 now - window[0][0] >= duration 永远不成立。
        grace = max(0.20, 2.0 / max(self.cfg.decision_hz, 1.0))
        return [h for h in self.history if now - h[0] <= duration + grace]


    def _stable_clear(self, now: float) -> bool:
        return self._stable_clear_for(now, self.cfg.confirm_s)


    def _stable_clear_for(self, now: float, duration: float) -> bool:
        window = self._window(now, duration)
        if len(window) < 3:
            return False
        if now - window[0][0] < duration:
            return False
        return all(valid and center >= self.cfg.clear_m
                   for _, center, _, valid in window)


    def _stable_obstacle(self, now: float) -> bool:
        window = self._window(now, self.cfg.confirm_s)
        if len(window) < 3:
            return False
        if now - window[0][0] < self.cfg.confirm_s:
            return False
        centers = [h[1] for h in window if h[3]]
        if not centers:
            return False
        nears = [h[2] for h in window if h[3]]
        if max(centers) - min(centers) > self.cfg.stable_delta_m:
            return False
        if max(nears) - min(nears) > self.cfg.stable_near_delta:
            return False
        return True


    def _is_approaching(self, now: float) -> bool:
        window = self._window(now, self.cfg.confirm_s)
        if len(window) < 3:
            return False
        first = window[0][1]
        last = window[-1][1]
        return last < self.cfg.approach_m and (first - last) > 0.15


    def _warn(self, now: float, message: str) -> str | None:
        if now - self.last_dynamic_warn_ts < self.cfg.warn_log_interval_s:
            return None
        self.last_dynamic_warn_ts = now
        return message



    def _step_blocked(self, ctx: StepContext) -> StepResult:
        if self.blocked_latched:
            return self._stop_result(BLOCKED, ctx.now)
        if ctx.valid:
            self._history_append(ctx.now, ctx.perception)
            if (
                self._stable_clear_for(ctx.now, self.cfg.blocked_recheck_s)
                and ctx.perception.min_m >= self.cfg.rotate_clear_m
            ):
                self.phase = HOLD
                self.history.clear()
                self.voice.clear_keys()
                self.last_stop_cmd_ts = ctx.now - self.cfg.stop_refresh_s
                return StepResult(HOLD, log="障碍解除，恢复检查")
        else:
            self.history.clear()

        return self._stop_result(BLOCKED, ctx.now)

