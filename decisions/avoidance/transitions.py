"""状态切换、停车/锁定、历史和稳定窗口。"""

import math

from decisions.types import BLOCKED, HOLD, StepContext, StepResult
from decisions.rules import wrap_pi
from messages import ActionRequest, Perception, Pose, VoiceCue


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


    def _history_append(self, now: float, perception: Perception,
                        pose: Pose | None = None):
        # 只统计不同相机帧，不把 50Hz 重复读取同一帧算成稳定证据。
        if self.last_history_frame_ts is not None and perception.ts <= self.last_history_frame_ts:
            return
        self.last_history_frame_ts = perception.ts
        if not self.history:
            # 历史被清空（刚进 HOLD/BLOCKED）后的第一帧：从这里开始算"停稳"
            self.hold_clean_ts = now
        if perception.valid:
            self.last_valid_m = perception.distance_m
            self.invalid_since = 0.0
            self.recover_attempts = 0
        # 同时记录自车位置：判定"障碍在接近"时必须扣掉机器人自己走过去的那部分距离。
        x = pose.x if pose is not None else float("nan")
        y = pose.y if pose is not None else float("nan")
        self.history.append(
            (now, perception.distance_m, perception.near_ratio, perception.valid, x, y)
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
        # 深度只有 2~3FPS：0.5s 窗口常常只有 1 帧，原来 len<3 会让状态机永远卡在 HOLD
        window = self._window(now, max(duration, 1.2))
        if len(window) < 2:
            return False
        if now - window[0][0] < duration:
            return False
        return all(h[3] and h[1] >= self.cfg.clear_m for h in window)


    def _stable_obstacle(self, now: float) -> bool:
        window = self._window(now, max(self.cfg.confirm_s, 1.2))
        if len(window) < 2:
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
        """判定障碍是否真的在靠近（而不是机器人自己走过去/深度噪声）。

        1) 只取有效帧，且窗口必须覆盖 confirm_s、帧数达标；
        2) 扣掉窗口内机器人自身位移（里程计 x/y），剩下的缩短量才算"它在动"；
        3) 连续 approach_confirm_n 次命中才成立，避免单帧跳变。
        """
        # 深度相机实测 2~8FPS，窗口放宽到 1.5s、跨度≥0.8s，才能拿到≥3个真帧
        # 刚停车的 1.2s 内机器人还在滑行，距离缩短是它自己造成的，不能算动态
        if now - self.hold_clean_ts < self.cfg.approach_settle_s:
            self.dynamic_hits = 0
            return False

        window = self._window(now, 1.5)
        need = max(3, self.cfg.approach_min_frames)
        samples = [h for h in window if h[3]]
        if len(samples) < need or now - samples[0][0] < 0.8:
            self.dynamic_hits = 0
            return False

        first, last = samples[0], samples[-1]
        drop = first[1] - last[1]

        ego = 0.0
        if math.isfinite(first[4]) and math.isfinite(first[5]) \
                and math.isfinite(last[4]) and math.isfinite(last[5]):
            ego = math.hypot(last[4] - first[4], last[5] - first[5])
        ego = max(0.0, ego - self.cfg.approach_ego_tol_m)

        closing = drop - ego
        self.last_approach_info = (closing, ego)

        hit = last[1] < self.cfg.approach_m and closing > self.cfg.approach_drop_m
        self.dynamic_hits = self.dynamic_hits + 1 if hit else 0
        if self.dynamic_hits >= self.cfg.approach_confirm_n:
            self.dynamic_until = now + 0.5
        return bool(hit or now < self.dynamic_until)


    def _warn(self, now: float, message: str) -> str | None:
        if now - self.last_dynamic_warn_ts < self.cfg.warn_log_interval_s:
            return None
        self.last_dynamic_warn_ts = now
        return message



    def _step_blocked(self, ctx: StepContext) -> StepResult:
        if self.blocked_latched:
            return self._stop_result(BLOCKED, ctx.now)
        if ctx.valid:
            self._history_append(ctx.now, ctx.perception, ctx.pose)
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

