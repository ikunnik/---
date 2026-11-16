"""SIGNAL：红/黄/绿、播报和超时。"""
import logging
from dataclasses import replace

from decisions.safety import depth_valid
from decisions.traffic.constants import (
    ALIGN, AT_LINE, DONE, LINE, PASS, SIGNAL, WAIT_STOP_LINE,
)
from decisions.types import PendingAction, StepResult
from messages import Perception, StopLineState, TrafficLightState, VoiceCue


class SignalMixin:
    def _step_signal(self, now, traffic) -> StepResult:
        if (
            traffic is None
            or (now - traffic.ts) > self.cfg.traffic_state_stale_s
        ):
            if self._signal_missing_since is None:
                self._signal_missing_since = now
            timeout = self.cfg.traffic_signal_timeout_s
            if (
                not self._signal_timeout_logged
                and now - self._signal_missing_since >= timeout
            ):
                self._signal_timeout_logged = True
                logging.warning(
                    "[停止线序列] 红绿灯数据 %.1fs 不可用，保持停车等待"
                    "（不交回避障，避免在红灯前恢复前进）",
                    timeout,
                )
            return self._stop(
                now, log=self._maybe_log(now, "等待红绿灯数据")
            )
        self._signal_missing_since = None
        self._signal_timeout_logged = False

        if traffic.stable and traffic.color in ("red", "yellow"):
            self._green_announced = False
            cue = self._signal_voice(now, traffic.color)
            return self._stop(
                now,
                log=self._maybe_log(
                    now,
                    "红灯，停车等待" if traffic.color == "red"
                    else "黄灯，停车等待",
                ),
                voice=cue,
            )
        if traffic.stable and traffic.color == "green":
            self._red_announced = False
            self._yellow_announced = False
            cue = self._signal_voice(now, "green", wait=False)
            self.phase = PASS
            self.pass_ts = now
            self._pass_started_ts = now
            self._pass_completed = False
            if self.cfg.traffic_pass_m <= 0:
                return self._complete_pass(now, "前进距离为 0")
            logging.info(
                "[停止线序列] 绿灯，通过 %.1fm（<%.2fm 提前掉头）",
                self.cfg.traffic_pass_m,
                self.cfg.traffic_pass_stop_m,
            )
            step = self._pass_request()
            if cue is not None:
                return StepResult(
                    step.state, step.request, step.interrupt,
                    step.log, cue,
                )
            return step
        return self._stop(
            now, log=self._maybe_log(now, "等待红灯/绿灯")
        )

    def _signal_voice(self, now: float, color: str, *,
                      wait: bool = True) -> VoiceCue | None:
        """红/黄/绿各只播一次；同一颜色重复稳定时不重复排队。"""
        if color == "red":
            if self._red_announced:
                return None
            self._red_announced = True
            cue = VoiceCue("traffic_red")
        elif color == "yellow":
            if self._yellow_announced:
                return None
            self._yellow_announced = True
            cue = VoiceCue("traffic_yellow")
        elif color == "green":
            if self._green_announced:
                return None
            self._green_announced = True
            cue = VoiceCue("traffic_green")
        else:
            return None
        # 每个颜色事件用独立标记去重；清掉旧 key，避免上一轮红灯
        # 挡住“绿灯后再变红”的第二次播报。
        self.sm.voice.clear_keys()
        return self.sm.voice.announce(cue, now, wait=wait)

