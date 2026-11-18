"""AT_LINE：停稳 + 停止线倾角对齐。"""
import logging
import math
from dataclasses import replace

from decisions.safety import depth_valid
from decisions.traffic.constants import (
    ALIGN, AT_LINE, DONE, LINE, PASS, SIGNAL, WAIT_STOP_LINE,
)
from decisions.types import PendingAction, StepResult
from messages import Perception, StopLineState, TrafficLightState


class AlignMixin:
    def _step_at_line(self, now, line) -> StepResult:
        if self.phase == ALIGN:
            return self._stop(now)
        # 只有第一次（真正停下后）需要等停稳；对齐转向本身已经停过并落稳，
        # 再等一遍 stop_settle_s 只是白等。
        if self._align_attempts == 0 and now - self.stop_ts < self.cfg.stop_settle_s:
            return self._stop(now)
        if self._needs_align(now, line):
            return self._start_align(now, line)
        if self.traffic_active is not None:
            self.traffic_active.set()
            self._traffic_opened = True
        self.phase = SIGNAL
        logging.info("[停止线序列] 停稳，开启红绿灯识别")
        return self._stop(now, log="停稳，开启红绿灯识别")

    def _needs_align(self, now: float, line: StopLineState | None) -> bool:
        if not self.cfg.stop_line_align_enabled:
            return False
        if self._align_attempts >= self.cfg.stop_line_align_max_attempts:
            return False
        if line is None or not line.visible or not self._line_fresh(now, line):
            return False
        angle = float(getattr(line, "angle_deg", 0.0))
        return (math.isfinite(angle)
                and abs(angle) >= self.cfg.stop_line_align_threshold_deg)


    def _start_align(self, now: float, line: StopLineState) -> StepResult:
        angle = float(getattr(line, "angle_deg", 0.0))
        delta = (-float(self.cfg.stop_line_align_sign) * angle
                 * float(self.cfg.stop_line_align_gain))
        limit = float(self.cfg.stop_line_align_max_deg)
        delta = max(-limit, min(limit, delta))
        request = self.actions.turn(delta, purpose="与停止线对齐")
        request = replace(request, timeout_s=self.cfg.turn_timeout_s)
        self._action = PendingAction(
            kind="align",
            tag=request.cmd.tag,
            started_ts=now,
            timeout_s=self.cfg.turn_timeout_s,
            expected_yaw_delta_deg=delta,
        )
        self.phase = ALIGN
        logging.info(
            "[停止线序列] 停止线倾角 %+.1f°，对齐转向 %+.1f°",
            angle, delta,
        )
        return StepResult(
            ALIGN, request,
            log=f"停止线倾角 {angle:+.1f}°，对齐 {delta:+.1f}°",
        )

