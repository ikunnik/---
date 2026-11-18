"""LINE_APPROACH：低速接近、row 触发、丢线/无进展锁存。"""
import logging
from dataclasses import replace

from decisions.safety import depth_valid
from decisions.traffic.constants import (
    ALIGN, AT_LINE, DONE, LINE, PASS, SIGNAL, WAIT_STOP_LINE,
)
from decisions.types import PendingAction, StepResult
from messages import Perception, StopLineState, TrafficLightState


class LineMixin:
    def _step_line(self, now, line, result, executor_busy) -> StepResult:
        if not self._line_fresh(now, line):
            if now - self.progress_ts >= self.cfg.traffic_stall_s:
                return self._latch(
                    now, f"停止线丢失 {self.cfg.traffic_stall_s:.1f}s",
                    "stop_line_lost_wait",
                )
            return self._safety_stop(
                now, self._maybe_log(now, "停止线丢失，停车等待")
            )
        if line.row >= self.cfg.stop_line_trigger_row:
            self.phase = AT_LINE
            self.stop_ts = now
            self._action = None
            # 红绿灯识别立刻打开，与"停稳 + 对齐"并行跑：否则绿灯还要再等
            # stop_settle_s + 对齐 + confirm_frames/confirm_s 才能进 PASS。
            opened = False
            if self.traffic_active is not None:
                opened = not self.traffic_active.is_set()
                self.traffic_active.set()
                self._traffic_opened = True
            logging.info(
                "[停止线序列] 到停止线 row=%.3f，停车（红绿灯识别已开启=%s）",
                line.row, opened,
            )
            return self._stop(
                now, interrupt=True,
                log=f"到停止线 row={line.row:.3f}，停车",
            )
        if line.row >= self.last_row + self.cfg.traffic_stall_row:
            self.last_row = line.row
            self.progress_ts = now
        elif now - self.progress_ts >= self.cfg.traffic_stall_s:
            return self._latch(
                now, f"停止线无进展 {self.cfg.traffic_stall_s:.1f}s",
                "stop_line_stall_wait",
            )
        if self._action is not None:
            return self._service_action(now, result)
        if executor_busy:
            return StepResult(LINE)
        return self._forward("停止线接近")
