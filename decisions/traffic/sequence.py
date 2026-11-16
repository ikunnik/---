"""停止线序列门面：按阶段分发给 line/pass/align/signal。"""
import logging

from decisions.safety import depth_valid
from decisions.traffic.align import AlignMixin
from decisions.traffic.constants import (
    ALIGN, AT_LINE, DONE, LINE, PASS, SIGNAL, WAIT_STOP_LINE,
)
from decisions.traffic.core import TrafficSequenceBase
from decisions.traffic.line import LineMixin
from decisions.traffic.pass_ import PassMixin
from decisions.traffic.signal_ import SignalMixin
from decisions.types import StepResult
from decisions.voice import WAITING
from messages import Perception, StopLineState, TrafficLightState


class TrafficSequence(LineMixin, PassMixin, AlignMixin, SignalMixin,
                      TrafficSequenceBase):
    def step(self, now: float, line: StopLineState | None,
             traffic: TrafficLightState | None, executor_busy: bool,
             perception: Perception | None = None,
             perception_ts: float | None = None, result=None,
             voice_done: bool = False) -> StepResult:
        if not self.active:
            return StepResult(DONE)

        # 语音播报期间：不产生任何运动。若正在前进，先打断一次。
        if self.sm.voice_pending:
            state, _ = self.sm.voice.poll(now, voice_done)
            if state == WAITING:
                if not self._voice_waiting:
                    self._voice_waiting = True
                    self._action = None
                    return self._stop(
                        now, interrupt=True, log="语音播报中，停车等待"
                    )
                return self._stop(now)
            self._voice_waiting = False
        else:
            self._voice_waiting = False

        if self.phase == WAIT_STOP_LINE:
            if self._action is not None:
                if self._action.kind == "stop":
                    return self._service_action(now, result)
                self._action = None
            return self._stop(
                now,
                log=self._maybe_log(now, f"{self._latch_reason}，保持锁存停车"),
            )

        if perception_ts is None:
            perception_ts = perception.ts if perception is not None else 0.0
        if not depth_valid(self.cfg, perception, perception_ts, now):
            if self._invalid_since is None:
                self._invalid_since = now
            if now - self._invalid_since >= self.cfg.traffic_stall_s:
                return self._abort(
                    now, f"深度数据失效 {self.cfg.traffic_stall_s:.1f}s"
                )
            return self._safety_stop(
                now, self._maybe_log(now, "深度数据失效，停车")
            )
        self._invalid_since = None

        if perception.distance_m < self.cfg.emergency_m:
            msg = self._maybe_log(
                now, f"前方 {perception.distance_m:.2f}m，序列紧急停车"
            )
            interrupt = not self._hard_stopped
            self._hard_stopped = True
            if interrupt:
                self._action = None
            return self._stop(now, interrupt=interrupt, log=msg)
        self._hard_stopped = False

        if self.phase == LINE:
            return self._step_line(now, line, result, executor_busy)
        if self.phase == PASS:
            return self._step_pass(
                now, traffic, perception, perception_ts, result
            )

        # 其他阶段：上一条动作还没结果时只等，不发新动作。
        if self._action is not None:
            return self._service_action(now, result)

        if self.phase in (AT_LINE, ALIGN):
            return self._step_at_line(now, line)
        if self.phase == SIGNAL:
            return self._step_signal(now, traffic)

        return StepResult(DONE)
