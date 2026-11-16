"""PASS：绿灯 2m、0.7m 提前停、红灯复检。"""
import logging
from dataclasses import replace

from decisions.safety import depth_valid
from decisions.traffic.constants import (
    ALIGN, AT_LINE, DONE, LINE, PASS, SIGNAL, WAIT_STOP_LINE,
)
from decisions.types import PendingAction, StepResult
from messages import Perception, StopLineState, TrafficLightState


class PassMixin:
    def _step_pass(self, now, traffic, perception, perception_ts,
                   result) -> StepResult:
        signal_fresh = (
            traffic is not None
            and (now - traffic.ts) <= self.cfg.traffic_state_stale_s
        )
        if (
            signal_fresh
            and traffic.stable
            and traffic.color in ("red", "yellow")
        ):
            self.phase = SIGNAL
            self._action = None
            self._pass_completed = False
            self._green_announced = False
            cue = self._signal_voice(now, traffic.color)
            logging.warning(
                "[停止线序列] 通过中检测到%s灯，停车等待",
                "红" if traffic.color == "red" else "黄",
            )
            return self._stop(
                now, interrupt=True,
                log=f"通过中{traffic.color}灯，停车等待", voice=cue,
            )

        if perception_ts is None:
            perception_ts = perception.ts if perception is not None else 0.0
        if (now - self._pass_started_ts >= self.cfg.traffic_pass_guard_s
                and depth_valid(self.cfg, perception, perception_ts, now)
                and perception.distance_m <= self.cfg.traffic_pass_stop_m):
            return self._complete_pass(
                now,
                f"前方 {perception.distance_m:.2f}m < "
                f"{self.cfg.traffic_pass_stop_m:.2f}m",
            )

        if self._action is not None:
            step = self._service_action(now, result)
            if self._pass_completed:
                return self._complete_pass(
                    now, f"已前进 {self.cfg.traffic_pass_m:.1f}m")
            return step
        return StepResult(PASS)

    def _pass_request(self) -> StepResult:
        """绿灯通过：按 SDK 距离走 traffic_pass_m 米，而不是按时间估算。"""
        request = self.actions.move(
            "forward", self.cfg.traffic_pass_m,
            purpose=f"绿灯通过 {self.cfg.traffic_pass_m:.1f}m",
        )
        request = replace(
            request, kind="forward",
            timeout_s=self.cfg.traffic_pass_timeout_s,
        )
        return StepResult(self.phase, request)


    def _complete_pass(self, now: float, reason: str = "") -> StepResult:
        self.active = False
        self.done = True
        self.completed = True
        self.phase = DONE
        self._action = None
        self._pass_completed = False
        if self.traffic_active is not None:
            self.traffic_active.clear()
            self._traffic_opened = False
        self.sm.reset(now, keep_voice=True)
        logging.info("[停止线序列] 通过完成（%s），交回避障", reason or "2m")
        return self._stop(
            now, interrupt=True, log=f"通过完成：{reason}" if reason else "通过完成，交回避障"
        )

