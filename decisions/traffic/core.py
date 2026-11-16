"""停止线序列核心：状态、守卫、动作结果和生命周期。"""
import logging
from dataclasses import replace

from actions import Actions
from decisions.safety import depth_valid, stop_line_alive, stop_line_visible
from decisions.traffic.constants import (
    ALIGN, AT_LINE, DONE, LINE, PASS, SIGNAL, WAIT_STOP_LINE,
)
from decisions.types import GO, HOLD, PendingAction, StepResult
from decisions.voice import WAITING
from messages import (
    ActionRequest, Perception, StopLineState, TrafficLightState, VoiceCue,
)


class TrafficSequenceBase:
    def __init__(self, cfg, traffic_active, sm):
        self.cfg = cfg
        self.actions = Actions(cfg)
        self.traffic_active = traffic_active
        self.sm = sm
        self.active = False
        self.done = False
        self.completed = False  # 只有正常完成通过才为True；异常退出不算通过
        self.phase = DONE
        self.stop_ts = 0.0
        self.pass_ts = 0.0
        self.last_stop_ts = -1e9
        self._last_log_ts = -1e9
        self._hard_stopped = False
        self.progress_ts = 0.0
        self.last_row = 0.0
        self._action: PendingAction | None = None
        self._invalid_since: float | None = None
        self._voice_waiting = False
        self._traffic_opened = False
        self._red_announced = False
        self._yellow_announced = False
        self._green_announced = False
        self._signal_missing_since: float | None = None
        self._signal_timeout_logged = False
        self._latch_reason = ""
        self._latch_voice_sent = False
        self._sensor_lost_since: float | None = None
        self._sensor_voice_sent = False
        self._pass_completed = False
        self._pass_started_ts = 0.0
        self._align_attempts = 0


    def maybe_start(self, now: float, line: StopLineState | None) -> bool:
        """停止线可见且新鲜、避障空闲时接管；已通过一次后不再重复触发。"""
        if self.active or self.done:
            return False
        if self.sm.has_pending_action or self.sm.voice_pending:
            return False
        if not self._line_fresh(now, line):
            return False
        if self.sm.phase not in (GO, HOLD):
            return False

        self.active = True
        self.phase = LINE
        self.stop_ts = 0.0
        self.pass_ts = 0.0
        self.last_stop_ts = -1e9
        self._last_log_ts = now
        self._hard_stopped = False
        self.progress_ts = now
        self.last_row = line.row
        self._action = None
        self._invalid_since = None
        self._voice_waiting = False
        self._traffic_opened = False
        self._red_announced = False
        self._yellow_announced = False
        self._green_announced = False
        self._signal_missing_since = None
        self._signal_timeout_logged = False
        self._latch_reason = ""
        self._latch_voice_sent = False
        self._sensor_lost_since = None
        self._sensor_voice_sent = False
        self._pass_completed = False
        self._pass_started_ts = 0.0
        self._align_attempts = 0
        logging.info(
            "[停止线序列] 启动（row=%.3f, trigger=%.3f）",
            line.row, self.cfg.stop_line_trigger_row,
        )
        return True


    def guard_sensor(self, now: float, line: StopLineState | None,
                     line_ts: float,
                     voice_done: bool = False) -> StepResult | None:
        """序列还没接管时，确认停止线检测器真的在出数据。

        区分两种情况（这是之前真机静默失败的根因）：

        - 新鲜状态：即使 ``visible=False`` 也算健康，说明检测器在跑，
          只是线还没进入视野，允许普通避障继续接近；
        - 完全没有数据，或 ``Latest`` 超过 ``stop_line_stale_s`` 没更新：
          检测器死了/卡住了，立即打断前进并停车，绝不交回普通避障；
          持续超过 ``traffic_stall_s`` 则进入 WAIT_STOP_LINE 永久锁存。

        返回 None 表示健康；返回 StepResult 表示必须停车。
        """
        if self.active or self.done:
            return None

        alive = stop_line_alive(self.cfg, line, line_ts, now)
        if alive:
            if self._sensor_lost_since is not None:
                logging.info("[停止线序列] 停止线检测恢复，继续接近")
            self._sensor_lost_since = None
            self._sensor_voice_sent = False
            return None

        # 守卫接管时普通避障的 step() 不再运行，这里补一次语音门控轮询，
        # 否则失效播报的 pending 永远不会结束。
        if self.sm.voice_pending:
            self.sm.voice.poll(now, voice_done)

        if self._sensor_lost_since is None:
            self._sensor_lost_since = now
            self._last_log_ts = now
            logging.error(
                "[停止线序列] 停止线检测无数据/已过期，立即停车"
                "（不交回普通避障继续前进）"
            )
            # 普通避障可能正挂着一条 forward；清掉它，避免检测器恢复后
            # 收到旧动作的超时结果，被误判成避障失败。
            self.sm.reset(now, keep_voice=True)
            cue = None
            if not self._sensor_voice_sent:
                self._sensor_voice_sent = True
                cue = self.sm.voice.announce(
                    VoiceCue("wait", reason="stop_line_sensor_wait"), now
                )
            return self._stop(
                now, interrupt=True,
                log="停止线检测失效，停车", voice=cue,
            )

        lost = now - self._sensor_lost_since
        if lost >= self.cfg.traffic_stall_s:
            self.active = True
            self.phase = WAIT_STOP_LINE
            self._latch_reason = "停止线检测失效"
            self._action = None
            # 首次失效时已经播报过，这里不重复。
            self._latch_voice_sent = True
            logging.error(
                "[停止线序列] 停止线检测失效 %.1fs，锁存停车"
                "（不交回避障，需人工处理）",
                lost,
            )
            return self._stop(
                now, interrupt=True,
                log="停止线检测失效，锁存停车（不交回避障）",
            )

        return self._stop(
            now,
            log=self._maybe_log(
                now, f"停止线检测失效 {lost:.1f}s，保持停车"
            ),
        )


    @property
    def has_pending_action(self) -> bool:
        """是否还在等序列自己发出的动作结果。"""
        return self._action is not None


    def note_sent(self, request: ActionRequest, now: float):
        """DecisionWorker 真正把动作交给执行器后登记，等结果用。"""
        self._action = PendingAction(
            kind=request.kind,
            tag=request.cmd.tag,
            started_ts=now,
            timeout_s=request.timeout_s or self.cfg.traffic_action_timeout_s,
        )


    def _line_fresh(self, now: float, line: StopLineState | None) -> bool:
        return stop_line_visible(self.cfg, line, now)


    def _maybe_log(self, now: float, message: str):
        if now - self._last_log_ts >= 1.0:
            self._last_log_ts = now
            return message
        return None


    def _abort(self, now: float, reason: str) -> StepResult:
        """深度/动作异常太久：停车并把控制权交回避障。

        停止线丢失/无进展不走这里，改用 _latch()，避免交回后立刻恢复前进。
        """
        self.active = False
        self.done = True
        self.phase = DONE
        self._action = None
        if self._traffic_opened and self.traffic_active is not None:
            self.traffic_active.clear()
            self._traffic_opened = False
        self.sm.reset(now, keep_voice=True)
        logging.warning("[停止线序列] %s，交回避障", reason)
        return self._stop(
            now, interrupt=True, log=f"{reason}，停车交回避障"
        )


    def _latch(self, now: float, reason: str, voice_reason: str) -> StepResult:
        """停止线丢失/无进展：锁存停车，不交回避障。

        交回避障会让状态机立刻恢复前进，在停止线附近等于盲走。锁存后序列
        保持 active，持续补发停车，直到人工处理并重启程序。
        """
        first = self.phase != WAIT_STOP_LINE
        self.phase = WAIT_STOP_LINE
        self._action = None
        self._latch_reason = reason
        if first:
            logging.error(
                "[停止线序列] %s，锁存停车（不交回避障，需人工处理）",
                reason,
            )

        cue = None
        if not self._latch_voice_sent:
            self._latch_voice_sent = True
            cue = self.sm.voice.announce(
                VoiceCue("wait", reason=voice_reason), now
            )

        if first:
            return self._stop(
                now, interrupt=True,
                log=f"{reason}，锁存停车（不交回避障）",
                voice=cue,
            )
        return self._stop(
            now, log=self._maybe_log(now, f"{reason}，保持锁存停车"),
            voice=cue,
        )


    def _stop(self, now: float, interrupt: bool = False, log=None,
              voice: VoiceCue | None = None) -> StepResult:
        if interrupt or (now - self.last_stop_ts) >= self.cfg.stop_refresh_s:
            self.last_stop_ts = now
            return StepResult(
                self.phase,
                self.actions.stop(timeout_s=self.cfg.traffic_action_timeout_s),
                interrupt=interrupt,
                log=log,
                voice=voice,
            )
        return StepResult(self.phase, None, log=log, voice=voice)


    def _safety_stop(self, now: float, log: str | None) -> StepResult:
        """安全条件丢失：如果正在前进，必须立刻打断，而不是等补发周期。"""
        interrupt = self._action is not None and self._action.kind == "forward"
        if interrupt:
            self._action = None
        return self._stop(now, interrupt=interrupt, log=log)


    def _forward(self, purpose: str) -> StepResult:
        return StepResult(self.phase, self.actions.traffic_forward(purpose))


    def _service_action(self, now: float, result) -> StepResult:
        """等上一条 forward/stop 的最终结果；失败就停车或退出。"""
        action = self._action
        if action is None:
            return StepResult(self.phase)

        if result is not None and result.tag == action.tag:
            self._action = None
            if self.phase == ALIGN and action.kind == "align":
                self._align_attempts += 1
                self.phase = AT_LINE
                self.stop_ts = now
                if result.status == "ok":
                    logging.info("[停止线序列] 姿态对齐完成")
                    return self._stop(
                        now, interrupt=True, log="与停止线对齐完成"
                    )
                logging.warning(
                    "[停止线序列] 对齐转向 %s（%s），跳过本次对齐",
                    result.status, result.message,
                )
                return self._stop(
                    now, interrupt=True, log="对齐失败，继续等待红绿灯"
                )
            if result.status == "ok":
                if self.phase == PASS and action.kind == "forward":
                    self._pass_completed = True
                return StepResult(self.phase)

            if action.kind == "stop":
                # 停车结果失败：继续停车，不能因为“结果丢了”就往前走。
                logging.warning(
                    "[停止线序列] 停车动作 %s（%s），重新停车",
                    result.status, result.message,
                )
                return self._stop(
                    now, interrupt=True, log="停车动作失败，重新停车"
                )
            return self._abort(
                now, f"{action.kind} 动作失败（{result.status}）"
            )

        if (now - action.started_ts) >= action.timeout_s:
            self._action = None
            if self.phase == ALIGN and action.kind == "align":
                self._align_attempts += 1
                self.phase = AT_LINE
                self.stop_ts = now
                logging.warning("[停止线序列] 对齐转向超时，跳过本次对齐")
                return self._stop(
                    now, interrupt=True, log="对齐超时，继续等待红绿灯"
                )
            if action.kind == "stop":
                logging.warning("[停止线序列] 停车结果超时，重新停车")
                return self._stop(
                    now, interrupt=True, log="停车结果超时，重新停车"
                )
            return self._abort(now, f"{action.kind} 动作结果超时")

        return StepResult(self.phase)

