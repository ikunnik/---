"""
停止线序列：camera2 黄线 -> 低速接近 -> 到线停车 -> 开红绿灯识别
            -> 红灯等 / 绿灯通过 -> 交给上层切换控制器。

本类只负责“停止线 + 红绿灯”这一段，不决定全局模式：单引擎由
DecisionEngine 交回下一控制器，多控制器串联由 ControllerSupervisor 推进。

与避障状态机的关系：为复用同一套播报门控，并避免在避障动作执行中接管，
构造时注入 AvoidStateMachine 实例；启动条件要求它空闲（无 pending、无播报、
处于 GO/HOLD）。切换控制器时上层会重建引擎，本类不持有全局状态。

安全约束：
    - 停止线/红绿灯/深度数据过期时一律停车；
    - 深度无效或距离进入 emergency_m 时立即打断前进；
    - 丢失停止线或行号长时间无进展时进入 WAIT_STOP_LINE 锁存停车，
      不把控制权交回普通前进（交回会立刻恢复前进，可能冲过停止线）；
    - 序列发出的 forward/stop 都要等执行器结果，失败/取消/超时则停车或退出；
    - 语音还在播报时不产生任何运动。

时间全部由外部传入，方便离线测试。
"""
import logging
import math
from dataclasses import replace

from actions import Actions
from decisions.safety import depth_valid, stop_line_alive, stop_line_visible
from messages import (
    ActionRequest,
    Perception,
    StopLineState,
    TrafficLightState,
    VoiceCue,
)
from decisions.types import (
    GO,
    HOLD,
    PendingAction,
    StepResult,
)
from decisions.voice import WAITING

LINE = "LINE_APPROACH"
AT_LINE = "AT_LINE"
ALIGN = "ALIGN"
SIGNAL = "SIGNAL"
PASS = "PASS"
WAIT_STOP_LINE = "WAIT_STOP_LINE"
DONE = "DONE"


class TrafficSequence:
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

    # ---------------------------------------------------------------- 启动
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

    # ---------------------------------------------------------------- 守卫
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

    # ---------------------------------------------------------------- 工具
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

    def _safety_stop(self, now: float, log: str | None) -> StepResult:
        """安全条件丢失：如果正在前进，必须立刻打断，而不是等补发周期。"""
        interrupt = self._action is not None and self._action.kind == "forward"
        if interrupt:
            self._action = None
        return self._stop(now, interrupt=interrupt, log=log)

    def _forward(self, purpose: str) -> StepResult:
        return StepResult(self.phase, self.actions.traffic_forward(purpose))

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

    # ---------------------------------------------------------------- 停止线对中
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
        delta = -float(self.cfg.stop_line_align_sign) * angle
        limit = float(self.cfg.stop_line_align_max_deg)
        delta = max(-limit, min(limit, delta))
        request = self.actions.turn(delta, purpose="与停止线对齐")
        request = replace(request, timeout_s=self.cfg.turn_timeout_s)
        self.pending = PendingAction(
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

    # ---------------------------------------------------------------- 动作结果
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

    # ---------------------------------------------------------------- 主步进
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

        # 锁存停车：停止线丢失/无进展后，后续任何异常都不能交回避障。
        if self.phase == WAIT_STOP_LINE:
            # 停车结果失败/超时要立刻重新打断；但绝不能让 _service_action
            # 走到 _abort()（那会把手动锁存又交回避障）。
            if self._action is not None:
                if self._action.kind == "stop":
                    return self._service_action(now, result)
                self._action = None
            return self._stop(
                now,
                log=self._maybe_log(now, f"{self._latch_reason}，保持锁存停车"),
            )

        # 深度无效/过期：立即停车；持续失效则退出序列交回避障。
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

        # 硬急停：序列期间也不允许撞上去（只停，不做避障绕行）
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

        # LINE 阶段的安全检查必须先于“等动作结果”：否则停止线到线/丢失时，
        # 还要等完当前 0.5s 前进才停车，可能已经冲过停止线。
        if self.phase == LINE:
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
                logging.info(
                    "[停止线序列] 到停止线 row=%.3f，停车", line.row
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

        # 通过期间仍然保留红绿灯检测：红灯/黄灯重新稳定时立刻停车，
        # 退回到 SIGNAL 等待下一次绿灯，而不是盲走完 traffic_pass_s。
        if self.phase == PASS:
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

        if self.phase == PASS:
            if perception_ts is None:
                perception_ts = perception.ts if perception is not None else 0.0
            # 2m 途中：越过起步保护时间后，前方 <0.7m 就提前停车掉头。
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

        # 其他阶段：上一条动作还没结果时只等，不发新动作。
        if self._action is not None:
            return self._service_action(now, result)

        if self.phase == AT_LINE:
            if now - self.stop_ts < self.cfg.stop_settle_s:
                return self._stop(now)
            if self._needs_align(now, line):
                return self._start_align(now, line)
            if self.traffic_active is not None:
                self.traffic_active.set()
                self._traffic_opened = True
            self.phase = SIGNAL
            logging.info("[停止线序列] 停稳，开启红绿灯识别")
            return self._stop(now, log="停稳，开启红绿灯识别")

        if self.phase == ALIGN:
            return self._stop(now)

        if self.phase == SIGNAL:
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

            # 黄灯和未知颜色都按“停车等待”处理；只有稳定绿灯才通过。
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
                # 通过期间保持 traffic_active，红灯复检才有数据。
                self._red_announced = False
                self._yellow_announced = False
                # 绿灯不等播报：短绿灯下等 2~3 秒语音会直接错过绿灯；
                # 播报与通过并行，红灯/黄灯仍然“播报期间停车”。
                cue = self._signal_voice(now, "green", wait=False)
                self.phase = PASS
                self.pass_ts = now
                self._pass_started_ts = now
                self._pass_completed = False
                if self.cfg.traffic_pass_m <= 0:
                    return self._complete_pass(now, "前进距离为 0")
                logging.info("[停止线序列] 绿灯，通过 %.1fm（<%.2fm 提前掉头）",
                             self.cfg.traffic_pass_m,
                             self.cfg.traffic_pass_stop_m)
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

        return StepResult(DONE)
