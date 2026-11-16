"""决策线程：读取感知快照 → DecisionEngine.evaluate() → 提交动作/播报。

行为判断统一维护在 decisions/；此处只处理队列、线程、回执匹配和执行器通信。
"""
import logging
import queue
import threading
import time
from dataclasses import dataclass, replace

from actions import Actions
from comm import Latest, put_latest
from workers.supervisor import ControllerStatus
from messages import VoiceDone
from decisions.engine import DecisionEngine, DecisionInput
from decisions.types import BLOCKED

__all__ = ["DecisionWorker"]

@dataclass
class _PendingAction:
    """决策层用来过滤执行器结果的本地记录。"""

    generation: int
    tag: str
    started_ts: float


class DecisionWorker(threading.Thread):
    def __init__(self, cfg, perception: Latest, executor, robot,
                 name="decision", voice_queue=None, voice_done_queue=None,
                 stop_line: Latest | None = None,
                 traffic: Latest | None = None,
                 traffic_active=None,
                 traffic_sequence: bool = False,
                 lane: Latest | None = None,
                 lane_follow: bool = False, supervisor=None):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.actions = Actions(cfg)
        self.supervisor = supervisor
        self.traffic_active = traffic_active
        self._mode_revision = 0
        self._mode = 'stopped'
        self._switching = None
        self._control_fault = None
        self._terminal = None
        self._last_hold_stop = -float('inf')
        self.perception = perception
        self.executor = executor
        self.robot = robot
        self.voice_queue = voice_queue
        self.voice_done_queue = voice_done_queue
        self.stop_line = stop_line
        self.traffic = traffic
        self.lane = lane
        self.engine = DecisionEngine(
            cfg,
            lane_follow=lane_follow and lane is not None,
            traffic_sequence=(traffic_sequence and stop_line is not None and traffic is not None),
            traffic_active=traffic_active,
            wait_for_voice=voice_queue is not None and voice_done_queue is not None,
        )
        self._stop_event = threading.Event()
        self._pending: _PendingAction | None = None
        self._voice_seq = 0
        self._voice_wait_seq: int | None = None

    def stop(self):
        self._stop_event.set()

    def _report(self, state, detail=''):
        if self.supervisor is not None:
            self.supervisor.status.set(ControllerStatus(
                self._mode_revision, self._mode, state, detail))

    def _hold_stopped(self, now, interrupt=False):
        if interrupt or now-self._last_hold_stop >= self.cfg.stop_refresh_s:
            self.executor.submit(self.actions.stop(), interrupt=interrupt)
            self._last_hold_stop = now

    def _fault(self, now, reason):
        if self._control_fault is None:
            self._control_fault = reason
            self._pending = None
            self._switching = None
            if self.traffic_active is not None:
                self.traffic_active.clear()
            logging.error('[调度切换] %s；保持停车，重启程序后才能恢复', reason)
            self._hold_stopped(now, interrupt=True)
            self._report('failed', reason)
        return False

    def _supervised_ready(self, now):
        """只有决策线程更换引擎；先取消并等零速成功，再开启新控制器。"""
        if self.supervisor is None:
            return True
        if self._control_fault is not None:
            self._hold_stopped(now)
            return False
        command, heartbeat = self.supervisor.snapshot()
        if time.monotonic()-heartbeat > self.cfg.supervisor_timeout_s:
            return self._fault(now, '调度线程心跳中断')
        if command.revision != self._mode_revision:
            self._mode_revision = command.revision
            self._mode = command.mode
            self._terminal = None
            self._pending = None
            self._voice_wait_seq = None
            if self.traffic_active is not None:
                self.traffic_active.clear()
            request = self.actions.stop()
            request = replace(request, cmd=replace(request.cmd, tag=f'mode_stop_{command.revision}'))
            generation = self.executor.submit(request, interrupt=True)
            self._switching = (generation, request.cmd.tag, now)
            self._last_hold_stop = now
            self._report('switching')
            return False
        if self._switching is not None:
            generation, tag, started = self._switching
            raw, _ = self.executor.result()
            matches = (raw is not None and raw.generation == generation
                       and raw.tag == tag and raw.ts >= started)
            if matches and raw.status in ('failed', 'cancelled'):
                return self._fault(now, '控制器切换停车失败')
            if now-started >= self.cfg.controller_switch_timeout_s:
                return self._fault(now, '控制器切换停车超时')
            if not (matches and raw.status == 'ok' and not self.executor.busy
                    and now-raw.ts >= self.cfg.settle_s):
                return False
            self.engine = DecisionEngine(
                self.cfg, lane_follow=self._mode == 'lane',
                traffic_sequence=self._mode == 'traffic', uturn=self._mode == 'uturn',
                traffic_active=self.traffic_active,
                wait_for_voice=self.voice_queue is not None and self.voice_done_queue is not None,
            )
            self.engine.start(now)
            self._switching = None
            self._report('running')
            logging.info('[控制器] 已开启 %s，其他行为控制器已停止', self._mode)
        if self._mode == 'stopped' or self._terminal is not None:
            self._hold_stopped(now)
            return False
        return True

    def _read_pose(self, refresh=False):
        try:
            return self.robot.get_pose(refresh=refresh)
        except Exception as e:
            logging.warning("[决策] 读取姿态异常: %s", e)
            return None

    def _poll_voice_done(self) -> bool:
        """读取语音进程回执；只接受当前待播报序号的回执。"""
        if self.voice_done_queue is None or self._voice_wait_seq is None:
            return False
        while True:
            try:
                item = self.voice_done_queue.get_nowait()
            except queue.Empty:
                return False
            except (EOFError, OSError, ValueError):
                return False
            if (
                isinstance(item, VoiceDone)
                and item.seq == self._voice_wait_seq
            ):
                self._voice_wait_seq = None
                return True

    def _matching_result(self):
        """只把“当前待执行动作”的最终结果交给状态机。"""
        raw, _ = self.executor.result()
        if raw is None or raw.status == "running" or self._pending is None:
            return None
        if raw.generation != self._pending.generation:
            return None
        if raw.tag != self._pending.tag:
            return None
        if raw.ts < self._pending.started_ts:
            return None
        return raw

    # ------------------------------------------------------------
    def run(self):
        period = 1.0 / self.cfg.decision_hz
        logging.info("[决策] 启动 (%.0f Hz)", self.cfg.decision_hz)

        now = time.time()
        self.engine.start(now)

        while not self._stop_event.is_set():
            # 单独的执行器看门狗需要确认决策线程仍在运行。
            self.executor.heartbeat()
            now = time.time()
            if not self._supervised_ready(now):
                self._stop_event.wait(period)
                continue
            perception, perception_ts = self.perception.get()
            result = self._matching_result()
            # 动作成功时要校验 yaw，必须绕过 50ms 缓存读一次新姿态，
            # 否则可能拿到动作开始前的旧值，误判成“没转到位”。
            pose = self._read_pose(refresh=(result is not None and result.status == "ok"))

            voice_done = self._poll_voice_done()
            line, line_ts = (
                self.stop_line.get()
                if self.stop_line is not None else (None, 0.0)
            )
            traffic, _ = (
                self.traffic.get()
                if self.traffic is not None else (None, 0.0)
            )

            lane_state, _ = (
                self.lane.get()
                if self.lane is not None else (None, 0.0)
            )

            now = time.time()  # 姿态RPC完成后的时刻，用于感知与姿态新鲜度校验
            output = self.engine.evaluate(DecisionInput(
                now=now, perception=perception, perception_ts=perception_ts,
                pose=pose, result=result, executor_busy=self.executor.busy,
                voice_done=voice_done, lane=lane_state, line=line,
                line_ts=line_ts, traffic=traffic,
            ))
            step = output.step
            if output.clear_pending:
                self._pending = None
            if self.voice_queue is not None:
                for notice in output.notices:
                    self._voice_seq += 1
                    put_latest(self.voice_queue, replace(notice, seq=self._voice_seq))

            if step.log:
                if step.state == BLOCKED:
                    logging.warning("[决策] %s", step.log)
                else:
                    logging.info("[决策] %s", step.log)

            if step.request is not None:
                generation = self.executor.submit(step.request, interrupt=step.interrupt)

                if output.track_result:
                    self._pending = _PendingAction(
                        generation=generation,
                        tag=step.request.cmd.tag,
                        started_ts=now,
                    )
                    self.engine.note_sent(step.request, now)
                    logging.info(
                        "[决策] 执行 %s（%s）",
                        step.request.cmd,
                        step.request.purpose,
                    )
            elif step.interrupt:
                # 保险：状态机要求打断但没有给停车指令
                self.executor.interrupt()

            if step.voice is not None and self.voice_queue is not None:
                self._voice_seq += 1
                cue = replace(step.voice, seq=self._voice_seq)
                self._voice_wait_seq = cue.seq
                put_latest(self.voice_queue, cue)

            if self.supervisor is not None:
                terminal = self.engine.terminal_status
                if terminal is not None:
                    self._terminal = terminal
                    self._report(terminal, step.log or step.state)
                    if terminal == 'failed':
                        self._control_fault = step.log or step.state
                    # 等调度线程选择下一控制器，期间绝不退回普通前进。

            # 判断层不再等待动作结果时，清除本地匹配记录。
            if not self.engine.has_pending_action:
                self._pending = None

            time.sleep(period)

        logging.info("[决策] 退出")
