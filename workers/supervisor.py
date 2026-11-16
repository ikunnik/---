"""控制器调度线程：只发布模式意图，不修改状态机、不向机器人发动作。"""
from dataclasses import dataclass
import logging
import threading
import time

from comm import Latest

MODES = frozenset(('stopped', 'avoidance', 'traffic', 'uturn', 'lane'))


@dataclass(frozen=True)
class ControllerCommand:
    revision: int
    mode: str


@dataclass(frozen=True)
class ControllerStatus:
    revision: int
    mode: str
    state: str  # switching / running / completed / failed
    detail: str = ''


class ControllerSupervisor(threading.Thread):
    """enable/disable 为线程安全入口；自动路线仅在正常完成回执后推进。"""
    def __init__(self, cfg, initial_mode, *, mission=False, available_modes=None):
        super().__init__(name='controller_supervisor', daemon=True)
        self.cfg = cfg
        self.available_modes = frozenset(available_modes or MODES) | {'stopped'}
        if initial_mode not in self.available_modes:
            raise ValueError('初始控制器未准备好')
        if mission and not {'traffic', 'uturn', 'lane'} <= self.available_modes:
            raise ValueError('自动流程需要交通、掉头和盲道控制器')
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._command = ControllerCommand(1, initial_mode)
        self._heartbeat = time.monotonic()
        self._mission = mission
        self._handled_revision = 0
        self.status = Latest()

    def start(self):
        with self._lock:
            self._heartbeat = time.monotonic()
        super().start()

    def enable(self, mode):
        """手动选择控制器，同时退出自动串联；切换须经决策线程停车确认。"""
        if mode not in MODES or mode not in self.available_modes:
            raise ValueError(f'控制器不可用: {mode}')
        with self._lock:
            self._mission = False
            self._command = ControllerCommand(self._command.revision + 1, mode)
            return self._command.revision

    def disable(self):
        return self.enable('stopped')

    def snapshot(self):
        with self._lock:
            return self._command, self._heartbeat

    def stop(self):
        self.disable()
        self._stop_event.set()

    def tick(self):
        """单步调度也可用于离线测试；自动推进与人工切换在同一锁下裁决。"""
        status, _ = self.status.get()
        with self._lock:
            self._heartbeat = time.monotonic()
            command = self._command
            if (status is None or status.revision != command.revision
                    or status.mode != command.mode or status.revision == self._handled_revision):
                return
            if status.state not in ('completed', 'failed'):
                return
            self._handled_revision = status.revision
            if status.state == 'failed':
                target = 'stopped'
                self._mission = False
            elif self._mission:
                target = {'traffic': 'uturn', 'uturn': 'lane', 'lane': 'stopped'}.get(command.mode, 'stopped')
            else:
                target = 'avoidance' if command.mode == 'traffic' else 'stopped'
            if target not in self.available_modes:
                target = 'stopped'
            self._command = ControllerCommand(command.revision + 1, target)
            logging.info('[调度] %s %s → %s', command.mode, status.state, target)

    def run(self):
        logging.info('[调度] 启动，初始控制器 %s', self._command.mode)
        try:
            while not self._stop_event.is_set():
                self.tick()
                self._stop_event.wait(self.cfg.supervisor_period_s)
        finally:
            self.disable()
            logging.info('[调度] 退出')
