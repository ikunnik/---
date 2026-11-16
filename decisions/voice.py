"""避障状态机的语音门控。

状态机只调用三个动作：
    announce(cue, now, deferred)  -> 去重并登记播报
    poll(now, voice_done)         -> 判断是否还在播、是否播完/超时
    clear_keys()                  -> 新障碍事件时允许重新播报

真正的 TTS/DDS 在 workers/voice.py，本文件不依赖音频库。
"""
from messages import VoiceCue
from decisions.types import DeferredAction

# poll() 的返回状态
IDLE = "idle"
WAITING = "waiting"
DONE = "done"
TIMEOUT = "timeout"


class VoiceGate:
    """播报去重 + 播报期间停车 + 播完后执行 deferred 动作。"""

    def __init__(self, cfg, wait_for_voice: bool = False):
        self.cfg = cfg
        self.wait_for_voice = wait_for_voice
        self.keys: set[str] = set()
        self.pending = False
        self.started_ts = 0.0
        self.deferred: DeferredAction | None = None

    def reset(self):
        self.keys.clear()
        self.pending = False
        self.started_ts = 0.0
        self.deferred = None

    def clear_keys(self):
        """新障碍事件/恢复检查时清空去重键。"""
        self.keys.clear()

    def announce(self, cue: VoiceCue, now: float,
                 deferred: DeferredAction | None = None, *,
                 wait: bool = True) -> VoiceCue | None:
        """去重并登记播报；需要停车播报时设置 pending。"""
        if cue.key in self.keys:
            return None
        self.keys.add(cue.key)

        if wait and self.wait_for_voice:
            self.pending = True
            self.started_ts = now
            self.deferred = deferred
        return cue

    def poll(self, now: float, voice_done: bool) -> tuple[str, DeferredAction | None]:
        """返回 (状态, deferred)。

        waiting  : 还在播报，调用方必须继续发停车
        done     : 收到回执，deferred 可以执行
        timeout  : 等待超时（语音进程异常），deferred 可以执行
        idle     : 当前没有播报
        """
        if not self.pending:
            return IDLE, None

        timed_out = now - self.started_ts >= self.cfg.voice_wait_timeout_s
        if not voice_done and not timed_out:
            return WAITING, None

        self.pending = False
        deferred = self.deferred
        self.deferred = None
        if voice_done:
            return DONE, deferred
        return TIMEOUT, deferred
