"""
语音提示进程：把状态机的 VoiceCue 变成机器人扬声器里的中文。

为什么单独一个进程：
    DDS 是原生库，和深度相机一样隔离出去；语音初始化失败也不影响避障主链路。
    主进程只往 mp.Queue 里丢很小的 VoiceCue，不导入 DDS。

音频链路（唯一）：
    VoiceCue -> assets/voice/*.wav（自然女声，24kHz/mono/16bit）
             -> VoiceCmd(type="streaming") -> rt/voice/cmd

注意：
    - 使用机器人 SDK 的低层 E7 语音接口，不经过主控电脑的扬声器。
    - 只播放预生成的 Xiaoxiao 女声 WAV；**没有 espeak 回退**，
      文件缺失时只报错/静音，绝不会退回机器声。
    - 每一句话用独立 task_id；最后发一包 data=[]、flag=True 表示流结束。
    - 播完（或失败）通过 done_queue 回执，决策线程据此恢复动作。
"""
import audioop
import collections
import io
import logging
import multiprocessing
import queue
import signal
import time
import wave
from pathlib import Path

from comm import setup_process_logging
from messages import VoiceCue, VoiceDone

SAMPLE_RATE = 24000
SAMPLE_WIDTH = 2          # 16 bit
CHANNELS = 1

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# 失败/停车原因 -> 中文短句。状态机只传 reason 字符串，翻译集中在这里。
BLOCKED_TEXT = {
    "too_close": "距离障碍太近，旋转空间不足，已停下。",
    "no_space": "左右两侧空间不足，无法绕行，已停下。",
    "no_pose": "无法获取机身方向，已停下。",
    "no_entry_yaw": "缺少避障前方向，无法回正，已停下。",
    "rotate_space": "旋转空间不足，已停下。",
    "turn_failed": "转向失败，已停下。",
    "advance_failed": "前进失败，已停下。",
    "realign_failed": "回正失败，已停下。",
    "turn_timeout": "转向超时，已停下。",
    "advance_timeout": "前进超时，已停下。",
    "realign_timeout": "回正超时，已停下。",
    "yaw_mismatch": "转向角度不符合要求，已停下。",
    "heading_drift": "前进中方向偏移过大，已停下。",
    "perception_lost": "深度数据丢失，已停下。",
    "emergency": "前方距离过近，已紧急停止。",
    "max_cycles": "绕行次数已达上限，已停下。",
    "realign_blocked": "回正后前方仍被挡，已停下。",
    "realign_limit": "回正次数已达上限，已停下。",
    "realign_space": "回正空间不足，已停下。",
    "pose_missing": "动作完成但无法读取机身方向，已停下。",
    "internal": "避障状态异常，已停下。",
}

WAIT_TEXT = {
    "no_pose_wait": "无法获取机身方向，暂不绕行。",
    "realign_space_wait": "回正空间不足，停步等待。",
    "clear_wait": "回正后距离不足，停步等待。",
    "stop_line_lost_wait": "停止线丢失，已停下，请寻求帮助。",
    "stop_line_stall_wait": "停止线无进展，已停下，请寻求帮助。",
    "stop_line_sensor_wait": "停止线检测失效，已停下，请检查。",
}


def render_text(cue: VoiceCue) -> str:
    """把语义事件翻译成一句中文；未知事件返回空串，不播报。"""
    if cue.kind == "bypass":
        side = "左" if cue.side == "left" else "右"
        if cue.angle_deg >= 60.0:
            return f"前方有障碍，请向{side}前方绕过。"
        return f"前方有障碍，请从{side}侧绕过。"
    if cue.kind == "bypass_done":
        return "已绕过障碍，继续前进。"
    if cue.kind == "dynamic":
        return "检测到移动障碍，停步等待。"
    if cue.kind == "blocked":
        return BLOCKED_TEXT.get(cue.reason, "避障失败，已停下。")
    if cue.kind == "wait":
        return WAIT_TEXT.get(cue.reason, "停步等待。")
    if cue.kind == "traffic_red":
        return "红灯，请停步等待。"
    if cue.kind == "traffic_yellow":
        return "黄灯，请停步等待。"
    if cue.kind == "traffic_green":
        return "绿灯，可以通行。"
    if cue.kind == "lane":
        return "已识别前方盲道，请跟随我。"
    if cue.kind == "help":
        return "快来人，帮帮我，快来人，帮帮我"
    return ""


def priority_name(cue: VoiceCue) -> str:
    """DDS VoicePriority 名称：停车/动态障碍用警告，绕行方向用提示。"""
    if cue.kind in ("blocked", "dynamic", "traffic_red", "traffic_yellow", "help"):
        return "kWarning"
    if (
        cue.kind == "wait"
        and cue.reason in (
            "stop_line_lost_wait",
            "stop_line_stall_wait",
            "stop_line_sensor_wait",
        )
    ):
        return "kWarning"
    if cue.kind == "bypass":
        return "kPROMPT"
    return "kNormal"


def voice_file_name(cue: VoiceCue) -> str:
    """VoiceCue -> 预生成 WAV 文件名；未知事件返回空串。"""
    if cue.kind == "bypass":
        side = "left" if cue.side == "left" else "right"
        suffix = "_front" if cue.angle_deg >= 60.0 else ""
        return f"bypass_{side}{suffix}.wav"
    if cue.kind == "bypass_done":
        return "bypass_done.wav"
    if cue.kind == "dynamic":
        return "dynamic.wav"
    if cue.kind == "blocked":
        return f"blocked_{cue.reason or 'internal'}.wav"
    if cue.kind == "wait":
        return f"wait_{cue.reason or 'generic'}.wav"
    if cue.kind == "traffic_red":
        return "traffic_red.wav"
    if cue.kind == "traffic_yellow":
        return "traffic_yellow.wav"
    if cue.kind == "traffic_green":
        return "traffic_green.wav"
    if cue.kind == "lane":
        return "lane.wav"
    if cue.kind == "help":
        return "help.wav"
    return ""


def voice_pack_path(cfg) -> Path:
    path = Path(cfg.voice_pack_dir)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def voice_file_for(cue: VoiceCue, cfg) -> Path | None:
    name = voice_file_name(cue)
    if not name:
        return None
    return voice_pack_path(cfg) / name


def normalize_pcm(frames: bytes, width: int, channels: int, rate: int) -> bytes:
    """转成 24kHz/mono/16bit，并在句尾留一点静音。"""
    if not frames:
        return b""

    if width != SAMPLE_WIDTH:
        frames = audioop.lin2lin(frames, width, SAMPLE_WIDTH)
        width = SAMPLE_WIDTH
    if channels != CHANNELS:
        frames = audioop.tomono(frames, width, 0.5, 0.5)
        channels = CHANNELS
    if rate != SAMPLE_RATE:
        frames, _ = audioop.ratecv(
            frames, width, channels, rate, SAMPLE_RATE, None
        )

    # 句尾留一点静音，避免最后一包被机器人当作截断。
    silence = b"\x00" * int(SAMPLE_RATE * 0.15) * SAMPLE_WIDTH
    return frames + silence


def wav_bytes_to_pcm(raw: bytes) -> bytes:
    """解析 WAV 字节并统一成 24kHz/mono/16bit PCM。"""
    try:
        with wave.open(io.BytesIO(raw), "rb") as wf:
            channels = wf.getnchannels()
            width = wf.getsampwidth()
            rate = wf.getframerate()
            frames = wf.readframes(wf.getnframes())
    except Exception as e:
        logging.warning("[语音进程] WAV 解析失败: %s", e)
        return b""

    return normalize_pcm(frames, width, channels, rate)


def load_wav_pcm(path: Path) -> bytes:
    try:
        return wav_bytes_to_pcm(path.read_bytes())
    except Exception as e:
        logging.warning("[语音进程] 读取语音包失败 %s: %s", path, e)
        return b""


def _make_voice_cmd(dds, priority, task_id: str, data: bytes, flag: bool):
    header = dds.Header()
    stamp = dds.Time()
    now = time.time()
    stamp.sec(int(now))
    stamp.nanosec(int((now - int(now)) * 1e9))
    header.stamp(stamp)
    header.frame_id("voice_cmd")

    cmd = dds.VoiceCmd()
    cmd.header(header)
    cmd.priority(priority)
    cmd.task_id(task_id)
    cmd.type("streaming")
    cmd.path("")
    cmd.data(list(data))
    cmd.flag(flag)
    return cmd


def publish_pcm(middleware, dds, pcm: bytes, cfg, priority, task_id: str,
                stop_event=None) -> None:
    """按 voice_chunk_ms 分块发布 PCM；结束时发送 flag=True 的空包。"""
    chunk_bytes = max(
        SAMPLE_WIDTH,
        int(SAMPLE_RATE * cfg.voice_chunk_ms / 1000) * SAMPLE_WIDTH,
    )
    chunk_s = chunk_bytes / SAMPLE_WIDTH / SAMPLE_RATE

    try:
        for offset in range(0, len(pcm), chunk_bytes):
            if stop_event is not None and stop_event.is_set():
                break
            cmd = _make_voice_cmd(
                dds, priority, task_id, pcm[offset:offset + chunk_bytes], False
            )
            middleware.publishVoiceCmd(cmd)

            if stop_event is not None:
                if stop_event.wait(chunk_s):
                    break
            else:
                time.sleep(chunk_s)
    finally:
        # 即使中途退出，也尽量把流结束掉，避免机器人一直播放最后一包。
        try:
            final = _make_voice_cmd(dds, priority, task_id, b"", True)
            middleware.publishVoiceCmd(final)
        except Exception as e:
            logging.warning("[语音进程] 发送结束包失败: %s", e)


class VoiceProcess(multiprocessing.Process):
    """子进程：读 VoiceCue -> 预生成女声 WAV -> DDS 发布。"""

    def __init__(self, cfg, in_queue, done_queue=None, name="voice"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.done_queue = done_queue
        self._stop_event = multiprocessing.Event()
        self._seq = 0

    def stop(self):
        self._stop_event.set()

    def _pcm_for(self, cue, text, cache):
        key = cue.key or text
        cached = cache.get(key)
        if cached is not None:
            cache.move_to_end(key)
            return cached

        pcm = b""
        path = voice_file_for(cue, self.cfg)
        if path is None:
            logging.error("[语音进程] 未知语音事件，无法定位 WAV: %s", cue)
            return b""
        if not path.is_file():
            logging.error("[语音进程] 女声语音包缺失: %s（不会回退机器声）", path)
            return b""
        pcm = load_wav_pcm(path)
        if not pcm:
            logging.error("[语音进程] 女声语音包为空: %s", path)
            return b""

        if pcm:
            cache[key] = pcm
            if len(cache) > self.cfg.voice_cache_size:
                cache.popitem(last=False)
        return pcm

    def _notify_done(self, cue: VoiceCue, ok: bool):
        if self.done_queue is None:
            return
        try:
            self.done_queue.put_nowait(
                VoiceDone(seq=cue.seq, ok=ok, key=cue.key, ts=time.time())
            )
        except queue.Full:
            logging.warning("[语音进程] 回执队列已满，丢弃 seq=%s", cue.seq)
        except Exception as e:
            logging.warning("[语音进程] 发送回执失败: %s", e)

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()

        if not self.cfg.voice_enabled:
            logging.info("[语音进程] 已禁用")
            return

        pack_dir = voice_pack_path(self.cfg)
        pack_count = len(list(pack_dir.glob("*.wav"))) if pack_dir.is_dir() else 0
        if pack_count:
            logging.info("[语音进程] 女声语音包: %s（%d 个 WAV）", pack_dir, pack_count)
        else:
            logging.error("[语音进程] 没有女声语音包，语音提示不可用（不会回退机器声）")

        try:
            # 只在子进程导入原生 DDS 库
            import dds_middleware_python as dds

            middleware = dds.PyDDSMiddleware(self.cfg.dds_config)
            qos = {
                "reliability": "reliable",
                "history_kind": "keep_last",
                "history_depth": 5,
                "durability": "volatile",
            }
            middleware.createVoiceCmdWriter(self.cfg.voice_topic, qos)
        except Exception as e:
            logging.error("[语音进程] DDS 初始化失败: %s", e)
            return

        logging.info("[语音进程] 发布 %s", self.cfg.voice_topic)
        if self._stop_event.wait(self.cfg.voice_startup_s):
            return

        cache = collections.OrderedDict()
        while not self._stop_event.is_set():
            try:
                cue = self.in_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            except (EOFError, OSError, ValueError):
                break

            if not isinstance(cue, VoiceCue):
                continue
            text = render_text(cue)
            if not text:
                self._notify_done(cue, False)
                continue

            pcm = self._pcm_for(cue, text, cache)
            if not pcm:
                logging.warning("[语音进程] 没有可播放音频：%s", text)
                self._notify_done(cue, False)
                continue

            self._seq += 1
            task_id = f"{self.cfg.voice_task_id}_{self._seq}"
            priority = getattr(dds.VoicePriority, priority_name(cue))
            ok = False
            try:
                logging.info("[语音进程] 播报：%s", text)
                publish_pcm(
                    middleware, dds, pcm, self.cfg, priority, task_id,
                    stop_event=self._stop_event,
                )
                ok = True
            except Exception as e:
                logging.warning("[语音进程] 发布失败: %s", e)
            finally:
                # 最后一包发出后，机器人可能还有少量缓冲；等尾音结束再回执。
                if ok:
                    self._stop_event.wait(self.cfg.voice_tail_s)
                self._notify_done(cue, ok)

        logging.info("[语音进程] 退出")
