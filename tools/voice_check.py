#!/usr/bin/env python3
"""语音链路自检：只读预生成女声 WAV，不合成、不回退机器声。

用法（项目根目录）：
    python3 tools/voice_check.py --dry-run
    python3 tools/voice_check.py --dry-run --wav assets/voice/lane.wav
    python3 tools/voice_check.py --play  --wav assets/voice/lane.wav

注意：
    --play 只发语音，不会 stand/walk，也不会让机器人移动。
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CFG                                      # noqa: E402
from workers.voice import (                                 # noqa: E402
    SAMPLE_RATE,
    load_wav_pcm,
    publish_pcm,
)

DEFAULT_WAV = "assets/voice/lane.wav"


def main() -> int:
    parser = argparse.ArgumentParser(description="Rover X1 语音链路自检")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run", action="store_true",
        help="只读取 WAV 检查 PCM（默认）",
    )
    mode.add_argument(
        "--play", action="store_true",
        help="通过 DDS 把 WAV 播到机器人扬声器",
    )
    parser.add_argument("--wav", default=DEFAULT_WAV,
                        help=f"要检查/播放的女声 WAV（默认 {DEFAULT_WAV}）")
    parser.add_argument(
        "--priority", default="kPROMPT",
        choices=("kNone", "kNormal", "kPROMPT", "kWarning", "kApp", "kAbnormal"),
        help="VoiceCmd 优先级（默认 kPROMPT）",
    )
    parser.add_argument(
        "--topic", default=CFG.voice_topic,
        help=f"语音话题（默认 {CFG.voice_topic}）",
    )
    args = parser.parse_args()

    wav_path = Path(args.wav)
    if not wav_path.is_absolute():
        wav_path = Path(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))) / wav_path
    print(f"WAV: {wav_path}")
    if not wav_path.is_file():
        print("文件不存在；不会回退机器声，请检查 assets/voice/")
        return 1

    pcm = load_wav_pcm(wav_path)
    if not pcm:
        print("WAV 解析失败或为空；不会回退机器声")
        return 1

    duration = len(pcm) / 2 / SAMPLE_RATE
    print(f"PCM: {len(pcm)} bytes, {duration:.2f}s, {SAMPLE_RATE}Hz/mono/16bit")

    if not args.play:
        print("dry-run：未发布 DDS。加 --play 才会播到机器人扬声器。")
        return 0

    try:
        import dds_middleware_python as dds
    except Exception as e:
        print(f"无法导入 dds_middleware_python: {e}")
        return 1

    try:
        middleware = dds.PyDDSMiddleware(CFG.dds_config)
        qos = {
            "reliability": "reliable",
            "history_kind": "keep_last",
            "history_depth": 5,
            "durability": "volatile",
        }
        middleware.createVoiceCmdWriter(args.topic, qos)
    except Exception as e:
        print(f"DDS 初始化失败: {e}")
        return 1

    priority = getattr(dds.VoicePriority, args.priority)
    task_id = f"voice_check_{int(time.time() * 1000)}"
    print(f"等待 DDS 发现 {CFG.voice_startup_s:.1f}s...")
    time.sleep(CFG.voice_startup_s)
    try:
        publish_pcm(middleware, dds, pcm, CFG, priority, task_id)
    except Exception as e:
        print(f"发布失败: {e}")
        return 1

    print(f"已发布到 {args.topic}（priority={args.priority}, task_id={task_id}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
