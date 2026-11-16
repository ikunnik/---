#!/usr/bin/env python3
"""
生成离线女声语音包：assets/voice/*.wav。

只用于开发阶段，需要联网安装 edge-tts 和 miniaudio：
    pip install --user edge-tts miniaudio
    python3 tools/voice_generate.py

运行时不需要 edge-tts、ffmpeg 或网络；VoiceProcess 只读 WAV。
默认音色：zh-CN-XiaoxiaoNeural（自然女声）。
"""
import argparse
import asyncio
import os
import sys
import tempfile
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CFG                                      # noqa: E402
from messages import VoiceCue                               # noqa: E402
from workers.voice import (                                 # noqa: E402
    BLOCKED_TEXT,
    WAIT_TEXT,
    SAMPLE_RATE,
    render_text,
    voice_file_name,
    voice_pack_path,
)

DEFAULT_VOICE = "zh-CN-XiaoxiaoNeural"


def all_cues() -> list[VoiceCue]:
    cues = [
        VoiceCue("lane"),
        VoiceCue("bypass", side="left", angle_deg=30),
        VoiceCue("bypass", side="left", angle_deg=60),
        VoiceCue("bypass", side="right", angle_deg=30),
        VoiceCue("bypass", side="right", angle_deg=60),
        VoiceCue("bypass_done"),
        VoiceCue("dynamic"),
        VoiceCue("traffic_red"),
        VoiceCue("traffic_yellow"),
        VoiceCue("traffic_green"),
    ]
    cues.extend(VoiceCue("blocked", reason=reason) for reason in BLOCKED_TEXT)
    cues.extend(VoiceCue("wait", reason=reason) for reason in WAIT_TEXT)
    return cues


async def synth_one(edge_tts, miniaudio, text: str, voice: str,
                    rate: str, out_path) -> float:
    communicate = edge_tts.Communicate(text, voice, rate=rate)
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
        mp3_path = tmp.name
    try:
        await communicate.save(mp3_path)
        decoded = miniaudio.decode_file(
            mp3_path,
            output_format=miniaudio.SampleFormat.SIGNED16,
            nchannels=1,
            sample_rate=SAMPLE_RATE,
        )
        if decoded.sample_width != 2 or decoded.nchannels != 1:
            raise RuntimeError(
                f"解码格式异常: width={decoded.sample_width}, "
                f"channels={decoded.nchannels}"
            )
        frames = decoded.samples.tobytes()
        with wave.open(str(out_path), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(frames)
        return len(frames) / 2 / SAMPLE_RATE
    finally:
        try:
            os.unlink(mp3_path)
        except OSError:
            pass


async def run(args):
    try:
        import edge_tts
        import miniaudio
    except ImportError as e:
        print(f"缺少开发依赖: {e}")
        print("请先执行: pip install --user edge-tts miniaudio")
        return 1

    out_dir = voice_pack_path(args.cfg)
    out_dir.mkdir(parents=True, exist_ok=True)

    made = 0
    skipped = 0
    failed = 0
    for cue in all_cues():
        name = voice_file_name(cue)
        text = render_text(cue)
        if not name or not text:
            print(f"跳过未知事件: {cue}")
            failed += 1
            continue

        out_path = out_dir / name
        if out_path.is_file() and not args.force:
            print(f"已存在，跳过: {name}")
            skipped += 1
            continue

        for attempt in range(1, 4):
            try:
                duration = await synth_one(
                    edge_tts, miniaudio, text, args.voice, args.rate, out_path
                )
                print(f"生成 {name}  {duration:.2f}s  {text}")
                made += 1
                break
            except Exception as e:
                print(f"失败 {name}（第 {attempt}/3 次）: {e}")
                if attempt < 3:
                    await asyncio.sleep(1.0 * attempt)
        else:
            failed += 1

    print(
        f"\n完成：生成 {made}，跳过 {skipped}，失败 {failed}；目录 {out_dir}"
    )
    return 0 if failed == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="生成离线女声语音包")
    parser.add_argument(
        "--voice", default=DEFAULT_VOICE,
        help=f"Edge TTS 音色（默认 {DEFAULT_VOICE}）",
    )
    parser.add_argument(
        "--rate", default="+0%",
        help="语速，例如 +10%%（默认 +0%%）",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="覆盖已存在的 WAV",
    )
    args = parser.parse_args()
    args.cfg = CFG

    started = time.time()
    rc = asyncio.run(run(args))
    print(f"耗时 {time.time() - started:.1f}s")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
