#!/usr/bin/env python3
"""
前置 RGB 相机体检：只订阅 DDS，不连接运动接口、不改机器人状态。

用法（项目根目录，容器 roverx1-dev 里）：
    python3 tools/rgb_probe.py
    python3 tools/rgb_probe.py --topic rt/camera/camera0/image_compressed --seconds 5
    python3 tools/rgb_probe.py --save rgb_images/probe.jpg

回答三个问题：
    1. camera0 有没有推流？
    2. 格式是不是 jpeg？一帧多大？
    3. 首帧能不能保存成文件？
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import CFG                          # noqa: E402
from workers.rgb_camera import decode_compressed  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="前置 RGB 相机体检（只读）")
    parser.add_argument(
        "--topic", default=CFG.rgb_topic,
        help=f"RGB 话题（默认 {CFG.rgb_topic}）",
    )
    parser.add_argument(
        "--seconds", type=float, default=5.0,
        help="采样时长，默认 5 秒",
    )
    parser.add_argument(
        "--save", default="",
        help="保存首帧的路径；留空则自动存到 rgb_images/rgb_probe.<ext>",
    )
    args = parser.parse_args()

    try:
        import dds_middleware_python as dds
    except Exception as e:
        print(f"无法导入 dds_middleware_python: {e}")
        print("请在 distrobox roverx1-dev 里运行，并先 source SDK 的 setup_cyclonedds_env.sh")
        return 1

    state = {"frame": None, "count": 0}

    def _on_image(msg):
        # 只解析，不打印、不解码；DDS 回调线程里做最少的事
        frame = decode_compressed(msg)
        state["count"] += 1
        if state["frame"] is None:
            state["frame"] = frame

    try:
        middleware = dds.PyDDSMiddleware(CFG.dds_config)
        middleware.subscribeCompressedImage(args.topic, _on_image)
    except Exception as e:
        print(f"DDS 订阅失败: {e}")
        return 1

    print(f"订阅 {args.topic}，采样 {args.seconds:g}s ...")
    time.sleep(args.seconds)

    frame = state["frame"]
    if frame is None:
        print("❌ 一帧都没收到：检查网线 / CYCLONEDDS_URI / 话题名（camera0 可能没有推流）")
        return 1

    fps = state["count"] / args.seconds
    print(f"收到 {state['count']} 帧（{fps:.1f} FPS）")
    print(
        f"  format={frame.format!r}  frame_id={frame.frame_id!r}  "
        f"size={len(frame.data)} bytes  ts={frame.ts:.3f}  seq={frame.seq}"
    )

    ext = ".jpg" if frame.format.lower() in ("jpeg", "jpg") else (
        f".{frame.format}" if frame.format else ".bin"
    )
    out = Path(args.save) if args.save else Path("rgb_images") / f"rgb_probe{ext}"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(frame.data)
    print(f"已保存首帧: {out}（{len(frame.data)} bytes）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
