"""
深度相机体检：只订阅、只打印，不保存、不控制机器人。

用法（项目根目录）：
    python3 tools/depth_probe.py

用来回答一个问题：为什么一直 valid=False？
    - 画面全是 0      → 镜头被挡 / 对着吸光材质 / 太近超出量程
    - 只有远处有值    → 正常，ROI 里没有障碍物
    - 有值但很乱      → 检查 ROI 或现场反光
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CFG                        # noqa: E402
from workers.depth_camera import compute_profile  # noqa: E402

import dds_middleware_python as dds           # noqa: E402

latest = {"frame": None, "count": 0}


def _on_depth(msg):
    if "16UC1" not in msg.encoding():
        return
    w, h = msg.width(), msg.height()
    step = msg.step() or w * 2
    raw = bytes(msg.data())
    dtype = np.dtype(">u2") if msg.is_bigendian() else np.dtype("<u2")
    frame = np.frombuffer(raw, dtype=dtype).reshape(h, step // 2)[:, :w]
    latest["frame"] = frame
    latest["count"] += 1


def main():
    m = dds.PyDDSMiddleware(CFG.dds_config)
    m.subscribeImage(CFG.depth_topic, _on_depth, {
        "reliability": "best_effort",
        "history_kind": "keep_last",
        "history_depth": 5,
        "durability": "volatile",
    })
    print(f"订阅 {CFG.depth_topic}，采样 3 秒...")
    time.sleep(3)

    frame = latest["frame"]
    if frame is None:
        print("❌ 一帧都没收到：检查网线 / CYCLONEDDS_URI / 话题名")
        return

    h, w = frame.shape
    print(f"收到 {latest['count']} 帧，分辨率 {w}x{h}，零值占比 {100 * (frame == 0).mean():.1f}%")

    p = compute_profile(frame, CFG)
    print(f"当前感知结果: {p}")
    print(f"  center={p.distance_m:.2f}m  left={p.left_m:.2f}m  right={p.right_m:.2f}m")
    print(f"  min={p.min_m:.2f}m  near_ratio={p.near_ratio:.2f}  valid={p.valid}")

    for i in range(5):
        band = frame[int(h * i / 5):int(h * (i + 1) / 5)]
        valid = band[band > 0]
        p5 = np.percentile(valid, 5) if valid.size else 0
        print(f"  第{i}段 (y {i * 20:2d}%~{(i + 1) * 20:2d}%)  "
              f"零值 {100 * (band == 0).mean():5.1f}%  最近5% {p5:7.0f} mm")


if __name__ == "__main__":
    main()
