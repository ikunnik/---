#!/usr/bin/env python3
"""停止线几何指标探针 —— 用真实帧定阈值，不要拍脑袋。

单组模式（列出每帧指标）：
    python3 tools/stop_line_probe.py rgb_images/calib/*.jpg

两组模式（算分离度并给出建议阈值，推荐）：
    python3 tools/stop_line_probe.py --pos /tmp/line --neg /tmp/cone

必须在容器里跑（需要 cv2 + numpy）：
    distrobox enter roverx1-dev -- bash -lc \
      'cd ~/rover_x1_v7.5 && python3 tools/stop_line_probe.py --pos ... --neg ...'

判读方法：
    真停止线的 elongation（主轴长/次轴厚）通常是几十，黄锥桶/水马只有个位数。
    两组分布若有清晰空隙，把 elongation_min 放在空隙中间即可；
    若两组重叠，说明该障碍在几何上就是细长条（横置黄栏杆、地面黄漆带），
    几何门救不了它，必须换手段：深度贴合地面校验，或只在红绿灯序列中消费。
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from workers.stop_line import (  # noqa: E402
    DEFAULT_SHAPE, _crop, _full_rows, _shape_metrics, detect_stop_line, load_profile,
)
from workers.traffic_light import range_mask, rgb_to_hsv  # noqa: E402

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def load_rgb(path: Path, width: int) -> np.ndarray:
    """图片 -> RGB float32 [0,1]，与 decode_jpeg 保持同一约定。"""
    import cv2

    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"无法解码: {path}")
    h, w = bgr.shape[:2]
    nh = max(2, int(round(h * width / w / 2.0)) * 2)
    bgr = cv2.resize(bgr, (width, nh), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0


def measure(rgb: np.ndarray, profile: dict) -> dict:
    """复算一次掩膜与几何指标，用于输出诊断信息。"""
    crop, _ = _crop(rgb, profile["roi"])
    if crop.size == 0:
        return {}
    h, s, v = rgb_to_hsv(crop)
    mask = range_mask(h, s, v, profile["ranges"])
    n = int(mask.sum())
    area = float(crop.shape[0] * crop.shape[1])
    out = {"area_ratio": n / area if area else 0.0, "pixels": n}
    out["full_rows"] = _full_rows(mask, crop.shape[1])
    if n:
        ys, xs = np.nonzero(mask)
        metrics = _shape_metrics(xs, ys, crop.shape[1], crop.shape[0])
        if metrics:
            out.update(metrics)
    return out


def collect(paths, width):
    frames = []
    for p in paths:
        try:
            frames.append((p, load_rgb(p, width)))
        except Exception as e:
            print(f"  跳过 {p}: {e}", file=sys.stderr)
    return frames


def iter_images(target: Path):
    if target.is_file():
        return [target]
    return sorted(p for p in target.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES)


def print_table(rows, shape):
    header = (f"{'文件':<30}{'面积%':>7}{'主轴':>7}{'次轴':>7}{'伸长比':>8}"
              f"{'横跨比':>8}{'全宽行':>7}{'角度':>8}   判定")
    print(header)
    print("-" * len(header))
    for name, m, verdict in rows:
        if not m:
            print(f"{name:<30}{'—':>7}{'':>7}{'':>7}{'':>8}{'':>8}{'':>7}{'':>8}   {verdict}")
            continue
        print(
            f"{name[:28]:<30}{m.get('area_ratio', 0) * 100:>6.2f}%"
            f"{m.get('length', 0):>7.0f}{m.get('thickness', 0):>7.0f}"
            f"{m.get('elongation', 0):>8.2f}{m.get('length_ratio', 0):>8.2f}"
            f"{m.get('full_rows', 0):>7}{m.get('angle', 0):>+8.1f}   {verdict}"
        )
    print(
        f"\n通路A(细长): elongation>={shape['elongation_min']} "
        f"length_ratio>={shape['length_ratio_min']} "
        f"thickness_ratio<={shape['thickness_ratio_max']} "
        f"|angle|<={shape['angle_max_deg']}"
    )
    print(
        f"通路B(全宽行): 连续长度>={shape.get('full_row_frac', 0.8):.0%} ROI宽 的行数"
        f" >= {shape.get('full_rows_min', 3)}"
    )


def separation(pos_vals, neg_vals, name, higher_is_line=True):
    """报告两组的取值范围与可分性；可分时给出建议阈值。"""
    if not pos_vals or not neg_vals:
        return
    p_lo, p_hi = min(pos_vals), max(pos_vals)
    n_lo, n_hi = min(neg_vals), max(neg_vals)
    if higher_is_line:
        ok = p_lo > n_hi
        suggest = (n_hi + p_lo) / 2.0
    else:
        ok = p_hi < n_lo
        suggest = (p_hi + n_lo) / 2.0
    mark = "可分" if ok else "重叠"
    print(
        f"  {name:<16} 真线 {p_lo:>7.2f}~{p_hi:<7.2f}(n={len(pos_vals):<3})"
        f" 黄障碍 {n_lo:>7.2f}~{n_hi:<7.2f}(n={len(neg_vals):<3})  {mark}"
        + (f"  建议 {suggest:.2f}" if ok else "  ← 该判据无法分离，别用它")
    )


def main():
    ap = argparse.ArgumentParser(description="停止线几何指标探针")
    ap.add_argument("paths", nargs="*", type=Path, help="图片文件或目录")
    ap.add_argument("--pos", type=Path, help="真停止线样本目录")
    ap.add_argument("--neg", type=Path, help="误检的黄障碍样本目录")
    ap.add_argument("--profile", type=Path, default=ROOT / "assets/stop_line_profile.json")
    ap.add_argument("--elongation-min", type=float, default=None, help="临时覆盖阈值试算")
    ap.add_argument("--length-ratio-min", type=float, default=None)
    args = ap.parse_args()

    profile = load_profile(args.profile)
    shape = {**DEFAULT_SHAPE, **(profile.get("shape") or {})}
    if args.elongation_min is not None:
        shape["elongation_min"] = args.elongation_min
    if args.length_ratio_min is not None:
        shape["length_ratio_min"] = args.length_ratio_min
    width = int(profile.get("camera_width", 640))

    if args.pos and args.neg:
        print("=== 分离度报告（用真帧标定，比调参猜更可靠）===")
        groups = {}
        for label, target in (("真线", args.pos), ("黄障碍", args.neg)):
            files = iter_images(target)
            if not files:
                print(f"  {label}: 目录里没有图片 — {target}", file=sys.stderr)
                continue
            vals = []
            for p, rgb in collect(files, width):
                m = measure(rgb, profile)
                if m.get("pixels"):
                    vals.append(m)
            groups[label] = vals
            print(f"  {label}: {len(vals)} 帧有黄色像素（共 {len(files)} 张）")

        if len(groups) == 2:
            pos = groups["真线"]
            neg = groups["黄障碍"]
            print()
            separation([m["elongation"] for m in pos], [m["elongation"] for m in neg], "elongation")
            separation([m["length_ratio"] for m in pos], [m["length_ratio"] for m in neg], "length_ratio")
            separation([m["thickness_ratio"] for m in pos], [m["thickness_ratio"] for m in neg],
                       "thickness_ratio", higher_is_line=False)
        return 0

    targets = list(args.paths)
    if not targets:
        ap.error("至少给一个图片路径/目录，或用 --pos/--neg 做分离度报告")
    rows = []
    for target in targets:
        files = iter_images(target)
        for p, rgb in collect(files, width):
            m = measure(rgb, profile)
            state = detect_stop_line(rgb, profile, now=0.0)
            if not m.get("pixels"):
                verdict = "无黄像素"
            elif state.visible:
                verdict = "→ 判为停止线"
            else:
                verdict = "→ 被形状门拒绝"
            rows.append((p.name, m, f"{verdict} row={state.row:.3f}"))
        print_table(rows, shape)
    return 0


if __name__ == "__main__":
    sys.exit(main())
