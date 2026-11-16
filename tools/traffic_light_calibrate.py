#!/usr/bin/env python3
"""
玩具红绿灯 HSV 标定（不训练模型）。

输入：一组带标签的 JPEG（red_*.jpg / green_*.jpg / yellow_*.jpg / bg_*.jpg）
输出：assets/traffic_light_profile.json（给后续 traffic_light.py 读）

采集（用已有的 probe 拍帧，camera0 或 camera2 都可以）：
    mkdir -p rgb_images/calib
    python3 tools/rgb_probe.py --seconds 3 --save rgb_images/calib/red_01.jpg
    python3 tools/rgb_probe.py --seconds 3 --save rgb_images/calib/red_02.jpg
    python3 tools/rgb_probe.py --seconds 3 --save rgb_images/calib/green_01.jpg
    python3 tools/rgb_probe.py --seconds 3 --save rgb_images/calib/yellow_01.jpg
    python3 tools/rgb_probe.py --seconds 3 --save rgb_images/calib/bg_01.jpg

分析（ROI 用归一化坐标 x,y,w,h，建议框住灯，不要把整面墙/地板框进来）：
    python3 tools/traffic_light_calibrate.py \
        --dir rgb_images/calib \
        --roi 0.35,0.20,0.30,0.30 \
        --out assets/traffic_light_profile.json

依赖：numpy + (cv2 或 ffmpeg)。容器里没有 cv2 时用 ffmpeg 解码，不需要额外安装。
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

try:
    import cv2  # 可选；容器里通常有
except Exception:  # pragma: no cover
    cv2 = None

LABELS = ("red", "green", "yellow", "bg")
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


# ---------------------------------------------------------------- 解码 / HSV
def decode_rgb(path: Path, width: int) -> np.ndarray:
    """读图并缩放到 width，返回 float32 RGB [0,1]。"""
    if cv2 is not None:
        bgr = cv2.imread(str(path))
        if bgr is None:
            raise RuntimeError(f"cv2 读图失败: {path}")
        h0, w0 = bgr.shape[:2]
        nh = max(2, int(round(h0 * width / w0 / 2.0)) * 2)
        bgr = cv2.resize(bgr, (width, nh), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

    proc = subprocess.run(
        [
            "ffmpeg", "-v", "error", "-i", str(path),
            "-vf", f"scale={width}:-2",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-",
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode("utf-8", "replace")[:300])
    height = len(proc.stdout) // (width * 3)
    if height <= 0:
        raise RuntimeError(f"ffmpeg 解码为空: {path}")
    return np.frombuffer(proc.stdout, np.uint8).reshape(height, width, 3) \
        .astype(np.float32) / 255.0


def rgb_to_hsv(img: np.ndarray):
    """向量化 RGB->HSV；H 用 0~360 度，S/V 用 0~1。"""
    r, g, b = img[..., 0], img[..., 1], img[..., 2]
    mx = img.max(axis=2)
    mn = img.min(axis=2)
    diff = mx - mn
    s = np.where(mx > 1e-6, diff / np.maximum(mx, 1e-6), 0.0)
    v = mx

    h = np.zeros_like(mx)
    m = diff > 1e-6
    rmax = m & (mx == r)
    gmax = m & (mx == g)
    bmax = m & (mx == b)
    h[rmax] = (60.0 * ((g - b) / np.maximum(diff, 1e-6)))[rmax] % 360.0
    h[gmax] = 60.0 * (((b - r) / np.maximum(diff, 1e-6))[gmax] + 2.0)
    h[bmax] = 60.0 * (((r - g) / np.maximum(diff, 1e-6))[bmax] + 4.0)
    return h, s, v


def crop_roi(img: np.ndarray, roi):
    """roi = (x, y, w, h)，归一化；返回裁剪后的图和像素边界。"""
    h, w = img.shape[:2]
    x, y, rw, rh = roi
    x0 = max(0, min(w - 1, int(round(x * w))))
    y0 = max(0, min(h - 1, int(round(y * h))))
    x1 = max(x0 + 1, min(w, int(round((x + rw) * w))))
    y1 = max(y0 + 1, min(h, int(round((y + rh) * h))))
    return img[y0:y1, x0:x1], (x0, y0, x1, y1)


def seed_mask(h, s, v, label: str) -> np.ndarray:
    """标定用的宽种子范围；尽量宽，真正的范围由样本算。"""
    if label == "red":
        return (((h < 20) | (h > 330)) & (s > 0.35) & (v > 0.25))
    if label == "yellow":
        return ((h >= 35) & (h <= 75) & (s > 0.35) & (v > 0.35))
    if label == "green":
        return ((h >= 70) & (h <= 170) & (s > 0.30) & (v > 0.20))
    # bg：所有颜色的并集，用于统计误检
    return (
        (((h < 20) | (h > 330)) & (s > 0.35) & (v > 0.25))
        | ((h >= 35) & (h <= 75) & (s > 0.35) & (v > 0.35))
        | ((h >= 70) & (h <= 170) & (s > 0.30) & (v > 0.20))
    )


def label_of(path: Path) -> str:
    prefix = path.stem.split("_", 1)[0].lower()
    return prefix if prefix in LABELS else "bg"


def analyze(path: Path, roi, width: int, min_pixels: int):
    img = decode_rgb(path, width)
    img, _ = crop_roi(img, roi)
    h, s, v = rgb_to_hsv(img)
    label = label_of(path)
    mask = seed_mask(h, s, v, label)
    n = int(mask.sum())
    ratio = n / float(mask.size) if mask.size else 0.0
    out = {
        "path": str(path), "label": label, "pixels": n, "ratio": ratio,
        "h": h[mask], "s": s[mask], "v": v[mask],
        "shape": img.shape[:2],
    }
    if n >= min_pixels:
        ys, xs = np.nonzero(mask)
        out["bbox"] = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
        out["median"] = (float(np.median(h[mask])),
                         float(np.median(s[mask])),
                         float(np.median(v[mask])))
    return out


# ---------------------------------------------------------------- 范围计算
def _expand(lo, hi, margin):
    return max(0.0, lo - margin), min(360.0, hi + margin)


def ranges_for(label: str, h, s, v):
    """从池化的像素里算 HSV 范围；红跨 0 度，拆成两段。"""
    if h.size == 0:
        return []
    s_min = max(0.0, float(np.percentile(s, 2)) - 0.05)
    v_min = max(0.0, float(np.percentile(v, 2)) - 0.05)

    if label == "red":
        # 0 度附近和 360 度附近分开算，避免 0/359 把范围拉满
        low = h[h < 180.0]
        high = h[h >= 180.0]
        total = max(1, h.size)
        ranges = []
        for cluster in (low, high):
            if cluster.size < max(10, 0.10 * total):
                continue
            lo, hi = np.percentile(cluster, [2, 98])
            lo, hi = _expand(float(lo), float(hi), 3.0)
            ranges.append([round(lo, 1), round(hi, 1),
                           round(s_min, 3), round(v_min, 3)])
        return ranges

    lo, hi = np.percentile(h, [2, 98])
    lo, hi = _expand(float(lo), float(hi), 3.0)
    return [[round(lo, 1), round(hi, 1), round(s_min, 3), round(v_min, 3)]]


def in_ranges(h, s, v, ranges) -> bool:
    for lo, hi, s_min, v_min in ranges:
        hue_ok = (lo <= h <= hi) if lo <= hi else (h >= lo or h <= hi)
        if hue_ok and s >= s_min and v >= v_min:
            return True
    return False


# ---------------------------------------------------------------- 主流程
def main() -> int:
    parser = argparse.ArgumentParser(description="玩具红绿灯 HSV 标定（不训练）")
    parser.add_argument("--dir", default="rgb_images/calib", help="样本目录")
    parser.add_argument("--roi", default="0,0,1,1",
                        help="归一化 ROI：x,y,w,h（默认整帧）")
    parser.add_argument("--out", default="assets/traffic_light_profile.json")
    parser.add_argument("--width", type=int, default=640, help="解码宽度")
    parser.add_argument("--min-pixels", type=int, default=20,
                        help="单张样本最少有效像素")
    parser.add_argument("--max-bg-ratio", type=float, default=0.01,
                        help="背景 ROI 里允许的误检面积比例")
    parser.add_argument("--dry-run", action="store_true", help="只打印，不写文件")
    args = parser.parse_args()

    try:
        roi = tuple(float(x) for x in args.roi.split(","))
        if len(roi) != 4:
            raise ValueError
    except ValueError:
        print("--roi 必须是 4 个数字：x,y,w,h（归一化，例如 0.35,0.2,0.3,0.3）")
        return 2
    if not (0 <= roi[0] < 1 and 0 <= roi[1] < 1 and roi[2] > 0 and roi[3] > 0
            and roi[0] + roi[2] <= 1.001 and roi[1] + roi[3] <= 1.001):
        print(f"--roi 越界: {roi}")
        return 2

    files = sorted(p for p in Path(args.dir).iterdir()
                   if p.suffix.lower() in IMAGE_EXTS)
    if not files:
        print(f"没找到图片: {args.dir}")
        return 2

    samples = {k: [] for k in LABELS}
    for path in files:
        label = label_of(path)
        try:
            item = analyze(path, roi, args.width, args.min_pixels)
        except Exception as e:
            print(f"跳过 {path}: {e}")
            continue
        samples[label].append(item)
        tag = "OK " if item["pixels"] >= args.min_pixels else "WARN"
        med = item.get("median")
        med_s = (f"H={med[0]:5.1f} S={med[1]:.2f} V={med[2]:.2f}"
                 if med else "无有效像素")
        print(f"{tag} {path.name:24s} label={label:6s} "
              f"px={item['pixels']:5d} ratio={item['ratio']:.4f}  {med_s}")

    ranges = {}
    for label in ("red", "green", "yellow"):
        items = [x for x in samples[label] if x["pixels"] >= args.min_pixels]
        if not items:
            print(f"缺少 {label} 样本（至少 1 张有效），无法生成该颜色范围")
            continue
        h = np.concatenate([x["h"] for x in items])
        s = np.concatenate([x["s"] for x in items])
        v = np.concatenate([x["v"] for x in items])
        ranges[label] = ranges_for(label, h, s, v)
        print(f"{label:6s}: {len(items)} 张, "
              f"{h.size} px -> {ranges[label]}")

    bg_items = [x for x in samples["bg"] if x["pixels"] > 0]
    max_bg = max((x["ratio"] for x in bg_items), default=0.0)
    print(f"背景  : {len(samples['bg'])} 张, 最大误检比例={max_bg:.4f} "
          f"(阈值 {args.max_bg_ratio})")
    if max_bg > args.max_bg_ratio:
        print("WARN 背景误检偏高：缩小 ROI 或收紧 S/V 下限")

    misses = [x for x in samples["red"] + samples["green"] + samples["yellow"]
              if x["pixels"] < args.min_pixels]
    if misses:
        print(f"WARN {len(misses)} 张颜色样本有效像素不足，"
              f"检查 ROI/曝光/灯是否在框内")

    if not ranges:
        print("没有生成任何颜色范围；先检查样本命名和 ROI")
        return 2

    profile = {
        "hsv_space": "degrees_0_360_s01_v01",
        "camera_width": args.width,
        "roi": [round(float(x), 4) for x in roi],
        "ranges": ranges,
        "min_area_ratio": 0.0008,
        "confirm_frames": 5,
        "confirm_s": 0.4,
        "stale_s": 0.5,
        "samples": {k: len(v) for k, v in samples.items()},
        "notes": "ranges: [h_lo, h_hi, s_min, v_min]；红可能两段，h_lo>h_hi 表示跨 0°",
    }

    # 用生成的范围回验颜色样本（只看中位色，不做连通域）
    for label in ("red", "green", "yellow"):
        items = [x for x in samples[label] if "median" in x]
        bad = [x["path"] for x in items
               if not in_ranges(x["median"][0], x["median"][1],
                                x["median"][2], ranges.get(label, []))]
        if bad:
            print(f"WARN {label} 有 {len(bad)} 张样本不在生成范围内: {bad[:3]}")

    if args.dry_run:
        print("dry-run：未写文件")
        return 0

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(profile, ensure_ascii=False, indent=2),
                   encoding="utf-8")
    print(f"已写入 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
