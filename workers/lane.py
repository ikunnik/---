"""盲道图像识别与队列转发；行为判断集中在 decisions/lane.py。
"""
import json
import logging
import math
import queue
import threading
import time
from pathlib import Path

import numpy as np

from comm import Latest, put_latest
from messages import LaneState
from workers.traffic_light import range_mask, rgb_to_hsv

LANE = "LANE"


def load_profile(path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("roi", "ranges", "near_row", "far_row"):
        if key not in data:
            raise ValueError(f"lane profile 缺少字段: {key}")
    return data


def _crop(img: np.ndarray, roi):
    h, w = img.shape[:2]
    x, y, rw, rh = roi
    x0 = max(0, min(w - 1, int(round(x * w))))
    y0 = max(0, min(h - 1, int(round(y * h))))
    x1 = max(x0 + 1, min(w, int(round((x + rw) * w))))
    y1 = max(y0 + 1, min(h, int(round((y + rh) * h))))
    return img[y0:y1, x0:x1], (x0, y0, x1, y1)


def detect_lane(rgb: np.ndarray, profile: dict,
                now: float | None = None) -> LaneState:
    """纯函数：RGB[0,1] -> LaneState；center/heading 都按整帧归一化。"""
    ts = time.time() if now is None else now
    h, w = rgb.shape[:2]
    crop, (x0, y0, _, _) = _crop(rgb, profile["roi"])
    if crop.size == 0:
        return LaneState(False, ts=ts)

    hh, ss, vv = rgb_to_hsv(crop)
    mask = range_mask(hh, ss, vv, profile["ranges"])
    ys, xs = np.nonzero(mask)
    min_pixels = int(profile.get("min_pixels", 80))
    if len(xs) < min_pixels:
        return LaneState(False, ts=ts)

    y_norm = (ys + y0) / float(h)
    x_norm = (xs + x0) / float(w)
    near_row = float(profile["near_row"])
    far_row = float(profile["far_row"])
    near_band = float(profile.get("near_band", 0.03))
    min_near = int(profile.get("min_near_pixels", 8))
    width_near = float(profile.get("width_near", 0.62))
    width_far = float(profile.get("width_far", 0.42))
    min_sep = float(profile.get("min_line_sep", 0.12))
    lo, hi = -0.35, 1.35

    def fit(sel):
        count = int(sel.sum())
        if count < min_pixels:
            return None
        if int(np.count_nonzero(sel & (np.abs(y_norm - near_row) <= near_band))) < min_near:
            return None
        a, b = np.polyfit(y_norm[sel], x_norm[sel], 1)
        xn = float(a * near_row + b)
        xf = float(a * far_row + b)
        if not (lo <= xn <= hi and lo <= xf <= hi):
            return None
        return (xn, xf, float(a), count)

    # 两条线可能都在画面中线同一侧（机器人偏得很远）；不能用 x<0.5 硬分。
    # 先对黄色像素的 x 做 2-means，再按簇拟合。
    c0, c1 = [float(v) for v in np.percentile(x_norm, [25, 75])]
    if abs(c1 - c0) < 1e-6:
        c0, c1 = float(x_norm.min()), float(x_norm.max())
    assign = np.abs(x_norm - c0) <= np.abs(x_norm - c1)
    for _ in range(12):
        if not assign.any() or assign.all():
            break
        n0 = float(x_norm[assign].mean())
        n1 = float(x_norm[~assign].mean())
        if abs(n0 - c0) + abs(n1 - c1) < 1e-5:
            c0, c1 = n0, n1
            break
        c0, c1 = n0, n1
        assign = np.abs(x_norm - c0) <= np.abs(x_norm - c1)
    if c0 > c1:
        c0, c1 = c1, c0
        assign = ~assign

    if profile.get("mode", "dual") == "single":
        if abs(c1 - c0) < min_sep:
            sel = np.ones_like(x_norm, dtype=bool)
        elif int(assign.sum()) >= int((~assign).sum()):
            sel = assign
        else:
            sel = ~assign
        ys_sel = y_norm[sel]
        xs_sel = x_norm[sel]
        count = int(sel.sum())
        if count < min_pixels:
            return LaneState(False, ts=ts)
        # 单线不要求固定 near_row：线可能只出现在 0.60~0.66 的远处。
        # 用线自身的最低点作为 near，向上 0.15 作为 far。
        y_bottom = float(ys_sel.max())
        y_top = float(ys_sel.min())
        if y_bottom - y_top < 0.03:
            return LaneState(False, ts=ts)
        near = y_bottom
        far = max(y_top, y_bottom - 0.15)
        a, b = np.polyfit(ys_sel, xs_sel, 1)
        xn = float(a * near + b)
        xf = float(a * far + b)
        if not (lo <= xn <= hi and lo <= xf <= hi):
            return LaneState(False, ts=ts)
        # 单线就是车道中心线：center 直接用黄线本身，不补车道宽度。
        center = xn
        center_far = xf
        width = 0.0
        left_x = right_x = xn
        confidence = max(0.0, min(1.0, count / float(min_pixels * 4)))
        dx = (xf - xn) * w
        dy = (near - far) * h
        heading_deg = math.degrees(math.atan2(dx, max(dy, 1e-6)))
        return LaneState(
            visible=True,
            center=center,
            center_far=center_far,
            heading_deg=heading_deg,
            width=width,
            left_x=left_x,
            right_x=right_x,
            ts=ts,
            confidence=confidence,
            single=True,
        )

    one_line = False
    if abs(c1 - c0) < min_sep:
        f = fit(np.ones_like(x_norm, dtype=bool))
        if f is None:
            return LaneState(False, ts=ts)
        left_fit = right_fit = None
        one_fit = f
        one_line = True
    else:
        left_fit = fit(assign)
        right_fit = fit(~assign)
        one_fit = None
        if left_fit is not None and right_fit is not None:
            ln, lf, la, lc = left_fit
            rn, rf, ra, rc = right_fit
            if ln <= rn:
                left_near, left_far = ln, lf
                right_near, right_far = rn, rf
                left_n, right_n = lc, rc
            else:
                left_near, left_far = rn, rf
                right_near, right_far = ln, lf
                left_n, right_n = rc, lc
        elif left_fit is not None or right_fit is not None:
            one_fit = left_fit if left_fit is not None else right_fit
            one_line = True
        else:
            return LaneState(False, ts=ts)

    if one_line:
        xn, xf, a, count = one_fit
        # 单线身份用斜率判断：左线 y 增大 x 减小(a<0)，右线 a>0。
        if a < -1e-4:
            side = "left"
            left_near, right_near = xn, xn + width_near
            left_far, right_far = xf, xf + width_far
        elif a > 1e-4:
            side = "right"
            left_near, right_near = xn - width_near, xn
            left_far, right_far = xf - width_far, xf
        elif xn < 0.5:
            side = "left"
            left_near, right_near = xn, xn + width_near
            left_far, right_far = xf, xf + width_far
        else:
            side = "right"
            left_near, right_near = xn - width_near, xn
            left_far, right_far = xf - width_far, xf
        width = width_near
        confidence = min(1.0, count / float(min_pixels * 4)) * 0.6
    else:
        side = ""
        width = right_near - left_near
        if not (
            float(profile.get("min_width", 0.25))
            <= width
            <= float(profile.get("max_width", 0.90))
        ):
            return LaneState(False, ts=ts)
        confidence = min(left_n, right_n) / float(min_pixels * 4)

    center = (left_near + right_near) / 2.0
    center_far = (left_far + right_far) / 2.0
    # far 中心在 near 中心右侧 -> heading 为正（图像 x 向右）
    dx = (center_far - center) * w
    dy = (near_row - far_row) * h
    heading_deg = math.degrees(math.atan2(dx, max(dy, 1e-6)))
    confidence = max(0.0, min(1.0, confidence))

    return LaneState(
        visible=True,
        center=center,
        center_far=center_far,
        heading_deg=heading_deg,
        width=width,
        left_x=left_near,
        right_x=right_near,
        ts=ts,
        confidence=confidence,
        one_line=one_line,
        one_line_side=side,
    )


class LaneWorker(threading.Thread):
    """把 LaneCameraProcess 输出的 LaneState 搬到 Latest。"""

    def __init__(self, cfg, in_queue, out: Latest, name="lane_worker"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.out = out
        self._stop_event = threading.Event()
        self._frames = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        logging.info("[车道感知] 启动")
        while not self._stop_event.is_set():
            try:
                state = self.in_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            except (EOFError, OSError, ValueError):
                break
            if not isinstance(state, LaneState):
                continue
            self.out.set(state)
            self._frames += 1
            if self._frames == 1:
                logging.info("[车道感知] 第一帧: %s", state)
        logging.info("[车道感知] 退出（共 %d 帧）", self._frames)
