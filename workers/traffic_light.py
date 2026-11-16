"""
红绿灯检测进程（camera0 RGB -> ROI/HSV -> 时序投票 -> TrafficLightState）。

为什么单独一个进程：
    cv2 解码 + 颜色计算是原生库和 CPU 工作；隔离出去，崩了不影响避障。
    输入是 RgbPerceptionWorker fan-out 的最新 RgbFrame（camera0 JPEG），
    输出 TrafficLightState；主进程的 TrafficLightWorker 把它搬到 Latest。

文件分三部分：
    纯函数：load_profile / rgb_to_hsv / range_mask / detect_hsv
    投票器：TrafficLightVoter（连续帧确认 + 丢帧保持）
    外壳：  TrafficLightProcess（进程）/ TrafficLightWorker（线程）

注意：
    - 本模块顶层不 import cv2；只有在进程 run()/decode_jpeg() 里才导入，
      这样宿主机没有 cv2 也能跑单元测试。
    - HSV 用 degrees_0_360 + S/V 0~1，和 assets/traffic_light_profile.json 一致。
    - 检测本身只输出状态；默认停止线序列在到线后设置 traffic_active 并消费它。
"""
import json
import logging
import multiprocessing
import queue
import signal
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from comm import Latest, put_latest, setup_process_logging
from messages import RgbFrame, TrafficLightState


# ---------------------------------------------------------------- profile
def load_profile(path) -> dict:
    """读取标定 profile；缺字段直接报错，避免静默用错阈值。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("roi", "ranges", "min_area_ratio"):
        if key not in data:
            raise ValueError(f"profile 缺少字段: {key}")
    return data


# ---------------------------------------------------------------- HSV / ROI
def _roi_crop(img: np.ndarray, roi) -> np.ndarray:
    h, w = img.shape[:2]
    x, y, rw, rh = roi
    x0 = max(0, min(w - 1, int(round(x * w))))
    y0 = max(0, min(h - 1, int(round(y * h))))
    x1 = max(x0 + 1, min(w, int(round((x + rw) * w))))
    y1 = max(y0 + 1, min(h, int(round((y + rh) * h))))
    return img[y0:y1, x0:x1]


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


def range_mask(h, s, v, ranges) -> np.ndarray:
    """ranges: [[h_lo, h_hi, s_min, v_min], ...]；h_lo>h_hi 表示跨 0°。"""
    mask = np.zeros(h.shape, dtype=bool)
    for lo, hi, s_min, v_min in ranges:
        hue_ok = ((h >= lo) & (h <= hi)) if lo <= hi else ((h >= lo) | (h <= hi))
        mask |= hue_ok & (s >= s_min) & (v >= v_min)
    return mask


def detect_hsv(rgb: np.ndarray, profile: dict,
               now: float | None = None) -> TrafficLightState:
    """纯函数：RGB [0,1] -> TrafficLightState（未做时序投票，stable=False）。"""
    ts = time.time() if now is None else now
    crop = _roi_crop(rgb, profile["roi"])
    if crop.size == 0:
        return TrafficLightState("unknown", False, ts)

    h, s, v = rgb_to_hsv(crop)
    area = float(crop.shape[0] * crop.shape[1])
    min_ratio = float(profile.get("min_area_ratio", 0.001))

    detected = {}  # color -> (ratio, bbox)
    for color, ranges in profile["ranges"].items():
        mask = range_mask(h, s, v, ranges)
        n = int(mask.sum())
        if n == 0:
            continue
        ratio = n / area
        if ratio < min_ratio:
            continue
        ys, xs = np.nonzero(mask)
        bbox = (
            int(xs.min()), int(ys.min()),
            int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1),
        )
        detected[color] = (ratio, bbox)

    # 红绿同时出现时判绿：场景里常见红灯灯罩反光/背景红色块与绿灯并存，
    # 以绿灯为准，避免明明绿灯却按红灯停住。
    if "green" in detected and "red" in detected:
        ratio, bbox = detected["green"]
        return TrafficLightState("green", False, ts, bbox, ratio)

    if not detected:
        return TrafficLightState("unknown", False, ts)
    color = max(detected, key=lambda c: detected[c][0])
    ratio, bbox = detected[color]
    return TrafficLightState(color, False, ts, bbox, ratio)


# ---------------------------------------------------------------- 时序投票
class TrafficLightVoter:
    """连续帧确认：同一颜色连续 N 帧且持续 confirm_s 才 stable。

    丢帧/unknown 时保持上一稳定颜色直到 stale_s，避免一帧闪烁就丢掉红灯。
    """

    def __init__(self, profile: dict):
        self.confirm_frames = max(1, int(profile.get("confirm_frames", 5)))
        self.confirm_s = max(0.0, float(profile.get("confirm_s", 0.4)))
        self.stale_s = max(0.0, float(profile.get("stale_s", 0.5)))
        self._last_color = "unknown"
        self._count = 0
        self._first_ts = 0.0
        self._last_seen = 0.0
        self._stable = "unknown"

    def update(self, raw: TrafficLightState, now: float) -> TrafficLightState:
        if raw.color == "unknown":
            if self._stable != "unknown" and (now - self._last_seen) <= self.stale_s:
                return replace(raw, color=self._stable, stable=True, ts=now)
            self._stable = "unknown"
            self._last_color = "unknown"
            self._count = 0
            return replace(raw, color="unknown", stable=False, ts=now)

        self._last_seen = now
        if raw.color == self._last_color:
            self._count += 1
        else:
            self._last_color = raw.color
            self._count = 1
            self._first_ts = now

        if (
            self._count >= self.confirm_frames
            and (now - self._first_ts) >= self.confirm_s
        ):
            self._stable = raw.color
            return replace(raw, stable=True, ts=now)
        return replace(raw, stable=False, ts=now)


# ---------------------------------------------------------------- 解码
def decode_jpeg(data: bytes, width: int) -> np.ndarray:
    """JPEG -> RGB float32 [0,1]，缩放到指定宽度；cv2 只在调用时导入。"""
    import cv2  # noqa: F401  (可选依赖，只在检测进程里用)

    arr = np.frombuffer(data, np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("cv2.imdecode 失败（不是 JPEG？）")
    h, w = bgr.shape[:2]
    nh = max(2, int(round(h * width / w / 2.0)) * 2)
    bgr = cv2.resize(bgr, (width, nh), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0


def detect_ros_frame(frame, profile, voter, *, clock=time.time):
    """Keep capture time through ROS input; slow decoding cannot refresh stale data."""
    if not 0 <= clock() - frame.ts < profile["input_stale_s"]:
        return None
    rgb = decode_jpeg(frame.data, int(profile.get("camera_width", 960)))
    raw = detect_hsv(rgb, profile, now=frame.ts)
    if not 0 <= clock() - frame.ts < profile["input_stale_s"]:
        return None
    return voter.update(raw, frame.ts)


# ---------------------------------------------------------------- 进程
class TrafficLightProcess(multiprocessing.Process):
    """消费 RgbFrame -> 解码 -> detect_hsv -> 投票 -> TrafficLightState。"""

    def __init__(self, cfg, in_queue, out_queue, active=None,
                 name="traffic_light"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.out_queue = out_queue
        # 跨进程布尔：False 时只收帧不处理（相机采集仍然常开）
        self.active = active
        self._stop_event = multiprocessing.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()

        try:
            import cv2  # noqa: F401
        except Exception as e:
            logging.error("[红绿灯] 无法导入 cv2: %s", e)
            return

        try:
            profile = load_profile(self.cfg.traffic_light_profile)
        except Exception as e:
            logging.error(
                "[红绿灯] 读取 profile 失败 %s: %s",
                self.cfg.traffic_light_profile, e,
            )
            return

        width = int(profile.get("camera_width", 960))
        voter = TrafficLightVoter(profile)
        logging.info(
            "[红绿灯] 已加载 %s（roi=%s, width=%d）",
            self.cfg.traffic_light_profile, profile.get("roi"), width,
        )

        fps_ts = time.time()
        fps_cnt = 0
        last = None
        was_active = False
        last_source_ts = 0.0
        ros_input = self.cfg.camera_backend == "ros2"
        profile = dict(profile, input_stale_s=self.cfg.traffic_state_stale_s)
        while not self._stop_event.is_set():
            active_now = self.active is None or self.active.is_set()
            if active_now and not was_active:
                # 到停止线后才开始处理：从干净投票器开始，避免旧颜色
                voter = TrafficLightVoter(profile)
                logging.info("[红绿灯] traffic_active=True，开始处理")
            elif was_active and not active_now:
                logging.info("[红绿灯] traffic_active=False，暂停处理")
            was_active = active_now

            try:
                frame = self.in_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            except (EOFError, OSError, ValueError):
                break

            if not active_now or not isinstance(frame, RgbFrame):
                continue

            try:
                if ros_input:
                    if frame.ts <= last_source_ts:
                        continue
                    last_source_ts = frame.ts
                    state = detect_ros_frame(frame, profile, voter)
                    if state is None:
                        continue
                else:
                    rgb = decode_jpeg(frame.data, width)
                    raw = detect_hsv(rgb, profile)
                    state = voter.update(raw, time.time())
            except Exception as e:
                logging.warning("[红绿灯] 解码失败: %s", e)
                continue

            if (
                last is None
                or state.color != last.color
                or state.stable != last.stable
            ):
                logging.info(
                    "[红绿灯] %s stable=%s area=%.3f%% bbox=%s",
                    state.color, state.stable,
                    state.area_ratio * 100.0, state.bbox,
                )
            last = state
            put_latest(self.out_queue, state)

            fps_cnt += 1
            now = time.time()
            if now - fps_ts >= self.cfg.rgb_log_s:
                logging.info(
                    "[红绿灯] %.1f FPS", fps_cnt / (now - fps_ts)
                )
                fps_ts, fps_cnt = now, 0

        logging.info("[红绿灯] 退出")


# ---------------------------------------------------------------- 线程
class TrafficLightWorker(threading.Thread):
    """把检测进程的输出搬到 Latest，供以后的红灯门控读取。"""

    def __init__(self, cfg, in_queue, out: Latest, name="traffic_light_worker"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.out = out
        self._stop_event = threading.Event()
        self._frames = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        logging.info("[红绿灯感知] 启动")
        while not self._stop_event.is_set():
            try:
                state = self.in_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            except (EOFError, OSError, ValueError):
                break

            if not isinstance(state, TrafficLightState):
                continue

            self.out.set(state)
            self._frames += 1
            if self._frames == 1:
                logging.info("[红绿灯感知] 第一帧: %s", state)

        logging.info("[红绿灯感知] 退出（共 %d 帧）", self._frames)
