"""
停止线检测进程（camera2 RGB -> 下方 ROI -> 黄线行号 -> StopLineState）。

为什么用 camera2：
    camera0（纯前置 RGB）看不到地面黄线；camera2（深度相机的 RGB）视角偏下，
    能看到停止线。camera2 和 camera2/image_depth 同模组，后续要测真实距离也方便。

检测本身只输出 StopLineState；默认停止线序列会消费它，在 row 到
stop_row 时停车并 set(traffic_active)。

纯函数：load_profile / detect_stop_line / StopLineVoter
外壳：  StopLineProcess（自己订阅 camera2 RGB）/ StopLineWorker（队列 -> Latest）
"""
import json
import math
import logging
import multiprocessing
import os
import queue
import signal
import threading
import time
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np

from comm import Latest, put_latest, setup_process_logging
from messages import RgbFrame, StopLineState
from workers.rgb_camera import decode_compressed
from workers.traffic_light import decode_jpeg, range_mask, rgb_to_hsv


# ---------------------------------------------------------------- profile
def load_profile(path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("roi", "ranges", "min_area_ratio"):
        if key not in data:
            raise ValueError(f"stop_line profile 缺少字段: {key}")
    return data


# ---------------------------------------------------------------- 检测
def _crop(img: np.ndarray, roi):
    h, w = img.shape[:2]
    x, y, rw, rh = roi
    x0 = max(0, min(w - 1, int(round(x * w))))
    y0 = max(0, min(h - 1, int(round(y * h))))
    x1 = max(x0 + 1, min(w, int(round((x + rw) * w))))
    y1 = max(y0 + 1, min(h, int(round((y + rh) * h))))
    return img[y0:y1, x0:x1], (x0, y0, x1, y1)


def detect_stop_line(rgb: np.ndarray, profile: dict,
                     now: float | None = None) -> StopLineState:
    """纯函数：RGB [0,1] -> StopLineState；row 是黄线底边/整帧高度。"""
    ts = time.time() if now is None else now
    full_h = rgb.shape[0]
    crop, (x0, y0, _, _) = _crop(rgb, profile["roi"])
    if crop.size == 0:
        return StopLineState(False, 0.0, ts)

    h, s, v = rgb_to_hsv(crop)
    mask = range_mask(h, s, v, profile["ranges"])
    n = int(mask.sum())
    area = float(crop.shape[0] * crop.shape[1])
    ratio = n / area if area else 0.0
    if ratio < float(profile.get("min_area_ratio", 0.002)):
        return StopLineState(False, 0.0, ts, ratio)

    ys, xs = np.nonzero(mask)
    # 黄线近边/底边：越靠近机器人，row 越大
    row = (y0 + int(ys.max()) + 1) / float(full_h)
    bbox = (
        int(xs.min()), int(ys.min()),
        int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1),
    )
    # PCA 主方向：黄线在图像里的倾角。机器人航向与停止线不垂直时，
    # 横线会变斜；把这个角度交给 TrafficSequence 在停车后微调 yaw。
    angle = 0.0
    if len(xs) >= 8:
        points = np.column_stack((xs, ys)).astype(np.float64)
        points -= points.mean(axis=0)
        try:
            _, vectors = np.linalg.eigh(points.T @ points)
            vx, vy = vectors[:, -1]
            angle = math.degrees(math.atan2(float(vy), float(vx)))
            if angle > 90.0:
                angle -= 180.0
            elif angle < -90.0:
                angle += 180.0
        except Exception:
            angle = 0.0
    return StopLineState(True, row, ts, ratio, bbox, angle)


class StopLineVoter:
    """连续 confirm_frames 帧可见才输出 visible；丢帧保持 stale_s。"""

    def __init__(self, profile: dict):
        self.confirm_frames = max(1, int(profile.get("confirm_frames", 3)))
        self.stale_s = max(0.0, float(profile.get("stale_s", 0.3)))
        self._history = deque(maxlen=max(10, self.confirm_frames))
        self._last_confirmed = None
        self._last_seen = 0.0

    def update(self, raw: StopLineState, now: float) -> StopLineState:
        if raw.visible:
            # 大间隔后重新看到线：旧帧/旧确认都不能和新帧拼成一次确认。
            if self._last_seen > 0.0 and (now - self._last_seen) > self.stale_s:
                self._history.clear()
                self._last_confirmed = None
            self._history.append(raw)
            self._last_seen = now
            if len(self._history) >= self.confirm_frames:
                recent = list(self._history)[-self.confirm_frames:]
                row = float(np.median([s.row for s in recent]))
                angle = float(np.median([s.angle_deg for s in recent]))
                self._last_confirmed = replace(
                    raw, visible=True, row=row, angle_deg=angle, ts=now
                )
                return self._last_confirmed
            return replace(raw, visible=False, ts=now)

        if (
            self._last_confirmed is not None
            and (now - self._last_seen) <= self.stale_s
        ):
            return replace(self._last_confirmed, ts=now)

        # 超过 stale：清空历史，避免相机恢复后旧帧参与新的确认。
        self._history.clear()
        self._last_confirmed = None
        return replace(raw, visible=False, row=0.0, ts=now)


# ---------------------------------------------------------------- 进程
class StopLineProcess(multiprocessing.Process):
    """订阅 camera2 RGB -> 检测黄线 -> StopLineState。"""

    def __init__(self, cfg, out_queue, ready=None, first_frame=None,
                 name="stop_line"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.out_queue = out_queue
        # 启动握手：main.py 在启动决策线程前等这两个事件，避免检测器
        # 悄悄卡死时仍然让普通避障带着 0.5 m/s 前进。
        self.ready = ready
        self.first_frame = first_frame
        self._stop_event = multiprocessing.Event()
        self._latest = None
        self._seq = 0

    def stop(self):
        self._stop_event.set()

    def _mark_ready(self):
        if self.ready is not None:
            self.ready.set()

    def _mark_first_frame(self):
        if self.first_frame is not None:
            self.first_frame.set()

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()
        try:
            self._run()
        except Exception:
            # 子进程里的异常默认只会打到 stderr，容易被管道/日志吞掉；
            # 这里显式打 traceback，父进程的启动握手随后会失败并中止运动。
            logging.exception("[停止线] 进程异常退出")
        finally:
            logging.info("[停止线] 进程结束 pid=%d", os.getpid())

    def _run(self):
        logging.info("[停止线] 进程启动 pid=%d", os.getpid())

        try:
            import cv2  # noqa: F401
            import dds_middleware_python as dds
        except Exception as e:
            logging.error("[停止线] 依赖导入失败: %s", e)
            raise SystemExit(1)
        logging.info("[停止线] 依赖导入完成")

        try:
            profile = load_profile(self.cfg.stop_line_profile)
        except Exception as e:
            logging.error(
                "[停止线] 读取 profile 失败 %s: %s",
                self.cfg.stop_line_profile, e,
            )
            raise SystemExit(1)
        logging.info(
            "[停止线] profile 已加载: %s", self.cfg.stop_line_profile
        )

        logging.info("[停止线] 正在初始化 DDS: %s", self.cfg.dds_config)
        try:
            middleware = dds.PyDDSMiddleware(self.cfg.dds_config)
            logging.info(
                "[停止线] DDS 已初始化，正在订阅 %s",
                self.cfg.stop_line_topic,
            )
            middleware.subscribeCompressedImage(
                self.cfg.stop_line_topic, self._on_image
            )
        except Exception as e:
            # 这里最容易出现“子进程没声音”的失败，exception() 会带上堆栈。
            logging.exception("[停止线] 订阅失败: %s", e)
            raise SystemExit(1)

        width = int(profile.get("camera_width", 640))
        voter = StopLineVoter(profile)
        logging.info(
            "[停止线] 已订阅 %s（profile=%s, ranges=%s, stop_row=%s）",
            self.cfg.stop_line_topic, self.cfg.stop_line_profile,
            profile.get("ranges"), profile.get("stop_row"),
        )
        self._mark_ready()

        fps_ts = time.time()
        fps_cnt = 0
        last = None
        last_status = time.time()
        last_frame_ts = time.time()
        first = True
        while not self._stop_event.is_set():
            frame = self._latest
            if frame is None:
                now = time.time()
                if now - last_frame_ts >= 5.0:
                    logging.warning(
                        "[停止线] 已订阅但 %.0fs 未收到图像帧（%s）",
                        now - last_frame_ts, self.cfg.stop_line_topic,
                    )
                    last_frame_ts = now
                time.sleep(0.01)
                continue
            self._latest = None
            last_frame_ts = time.time()

            try:
                rgb = decode_jpeg(frame.data, width)
            except Exception as e:
                logging.warning("[停止线] 解码失败: %s", e)
                continue

            raw = detect_stop_line(rgb, profile)
            state = voter.update(raw, time.time())

            changed = (
                last is None
                or state.visible != last.visible
                or abs(state.row - last.row) >= 0.01
            )
            if changed:
                logging.info("[停止线] %s", state)
            last = state
            put_latest(self.out_queue, state)

            if first:
                first = False
                self._mark_first_frame()
                logging.info("[停止线] 第一帧已产出: %s", state)

            fps_cnt += 1
            now = time.time()
            if now - last_status >= 1.0:
                # 行号不变时也每秒打一条，方便判断“机器狗到底有没有在接近”
                logging.info("[停止线] 状态 %s", state)
                last_status = now
            if now - fps_ts >= self.cfg.rgb_log_s:
                logging.info("[停止线] %.1f FPS", fps_cnt / (now - fps_ts))
                fps_ts, fps_cnt = now, 0

        logging.info("[停止线] 退出")

    def _on_image(self, msg):
        try:
            self._seq += 1
            frame = decode_compressed(msg, seq=self._seq, now=time.time())
            if frame.data:
                self._latest = frame
        except Exception as e:
            logging.warning("[停止线] 解析失败: %s", e)


# ---------------------------------------------------------------- 线程
class StopLineWorker(threading.Thread):
    """把停止线进程的输出搬到 Latest。"""

    def __init__(self, cfg, in_queue, out: Latest, name="stop_line_worker"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.out = out
        self._stop_event = threading.Event()
        self._frames = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        logging.info("[停止线感知] 启动")
        while not self._stop_event.is_set():
            try:
                state = self.in_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            except (EOFError, OSError, ValueError):
                break

            if not isinstance(state, StopLineState):
                continue
            self.out.set(state)
            self._frames += 1
            if self._frames == 1:
                logging.info("[停止线感知] 第一帧: %s", state)

        logging.info("[停止线感知] 退出（共 %d 帧）", self._frames)
