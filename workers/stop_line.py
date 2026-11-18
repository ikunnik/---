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


# ---------------------------------------------------------------- 形状门
# 立体黄色障碍（锥桶、水马、路沿、黄箱）与地面停止线在 HSV 上高度重叠，
# 单靠颜色阈值治不了：收紧到能滤掉锥桶的程度，真线也会一起丢。
# 唯一稳定的区分是形状 —— 停止线是「细长的横向带」。
#
# 判据必须旋转不变。机器人航向偏斜时停止线在图像里会变斜，若用 bbox
# 宽高比判断，倾斜 20° 就足以让真线被误杀（bbox 高度会暴涨 3 倍，
# 而 shape 本身没变）。所以这里用 PCA 的主轴长 / 次轴厚：
# 这两个量是刚体量，不随图像内旋转变化。
DEFAULT_SHAPE = {
    "enabled": True,
    "elongation_min": 6.0,       # 主轴长/次轴厚：真线 40+，锥桶≈1.8，水马≈1.9
    "length_ratio_min": 0.45,    # 主轴长/ROI 宽：线要横贯视场
    "thickness_ratio_max": 1.0,  # 次轴厚/ROI 高：默认不设限，近处真线会很厚
    "angle_max_deg": 45.0,       # 与水平夹角上限，防止把竖杆当横线
    "full_row_frac": 0.8,        # 判定「全宽行」的连续长度门槛
    "full_rows_min": 3,          # 至少要有几行接近全宽
}


def _shape_metrics(xs, ys, roi_w, roi_h):
    """旋转不变的形状指标；点数太少无法判断时返回 None（由调用方拒绝）。"""
    if xs.size < 8:
        return None
    pts = np.column_stack((xs, ys)).astype(np.float64)
    centered = pts - pts.mean(axis=0)
    try:
        _, vectors = np.linalg.eigh(centered.T @ centered)
    except np.linalg.LinAlgError:
        return None
    major = vectors[:, -1]          # 主轴（线延伸方向）
    minor = vectors[:, 0]           # 次轴（线的厚度方向）
    along = centered @ major
    across = centered @ minor
    length = float(along.max() - along.min())
    thickness = float(across.max() - across.min())
    angle = math.degrees(math.atan2(float(major[1]), float(major[0])))
    if angle > 90.0:
        angle -= 180.0
    elif angle < -90.0:
        angle += 180.0
    return {
        "length": length,
        "thickness": thickness,
        "elongation": length / max(thickness, 1.0),
        "length_ratio": length / max(float(roi_w), 1.0),
        "thickness_ratio": thickness / max(float(roi_h), 1.0),
        "angle": angle,
    }


def _full_rows(mask, roi_w, frac=0.8) -> int:
    """「连续黄色接近 ROI 全宽」的行数。

    停止线横贯整个视场，而任何障碍物（除非贴脸挡满视野）都做不到让某一行
    的连续黄色占到 80% 以上。这条判据专治「障碍物与停止线在图像里粘连」：
    此时两者的点云连成一片，PCA 伸长比被拽到 4 左右，只有按行看才分得开。
    """
    thr = frac * float(roi_w)
    count = 0
    for runs in _row_runs(mask):
        if runs and max(e - s for s, e in runs) >= thr:
            count += 1
    return count


def _shape_ok(metrics, shape) -> bool:
    """通路 A：细长 + 横贯 + 不过厚 + 不竖立。

    四条都用 PCA 旋转不变量，航向偏斜不会误杀；通路 B（全宽行）在
    detect_stop_line 里单独判，两条是并列关系，不是与关系。
    """
    if metrics is None:
        return False
    return (
        metrics["elongation"] >= float(shape["elongation_min"])
        and metrics["length_ratio"] >= float(shape["length_ratio_min"])
        and metrics["thickness_ratio"] <= float(shape["thickness_ratio_max"])
        and abs(metrics["angle"]) <= float(shape["angle_max_deg"])
    )


def _row_runs(mask):
    """每行连续 True 段 [(x_start, x_end_exclusive), ...]，向量化提取。"""
    pad = np.zeros((mask.shape[0], mask.shape[1] + 2), dtype=np.int8)
    pad[:, 1:-1] = mask
    diff = np.diff(pad, axis=1)
    rs, cs = np.nonzero(diff == 1)
    _, ce = np.nonzero(diff == -1)
    out = [[] for _ in range(mask.shape[0])]
    for r, s, e in zip(rs.tolist(), cs.tolist(), ce.tolist()):
        out[r].append((s, e))
    return out


def _connected_regions(mask, min_pixels=60):
    """按「相邻行 x 区间重叠」把 mask 拆成空间区域。

    必须拆，不能对整张 mask 做一次 PCA：黄牌和真停止线同时落在 ROI 里时，
    合并点云的主轴会被扯成斜向，实测伸长比从 58 掉到 4.2 —— 真停止线
    被判掉，机器狗会直接冲过路口。分开逐区域判定才能既滤掉黄牌、又保住真线。
    """
    rows = _row_runs(mask)
    active = []      # [x0, x1, y0, y1, pixels, 本行被延续]
    done = []
    for y, runs in enumerate(rows):
        for s, e in runs:
            for reg in active:
                if not reg[5] and s <= reg[1] and e >= reg[0]:
                    reg[0] = min(reg[0], s)
                    reg[1] = max(reg[1], e)
                    reg[3] = y
                    reg[4] += e - s
                    reg[5] = True
                    break
            else:
                active.append([s, e, y, y, e - s, True])
        alive = []
        for reg in active:
            if reg[5]:
                reg[5] = False
                alive.append(reg)
            else:
                done.append(reg)
        active = alive
    done.extend(active)
    return [(r[0], r[1], r[2], r[3], r[4]) for r in done if r[4] >= min_pixels]


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
    bbox = (
        int(xs.min()), int(ys.min()),
        int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1),
    )
    shape = {**DEFAULT_SHAPE, **(profile.get("shape") or {})}

    if not shape.get("enabled", True):
        # 形状门关闭：完全退回旧行为，保证现场可一键回滚。
        metrics = _shape_metrics(xs, ys, crop.shape[1], crop.shape[0])
        angle = metrics["angle"] if metrics else 0.0
        row = (y0 + int(ys.max()) + 1) / float(full_h)
        return StopLineState(True, row, ts, ratio, bbox, angle)

    # 逐区域判定：任一区域像停止线就算看到线，取最近（下边界最大）的那个。
    # 整体判定的失败模式见 _connected_regions 的注释（黄牌会把真线拽歪）。
    roi_w, roi_h = crop.shape[1], crop.shape[0]
    best_metrics = None
    best_bottom = -1
    for x0r, x1r, y0r, y1r, _npx in _connected_regions(mask):
        rys, rxs = np.nonzero(mask[y0r:y1r + 1, x0r:x1r + 1])
        metrics = _shape_metrics(rxs, rys, roi_w, roi_h)
        if not _shape_ok(metrics, shape):
            continue
        if y1r > best_bottom:
            best_bottom, best_metrics = y1r, metrics

    if best_metrics is not None:
        # row 取通过区域的下边界：越靠近机器人越大，语义与旧版一致。
        row = (y0 + best_bottom + 1) / float(full_h)
        return StopLineState(True, row, ts, ratio, bbox, best_metrics["angle"])

    # 通路 B：有若干行连续黄色接近全宽。障碍物与停止线在图像里粘连时，
    # 两者的点云连成一片，A 通路会被拽到伸长比 4 左右而漏检 —— 此时只能
    # 按行看。row 用整体底边，宁可提前一点停车，也不能冲过路口。
    if _full_rows(mask, roi_w, float(shape.get("full_row_frac", 0.8))) >= int(
        shape.get("full_rows_min", 3)
    ):
        metrics = _shape_metrics(xs, ys, roi_w, roi_h)
        row = (y0 + int(ys.max()) + 1) / float(full_h)
        return StopLineState(
            True, row, ts, ratio, bbox, metrics["angle"] if metrics else 0.0
        )

    # 拒绝时 row 仍给 0.0：下游 _line_fresh 会先判 visible，但这里不向
    # 下游暗示「黄色就在眼前」。area_ratio / bbox / angle 照常返回，
    # 现场日志可据此区分「没有黄色像素」与「有黄色但不像停止线」——
    # 这正是排查误检需要的信息。
    metrics = _shape_metrics(xs, ys, roi_w, roi_h)
    return StopLineState(
        False, 0.0, ts, ratio, bbox, metrics["angle"] if metrics else 0.0
    )


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
