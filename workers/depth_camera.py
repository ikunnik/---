"""
深度相机进程（整个项目里唯一的"进程"，其他都是线程）。

为什么用进程而不是线程？
    DDS 是原生库，万一它崩了/卡了，不会把主控一起拖死。
    而且它只往外送一个小结果，用 Queue 传一条数据就够了。

文件分两半：
    上半部分 compute_profile() —— 纯函数，输入深度图，输出多区域剖面，离线回放也能用
    下半部分 DepthCameraProcess —— 进程外壳，负责订阅 DDS 并调用上面的函数

【运行顺序（进程）】
    __init__()             只保存参数，不启动任何东西
    start()（main 里调用）   操作系统真正创建子进程，随后子进程执行 run()
    run()（子进程）          ① 忽略 Ctrl+C ② 导入并初始化 DDS ③ 订阅话题
                            ④ 进入 while：取回调写来的帧 -> 算距离 -> 放进队列
    stop()（父进程调用）     设置 Event，run() 的 while 下一轮退出

【两个线程交错】
    DDS 回调线程: DDS 收到数据 -> _on_depth() 解析 -> 写 self._latest
    run() 线程:   读 self._latest -> compute_profile() -> 放进 out_queue
"""
import logging
import multiprocessing
import signal
import time

import numpy as np

from comm import put_latest, setup_process_logging
from messages import Perception


def _zone_distance(depth, cfg, left_ratio, right_ratio, min_pixels):
    """算一个垂直带的最近距离（米）。有效像素不够就返回 (inf, False)。"""
    h, w = depth.shape
    top = int(h * cfg.roi_top)
    bottom = int(h * cfg.roi_bottom)
    left = int(w * left_ratio)
    right = int(w * right_ratio)
    roi = depth[top:bottom, left:right]

    valid = roi[roi > 0]                      # 0 = 无回波 / 超量程
    if valid.size < min_pixels:
        return float("inf"), False

    dist_mm = float(np.percentile(valid, cfg.percentile))
    return dist_mm / 1000.0, True


def compute_profile(depth, cfg, ts=None) -> Perception:
    """
    纯函数：从 480x640 的 uint16 深度图（单位 mm）算出多区域剖面。

    中心 → 正前方距离、near_ratio
    左侧 → 左前空间
    右侧 → 右前空间
    min_m → 三个区域里最近的一个，用于判断原地转身是否安全
    """
    if ts is None:
        ts = time.time()

    center_m, center_valid = _zone_distance(
        depth, cfg, cfg.roi_left, cfg.roi_right, cfg.min_valid_pixels
    )
    left_m, left_valid = _zone_distance(
        depth, cfg, cfg.roi_side_left_l, cfg.roi_side_left_r, cfg.min_zone_pixels
    )
    right_m, right_valid = _zone_distance(
        depth, cfg, cfg.roi_side_right_l, cfg.roi_side_right_r, cfg.min_zone_pixels
    )

    # near_ratio：中心 ROI 内低于 danger_m 的像素占比。
    # 细杆只占一小块；整面墙/门会占很大比例。
    near_ratio = 0.0
    if center_valid:
        h, w = depth.shape
        top = int(h * cfg.roi_top)
        bottom = int(h * cfg.roi_bottom)
        left = int(w * cfg.roi_left)
        right = int(w * cfg.roi_right)
        roi = depth[top:bottom, left:right]
        valid = roi[roi > 0]
        if valid.size:
            near_ratio = float(np.count_nonzero(valid < cfg.danger_m * 1000.0)) / valid.size

    candidates = []
    if center_valid:
        candidates.append(center_m)
    if left_valid:
        candidates.append(left_m)
    if right_valid:
        candidates.append(right_m)
    min_m = min(candidates) if candidates else float("inf")

    return Perception(
        distance_m=center_m,
        valid=center_valid,
        ts=ts,
        left_m=left_m,
        right_m=right_m,
        left_valid=left_valid,
        right_valid=right_valid,
        min_m=min_m,
        near_ratio=near_ratio,
    )


def compute_distance(depth, cfg, ts=None) -> Perception:
    """兼容旧调用：返回同一份多区域剖面。"""
    return compute_profile(depth, cfg, ts)


class DepthCameraProcess(multiprocessing.Process):
    """订阅深度图 → 每帧算一次距离 → 放进 out_queue（只保留最新一条）。"""

    def __init__(self, cfg, out_queue, name="depth_camera"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.out_queue = out_queue
        self._stop_event = multiprocessing.Event()
        self._latest = None          # DDS 回调线程写，run() 读
        self._frames = 0

    def stop(self):
        self._stop_event.set()

    # ------------------------------------------------------------
    def run(self):
        # ① Ctrl+C 会发给整个前台进程组，子进程也收得到。
        # 这里忽略它，退出统一由父进程 stop() + join() 控制，避免打印 traceback。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()

        # ② 原生库只在子进程里导入：即使 DDS 崩溃，也不会影响父进程
        import dds_middleware_python as dds

        try:
            middleware = dds.PyDDSMiddleware(self.cfg.dds_config)
        except Exception as e:
            logging.error("[深度进程] DDS 初始化失败: %s", e)
            return

        qos = {
            "reliability": "best_effort",
            "history_kind": "keep_last",
            "history_depth": 5,
            "durability": "volatile",
        }
        # ③ 订阅；之后每来一帧，DDS 会在它自己的回调线程里调用 _on_depth()
        middleware.subscribeImage(self.cfg.depth_topic, self._on_depth, qos)
        logging.info("[深度进程] 已订阅 %s", self.cfg.depth_topic)

        fps_ts = time.time()
        fps_cnt = 0

        # ④ 主循环：只处理"最新一帧"，处理不过来就丢掉旧的
        while not self._stop_event.is_set():
            depth = self._latest          # 取回调线程刚写进来的帧
            if depth is None:             # 还没收到帧，等 5ms 再看
                time.sleep(0.005)
                continue
            self._latest = None

            # 纯计算：中心/左/右三个区域 -> 去 0 -> 5 分位数 -> 米
            result = compute_profile(depth, self.cfg)
            # 放进队列；父进程的感知线程会取走
            put_latest(self.out_queue, result)

            self._frames += 1
            fps_cnt += 1
            now = time.time()
            if now - fps_ts >= 5.0:
                logging.info("[深度进程] %.1f FPS", fps_cnt / (now - fps_ts))
                fps_ts, fps_cnt = now, 0

        logging.info("[深度进程] 退出（共处理 %d 帧）", self._frames)

    # ------------------------------------------------------------
    def _on_depth(self, msg):
        """
        DDS 回调：只做格式转换，不计算、不打印。

        注意：这个函数运行在 DDS 的回调线程，不是 run() 那个线程。
        它只负责把 msg 解析成 numpy 帧，然后写进 self._latest；
        run() 会在自己的循环里把帧取走并计算。
        """
        try:
            if "16UC1" not in msg.encoding():
                return
            w, h = msg.width(), msg.height()
            step = msg.step() or w * 2

            # msg.data() 返回的是 list[int]（61 万个元素）。
            # bytes(list) 是 C 层批量转换，比 np.asarray(list) 快约 10 倍：
            #   35 ms/帧 → 4 ms/帧，20 FPS 下 CPU 从 ~70% 降到 ~8%
            raw = bytes(msg.data())
            dtype = np.dtype(">u2") if msg.is_bigendian() else np.dtype("<u2")
            frame = np.frombuffer(raw, dtype=dtype).reshape(h, step // 2)[:, :w]
            if msg.is_bigendian():
                frame = frame.astype(np.uint16)     # 转成本机字节序

            self._latest = frame
        except Exception as e:
            logging.warning("[深度进程] 解析失败: %s", e)
