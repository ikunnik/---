"""
前置 RGB 相机进程（camera0，DDS CompressedImage）。

为什么单独一个进程：
    DDS 是原生库，和深度相机/语音一样隔离出去；RGB 进程崩了不影响避障。
    而且这里只把压缩字节往外送，mp.Queue 传一条很小的 RgbFrame 就够了。

文件分两半：
    上半部分 decode_compressed() —— 纯函数，输入 CompressedImage，输出 RgbFrame
    下半部分 RgbCameraProcess   —— 进程外壳，负责订阅 DDS 并调用上面的函数

【运行顺序（进程）】
    __init__()             只保存参数，不启动
    start()（main 调用）    操作系统创建子进程，随后子进程执行 run()
    run()（子进程）          ① 忽略 Ctrl+C ② 导入并初始化 DDS ③ 订阅话题
                            ④ 进入 while：取回调写来的帧 -> 放进队列
    stop()（父进程调用）     设置 Event，run() 的 while 下一轮退出

【两个线程交错】
    DDS 回调线程: DDS 收到数据 -> _on_image() 解析 -> 写 self._latest
    run() 线程:   读 self._latest -> put_latest() -> 父进程的 RGB 感知线程
"""
import logging
import multiprocessing
import signal
import time

from comm import put_latest, setup_process_logging
from messages import RgbFrame


def _stamp_to_time(msg) -> float:
    """读 header.stamp()；失败或为 0 时返回 0，由调用方回退到 time.time()。"""
    try:
        stamp = msg.header().stamp()
        return float(stamp.sec()) + float(stamp.nanosec()) / 1e9
    except Exception:
        return 0.0


def decode_compressed(msg, seq: int = 0, now: float | None = None) -> RgbFrame:
    """纯函数：CompressedImage -> RgbFrame；不依赖 DDS，便于离线测试。

    注意：msg.data() 在 DDS Python 绑定里是 list[int]，用 bytes() 批量转换，
    比 np.array(list) 快很多（和深度图的优化同理）。
    """
    data = bytes(msg.data())
    fmt = str(msg.format() or "")
    ts = _stamp_to_time(msg)
    if ts <= 0.0:
        ts = time.time() if now is None else now
    try:
        frame_id = str(msg.header().frame_id())
    except Exception:
        frame_id = ""
    return RgbFrame(
        data=data,
        ts=ts,
        format=fmt,
        seq=seq,
        frame_id=frame_id,
    )


class RgbCameraProcess(multiprocessing.Process):
    """订阅 camera0 的压缩 RGB -> 每帧一个 RgbFrame -> out_queue（只留最新）。"""

    def __init__(self, cfg, out_queue, name="rgb_camera"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.out_queue = out_queue
        self._stop_event = multiprocessing.Event()
        self._latest = None          # DDS 回调线程写，run() 读
        self._seq = 0
        self._frames = 0

    def stop(self):
        self._stop_event.set()

    # ------------------------------------------------------------
    def run(self):
        # ① Ctrl+C 发给整个进程组，子进程忽略，退出统一由父进程 stop() 控制
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()

        # ② 原生库只在子进程里导入
        try:
            import dds_middleware_python as dds
        except Exception as e:
            logging.error("[RGB进程] 无法导入 dds_middleware_python: %s", e)
            return

        try:
            middleware = dds.PyDDSMiddleware(self.cfg.dds_config)
        except Exception as e:
            logging.error("[RGB进程] DDS 初始化失败: %s", e)
            return

        # ③ 订阅；之后每来一帧，DDS 会在自己的回调线程里调用 _on_image()
        try:
            middleware.subscribeCompressedImage(
                self.cfg.rgb_topic, self._on_image
            )
        except Exception as e:
            logging.error("[RGB进程] 订阅 %s 失败: %s", self.cfg.rgb_topic, e)
            return

        logging.info("[RGB进程] 已订阅 %s", self.cfg.rgb_topic)
        fps_ts = time.time()
        fps_cnt = 0
        last_warn = fps_ts

        # ④ 主循环：只处理"最新一帧"，处理不过来就丢掉旧的
        while not self._stop_event.is_set():
            frame = self._latest
            if frame is None:
                now = time.time()
                if now - last_warn >= self.cfg.rgb_log_s:
                    logging.warning(
                        "[RGB进程] 还没有收到帧（检查 DDS 网卡/话题 %s）",
                        self.cfg.rgb_topic,
                    )
                    last_warn = now
                time.sleep(0.01)
                continue
            self._latest = None
            last_warn = time.time()   # 收到帧就重置“无帧”告警计时

            put_latest(self.out_queue, frame)
            self._frames += 1
            fps_cnt += 1
            now = time.time()
            if now - fps_ts >= self.cfg.rgb_log_s:
                logging.info(
                    "[RGB进程] %.1f FPS，%.1f KB/帧",
                    fps_cnt / (now - fps_ts),
                    len(frame.data) / 1024.0,
                )
                fps_ts, fps_cnt = now, 0

        logging.info("[RGB进程] 退出（共处理 %d 帧）", self._frames)

    # ------------------------------------------------------------
    def _on_image(self, msg):
        """DDS 回调：只做格式转换，不计算、不打印。"""
        try:
            self._seq += 1
            frame = decode_compressed(msg, seq=self._seq, now=time.time())
            if frame.data:
                self._latest = frame
        except Exception as e:
            logging.warning("[RGB进程] 解析失败: %s", e)
