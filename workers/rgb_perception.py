"""
RGB 感知线程：把 RGB 进程的队列搬进 Latest 白板。

职责只有两件：
    1. 从 multiprocessing.Queue 里取（阻塞等，不空转烧 CPU）
    2. 写到 Latest，并顺手打一条限流日志

它不解码、不判断，只负责"搬运"；以后预览/检测都从这个 Latest 取最新帧。

【运行顺序（线程）】
    __init__()        只保存队列和 Latest，不启动
    start()（main）    创建线程，随后执行 run()
    run()             while：
                        ① queue.get(timeout=0.1) 阻塞等子进程送帧
                        ② Latest.set(frame) 写到白板
                        ③ 第一帧和之后每 rgb_log_s 打一条日志
    stop()            设置 Event，最多 0.1s 后 while 退出
"""
import logging
import queue
import threading
import time

from comm import Latest, put_latest
from messages import RgbFrame


class RgbPerceptionWorker(threading.Thread):
    def __init__(self, cfg, in_queue, out: Latest, detect_queue=None,
                 detect_active=None, name="rgb_perception"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.out = out
        self.detect_queue = detect_queue   # 可选：fan-out 给红绿灯检测进程
        self.detect_active = detect_active  # 可选：跨进程布尔（到达停止线）
        self._stop_event = threading.Event()
        self._frames = 0
        self._last_log = 0.0

    def stop(self):
        self._stop_event.set()

    def run(self):
        logging.info("[RGB感知] 启动")
        while not self._stop_event.is_set():
            try:
                # ① 阻塞等子进程的帧；最多等 0.1s，方便及时响应 stop()
                frame = self.in_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            except (EOFError, OSError):
                break

            if not isinstance(frame, RgbFrame):
                continue

            # ② 写到 Latest 白板；预览/检测线程以后从这里读
            self.out.set(frame)
            # ②b 可选 fan-out：只有 detect_active 为 True 时才送检测进程。
            #    未到停止线时只采集、不处理（检测进程完全空闲）。
            active = (
                self.detect_active is None
                or self.detect_active.is_set()
            )
            if self.detect_queue is not None and active:
                put_latest(self.detect_queue, frame)
            self._frames += 1
            now = time.time()
            # ③ 第一帧和之后每 rgb_log_s 打一条，证明队列 -> Latest 是通的
            if self._frames == 1:
                logging.info("[RGB感知] 第一帧: %s", frame)
                self._last_log = now
            elif now - self._last_log >= self.cfg.rgb_log_s:
                logging.info(
                    "[RGB感知] 已转发 %d 帧，最新 %s", self._frames, frame
                )
                self._last_log = now

        logging.info("[RGB感知] 退出（共 %d 帧）", self._frames)
