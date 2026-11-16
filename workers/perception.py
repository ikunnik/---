"""
感知线程：把子进程放进队列的结果搬进 Latest 白板。

职责只有两件：
    1. 从 multiprocessing.Queue 里取（阻塞等，不空转烧 CPU）
    2. 写到 Latest，并顺手打一条限流日志

注意：它不认识机器人，也不做判断，只负责"搬运"。

【运行顺序（线程）】
    __init__()        只保存队列和 Latest，不启动
    start()（main）    创建线程，随后执行 run()
    run()              while 循环：
                         ① queue.get(timeout=0.1) 阻塞等子进程送结果
                         ② Latest.set(result) 写到白板
                         ③ 需要时打一条限流日志
    stop()             设置 Event，最多 0.1s 后 while 退出
"""
import logging
import queue
import threading
import time

from comm import Latest


class PerceptionWorker(threading.Thread):
    def __init__(self, cfg, in_queue, out: Latest, name="perception"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.in_queue = in_queue
        self.out = out
        self._stop_event = threading.Event()
        self._last_warn = 0.0

    def stop(self):
        self._stop_event.set()

    def run(self):
        logging.info("[感知] 启动")
        while not self._stop_event.is_set():
            try:
                # ① 阻塞等子进程的结果；最多等 0.1s，方便及时响应 stop()
                result = self.in_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            except (EOFError, OSError):
                break

            # ② 写到 Latest 白板，决策线程会来读
            self.out.set(result)
            # ③ 危险/无效才打日志，并且限流
            self._log(result)

        logging.info("[感知] 退出")

    # ------------------------------------------------------------
    def _log(self, result):
        """危险 / 无效才打日志，并且最多每秒一条，避免刷屏。"""
        if result.valid and result.distance_m >= self.cfg.danger_m:
            return

        now = time.time()
        if now - self._last_warn < self.cfg.warn_log_interval_s:
            return
        self._last_warn = now

        if not result.valid:
            logging.warning("[感知] 深度无效（valid=False）")
        else:
            logging.warning("[感知] 危险 %s", result)
