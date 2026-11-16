"""
线程 / 进程之间通信的 3 个小工具。

记住三句话就够了：
    Latest   = 只保留最新一条的"白板"，谁都能看，看不到就等下次
    Mailbox  = 只能放一张纸的"信箱"，放新的会盖掉旧的，取的时候没有就等
    put_latest = 专门给 multiprocessing.Queue 用的"放最新"函数

【谁写、谁读】
    深度进程  --put_latest()-->  mp.Queue  --PerceptionWorker.get()-->  Latest
    PerceptionWorker  --Latest.set()-->  Latest  --DecisionWorker.get()-->  决策
    DecisionWorker  --Mailbox.put()-->  Mailbox  --MotionExecutor.get()-->  执行器
"""
import logging
import queue
import threading
import time


def setup_process_logging():
    """spawn 出来的子进程不会继承父进程的 logging 配置，重新初始化。"""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(processName)s] %(message)s",
        force=True,
    )


class Latest:
    """单值信箱：写的人一直覆盖，读的人拿到的永远是最新一条，不会阻塞。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._value = None
        self._ts = 0.0

    def set(self, value):
        # 写的人：直接覆盖旧值，不等待、不阻塞
        with self._lock:
            self._value = value
            self._ts = time.time()

    def get(self):
        """返回 (值, 写入时间)。从没写过时返回 (None, 0.0)。"""
        # 读的人：立刻拿到当前最新值，不会卡住
        with self._lock:
            return self._value, self._ts


class Mailbox:
    """单槽指令箱：put 覆盖旧值，get 没有就等（最多等 timeout 秒）。"""

    def __init__(self):
        self._cond = threading.Condition()
        self._item = None
        self._closed = False

    def put(self, item):
        # 写的人：放新的会盖掉还没被取走的旧指令
        with self._cond:
            if self._closed:
                return
            self._item = item
            self._cond.notify()

    def get(self, timeout=None):
        """取一条指令；超时或已关闭返回 None。"""
        # 读的人：没指令就在这里等，直到 put() 通知或超时
        with self._cond:
            ok = self._cond.wait_for(
                lambda: self._item is not None or self._closed, timeout
            )
            if not ok or self._closed:
                return None
            item, self._item = self._item, None
            return item

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()


def put_latest(q, item):
    """往 multiprocessing.Queue(maxsize=1) 里放最新值：满了就丢掉旧的那条。"""
    # 顺序：先把旧的取走 -> 再放新的。这样队列里永远只有最新一帧。
    try:
        q.get_nowait()
    except queue.Empty:
        pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass
