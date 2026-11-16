"""
统一运动执行器：各业务模式共用提交、取消、结果和循环看门狗。
起立/切行走模式在启动阶段完成；退出刹车由 RobotLink 同步兜底。

对外接口：
    submit(request, interrupt=False)  提交 ActionRequest，返回 generation
    send(cmd)     底层兼容入口，放一条运动指令进信箱
    interrupt()   立刻打断正在跑的那条流（可以跨线程调用）
    busy          现在有没有指令在跑（决策层靠它决定要不要补发）
    result()      最近一条指令的最终结果（CommandResult）
    stop()        退出前调用：先打断，再关信箱

为什么要"版本号"（generation）？
    决策层刚把"前进"放进信箱，紧接着发现危险，于是 interrupt + 放"停车"。
    但执行器可能已经把"前进"取走了。版本号让这条过期指令自己作废，
    避免"刚停下又被前进指令带跑"。

【运行顺序（线程）】
    __init__()        只保存 robot / mailbox，不启动
    start()（main）    创建线程，随后执行 run()
    run()             while：
                        ① mailbox.get() 等指令（最多 0.2s）
                        ② 检查 generation 是否过期，过期就丢弃并报告 cancelled
                        ③ _run_one()：发起 gRPC 流 -> 阻塞读 progress -> 清 busy
    interrupt()        可以从决策线程随时调用，立刻 cancel() 当前流
    stop()             设置 Event + interrupt + 关闭信箱
"""
import logging
import threading
import time

from actions import stop
from motion.api import MotionCancelled
from comm import Latest, Mailbox
from messages import ActionRequest, CommandResult, Motion, MotionCommand, Velocity, VelocityPlan


class MotionExecutor(threading.Thread):
    def __init__(self, cfg, robot, mailbox: Mailbox, name="executor"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.robot = robot
        self.mailbox = mailbox
        self._stop_event = threading.Event()
        self._busy = threading.Event()
        self._lock = threading.Lock()
        self._call = None
        self._current_tag = None
        self._generation = 0
        self._fail_count = 0
        self._result = Latest()
        self._heartbeat_monotonic = time.monotonic()
        self._watchdog_trip = False
        self._loop_active = False

    def heartbeat(self):
        """决策线程每次循环调用；仅用于循环速度指令的独立看门狗。"""
        with self._lock:
            self._heartbeat_monotonic = time.monotonic()

    def _watchdog(self):
        timeout = float(getattr(self.cfg, "lane_loop_heartbeat_timeout_s", 0.7))
        while not self._stop_event.wait(0.05):
            with self._lock:
                active = self._loop_active
                age = time.monotonic() - self._heartbeat_monotonic
            if active and age > timeout and not self._watchdog_trip:
                self._watchdog_trip = True
                logging.critical("[执行器] 决策心跳中断 %.2fs，取消循环运动并发送零速", age)
                self.interrupt()
                self.send(stop(max(0.3, self.cfg.stop_s), "watchdog_stop"))

    # ---------------- 给决策层用的接口 ----------------
    @property
    def busy(self) -> bool:
        return self._busy.is_set()

    @property
    def current_tag(self):
        """当前正在执行指令的 tag；空闲时为 None。"""
        with self._lock:
            return self._current_tag

    def submit(self, request: ActionRequest, *, interrupt=False) -> int:
        """所有模式的统一提交入口；需要替换动作时先作废旧指令。"""
        if not isinstance(request, ActionRequest):
            raise TypeError("submit 需要 ActionRequest")
        if interrupt:
            self.interrupt()
        return self.send(request.cmd)

    def send(self, cmd: MotionCommand) -> int:
        """cmd 可以是 Velocity（前进/停车），也可以是 Motion（旋转等动作）。"""
        with self._lock:
            generation = self._generation
        if self._watchdog_trip and (
            isinstance(cmd, VelocityPlan)
            or (isinstance(cmd, Velocity) and any(abs(x) > 1e-6 for x in (cmd.vx, cmd.vy, cmd.vyaw)))
            or isinstance(cmd, Motion)
        ):
            logging.critical("[执行器] 看门狗已触发，拒绝后续运动指令；必须重新启动程序")
            cmd = stop(max(0.3, self.cfg.stop_s), "watchdog_latched_stop")
        self.mailbox.put((generation, cmd))
        return generation

    def result(self):
        """返回最近一条 CommandResult；没有时返回 (None, 0.0)。"""
        return self._result.get()

    def interrupt(self):
        with self._lock:
            self._generation += 1          # 让已取走的旧指令作废
            call = self._call
        if call is not None:
            try:
                call.cancel()
            except Exception as e:
                logging.warning("[执行器] cancel 失败: %s", e)

    def stop(self):
        self._stop_event.set()
        self.interrupt()
        self.mailbox.close()

    # ---------------- 线程主体 ----------------
    def run(self):
        logging.info("[执行器] 启动")
        watcher = threading.Thread(target=self._watchdog, name="motion_watchdog", daemon=True)
        watcher.start()
        while not self._stop_event.is_set():
            # ① 等决策线程放指令；超时就再检查一次 stop 标志
            item = self.mailbox.get(timeout=0.2)
            if item is None:
                continue

            # ② 版本号对不上 = 这条指令在排队期间已经过期（例如刚被 interrupt）
            generation, cmd = item
            with self._lock:
                stale = generation != self._generation
            if stale:
                tag = getattr(cmd, "tag", "")
                logging.info("[执行器] 丢弃过期指令 %s %s", tag, cmd)
                self._publish(generation, tag, "cancelled", "stale")
                continue

            # ③ 真正执行这一条
            self._run_one(generation, cmd)

        logging.info("[执行器] 退出")

    def _publish(self, generation, tag, status, message=""):
        self._result.set(
            CommandResult(
                generation=generation,
                tag=tag,
                status=status,
                message=message,
                ts=time.time(),
            )
        )

    def _run_one(self, generation, cmd):
        # busy=True 期间，决策层不会补发新的前进指令
        tag = getattr(cmd, "tag", "")
        with self._lock:
            self._current_tag = tag
            self._loop_active = isinstance(cmd, VelocityPlan) and cmd.loop
        self._busy.set()
        self._publish(generation, tag, "running")
        started = time.time()
        call = None
        saw_final = False
        final_success = False

        try:
            # 发起一次流式调用；call 可以跨线程 cancel()
            call = self.robot.execute_stream(cmd)
            with self._lock:
                self._call = call
                stale = generation != self._generation
            if stale:
                # 刚发起就发现过期（例如决策层已经 interrupt），立刻取消
                call.cancel()

            # 阻塞在这里，直到流结束 / 被 cancel / 出错
            for progress in call:
                if progress.is_final:
                    saw_final = True
                    final_success = bool(progress.success)
                    if not final_success:
                        logging.error("[执行器] 动作失败: %s", progress.message)
                    break

            if final_success:
                self._fail_count = 0
                self._publish(generation, tag, "ok")
                logging.info(
                    "[执行器] 完成 %s %s (%.2fs)",
                    tag, cmd, time.time() - started,
                )
            else:
                # 没有 final 也算失败：不能把"流断了"当成动作成功
                message = "动作流未返回成功结果"
                if saw_final:
                    message = "动作失败"
                self._publish(generation, tag, "failed", message)

        except MotionCancelled:
            self._publish(generation, tag, "cancelled", "interrupted")
            logging.info(
                "[执行器] 已中断 %s %s (%.2fs)",
                tag, cmd, time.time() - started,
            )

        except Exception as e:
            self._publish(generation, tag, "failed", str(e))
            self._on_failure(e)

        finally:
            # 不管成功、失败还是被取消，都要清掉 call 和 busy，
            # 否则决策层会一直以为执行器还在忙，永远不补发
            with self._lock:
                self._call = None
                self._current_tag = None
                self._loop_active = False
            self._busy.clear()

    def _on_failure(self, err):
        """失败后稍微退避一下，避免每 20ms 重试一次把 RPC 打爆。"""
        self._fail_count += 1
        logging.error("[执行器] 第 %d 次失败: %s", self._fail_count, err)
        self._stop_event.wait(min(1.0, 0.1 * self._fail_count))

