"""应用入口：装配 → 连接 → 启动 → 就绪 → 监视 → 逆序停止。"""
import logging
import threading

from runtime import assembly, dds, lifecycle, readiness


def run(cfg, args, robot, *, stop_event=None):
    dds.ensure(cfg)
    rt = assembly.build(cfg, args, robot)
    stop = stop_event if stop_event is not None else threading.Event()
    try:
        lifecycle.connect(rt)
        lifecycle.start_workers(rt)
        if readiness.wait(rt) is False:
            raise SystemExit("检测器未就绪，拒绝启动运动序列")

        rt.executor.start()
        rt.supervisor.start()
        rt.decision.start()

        lifecycle.install_signals(stop)
        lifecycle.print_status(rt)
        lifecycle.watch(rt, stop)
    finally:
        lifecycle.shutdown(rt)
        logging.info("[主进程] 已退出")
