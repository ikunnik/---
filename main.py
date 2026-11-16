"""Rover X1 入口：兼容层。

真正的 CLI 在 app/cli.py，装配在 runtime/assembly.py，
启动/停止在 runtime/lifecycle.py，启动握手在 runtime/readiness.py。
"""
import logging
import multiprocessing

from app.cli import build_cfg, parse_args
from config import CFG
from robot_factory import build_robot
from runtime import assembly, dds, lifecycle, readiness
from runtime.app import run


# 兼容旧测试/工具的入口
def _parse_args():
    return parse_args(CFG)


def _build_cfg(args):
    return build_cfg(args, CFG)


def _ensure_dds_config(cfg):
    return dds.ensure(cfg)


def _build_runtime(cfg, args, robot):
    return assembly.build(cfg, args, robot)


_wait_detector_ready = readiness.wait_detector_ready
_shutdown = lifecycle.shutdown


def main():
    # 子进程用 spawn：避免 fork 继承父进程的锁/原生库状态。
    multiprocessing.set_start_method("spawn", force=True)

    args = _parse_args()
    cfg = _build_cfg(args)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(threadName)s] %(message)s",
    )

    robot = build_robot(args.dry_run, cfg)
    run(cfg, args, robot)


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
