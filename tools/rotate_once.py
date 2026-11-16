"""
手动执行一次旋转动作，不启动相机、不启动决策线程。

用法（项目根目录）：
    python3 tools/rotate_once.py --dry-run --left 90
    python3 tools/rotate_once.py --right 45
    python3 tools/rotate_once.py --circle left --turns 2

它只做：连接 -> 起立 -> 切 WALK -> 启动执行器 -> 发一条 Motion -> 等结束 -> 刹车。
不会自动触发，也不会和 main.py 的避障状态机同时运行。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from actions import Actions                                       # noqa: E402
from comm import Mailbox                             # noqa: E402
from config import CFG                               # noqa: E402
from robot_factory import build_robot                  # noqa: E402
from workers.executor import MotionExecutor        # noqa: E402


def _build_action(args, cfg):
    actions = Actions(cfg)
    if args.left is not None:
        return actions.turn(max(0.0, args.left))
    if args.right is not None:
        return actions.turn(-max(0.0, args.right))
    return actions.circle(args.circle, args.turns)


def main():
    parser = argparse.ArgumentParser(description="手动执行一次旋转")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--left", nargs="?", const=90.0, type=float, metavar="ANGLE",
                       help="左转，可省略角度（默认 90）")
    group.add_argument("--right", nargs="?", const=90.0, type=float, metavar="ANGLE",
                       help="右转，可省略角度（默认 90）")
    group.add_argument("--circle", choices=("left", "right"),
                       help="按圈数旋转，配合 --turns")
    parser.add_argument("--turns", type=int, default=1, help="圈数 1~10，默认 1")
    parser.add_argument("--dry-run", action="store_true", help="不连接机器人")
    parser.add_argument("--ip", default=CFG.robot_ip, help="机器人 gRPC 地址")
    parser.add_argument("--timeout", type=float, default=20.0, help="等待动作结束的秒数")
    args = parser.parse_args()

    from dataclasses import replace
    cfg = replace(CFG, robot_ip=args.ip)
    action = _build_action(args, cfg)
    robot = build_robot(args.dry_run, cfg)
    executor = None

    print(f"动作: {action.cmd}")
    try:
        robot.connect()
        if not args.dry_run:
            robot.stand()
            robot.walk()

        executor = MotionExecutor(cfg, robot, Mailbox())
        executor.start()
        executor.submit(action)

        # 先等执行器把指令取走，再等它执行完；避免“还没开始就退出”
        pick_deadline = time.time() + 1.0
        while not executor.busy and time.time() < pick_deadline:
            time.sleep(0.01)
        if not executor.busy:
            print("执行器没有取走指令")
            return 1

        deadline = time.time() + args.timeout
        while executor.busy and time.time() < deadline:
            time.sleep(0.05)
        if executor.busy:
            print(f"等待超时（{args.timeout:.0f}s），将取消动作")
            return 1

        print("动作完成")
        return 0

    except KeyboardInterrupt:
        print("\n已中断")
        return 130
    finally:
        if executor is not None:
            executor.stop()
            if executor.is_alive():
                executor.join(2)
        robot.brake()
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
