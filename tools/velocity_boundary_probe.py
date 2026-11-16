"""诊断速度序列边界停顿；不依赖摄像头/盲道代码。

仅在平整、空旷、有急停的场地执行。每次总行走命令时间 2.4 秒。
A: 单次 gRPC / 单条 2.4s；B: 单次 gRPC / 内含两条 1.2s；
C: 两次独立 RPC / 每条 1.2s。
每个方案要求重新确认，方便比较在哪种边界出现顿挫。
本脚本使用官方 RobotClient.velocity_sequence，并不更改内置避障设置。
"""
import argparse
import time

from dobot_quad.robot_client import RobotClient


def check(res, what):
    if res is not None and hasattr(res, "success") and not res.success:
        raise RuntimeError(f"{what} 失败: {getattr(res, 'message', '')}")


def invoke(robot, steps):
    t = time.monotonic()
    result = robot.velocity_sequence(
        steps, gait="walk", stand_down_after=False, show_progress=False
    )
    check(result, "速度序列")
    print(f"  速度序列完成，墙钟耗时 {time.monotonic()-t:.3f}s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.5.2:50051")
    parser.add_argument("--vx", type=float, default=0.40)
    args = parser.parse_args()
    if not (0.40 <= args.vx <= 0.50):
        parser.error("测试速度限制为 0.40~0.50m/s")
    print("测试仅用于区分速度指令边界顿挫原因，不提供盲道跟踪/避障控制。")
    print("请在无障碍场地、急停可触达并有人监护时进行。")
    robot = RobotClient(args.ip)
    try:
        check(robot.balance_stand(), "起立")
        check(robot.walk(), "行走模式")
        cases = [
            ("A: 单条 2.4s", [[(args.vx, 0, 0, 2.4)]]),
            ("B: 单 RPC 两条连续 1.2s", [[(args.vx, 0, 0, 1.2), (args.vx, 0, 0, 1.2)]]),
            ("C: 两个独立 RPC 各 1.2s", [[(args.vx, 0, 0, 1.2)], [(args.vx, 0, 0, 1.2)]]),
        ]
        for title, requests in cases:
            resp = input(f"\n{title}，输入 RUN 才开始，其他输入跳过：").strip()
            if resp != "RUN":
                continue
            for index, steps in enumerate(requests, 1):
                print(f"  发起第 {index}/{len(requests)} 条 RPC：{steps}")
                invoke(robot, steps)
            print("本轮结束。请记录是否有顿挫、出现在哪个位置。")
            time.sleep(1)
    finally:
        try:
            robot.velocity_sequence("0,0,0,0.5", gait="walk", stand_down_after=False, show_progress=False)
        except Exception as err:
            print(f"警告：零速发送失败，请使用实体急停：{err}")
        try:
            robot.channel.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
