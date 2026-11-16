# -*- coding: utf-8 -*-
"""
傻瓜式实时 Velocity 更新测试
直接运行本文件即可，不需要填写命令行参数。

测试内容：
  1. 先发送一条 2.4 秒直行速度指令；
  2. 0.8 秒后，在第一条指令尚未结束时，再发送一条带小幅 yaw 的独立速度指令；
  3. 观察机器狗是否停顿、排队，或能在保持步态时改变方向。

注意：这只是 SDK 能力诊断，不是盲道跟踪程序。
"""
import math
import sys
import threading
import time
from pathlib import Path

import grpc
from dobot_quad.robot_client import RobotClient

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_link import RobotLink
from messages import Velocity


def ok_or_raise(result, action):
    if result is None or not bool(getattr(result, "success", False)):
        detail = "" if result is None else getattr(result, "message", "")
        raise RuntimeError(f"{action}失败：{detail}")


class StreamReader(threading.Thread):
    def __init__(self, name, call):
        super().__init__(name=name, daemon=True)
        self.name_text = name
        self.call = call
        self.done = threading.Event()
        self.final_success = None
        self.final_message = ""
        self.error = ""

    def run(self):
        try:
            for item in self.call:
                if getattr(item, "is_final", False):
                    self.final_success = bool(getattr(item, "success", False))
                    self.final_message = str(getattr(item, "message", ""))
                    break
        except grpc.RpcError as exc:
            self.error = f"{exc.code()}: {exc.details()}"
        except Exception as exc:
            self.error = repr(exc)
        finally:
            self.done.set()

    def cancel_safely(self):
        try:
            self.call.cancel()
        except Exception:
            pass


def wrap_angle(rad):
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def main():
    print("\n" + "=" * 58)
    print("       机器狗实时速度更新测试（傻瓜式）")
    print("=" * 58)
    print("这个测试只回答一个问题：")
    print("机器狗正在前进时，再发一条新速度指令，能否不停步地改变方向？")
    print("\n测试动作：")
    print("  第1条：直行 2.4 秒")
    print("  第2条：0.8 秒后发送，保持前进并轻微转向")
    print("\n安全要求：")
    print("  ① 只在封闭、平整、空旷场地测试，前方至少留出 2 米。")
    print("  ② 人和宠物离开机器狗运动范围。")
    print("  ③ 操作员站在实体急停旁，随时准备按下急停。")
    print("  ④ 两条指令可能被固件排队、覆盖或并行执行，结果尚未确认。")
    print("  ⑤ 不要在楼梯、道路、盲道开放通道或人员附近测试。")
    print("-" * 58)

    ip = input("机器狗 IP（直接回车使用 192.168.5.2:50051）：").strip()
    if not ip:
        ip = "192.168.5.2:50051"

    speed_choice = input("前进速度：输入 1=0.40 m/s（推荐），2=0.50 m/s（默认1）：").strip()
    vx = 0.50 if speed_choice == "2" else 0.40

    print("\n选择一个很小的转向方向：")
    print("  1 = 正 yaw（+0.06 rad/s）")
    print("  2 = 负 yaw（-0.06 rad/s）")
    turn_choice = input("输入 1 或 2（默认1）：").strip()
    vyaw = -0.06 if turn_choice == "2" else 0.06

    print("\n即将连接：", ip)
    print(f"前进速度：{vx:.2f} m/s；第二条指令 yaw：{vyaw:+.2f} rad/s")
    confirm = input("确认场地安全、实体急停可触达后，输入 YES 开始：").strip()
    if confirm != "YES":
        print("已取消，没有向机器狗发送运动指令。")
        return 0

    robot = None
    client = None
    first_reader = None
    second_reader = None
    try:
        print("\n[1/5] 正在连接机器人……")
        client = RobotClient(ip)
        robot = RobotLink(ip)
        robot._open_channel()

        print("[2/5] 机器人起立……")
        ok_or_raise(client.balance_stand(), "起立")
        print("[3/5] 切换行走模式……")
        ok_or_raise(client.walk(), "切换行走模式")

        # 读取起始 yaw；读不到也允许继续测试，但会提示。
        start_yaw = None
        pose = robot.get_pose(refresh=True)
        if pose is not None:
            start_yaw = pose.yaw_rad
            print(f"起始航向：{pose.yaw_deg:.2f}°")
        else:
            print("提示：暂时读不到 yaw，仍可通过肉眼观察步态是否中断。")

        print("\n[4/5] 发送第1条直行指令（2.4秒）……")
        first_call = robot._stub.ExecuteSequence(
            robot.build_request(Velocity(vx, 0.0, 0.0, 2.4, "simple_test_straight"))
        )
        first_reader = StreamReader("第一条直行指令", first_call)
        first_reader.start()

        print("机器狗开始直行。请观察它的步态……")
        time.sleep(0.8)

        if first_reader.done.is_set():
            print("第1条指令在0.8秒前已经结束，无法进行在线更新测试。")
            return 2

        print("\n[5/5] 现在发送第2条指令：保持前进，同时轻微转向……")
        second_call = robot._stub.ExecuteSequence(
            robot.build_request(Velocity(vx, 0.0, vyaw, 0.8, "simple_test_yaw_update"))
        )
        second_reader = StreamReader("第二条转向指令", second_call)
        second_reader.start()

        print("观察约2秒：")
        print("  - 有没有停步、顿一下、重新起步？")
        print("  - 有没有在继续走的同时改变方向？")

        yaw_samples = []
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            pose = robot.get_pose(refresh=True)
            if pose is not None:
                yaw_samples.append(pose.yaw_rad)
            time.sleep(0.10)

        # 汇总 RPC 流状态
        print("\n" + "-" * 58)
        print("指令返回情况：")
        for reader in (first_reader, second_reader):
            if reader is None:
                continue
            if reader.error:
                print(f"  {reader.name_text}：通信错误：{reader.error}")
            elif reader.done.is_set():
                print(
                    f"  {reader.name_text}：已结束，success={reader.final_success} "
                    f"{reader.final_message}"
                )
            else:
                print(f"  {reader.name_text}：仍在执行/未收到结束回执")

        if start_yaw is not None and yaw_samples:
            delta = math.degrees(wrap_angle(yaw_samples[-1] - start_yaw))
            print(f"航向变化约：{delta:+.2f}°（仅作参考，需结合实机观察）")
        else:
            print("航向变化：没有足够的姿态数据。")

        print("\n请按你看到的实际情况选择：")
        print("  1 = 没有停步，边走边改变方向")
        print("  2 = 发第2条指令时明显停顿/重新起步")
        print("  3 = 第2条指令排队，第1条结束后才转向")
        print("  4 = 第2条指令报错或被拒绝")
        print("  5 = 行为异常，无法判断")
        observation = input("输入 1/2/3/4/5：").strip()
        labels = {
            "1": "在线更新初步成功：需要再验证持续运行和急停",
            "2": "新指令可能打断了当前步态",
            "3": "新指令可能被排队，不支持实时更新",
            "4": "SDK/固件拒绝并发速度请求",
            "5": "结果不明确，不建议重复盲测",
        }
        print("\n测试结论：", labels.get(observation, "未记录人工观察结果"))
        print("请保存以上输出，并把测试结论告诉 ChatGPT。")
        return 0

    except Exception as exc:
        print(f"\n测试失败：{type(exc).__name__}: {exc}")
        print("如果机器狗仍在运动，立即使用实体急停。")
        return 1

    finally:
        print("\n正在执行清理……")
        # 先取消尚未结束的两条流。
        for reader in (second_reader, first_reader):
            if reader is not None and not reader.done.is_set():
                reader.cancel_safely()
        time.sleep(0.15)

        # 再尝试发送零速。若失败，必须使用实体急停。
        if client is not None:
            try:
                stop_result = client.velocity_sequence(
                    "0,0,0,0.5",
                    stand_down_after=False,
                    show_progress=False,
                )
                if getattr(stop_result, "success", False):
                    print("零速停车请求已返回，请确认机器狗完全停止。")
                else:
                    print("警告：零速停车未确认成功，请立即检查并准备实体急停。")
            except Exception as exc:
                print(f"严重警告：零速停车调用失败：{exc}")
                print("如果机器狗仍在运动，立即使用实体急停。")

        if robot is not None and robot._channel is not None:
            try:
                robot._channel.close()
            except Exception:
                pass
        if client is not None:
            try:
                client.channel.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
