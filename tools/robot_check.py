"""
机器人只读体检：不改状态、不发动作。

用法（项目根目录）：
    python3 tools/robot_check.py

回答两个问题：
    1. 机器人在什么状态？（READY = 趴着，WALK = 可以走）
    2. 速度档位 / 避障开关 / 机身姿态是否正常？
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import CFG                       # noqa: E402
from dobot_quad.robot_client import RobotClient  # noqa: E402

# 状态含义（来自 SDK 文档）
STATE_MEANING = {
    "PASSIVE": "完全断电/软趴",
    "READY": "缓慢趴下（安全停止）",
    "STAND_DOWN": "站立准备",
    "BALANCE_STAND": "站立平衡",
    "WALK": "行走",
    "FLYING_TROT": "奔跑",
}


def main():
    print(f"连接 {CFG.robot_ip} ...")
    robot = RobotClient(CFG.robot_ip)
    res = robot.get_state()

    if not res.success:
        print(f"❌ 读取失败: {res.message}")
        return 1

    state = res.current_state.upper()
    print(f"状态        : {state}  ({STATE_MEANING.get(state, '未知')})")
    print(f"机型        : {res.robot_type}")
    print(f"速度档位    : {res.current_speed_ratio}")
    print(f"内置避障    : {'开' if res.obstacle_avoidance_enabled else '关'}")

    body = res.robot_state
    if body.pos_body:
        x, y, z = body.pos_body
        print(f"机身位置    : x={x / 1000:.2f}m y={y / 1000:.2f}m 高度={z / 1000:.2f}m")
    if body.ori_body:
        print(f"机身姿态    : roll={body.ori_body[0]:.2f} "
              f"pitch={body.ori_body[1]:.2f} yaw={body.ori_body[2]:.2f}")

    if state in ("READY", "PASSIVE"):
        print("\n⚠️ 机器人正趴着，前置深度相机离地只有几厘米，"
              "深度图会大量为 0（低于最小量程），这是正常的。")
        print("   要测试避障，需要先让它站起来（balance_stand + walk）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
