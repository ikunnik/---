"""命令行与 Config 生成（从 main.py 拆出）。"""
import argparse
from dataclasses import replace

from config import MIN_TRAFFIC_VX


def _speed(text: str, max_vx: float) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"不是数字: {text}")
    if not 0.0 < value <= max_vx:
        raise argparse.ArgumentTypeError(f"速度需在 (0, {max_vx}] m/s 之间")
    return value


def parse_args(base_cfg, argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rover X1 深度避障")

    # 运行方式
    parser.add_argument(
        "--robot-ip", default=base_cfg.robot_ip,
        help=f"机器人 SDK 地址（默认 {base_cfg.robot_ip}）",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="不连接机器人，只跑框架和状态机（没硬件时用）",
    )
    parser.add_argument(
        "--vx", type=lambda text: _speed(text, base_cfg.max_forward_vx), default=base_cfg.forward_vx,
        help=f"前进速度 m/s（默认 {base_cfg.forward_vx}，首次真机建议 0.2）",
    )
    parser.add_argument(
        "--no-voice", action="store_true",
        help="关闭机器人语音提示（只影响播报，不影响避障）",
    )
    parser.add_argument(
        "--motion-backend", choices=("sdk", "ros2"),
        default=base_cfg.motion_backend,
        help="运动后端：ros2=/planner/vel_cmd（默认）；sdk=gRPC 回退",
    )

    # RGB / 红绿灯 / 停止线
    parser.add_argument(
        "--rgb", action="store_true",
        help="启用前置 RGB 采集（camera0，DDS CompressedImage）",
    )
    parser.add_argument(
        "--rgb-topic", default=base_cfg.rgb_topic,
        help=f"RGB 话题（默认 {base_cfg.rgb_topic}）",
    )
    parser.add_argument(
        "--traffic-light", action="store_true",
        help="启用红绿灯检测（camera0；等到停止线后才处理）",
    )
    parser.add_argument(
        "--traffic-light-now", action="store_true",
        help="调试：立即开始红绿灯处理，不等停止线",
    )
    parser.add_argument(
        "--traffic-light-profile", default=base_cfg.traffic_light_profile,
        help=f"红绿灯标定 profile（默认 {base_cfg.traffic_light_profile}）",
    )
    parser.add_argument(
        "--stop-line", action="store_true",
        help="显式启用停止线检测（camera2；默认序列已包含）",
    )
    parser.add_argument(
        "--stop-line-profile", default=base_cfg.stop_line_profile,
        help=f"停止线标定 profile（默认 {base_cfg.stop_line_profile}）",
    )

    # 串联任务
    mission = parser.add_mutually_exclusive_group()
    mission.add_argument(
        "--mission", dest="mission", action="store_true", default=None,
        help="串联：红绿灯 → 掉头 → 盲道（默认启用）",
    )
    mission.add_argument(
        "--no-mission", dest="mission", action="store_false",
        help="关闭默认串联，只跑红绿灯/避障（或配合 --no-traffic-sequence 只避障）",
    )
    parser.add_argument(
        "--uturn-check-forward-space", action="store_true",
        help="恢复掉头直行前的 1.9m 净空检查（默认关闭）",
    )
    parser.add_argument(
        "--uturn-ignore-obstacles", action="store_true",
        help="掉头阶段跳过深度净空检查，只保留姿态/角度校验（仅空旷场地）",
    )
    parser.add_argument(
        "--uturn-forward", type=float, default=base_cfg.uturn_forward_m,
        help=(
            "掉头两次左转90°之间的直行距离，"
            f"默认{base_cfg.uturn_forward_m:.2f}米（范围0.1~1.0米）"
        ),
    )

    # 盲道
    parser.add_argument(
        "--mangdao", "--lane-follow", dest="mangdao", action="store_true",
        help="独立盲道模式（别名 --lane-follow）：稳定循环纠偏；关闭停止线和红绿灯流程",
    )
    parser.add_argument(
        "--lane-stop-on-loss", action="store_true",
        help="恢复旧行为：图像中未检出盲道时停车（默认无盲道直行）",
    )
    parser.add_argument(
        "--lane-profile", default=base_cfg.lane_profile,
        help=f"盲道 profile（默认 {base_cfg.lane_profile}）",
    )
    parser.add_argument(
        "--lane-vx", type=float, default=base_cfg.lane_forward_vx,
        help=f"盲道跟踪前进速度（默认 {base_cfg.lane_forward_vx} m/s）",
    )
    parser.add_argument(
        "--lane-trial-seconds", type=float, default=base_cfg.lane_trial_seconds,
        help="盲道最长连续时间，默认0=不限时持续运行；>0 到时停车并锁定",
    )
    parser.add_argument(
        "--lane-steer-sign", type=float, default=base_cfg.lane_steer_sign,
        help=f"转向符号（默认 {base_cfg.lane_steer_sign}；实机方向反了改 +1）",
    )

    # 停止线序列参数
    parser.add_argument(
        "--traffic-sequence", action="store_true",
        help="强制启用停止线序列（默认已启用，保留兼容）",
    )
    parser.add_argument(
        "--no-traffic-sequence", action="store_true",
        help="关闭默认启用的红绿灯/停止线序列（只跑普通避障）",
    )
    parser.add_argument(
        "--traffic-vx", type=float, default=base_cfg.traffic_vx,
        help=(
            f"停止线序列速度（默认 {base_cfg.traffic_vx} m/s，"
            f"最低 {MIN_TRAFFIC_VX}；更低会原地踏步）"
        ),
    )
    parser.add_argument(
        "--stop-line-trigger-row", type=float,
        default=base_cfg.stop_line_trigger_row,
        help=f"黄线触发行号（默认 {base_cfg.stop_line_trigger_row}）",
    )
    return parser.parse_args(argv)


def build_cfg(args, base_cfg):
    """校验参数并生成本次运行的 Config；不启动任何线程/进程。"""
    parser = argparse.ArgumentParser(add_help=False)

    if not MIN_TRAFFIC_VX <= args.traffic_vx <= 0.5:
        parser.error(
            f"--traffic-vx 需在 [{MIN_TRAFFIC_VX}, 0.5] m/s 之间"
            f"（低于 {MIN_TRAFFIC_VX} 步态只原地踏步，实测走不动）"
        )
    if not 0.0 < args.stop_line_trigger_row < 1.0:
        parser.error("--stop-line-trigger-row 需在 (0, 1) 之间")
    if not 0.1 <= args.uturn_forward <= 1.0:
        parser.error("--uturn-forward 必须在 [0.1,1.0] 米之间")
    if not 0.40 <= args.lane_vx <= 0.50:
        parser.error("--lane-vx 需在 [0.40, 0.50] m/s 之间")
    if not 0.0 <= args.lane_trial_seconds <= 3600.0:
        parser.error("--lane-trial-seconds 必须在 [0,3600] 秒；0 表示不限时")
    if args.lane_steer_sign not in (-1.0, 1.0):
        parser.error("--lane-steer-sign 只能是 -1 或 +1")

    mission = base_cfg.mission_enabled if args.mission is None else args.mission
    if mission and (args.mangdao or args.no_traffic_sequence):
        # 默认串联可以被 --mangdao / --no-traffic-sequence 自动关掉；
        # 只有显式 --mission 才报互斥。
        if args.mission:
            parser.error("--mission 与 --mangdao/--no-traffic-sequence 互斥")
        mission = False

    lane_follow_enabled = args.mangdao
    if lane_follow_enabled and (
        args.stop_line or args.traffic_sequence
        or args.traffic_light or args.traffic_light_now
    ):
        parser.error(
            "--mangdao 与 --stop-line/--traffic-sequence/"
            "--traffic-light/--traffic-light-now 互斥"
        )

    traffic_sequence_enabled = (
        (base_cfg.traffic_sequence_enabled or args.traffic_sequence or mission)
        and not args.no_traffic_sequence
        and not lane_follow_enabled
    )
    traffic_light_enabled = (
        not lane_follow_enabled
        and (
            base_cfg.traffic_light_enabled
            or args.traffic_light
            or args.traffic_light_now
            or traffic_sequence_enabled
        )
    )
    stop_line_enabled = (
        not lane_follow_enabled
        and (base_cfg.stop_line_enabled or args.stop_line or traffic_sequence_enabled)
    )

    return replace(
        base_cfg,
        forward_vx=args.vx,
        robot_ip=args.robot_ip,
        motion_backend=args.motion_backend,
        mission_enabled=mission,
        uturn_forward_m=args.uturn_forward,
        uturn_ignore_obstacles=(
            base_cfg.uturn_ignore_obstacles or args.uturn_ignore_obstacles
        ),
        uturn_check_forward_space=(
            base_cfg.uturn_check_forward_space or args.uturn_check_forward_space
        ),
        voice_enabled=base_cfg.voice_enabled and not args.no_voice,
        rgb_enabled=base_cfg.rgb_enabled or args.rgb or traffic_light_enabled,
        rgb_topic=args.rgb_topic,
        traffic_light_enabled=traffic_light_enabled,
        traffic_light_profile=args.traffic_light_profile,
        stop_line_enabled=stop_line_enabled,
        stop_line_profile=args.stop_line_profile,
        traffic_sequence_enabled=traffic_sequence_enabled,
        traffic_vx=args.traffic_vx,
        stop_line_trigger_row=args.stop_line_trigger_row,
        lane_follow_enabled=lane_follow_enabled,
        lane_no_line_straight=(
            base_cfg.lane_no_line_straight and not args.lane_stop_on_loss
        ),
        lane_profile=args.lane_profile,
        lane_forward_vx=args.lane_vx,
        lane_trial_seconds=args.lane_trial_seconds,
        lane_steer_sign=args.lane_steer_sign,
    )

