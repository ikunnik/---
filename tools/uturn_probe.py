"""单独测试三段式掉头：左转90° → 直行 → 左转90°。

两种模式：
    --simulate（默认）  纯离线，不需要相机/机器人；可注入角度误差验证补转
    --real              真机：连接 → 起立 → WALK → 深度相机 → 掉头 → 恢复退出

真机前置：
    - 场地至少 2m×2m，地面平整，人和宠物离开运动范围；
    - 实体急停可触达，操作员全程监护；
    - 机器人已上电，深度相机无遮挡，DDS 环境已 source。

用法：
    python3 tools/uturn_probe.py --simulate
    python3 tools/uturn_probe.py --simulate --yaw-error 15
    python3 tools/uturn_probe.py --real --forward 0.4
"""
import argparse
import logging
import math
import os
import sys
import time
from dataclasses import replace
from pathlib import Path as _Path

sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))

from actions import Actions
from config import CFG
from decisions.engine import DecisionInput
from decisions.uturn import UTurnController
from messages import CommandResult, Perception, Pose


def build_cfg(args):
    if not 0.1 <= args.forward <= 1.0:
        raise SystemExit("--forward 必须在 [0.1, 1.0] 米之间")
    if args.yaw_error < 0:
        raise SystemExit("--yaw-error 不能为负")
    cfg = replace(
        CFG,
        uturn_forward_m=args.forward,
        uturn_ignore_obstacles=(
            CFG.uturn_ignore_obstacles or args.ignore_obstacles
        ),
    )
    return replace(cfg, robot_ip=args.ip)


def _sim_snapshot(now, yaw_deg, result=None, busy=False):
    per = Perception(
        2.0, True, now,
        left_m=2.0, right_m=2.0, left_valid=True, right_valid=True, min_m=2.0,
    )
    return DecisionInput(
        now=now, perception=per, perception_ts=now,
        pose=Pose(math.radians(yaw_deg), now),
        result=result, executor_busy=busy,
    )


def run_sim(args, cfg):
    """离线仿真：假机器人每次转向少转 --yaw-error 度，验证补转/锁停。"""
    ctrl = UTurnController(cfg)
    now, yaw = 100.0, 0.0
    pending = None
    finish_ts = 0.0
    generation = 0
    result = None
    injected = False
    last_state = None

    print(f"[sim] 目标：左90° → 直行 {cfg.uturn_forward_m:.2f}m → 左90°")
    for _ in range(600):
        if pending is not None and now >= finish_ts:
            expected = pending.expected_yaw_delta_deg or 0.0
            if pending.kind == "turn":
                err = args.yaw_error if (args.yaw_error and not injected) else 0.0
                injected = injected or bool(err)
                actual = expected - err if expected >= 0 else expected + err
                yaw += actual
                print(
                    f"[sim] {now:6.1f}s 执行 {pending.cmd.tag}："
                    f"期望 {expected:+.1f}°，实际 {actual:+.1f}°"
                )
            else:
                print(f"[sim] {now:6.1f}s 执行 {pending.cmd.tag}")
            generation += 1
            result = CommandResult(generation, pending.cmd.tag, "ok", ts=now)
            pending = None

        step = ctrl.step(_sim_snapshot(now, yaw, result, busy=pending is not None))
        if step.state != last_state:
            print(f"[sim] {now:6.1f}s 状态 {step.state}")
            last_state = step.state
        if step.log:
            print(f"[sim]   {step.log}")
        if step.request is not None and step.request.kind != "stop":
            pending = step.request
            finish_ts = now + 0.2

        if ctrl.completed:
            print(f"[sim] 完成，最终航向 {yaw:.1f}°")
            return 0
        if ctrl.failed:
            print(f"[sim] 失败：{ctrl.reason}")
            return 1

        result = None
        now += 0.1

    print("[sim] 超时未完成")
    return 2


def _confirm():
    print("\n真机掉头将依次执行：左转90° → 直行 → 左转90°。")
    print("请确认：")
    print("  ① 场地 ≥ 2m×2m，地面平整，无人/宠物；")
    print("  ② 实体急停可触达，操作员全程监护；")
    print("  ③ 深度相机无遮挡，机器人已上电。")
    try:
        return input("确认安全后输入 YES 开始：").strip() == "YES"
    except EOFError:
        return False


def _dds_uri_has_topic_discovery(uri: str) -> bool:
    if not uri:
        return False
    if uri.lstrip().startswith("<"):
        return "EnableTopicDiscoveryEndpoints" in uri
    path = uri[7:] if uri.startswith("file://") else uri
    path = os.path.expanduser(path)
    try:
        with open(path, "r", encoding="utf-8") as f:
            return "EnableTopicDiscoveryEndpoints" in f.read()
    except OSError:
        return False


def _ensure_dds_config():
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if _dds_uri_has_topic_discovery(uri):
        return
    path = os.path.expanduser("~/dobot_quad_sdk-main/cyclonedds.xml")
    if os.path.isfile(path):
        os.environ["CYCLONEDDS_URI"] = "file://" + path
        print(f"[real] 使用 CYCLONEDDS_URI=file://{path}")
    elif not uri:
        print("[real] 警告：未设置 CYCLONEDDS_URI，相机可能收不到数据")


def run_real(args, cfg):
    import multiprocessing

    from comm import Latest, Mailbox
    from robot_factory import build_robot
    from workers.depth_camera import DepthCameraProcess
    from workers.executor import MotionExecutor
    from workers.perception import PerceptionWorker

    if not args.yes and not _confirm():
        print("[real] 已取消，没有向机器人发送运动指令")
        return 2
    _ensure_dds_config()

    actions = Actions(cfg)
    robot = build_robot(False, cfg)
    executor = perc = cam = None
    ctrl = UTurnController(cfg)
    status = 2

    try:
        print("[real] 连接机器人…")
        robot.connect()
        print("[real] 起立…")
        robot.stand()
        print("[real] 切换 WALK…")
        robot.walk()

        depth_queue = multiprocessing.Queue(maxsize=1)
        latest = Latest()
        cam = DepthCameraProcess(cfg, depth_queue)
        perc = PerceptionWorker(cfg, depth_queue, latest)
        cam.start()
        perc.start()

        if cfg.uturn_ignore_obstacles:
            print("[real] 警告：已禁用掉头深度净空检查，仅依赖姿态/角度校验")
        else:
            deadline = time.time() + args.startup_timeout
            ready = False
            while time.time() < deadline:
                per, ts = latest.get()
                now = time.time()
                if per is not None and ts > 0 and (now - ts) < cfg.stale_s and per.valid:
                    ready = True
                    print(f"[real] 深度就绪：{per}")
                    break
                time.sleep(0.05)
            if not ready:
                raise RuntimeError("深度未就绪：确认已起立、镜头无遮挡、DDS 有数据")
        if robot.get_pose(refresh=True) is None:
            raise RuntimeError("读不到姿态（GetRobotState 失败）")

        mailbox = Mailbox()
        executor = MotionExecutor(cfg, robot, mailbox)
        executor.start()

        start = time.time()
        last_state = None
        while True:
            if args.timeout and time.time() - start > args.timeout:
                print(f"[real] 超时 {args.timeout:.0f}s，停车")
                break
            executor.heartbeat()
            per, ts = latest.get()
            raw, _ = executor.result()
            result = raw if raw is not None and raw.status != "running" else None
            refresh = bool(
                result is not None and result.status == "ok" and ctrl.has_pending
            )
            pose = robot.get_pose(refresh=refresh)
            # 姿态 RPC 会晚于上面的取帧；now 必须在读取姿态之后再取，
            # 否则 pose.ts 比 now 新，uturn 会判成“姿态来自未来”而锁停。
            now = time.time()
            step = ctrl.step(DecisionInput(
                now=now, perception=per, perception_ts=ts, pose=pose,
                result=result, executor_busy=executor.busy,
            ))
            if step.state != last_state:
                print(f"[real] {now - start:5.1f}s 状态 {step.state}")
                last_state = step.state
            if step.log:
                print(f"[real]   {step.log}")
            if step.request is not None:
                executor.submit(step.request, interrupt=step.interrupt)

            if ctrl.completed:
                print("[real] 掉头完成")
                status = 0
                break
            if ctrl.failed:
                print(f"[real] 掉头失败：{ctrl.reason}")
                status = 1
                break
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[real] 收到中断，停车")
        status = 2
    except Exception as exc:
        print(f"[real] 失败：{type(exc).__name__}: {exc}")
        status = 1
    finally:
        if executor is not None:
            try:
                executor.submit(actions.stop(), interrupt=True)
            except Exception:
                pass
            time.sleep(0.3)
            executor.stop()
            if executor.is_alive():
                executor.join(2)
        if perc is not None:
            perc.stop()
            if perc.is_alive():
                perc.join(2)
        if cam is not None:
            cam.stop()
            if cam.is_alive():
                cam.join(2)
        try:
            robot.brake()
        finally:
            robot.close()
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description="单独测试三段式掉头")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--simulate", action="store_true", help="离线仿真（默认）")
    mode.add_argument("--real", action="store_true", help="真机掉头（需确认）")
    parser.add_argument("--ignore-obstacles", action="store_true",
                        help="掉头时跳过深度净空检查，只保留姿态/角度校验")
    parser.add_argument("--forward", type=float, default=CFG.uturn_forward_m,
                        help=f"两次左转之间的直行距离，默认 {CFG.uturn_forward_m}m")
    parser.add_argument("--yaw-error", type=float, default=0.0,
                        help="仿真：第一段少转的角度（验证补转/锁停）")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="真机整段超时秒数，默认 60")
    parser.add_argument("--startup-timeout", type=float, default=10.0,
                        help="真机等待深度首帧的秒数，默认 10")
    parser.add_argument("--ip", default=CFG.robot_ip, help=f"机器人地址，默认 {CFG.robot_ip}")
    parser.add_argument("--yes", action="store_true", help="真机模式跳过交互确认")
    args = parser.parse_args(argv)

    cfg = build_cfg(args)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
    )
    if args.real:
        return run_real(args, cfg)
    return run_sim(args, cfg)


if __name__ == "__main__":
    raise SystemExit(main())
