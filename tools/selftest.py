"""
不需要机器人的自检：验证多区域剖面、候选方向、状态机、执行结果、航向回正和语音事件。

用法（项目根目录）：
    python3 tools/selftest.py

它会把真实的 worker 跑起来，只是把“机器人”换成 RecordingRobot；
也用纯状态机模拟器覆盖失败动作、超时、动态障碍等难以在真机上复现的路径。
"""
import math
import os
import queue
import subprocess
import sys
import threading
import time
from dataclasses import replace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import actions                                       # noqa: E402
from comm import Latest, Mailbox                     # noqa: E402
from config import CFG, MIN_TRAFFIC_VX              # noqa: E402
from messages import (  # noqa: E402
    CommandResult, LaneState, Motion, Perception, Pose, RgbFrame, StopLineState, Velocity, VelocityPlan,
    TrafficLightState, VoiceCue, VoiceDone,
)
from motion.api import MotionCancelled               # noqa: E402
from robot_link import DryRunRobot, RobotLink       # noqa: E402
from decisions.avoidance import (                          # noqa: E402
    ADVANCE,
    ANNOUNCE,
    BLOCKED,
    CHECK,
    GO,
    HOLD,
    PLAN,
    REALIGN,
    TURN,
    AvoidStateMachine,
    wrap_pi,
)
from workers.decision import DecisionWorker  # noqa: E402
from workers.depth_camera import compute_distance, compute_profile  # noqa: E402
from workers.executor import MotionExecutor        # noqa: E402
from workers.rgb_camera import decode_compressed     # noqa: E402
from workers.rgb_perception import RgbPerceptionWorker  # noqa: E402
from workers.stop_line import (                      # noqa: E402
    StopLineVoter, detect_stop_line, load_profile,
)
from decisions.lane import LaneFollower
from workers.lane import (                            # noqa: E402
    detect_lane, load_profile as load_lane_profile,
)
from workers.traffic_light import (                  # noqa: E402
    TrafficLightVoter, detect_hsv,
)
from decisions.traffic import (                # noqa: E402
    AT_LINE,
    SIGNAL,
    WAIT_STOP_LINE,
    TrafficSequence,
)
from workers.voice import (  # noqa: E402
    BLOCKED_TEXT, WAIT_TEXT, load_wav_pcm, priority_name, publish_pcm,
    render_text, voice_file_for, voice_file_name,
)

FAILS = []
INF = float("inf")


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


def fast_cfg(**overrides):
    base = replace(
        CFG,
        forward_s=0.3,
        stop_s=0.05,
        stop_refresh_s=5.0,
        confirm_s=0.20,
        settle_s=0.05,
        blocked_recheck_s=0.20,
        turn_timeout_s=1.0,
        step_timeout_s=1.0,
        motion_backend="sdk",
        realign_timeout_s=1.0,
        max_avoid_cycles=24,
        min_bypass_advance_m=0.50,
    )
    return replace(base, **overrides) if overrides else base


# =====================================================================
# 2. 多区域剖面
# =====================================================================
def test_profile():
    print("\n[2] compute_profile() 多区域剖面")
    cfg = CFG
    depth = np.full((480, 640), 5000, dtype=np.uint16)
    depth[20:200, 250:350] = 500          # 中心靠左一块 0.5m 障碍

    p = compute_profile(depth, cfg)
    check("中心距离 0.5m", abs(p.distance_m - 0.5) < 0.05, f"{p.distance_m:.2f}")
    check("左侧空间 >= 4m", p.left_m > 4.0, f"{p.left_m:.2f}")
    check("右侧空间 >= 4m", p.right_m > 4.0, f"{p.right_m:.2f}")
    check("min 取中心", abs(p.min_m - 0.5) < 0.05, f"{p.min_m:.2f}")
    check("near_ratio 在合理区间", 0.1 < p.near_ratio < 0.6, f"{p.near_ratio:.2f}")
    check("中心有效", p.valid)
    check("compute_distance 兼容", compute_distance(depth, cfg).distance_m == p.distance_m)

    empty = compute_profile(np.zeros((480, 640), dtype=np.uint16), cfg)
    check("全零帧中心无效", not empty.valid and math.isinf(empty.distance_m))

    left_blank = np.full((480, 640), 5000, dtype=np.uint16)
    left_blank[:, :int(640 * cfg.roi_side_left_r)] = 0
    p3 = compute_profile(left_blank, cfg)
    check("左侧无回波 → left_valid=False", not p3.left_valid)
    check("左侧无回波不影响中心", p3.valid)


# =====================================================================
# 3. 动态步长 + 候选方向
# =====================================================================
def scene(center=1.5, left=1.5, right=1.5, near=0.0,
          center_valid=True, left_valid=True, right_valid=True):
    values = []
    if center_valid:
        values.append(center)
    if left_valid and math.isfinite(left):
        values.append(left)
    if right_valid and math.isfinite(right):
        values.append(right)
    return Perception(
        distance_m=center,
        valid=center_valid,
        ts=0.0,
        left_m=left,
        right_m=right,
        left_valid=left_valid,
        right_valid=right_valid,
        min_m=min(values) if values else INF,
        near_ratio=near,
    )


def clear_scene():
    return scene()


def obstacle_scene(center=0.5):
    # 左边空，右边近；机器人应该选择左转
    return scene(center=center, left=1.5, right=0.45, near=0.2)








# =====================================================================
# 4. actions 纯函数
# =====================================================================
def test_actions():
    print("\n[4] actions 动作构造")

    left = actions.rotate_left(90)
    left_params = dict(left.params)
    check("rotate_left 使用 rotation", left.motion_id == "rotation")
    check("rotate_left direction=0", left_params["direction"] == 0)
    check("rotate_left angle=90", left_params["angle"] == 90.0)

    right = actions.rotate_right(45)
    right_params = dict(right.params)
    check("rotate_right direction=1", right_params["direction"] == 1)
    check("rotate_right angle=45", right_params["angle"] == 45.0)

    circle = actions.circle("left", 3)
    check("circle 3 圈 = 1080°", dict(circle.params)["angle"] == 1080.0)
    check("circle direction=0", dict(circle.params)["direction"] == 0)

    forward = actions.move_forward(0.2)
    forward_params = dict(forward.params)
    check("move_forward 使用 line_walk", forward.motion_id == "line_walk")
    check("move_forward direction=0", forward_params["direction"] == 0)
    check("move_forward distance=0.2", forward_params["distance"] == 0.2)
    check("move_backward direction=1",
          dict(actions.move_backward(0.2).params)["direction"] == 1)
    check("move_left direction=2",
          dict(actions.move_left(0.2).params)["direction"] == 2)
    check("move_right direction=3",
          dict(actions.move_right(0.2).params)["direction"] == 3)

    check("angle 上限 3600", dict(actions.rotate_left(9999).params)["angle"] == 3600.0)
    check("turns 上限 10", dict(actions.circle("right", 99).params)["angle"] == 3600.0)

    try:
        actions.rotate("up", 90)
    except ValueError:
        bad_direction_raised = True
    else:
        bad_direction_raised = False
    check("非法方向报错", bad_direction_raised)

    try:
        actions.move("up", 0.2)
    except ValueError:
        bad_move_raised = True
    else:
        bad_move_raised = False
    check("非法移动方向报错", bad_move_raised)


# =====================================================================
# 5. 纯状态机模拟器：覆盖失败动作、超时、动态障碍、回正
# =====================================================================
class PureSim:
    """不启动线程，直接把 ActionRequest 变成 yaw/结果，喂回状态机。"""

    def __init__(self, cfg, yaw_deg=0.0, scene_=None, wait_for_voice=False):
        self.cfg = cfg
        self.sm = AvoidStateMachine(cfg, wait_for_voice=wait_for_voice)
        self.now = 1000.0
        self.yaw = math.radians(yaw_deg)
        self.x = self.y = 0.0
        self.scene = scene_ if scene_ is not None else clear_scene()
        self.requests = []
        self.result = None
        self.logs = []
        self.voices = []
        self.voice_done = False
        self.sm.reset(self.now)

    def pose(self):
        return Pose(self.yaw, self.now, x=self.x, y=self.y)

    def step(self, dt=0.0):
        self.now += dt
        p = replace(self.scene, ts=self.now)
        res = self.sm.step(
            self.now, p, p.ts, self.pose(), self.result, executor_busy=False,
            voice_done=self.voice_done,
        )
        self.result = None
        self.voice_done = False
        if res.log:
            self.logs.append(res.log)
        if res.voice is not None:
            self.voices.append(res.voice)
        if res.request is not None:
            self.requests.append(res.request)
        return res

    def ack_voice(self):
        """模拟语音进程回执，并立即跑一步。"""
        self.voice_done = True
        return self.step(0.0)

    def resolve(self, status="ok", apply=True):
        req = self.requests[-1]
        if apply and status == "ok" and req.kind in ("turn", "realign"):
            params = dict(req.cmd.params)
            angle = math.radians(float(params["angle"]))
            delta = angle if int(params["direction"]) == 0 else -angle
            self.yaw = wrap_pi(self.yaw + delta)
        if apply and status == "ok" and req.kind == "advance":
            distance = float(dict(req.cmd.params)["distance"])
            self.x += distance * math.cos(self.yaw)
            self.y += distance * math.sin(self.yaw)
        self.result = CommandResult(
            generation=1,
            tag=req.cmd.tag,
            status=status,
            message="" if status == "ok" else "test failure",
            ts=self.now,
        )
        return self.step(0.0)


def run_cycle(sim, max_steps=400, dt=0.1):
    """跑完一次完整避障：障碍 -> 转向 -> 前进 -> 回正 -> GO。"""
    seen_turn = False
    for _ in range(max_steps):
        before = len(sim.requests)
        sim.step(dt)
        if len(sim.requests) > before:
            req = sim.requests[-1]
            if req.kind == "turn":
                seen_turn = True
                sim.scene = clear_scene()
            if req.kind in ("turn", "advance", "realign"):
                sim.resolve()
        if seen_turn and sim.sm.phase == GO:
            return True
    return False


def run_until_blocked(sim, status="ok", apply=True, max_steps=300, dt=0.1):
    for _ in range(max_steps):
        before = len(sim.requests)
        sim.step(dt)
        if len(sim.requests) > before:
            req = sim.requests[-1]
            if req.kind in ("turn", "advance", "realign"):
                sim.resolve(status=status, apply=apply)
        if sim.sm.phase == BLOCKED:
            return True
    return False


def test_state_machine_success():
    print("\n[5] 状态机：启动即障碍 → 转向 → 前进 → 回正")
    cfg = fast_cfg()
    sim = PureSim(cfg, yaw_deg=10.0, scene_=obstacle_scene())

    ok = run_cycle(sim)
    check("完整避障后回到 GO", ok, f"phase={sim.sm.phase}")
    check("有转向动作", any(r.kind == "turn" for r in sim.requests))
    check("有短距离前进", any(r.kind == "advance" for r in sim.requests))
    check("有回正动作", any(r.kind == "realign" for r in sim.requests))
    check("最终航向回到 10°",
          abs(math.degrees(sim.yaw) - 10.0) < 2.0,
          f"{math.degrees(sim.yaw):.1f}°")
    check("回正后 entry_yaw 清空", sim.sm.entry_yaw_rad is None)


def test_startup_near_obstacle_bypass():
    print("\n[6] 启动时 0.9m 障碍（未到危险线）也能绕行")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=obstacle_scene(center=0.9))
    ok = run_cycle(sim)
    check("0.9m 障碍稳定后进入 PLAN/TURN", ok, f"phase={sim.sm.phase}")
    check("0.9m 障碍确实转向", any(r.kind == "turn" for r in sim.requests))
    check("0.9m 绕行后回到 GO", sim.sm.phase == GO)


def test_second_entry_yaw():
    print("\n[7] 第二次避障使用新的 entry_yaw")
    cfg = fast_cfg()
    sim = PureSim(cfg, yaw_deg=10.0, scene_=obstacle_scene())
    check("第一次避障完成", run_cycle(sim))

    # 模拟机器人在两次避障之间被手动转了 15°
    sim.yaw = math.radians(25.0)
    sim.scene = obstacle_scene()
    check("第二次避障完成", run_cycle(sim))
    check("第二次回到自己的 entry_yaw=25°",
          abs(math.degrees(sim.yaw) - 25.0) < 2.0,
          f"{math.degrees(sim.yaw):.1f}°")


def test_two_step_advance():
    print("\n[7b] 转向前方 1.2m：分两步补足到 0.5m 再回正")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=obstacle_scene())
    steps = []

    for _ in range(400):
        before = len(sim.requests)
        sim.step(0.1)
        if len(sim.requests) > before:
            req = sim.requests[-1]
            if req.kind == "turn":
                # 转完后前方只有 1.2m：动态步长 0.3m，需要两步才够 0.5m
                sim.scene = scene(center=1.2, left=1.5, right=1.5, near=0.1)
            if req.kind in ("turn", "advance", "realign"):
                if req.kind == "advance":
                    steps.append(dict(req.cmd.params)["distance"])
                sim.resolve()
        if sim.sm.phase == GO:
            break

    check("确实走了两步", len(steps) == 2, f"{steps}")
    check("每步 0.30m", all(abs(s - 0.30) < 1e-9 for s in steps), f"{steps}")
    check("累计绕出 >= 0.50m", sum(steps) >= cfg.min_bypass_advance_m,
          f"{sum(steps):.2f}m")
    check("补足后回到 GO", sim.sm.phase == GO, f"phase={sim.sm.phase}")
    check("最终航向回到 0°",
          abs(math.degrees(sim.yaw)) < 2.0, f"{math.degrees(sim.yaw):.1f}°")


def test_failed_turn_realigns_first():
    print("\n[7c] 右转后空间不足：先回正，再重新扫描")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=scene(center=0.5, left=0.45, right=1.5, near=0.2))

    turn = None
    for _ in range(100):
        before = len(sim.requests)
        sim.step(0.1)
        if len(sim.requests) > before:
            req = sim.requests[-1]
            if req.kind == "turn":
                turn = req
                # 转过去后中心仍只有 0.5m：两侧空间够，但前方不能走
                sim.scene = scene(center=0.5, left=2.0, right=1.5, near=0.2)
                sim.resolve()
                break

    check("先右转 30°",
          turn is not None and turn.cmd.tag == actions.rotate_right(30).tag,
          f"{turn.cmd if turn else None}")

    nxt = None
    for _ in range(100):
        before = len(sim.requests)
        sim.step(0.1)
        if len(sim.requests) > before:
            nxt = sim.requests[-1]
            break

    check("右转失败后先回正（不是直接换边转向）",
          nxt is not None and nxt.kind == "realign",
          f"{nxt.kind if nxt else None}")
    check("回正目标为 entry_yaw（+30°）",
          nxt is not None and nxt.expected_yaw_delta_deg is not None
          and abs(nxt.expected_yaw_delta_deg - 30.0) < 1e-6,
          f"{nxt.expected_yaw_delta_deg if nxt else None}")


def test_failed_turn_blocks():
    print("\n[8] 转向失败 → 锁定，绝不前进")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=obstacle_scene())
    ok = run_until_blocked(sim, status="failed")
    check("转向失败后 BLOCKED", ok, f"phase={sim.sm.phase}")
    check("转向失败后没有前进",
          not any(r.kind == "advance" for r in sim.requests),
          f"{[r.kind for r in sim.requests]}")


def test_yaw_mismatch_blocks():
    print("\n[9] 转向成功但 yaw 对不上 → 锁定")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=obstacle_scene())
    ok = run_until_blocked(sim, status="ok", apply=False)
    check("yaw 校验失败后 BLOCKED", ok, f"phase={sim.sm.phase}")
    check("yaw 校验失败后没有前进",
          not any(r.kind == "advance" for r in sim.requests))


def test_action_timeout_blocks():
    print("\n[10] 动作超时 → 锁定")
    cfg = fast_cfg(turn_timeout_s=0.05)
    sim = PureSim(cfg, scene_=obstacle_scene())

    for _ in range(100):
        sim.step(0.1)
        if sim.requests and sim.requests[-1].kind == "turn":
            break

    check("已经发出转向动作", bool(sim.requests and sim.requests[-1].kind == "turn"))
    sim.now += 0.1
    res = sim.step(0.0)
    check("超时后 BLOCKED", sim.sm.phase == BLOCKED)
    check("超时后发停车", res.request is not None and res.request.kind == "stop")


def test_action_perception_loss_blocks():
    print("\n[11] 动作执行中感知失效 → 立即锁定")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=obstacle_scene())

    for _ in range(100):
        sim.step(0.1)
        if sim.requests and sim.requests[-1].kind == "turn":
            break
    check("动作已经开始", bool(sim.requests and sim.requests[-1].kind == "turn"))

    sim.scene = Perception(INF, False, 0.0)
    res = sim.step(0.1)
    check("动作中感知失效 → BLOCKED", sim.sm.phase == BLOCKED)
    check("动作中感知失效 → 发停车",
          res.request is not None and res.request.kind == "stop")


def test_dynamic_obstacle_waits():
    print("\n[12] 动态障碍 → 只停车等待")
    cfg = fast_cfg()
    sim = PureSim(cfg, scene_=obstacle_scene(0.5))
    for i in range(60):
        sim.scene = obstacle_scene(0.5 if i % 2 == 0 else 0.7)
        sim.step(0.1)
    check("动态障碍不转向",
          not any(r.kind == "turn" for r in sim.requests))
    check("动态障碍保持 HOLD", sim.sm.phase == HOLD, f"phase={sim.sm.phase}")


def test_too_close_blocks():
    print("\n[13] 离障碍太近、没有转身净空 → 锁定")
    cfg = fast_cfg()
    close = cfg.rotate_clear_m * 0.5
    too_close = scene(center=close, left=close, right=close, near=0.5)
    sim = PureSim(cfg, scene_=too_close)
    ok = run_until_blocked(sim)
    check("太近时 BLOCKED", ok, f"phase={sim.sm.phase}")
    check("太近时不尝试转身",
          not any(r.kind == "turn" for r in sim.requests))


def test_voice_cues():
    print("\n[13b] 语音事件：方向 / 完成 / 具体失败原因")
    cfg = fast_cfg()

    sim = PureSim(cfg, scene_=obstacle_scene())
    ok = run_cycle(sim)
    bypass = [v for v in sim.voices if v.kind == "bypass"]
    done = [v for v in sim.voices if v.kind == "bypass_done"]
    check("完整避障产生绕行提示", ok and len(bypass) == 1, f"{sim.voices}")
    check("左侧空间大时选左", bool(bypass) and bypass[0].side == "left")
    check("绕行角度 45°", bool(bypass) and abs(bypass[0].angle_deg - 45) < 1e-6,
          f"{bypass}")
    check("完成提示只播一次", len(done) == 1, f"{done}")

    sim = PureSim(cfg, scene_=scene(center=0.5, left=0.45, right=1.5, near=0.2))
    ok = run_cycle(sim)
    bypass = [v for v in sim.voices if v.kind == "bypass"]
    check("右侧空间大时选右", ok and bool(bypass) and bypass[0].side == "right",
          f"{bypass}")

    close = cfg.rotate_clear_m * 0.5
    sim = PureSim(cfg, scene_=scene(center=close, left=close, right=close, near=0.5))
    run_until_blocked(sim)
    blocked = [v for v in sim.voices if v.kind == "blocked"]
    check("太近的停车原因 too_close",
          bool(blocked) and blocked[-1].reason == "too_close", f"{blocked}")

    sim = PureSim(cfg, scene_=scene(center=0.5, left=0.45, right=0.45, near=0.1))
    run_until_blocked(sim)
    blocked = [v for v in sim.voices if v.kind == "blocked"]
    check("无绕行空间的停车原因 no_space",
          bool(blocked) and blocked[-1].reason == "no_space", f"{blocked}")

    sim = PureSim(cfg, scene_=obstacle_scene())
    run_until_blocked(sim, status="failed")
    blocked = [v for v in sim.voices if v.kind == "blocked"]
    check("转向失败的停车原因 turn_failed",
          bool(blocked) and blocked[-1].reason == "turn_failed", f"{blocked}")

    sim = PureSim(cfg, scene_=obstacle_scene())
    run_until_blocked(sim, status="ok", apply=False)
    blocked = [v for v in sim.voices if v.kind == "blocked"]
    check("航向不符的停车原因 yaw_mismatch",
          bool(blocked) and blocked[-1].reason == "yaw_mismatch", f"{blocked}")

    sim = PureSim(fast_cfg(turn_timeout_s=0.05), scene_=obstacle_scene())
    for _ in range(100):
        sim.step(0.1)
        if sim.requests and sim.requests[-1].kind == "turn":
            break
    sim.now += 0.1
    sim.step(0.0)
    blocked = [v for v in sim.voices if v.kind == "blocked"]
    check("转向超时的停车原因 turn_timeout",
          bool(blocked) and blocked[-1].reason == "turn_timeout", f"{blocked}")

    sim = PureSim(cfg, scene_=obstacle_scene(0.5))
    for i in range(60):
        sim.scene = obstacle_scene(0.5 if i % 2 == 0 else 0.7)
        sim.step(0.1)
    dynamic = [v for v in sim.voices if v.kind == "dynamic"]
    check("动态障碍只播一次等待提示", len(dynamic) == 1, f"{dynamic}")

    check("左侧 30° 文案",
          render_text(VoiceCue("bypass", side="left", angle_deg=30))
          == "前方有障碍，请从左侧绕过。")
    check("左侧 60° 文案",
          render_text(VoiceCue("bypass", side="left", angle_deg=60))
          == "前方有障碍，请向左前方绕过。")
    check("完成文案",
          render_text(VoiceCue("bypass_done")) == "已绕过障碍，继续前进。")
    check("具体失败文案",
          render_text(VoiceCue("blocked", reason="yaw_mismatch"))
          == "转向角度不符合要求，已停下。")
    check("停止线丢失锁存文案",
          render_text(VoiceCue("wait", reason="stop_line_lost_wait"))
          == "停止线丢失，已停下，请寻求帮助。")
    check("停止线无进展锁存文案",
          render_text(VoiceCue("wait", reason="stop_line_stall_wait"))
          == "停止线无进展，已停下，请寻求帮助。")
    check("锁存停车用警告优先级",
          priority_name(VoiceCue("wait", reason="stop_line_lost_wait"))
          == "kWarning")
    check("停车原因用警告优先级",
          priority_name(VoiceCue("blocked", reason="too_close")) == "kWarning")
    check("绕行方向用提示优先级",
          priority_name(VoiceCue("bypass", side="left", angle_deg=30)) == "kPROMPT")
    check("识别盲道播报文案",
          render_text(VoiceCue("lane")) == "继续前进。")
    check("识别盲道播报文件",
          voice_file_name(VoiceCue("lane")) == "lane.wav")


def test_voice_pack():
    print("\n[13d] 女声语音包：文件名映射、WAV 采样率、全部短语存在")
    cfg = fast_cfg()

    cue = VoiceCue("bypass", side="left", angle_deg=30)
    path = voice_file_for(cue, cfg)
    check("左 30° 映射到 bypass_left.wav",
          path is not None and path.name == "bypass_left.wav",
          f"{path.name if path else None}")
    pcm = load_wav_pcm(path) if path is not None else b""
    duration = len(pcm) / 2 / 24000
    check("自然女声 WAV 可解析且有时长", duration > 1.0, f"{duration:.2f}s")
    check("60° 使用前方绕行文件",
          voice_file_name(VoiceCue("bypass", side="right", angle_deg=60))
          == "bypass_right_front.wav")
    check("失败原因映射到独立文件",
          voice_file_name(VoiceCue("blocked", reason="yaw_mismatch"))
          == "blocked_yaw_mismatch.wav")
    check("红灯映射到 traffic_red.wav",
          voice_file_name(VoiceCue("traffic_red")) == "traffic_red.wav")
    check("绿灯映射到 traffic_green.wav",
          voice_file_name(VoiceCue("traffic_green")) == "traffic_green.wav")
    check("黄灯映射到 traffic_yellow.wav",
          voice_file_name(VoiceCue("traffic_yellow"))
          == "traffic_yellow.wav")

    cues = [
        VoiceCue("bypass", side="left", angle_deg=30),
        VoiceCue("bypass", side="left", angle_deg=60),
        VoiceCue("bypass", side="right", angle_deg=30),
        VoiceCue("bypass", side="right", angle_deg=60),
        VoiceCue("bypass_done"),
        VoiceCue("dynamic"),
        VoiceCue("traffic_red"),
        VoiceCue("traffic_yellow"),
        VoiceCue("traffic_green"),
    ]
    cues.extend(VoiceCue("blocked", reason=r) for r in BLOCKED_TEXT)
    cues.extend(VoiceCue("wait", reason=r) for r in WAIT_TEXT)
    missing = [
        voice_file_name(c) for c in cues
        if voice_file_for(c, cfg) is None or not voice_file_for(c, cfg).is_file()
    ]
    check(f"{len(cues)} 条女声短句全部存在", not missing, f"missing={missing}")


def test_voice_gate():
    print("\n[13e] 语音门控：播报期间停车，播完再转向/恢复前进")
    cfg = fast_cfg(voice_wait_timeout_s=10.0)

    sim = PureSim(cfg, scene_=obstacle_scene(), wait_for_voice=True)
    res = None
    for _ in range(150):
        res = sim.step(0.1)
        if sim.sm.voice_pending and any(v.kind == "bypass" for v in sim.voices):
            break

    check("绕行提示进入 ANNOUNCE", res is not None and res.state == ANNOUNCE,
          f"{res.state if res else None}")
    check("播报期间不产生转向", not any(r.kind == "turn" for r in sim.requests),
          f"{[r.kind for r in sim.requests]}")

    for _ in range(8):
        sim.step(0.1)
    check("没有回执时持续停车", not any(r.kind == "turn" for r in sim.requests),
          f"{[r.kind for r in sim.requests]}")
    check("播报期间有停车心跳", any(r.kind == "stop" for r in sim.requests),
          f"{[r.kind for r in sim.requests]}")

    sim.ack_voice()
    check("收到回执后才转向",
          bool(sim.requests) and sim.requests[-1].kind == "turn",
          f"{sim.requests[-1].kind if sim.requests else None}")

    # 转向完成、前方清空后，完成提示也必须先停车播完，再恢复 GO。
    sim.scene = clear_scene()
    sim.resolve()
    saw_done = False
    for _ in range(400):
        before = len(sim.requests)
        sim.step(0.1)
        if sim.sm.voice_pending and any(v.kind == "bypass_done" for v in sim.voices):
            saw_done = True
            break
        if sim.sm.voice_pending:
            sim.ack_voice()
        if len(sim.requests) > before:
            req = sim.requests[-1]
            if req.kind in ("turn", "advance", "realign"):
                sim.resolve()

    check("完成提示进入停车播报", saw_done and sim.sm.voice_pending,
          f"phase={sim.sm.phase}, voices={sim.voices[-2:]}")
    check("完成提示播报时未进入 GO", sim.sm.phase != GO, f"{sim.sm.phase}")
    last_kind = sim.requests[-1].kind if sim.requests else None
    check("完成提示播报时没有前进请求", last_kind != "forward", f"{last_kind}")

    sim.ack_voice()
    check("完成提示播完后恢复 GO", sim.sm.phase == GO, f"{sim.sm.phase}")
    next_res = sim.step(0.1)
    check("恢复 GO 后才发前进", next_res.request is not None
          and next_res.request.kind == "forward",
          f"{next_res.request.kind if next_res.request else None}")

    # 语音进程异常：超时后不能永久停死，但超时前仍必须停车。
    timeout_cfg = fast_cfg(voice_wait_timeout_s=0.2)
    timeout_sim = PureSim(timeout_cfg, scene_=obstacle_scene(), wait_for_voice=True)
    for _ in range(150):
        timeout_sim.step(0.1)
        if timeout_sim.sm.voice_pending:
            break
    check("语音超时测试进入等待", timeout_sim.sm.voice_pending)
    for _ in range(4):
        timeout_sim.step(0.1)
    check("语音超时后继续执行",
          any(r.kind == "turn" for r in timeout_sim.requests),
          f"{[r.kind for r in timeout_sim.requests]}")


class _FakeTime:
    def sec(self, value):
        self._sec = value

    def nanosec(self, value):
        self._nanosec = value


class _FakeHeader:
    def stamp(self, value):
        self._stamp = value

    def frame_id(self, value):
        self._frame_id = value


class _FakeVoiceCmd:
    def __init__(self):
        self._data = []
        self._flag = False
        self._type = ""
        self._task_id = ""
        self._priority = None

    def header(self, value):
        self._header = value

    def priority(self, value):
        self._priority = value

    def task_id(self, value=None):
        if value is not None:
            self._task_id = value
        return self._task_id

    def type(self, value=None):
        if value is not None:
            self._type = value
        return self._type

    def path(self, value):
        self._path = value

    def data(self, value=None):
        if value is not None:
            self._data = value
        return self._data

    def flag(self, value=None):
        if value is not None:
            self._flag = value
        return self._flag


class _FakeDDS:
    Time = _FakeTime
    Header = _FakeHeader
    VoiceCmd = _FakeVoiceCmd


class _FakeMiddleware:
    def __init__(self):
        self.published = []

    def publishVoiceCmd(self, cmd):
        self.published.append(cmd)


def test_voice_publish():
    print("\n[13c] 语音 DDS 分块 + 结束包")
    cfg = replace(CFG, voice_chunk_ms=10)
    pcm = b"\x00" * 480                       # 10ms，24kHz/16bit/mono
    middleware = _FakeMiddleware()

    publish_pcm(middleware, _FakeDDS, pcm, cfg, 2, "test_task")
    sent = middleware.published
    check("一包音频 + 一包结束", len(sent) == 2, f"{len(sent)}")
    check("音频包长度正确", bool(sent) and len(sent[0].data()) == 480,
          f"{len(sent[0].data()) if sent else None}")
    check("音频包不是结束包", bool(sent) and sent[0].flag() is False)
    check("音频包是 streaming", bool(sent) and sent[0].type() == "streaming")
    check("task_id 一致", bool(sent) and sent[0].task_id() == "test_task")
    check("结束包为空且 flag=True",
          bool(sent) and sent[-1].data() == [] and sent[-1].flag() is True)


# =====================================================================
# 6. 执行器结果 / 取消
# =====================================================================
class _Progress:
    def __init__(self, success=True, message=""):
        self.is_final = True
        self.success = success
        self.message = message


class _Cancelled(MotionCancelled):
    pass


class RecordingCall:
    def __init__(self, duration, on_done=None, on_cancel=None, fail=False):
        self._duration = duration
        self._cancelled = threading.Event()
        self._on_done = on_done
        self._on_cancel = on_cancel
        self._fail = fail

    def cancel(self):
        if not self._cancelled.is_set():
            self._cancelled.set()
            if self._on_cancel is not None:
                self._on_cancel()

    def __iter__(self):
        deadline = time.time() + self._duration
        while time.time() < deadline and not self._cancelled.is_set():
            time.sleep(0.005)
        if self._cancelled.is_set():
            raise _Cancelled()
        if not self._fail and self._on_done is not None:
            self._on_done()
        yield _Progress(
            success=not self._fail,
            message="" if not self._fail else "test failure",
        )


class RecordingRobot(DryRunRobot):
    def __init__(self, fail_rotations=False, cap_duration=0.2, fail_tags=None):
        super().__init__()
        self.commands = []      # [(时间, Velocity 或 Motion)]
        self.cancelled = 0
        self.rotate_count = 0
        self.cmd_lock = threading.Lock()
        self.fail_rotations = fail_rotations
        self.fail_tags = set(fail_tags or ())
        self.cap_duration = cap_duration

    def execute_stream(self, cmd):
        with self.cmd_lock:
            self.commands.append((time.time(), cmd))

        is_rotation = isinstance(cmd, Motion) and cmd.motion_id == "rotation"
        fail = bool(
            (self.fail_rotations and is_rotation)
            or (cmd.tag in self.fail_tags)
        )

        on_done = None
        if is_rotation:
            params = dict(cmd.params)
            direction = int(params["direction"])
            angle = float(params["angle"])

            def on_done():
                with self._lock:
                    delta = angle if direction == 0 else -angle
                    self._yaw = wrap_pi(self._yaw + math.radians(delta))
                with self.cmd_lock:
                    self.rotate_count += 1

        elif isinstance(cmd, Motion) and cmd.motion_id == "line_walk":
            distance = float(dict(cmd.params)["distance"])

            def on_done():
                with self._lock:
                    self._x += distance * math.cos(self._yaw)
                    self._y += distance * math.sin(self._yaw)

        duration = min(getattr(cmd, "duration", 1.0), self.cap_duration)
        return RecordingCall(
            duration,
            on_done=on_done,
            on_cancel=self._count_cancel,
            fail=fail,
        )

    def _count_cancel(self):
        with self.cmd_lock:
            self.cancelled += 1

    def count(self, tag, since=0.0, until=INF):
        with self.cmd_lock:
            return sum(1 for t, v in self.commands
                       if v.tag == tag and since <= t < until)


def _wait_result(executor, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        result, _ = executor.result()
        if result is not None and result.status != "running":
            return result
        time.sleep(0.01)
    return None


def test_executor_results():
    print("\n[14] 执行器结果：失败 / 取消")
    robot = RecordingRobot(fail_rotations=True, cap_duration=0.05)
    executor = MotionExecutor(fast_cfg(), robot, Mailbox())
    executor.start()
    try:
        executor.send(actions.rotate_left(30))
        result = _wait_result(executor)
        check("失败动作结果 status=failed",
              result is not None and result.status == "failed",
              f"{result.status if result else None}")
        check("失败动作结果带 tag",
              result is not None and result.tag == actions.rotate_left(30).tag)
    finally:
        executor.stop()
        executor.join(2)

    robot2 = RecordingRobot(cap_duration=1.0)
    executor2 = MotionExecutor(fast_cfg(), robot2, Mailbox())
    executor2.start()
    try:
        action = actions.rotate_left(90)
        executor2.send(action)

        deadline = time.time() + 1.0
        while not executor2.busy and time.time() < deadline:
            time.sleep(0.005)

        check("旋转指令到达机器人", robot2.count(action.tag) >= 1)
        check("旋转期间 busy", executor2.busy)

        executor2.interrupt()
        deadline = time.time() + 1.0
        while executor2.busy and time.time() < deadline:
            time.sleep(0.005)

        result = _wait_result(executor2)
        check("取消被记录", robot2.cancelled >= 1, f"{robot2.cancelled}")
        check("取消后 busy 清零", not executor2.busy)
        check("取消结果 status=cancelled",
              result is not None and result.status == "cancelled",
              f"{result.status if result else None}")
    finally:
        executor2.stop()
        executor2.join(2)


# =====================================================================
# 7. 决策 + 执行器联调
# =====================================================================
def _start_stack(cfg, robot, perception, feeder, feeder_stop, voice_queue=None,
                 voice_done_queue=None, stop_line=None, traffic=None,
                 traffic_active=None, traffic_sequence=False):
    executor = MotionExecutor(cfg, robot, Mailbox())
    decision = DecisionWorker(
        cfg, perception, executor, robot,
        voice_queue=voice_queue, voice_done_queue=voice_done_queue,
        stop_line=stop_line, traffic=traffic,
        traffic_active=traffic_active,
        traffic_sequence=traffic_sequence,
    )

    feeder_thread = threading.Thread(target=feeder, daemon=True)
    feeder_thread.start()
    executor.start()
    decision.start()
    return executor, decision, feeder_thread


def _stop_stack(executor, decision, feeder_thread, feeder_stop):
    feeder_stop.set()
    decision.stop()
    decision.join(2)
    executor.stop()
    executor.join(3)
    feeder_thread.join(1)


def test_workers():
    print("\n[15] 决策 + 执行器（前进补发 / 危险停车 / 恢复）")
    cfg = fast_cfg(confirm_s=0.15, settle_s=0.10, blocked_recheck_s=0.20)
    perception = Latest()
    robot = RecordingRobot(cap_duration=0.2)

    target = {"p": Perception(1.5, True, time.time())}
    feeder_stop = threading.Event()

    def feeder():
        while not feeder_stop.is_set():
            target["p"] = replace(target["p"], ts=time.time())
            perception.set(target["p"])
            time.sleep(0.02)

    t_start = time.time()
    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop
    )

    try:
        t0 = time.time()
        time.sleep(1.2)                        # 阶段 1：安全，应该持续前进
        t1 = time.time()
        target["p"] = Perception(0.3, True, time.time())
        time.sleep(0.7)                        # 阶段 2：危险，应该停车
        t2 = time.time()
        target["p"] = Perception(1.5, True, time.time())
        time.sleep(0.9)                        # 阶段 3：恢复，应该重新前进
        t3 = time.time()
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    startup_stops = robot.count("stop", t_start, t_start + 0.5)
    fwd1 = robot.count("forward", t0, t1)
    stop2 = robot.count("stop", t1, t2)
    fwd2_early = robot.count("forward", t1 + 0.2, t2)
    fwd3 = robot.count("forward", t2, t3)

    check("启动后立即发送停车", startup_stops >= 1, f"{startup_stops}")
    check("阶段1 前进指令自动补发 >= 2 条", fwd1 >= 2, f"{fwd1}")
    check("阶段2 收到停车指令", stop2 >= 1, f"{stop2}")
    check("阶段2 周期补发停车", stop2 >= 2, f"{stop2}")
    check("阶段2 打断正在执行的流", robot.cancelled >= 1, f"{robot.cancelled}")
    check("阶段2 停车后不再补发前进", fwd2_early == 0, f"{fwd2_early}")
    check("阶段3 恢复前进", fwd3 >= 1, f"{fwd3}")


def test_avoid_sequence_integration():
    print("\n[16] 联调：启动即障碍 → 左转 → 前进 → 右转回正")
    cfg = fast_cfg(confirm_s=0.15, settle_s=0.10, stop_refresh_s=5.0)
    perception = Latest()
    robot = RecordingRobot(cap_duration=0.2)

    feeder_stop = threading.Event()

    def feeder():
        while not feeder_stop.is_set():
            with robot.cmd_lock:
                n = robot.rotate_count
            p = obstacle_scene() if n == 0 else clear_scene()
            p = replace(p, ts=time.time())
            perception.set(p)
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop
    )

    left_tag = actions.rotate_left(45).tag
    right_tag = actions.rotate_right(45).tag
    step_tag = actions.move_forward(0.8).tag

    def wait_count(tag, n=1, timeout=4.0):
        deadline = time.time() + timeout
        while robot.count(tag) < n and time.time() < deadline:
            time.sleep(0.02)
        return robot.count(tag) >= n

    try:
        got_left = wait_count(left_tag)
        got_step = wait_count(step_tag)
        got_right = wait_count(right_tag)
        t_after = time.time()
        time.sleep(2.0)
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    check("启动即障碍 → 左转 30°", got_left, f"{robot.count(left_tag)}")
    check("转向后整段前进 0.8m", got_step, f"{robot.count(step_tag)}")
    check("绕出后右转 30° 回正", got_right, f"{robot.count(right_tag)}")
    check("回正后继续前进", robot.count("forward", since=t_after) >= 1,
          f"{robot.count('forward', since=t_after)}")
    check("最终航向回到 0°",
          abs(math.degrees(robot.get_pose().yaw_rad)) < 2.0,
          f"{math.degrees(robot.get_pose().yaw_rad):.1f}°")


def test_voice_routing_integration():
    print("\n[16b] 联调：状态机语音事件进入语音队列")
    cfg = fast_cfg(confirm_s=0.15, settle_s=0.10, stop_refresh_s=5.0)
    perception = Latest()
    robot = RecordingRobot(cap_duration=0.2)
    voice_queue = queue.Queue(maxsize=1)
    feeder_stop = threading.Event()

    def feeder():
        while not feeder_stop.is_set():
            with robot.cmd_lock:
                n = robot.rotate_count
            p = obstacle_scene() if n == 0 else clear_scene()
            perception.set(replace(p, ts=time.time()))
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop, voice_queue=voice_queue
    )

    left_tag = actions.rotate_left(45).tag
    right_tag = actions.rotate_right(45).tag
    step_tag = actions.move_forward(0.8).tag

    def wait_count(tag, n=1, timeout=4.0):
        deadline = time.time() + timeout
        while robot.count(tag) < n and time.time() < deadline:
            time.sleep(0.02)
        return robot.count(tag) >= n

    cues = []
    try:
        if wait_count(left_tag):
            try:
                cues.append(voice_queue.get(timeout=2.0))
            except queue.Empty:
                pass
        wait_count(step_tag)
        wait_count(right_tag)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if any(c.kind == "bypass_done" for c in cues):
                break
            try:
                cues.append(voice_queue.get(timeout=0.2))
            except queue.Empty:
                pass
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    check("语音队列收到绕行提示",
          any(c.kind == "bypass" and c.side == "left" for c in cues), f"{cues}")
    check("语音队列收到完成提示",
          any(c.kind == "bypass_done" for c in cues), f"{cues}")


def test_voice_ack_integration():
    print("\n[16c] 联调：收到语音回执前不转向，回执后才执行")
    cfg = fast_cfg(confirm_s=0.15, settle_s=0.10, stop_refresh_s=5.0)
    perception = Latest()
    robot = RecordingRobot(cap_duration=0.2)
    voice_queue = queue.Queue(maxsize=1)
    voice_done_queue = queue.Queue()
    feeder_stop = threading.Event()

    def feeder():
        while not feeder_stop.is_set():
            perception.set(replace(obstacle_scene(), ts=time.time()))
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop,
        voice_queue=voice_queue, voice_done_queue=voice_done_queue,
    )

    left_tag = actions.rotate_left(45).tag
    cue = None
    try:
        deadline = time.time() + 4.0
        while time.time() < deadline:
            try:
                cue = voice_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if cue.kind == "bypass":
                break
        time.sleep(0.8)
        blocked_before_ack = robot.count(left_tag)
        if cue is not None:
            voice_done_queue.put(VoiceDone(seq=cue.seq, ok=True))
        deadline = time.time() + 3.0
        while robot.count(left_tag) == 0 and time.time() < deadline:
            time.sleep(0.02)
        turned_after_ack = robot.count(left_tag)
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    check("语音队列收到带序号的绕行提示",
          cue is not None and cue.kind == "bypass" and cue.seq > 0, f"{cue}")
    check("回执前保持停车、不转向", blocked_before_ack == 0,
          f"{blocked_before_ack}")
    check("回执后才转向", turned_after_ack >= 1, f"{turned_after_ack}")


def test_failed_turn_integration():
    print("\n[17] 联调：真实执行器返回失败 → 不前进")
    cfg = fast_cfg(confirm_s=0.15, settle_s=0.10)
    perception = Latest()
    robot = RecordingRobot(fail_rotations=True, cap_duration=0.05)
    feeder_stop = threading.Event()

    def feeder():
        while not feeder_stop.is_set():
            perception.set(replace(obstacle_scene(), ts=time.time()))
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop
    )

    left_tag = actions.rotate_left(45).tag
    right_tag = actions.rotate_right(45).tag
    deadline = time.time() + 4.0
    while (robot.count(left_tag) + robot.count(right_tag) == 0
           and time.time() < deadline):
        time.sleep(0.02)

    time.sleep(0.4)

    try:
        phase = decision.engine.avoidance.phase
        advanced = robot.count(actions.move_forward(1.0).tag)
        forward = robot.count("forward")
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    check("失败转向被发出", robot.count(left_tag) + robot.count(right_tag) >= 1)
    check("失败转向后 BLOCKED", phase == BLOCKED, f"phase={phase}")
    check("失败转向后没有绕障前进", advanced == 0, f"{advanced}")
    check("启动即障碍时没有普通前进", forward == 0, f"{forward}")


def test_avoid_cancelled():
    print("\n[18] 等待稳定期间障碍消失 → 不转向")
    cfg = fast_cfg(confirm_s=0.20, settle_s=0.10, stop_refresh_s=5.0)
    perception = Latest()
    robot = RecordingRobot(cap_duration=0.2)
    feeder_stop = threading.Event()

    target = {"p": Perception(1.5, True, time.time())}

    def feeder():
        while not feeder_stop.is_set():
            target["p"] = replace(target["p"], ts=time.time())
            perception.set(target["p"])
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop
    )

    try:
        time.sleep(0.4)
        target["p"] = Perception(0.5, True, time.time())
        time.sleep(0.10)                  # 小于 confirm_s=0.20
        target["p"] = Perception(1.5, True, time.time())
        time.sleep(0.8)
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    left_tag = actions.rotate_left(45).tag
    right_tag = actions.rotate_right(45).tag
    check("障碍消失后不左转", robot.count(left_tag) == 0,
          f"{robot.count(left_tag)}")
    check("障碍消失后不右转", robot.count(right_tag) == 0,
          f"{robot.count(right_tag)}")


def test_rgb_capture():
    print("\n[19] RGB 采集/通信：CompressedImage -> RgbFrame -> Latest")
    payload = b"\xff\xd8\xff\xe0" + b"0" * 40

    class _Stamp:
        def __init__(self, sec, nanosec):
            self._sec, self._nanosec = sec, nanosec

        def sec(self):
            return self._sec

        def nanosec(self):
            return self._nanosec

    class _Header:
        def __init__(self, sec, nanosec, frame_id):
            self._sec, self._nanosec = sec, nanosec
            self._frame_id = frame_id

        def stamp(self):
            return _Stamp(self._sec, self._nanosec)

        def frame_id(self):
            return self._frame_id

    class _Compressed:
        def __init__(self, data, fmt="jpeg", sec=100, nanosec=500_000_000,
                     frame_id="camera0_optical_frame"):
            self._data, self._fmt = data, fmt
            self._sec, self._nanosec = sec, nanosec
            self._frame_id = frame_id

        def header(self):
            return _Header(self._sec, self._nanosec, self._frame_id)

        def format(self):
            return self._fmt

        def data(self):
            return list(self._data)      # 模拟 DDS 绑定返回的 list[int]

    msg = _Compressed(payload, "jpeg", 100, 500_000_000)
    frame = decode_compressed(msg, seq=7)
    check("list[int] -> bytes", frame.data == payload, f"{type(frame.data)}")
    check("format/frame_id/seq 透传",
          frame.format == "jpeg"
          and frame.frame_id == "camera0_optical_frame"
          and frame.seq == 7,
          f"{frame}")
    check("时间戳来自 header", abs(frame.ts - 100.5) < 1e-6, f"{frame.ts:.3f}")

    zero = _Compressed(payload, "jpeg", 0, 0)
    fallback = decode_compressed(zero, now=123.0)
    check("header 时间为 0 时回退 now", abs(fallback.ts - 123.0) < 1e-6,
          f"{fallback.ts:.3f}")

    q = queue.Queue()
    latest = Latest()
    worker = RgbPerceptionWorker(CFG, q, latest)
    worker.start()
    try:
        q.put(frame)
        got = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            got, _ = latest.get()
            if got is not None:
                break
            time.sleep(0.01)
    finally:
        worker.stop()
        worker.join(1)
    check("队列 -> Latest 搬运成功",
          got is not None and got.data == payload, f"{got}")


def test_traffic_light_detection():
    print("\n[20] 红绿灯检测：ROI + HSV + 时序投票")
    profile = {
        "hsv_space": "degrees_0_360_s01_v01",
        "camera_width": 640,
        "roi": [0.0, 0.0, 1.0, 1.0],
        "ranges": {
            "red": [[335.0, 360.0, 0.4, 0.25],
                    [0.0, 10.0, 0.4, 0.25]],
            "green": [[125.0, 165.0, 0.3, 0.25]],
        },
        "min_area_ratio": 0.01,
        "confirm_frames": 3,
        "confirm_s": 0.2,
        "stale_s": 0.2,
    }
    h, w = 100, 200
    red_img = np.full((h, w, 3), 0.5, np.float32)
    red_img[10:40, 10:40] = (1.0, 0.0, 0.3)       # 粉红，H≈342
    green_img = np.full((h, w, 3), 0.5, np.float32)
    green_img[50:80, 100:130] = (0.0, 1.0, 0.2)   # 绿，H≈132
    off_img = np.full((h, w, 3), 0.5, np.float32)

    red = detect_hsv(red_img, profile)
    green = detect_hsv(green_img, profile)
    off = detect_hsv(off_img, profile)
    check("红灯被识别", red.color == "red" and red.area_ratio > 0.01, f"{red}")
    check("绿灯被识别", green.color == "green" and green.area_ratio > 0.01,
          f"{green}")
    check("灰底无灯 → unknown", off.color == "unknown", f"{off}")
    check("面积比例符合预期", abs(red.area_ratio - 0.045) < 1e-6,
          f"{red.area_ratio:.4f}")

    voter = TrafficLightVoter(profile)
    state = None
    for i in range(3):
        state = voter.update(red, i * 0.1)
    check("连续 3 帧红灯 → stable",
          state is not None and state.stable and state.color == "red",
          f"{state}")
    one = voter.update(green, 0.4)
    check("单帧绿灯不立即切换", not one.stable, f"{one}")
    for i in range(3):
        state = voter.update(green, 0.5 + i * 0.1)
    check("连续 3 帧绿灯 → stable",
          state is not None and state.stable and state.color == "green",
          f"{state}")
    stale = voter.update(TrafficLightState("unknown", False, 2.0), 2.0)
    check("超过 stale → unknown",
          stale.color == "unknown" and not stale.stable, f"{stale}")


def test_rgb_detect_gate():
    print("\n[21] camera0 采集常开；traffic_active 门控检测")
    q = queue.Queue()
    latest = Latest()
    detect = queue.Queue()
    active = threading.Event()
    worker = RgbPerceptionWorker(
        CFG, q, latest, detect_queue=detect, detect_active=active
    )
    worker.start()
    frame = RgbFrame(b"\xff\xd8\xff\xd9", 0.0, "jpeg", 1)
    try:
        q.put(frame)
        got = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            got, _ = latest.get()
            if got is not None:
                break
            time.sleep(0.01)
        time.sleep(0.1)
        check("未到停止线：Latest 仍然有帧", got is not None)
        check("未到停止线：检测队列为空", detect.empty())

        active.set()
        q.put(frame)
        got2 = None
        try:
            got2 = detect.get(timeout=2.0)
        except queue.Empty:
            pass
        check("到停止线后：帧进入检测队列",
              got2 is not None and got2.data == frame.data, f"{got2}")
    finally:
        worker.stop()
        worker.join(1)


def test_stop_line_detection():
    print("\n[22] 黄线停止线：camera2 下方 ROI + 行号 + 投票")
    profile = {
        "roi": [0.0, 0.60, 1.0, 0.35],
        "ranges": [[18.0, 48.0, 0.35, 0.45]],
        "min_area_ratio": 0.002,
        "confirm_frames": 3,
        "stale_s": 0.3,
    }
    img = np.full((480, 640, 3), 0.5, np.float32)
    img[400:410, :, :] = (0.80, 0.55, 0.26)    # 橙黄线，H≈32（实测中位数）
    off = np.full((480, 640, 3), 0.5, np.float32)

    line = detect_stop_line(img, profile)
    none = detect_stop_line(off, profile)
    check("黄线可见", line.visible and line.area_ratio > 0.05,
          f"{line}")
    check("行号在画面下半部", line.visible and line.row > 0.8,
          f"row={line.row:.3f}")
    check("灰底不可见", not none.visible, f"{none}")

    voter = StopLineVoter(profile)
    state = None
    for i in range(3):
        state = voter.update(line, i * 0.1)
    check("连续 3 帧 -> 可见", state.visible, f"{state}")
    held = voter.update(StopLineState(False, 0.0, 0.3), 0.3)
    check("短暂丢帧保持可见", held.visible, f"{held}")
    lost = voter.update(StopLineState(False, 0.0, 1.0), 1.0)
    check("超过 stale -> 不可见", not lost.visible, f"{lost}")


def test_stop_line_profile_file():
    print("\n[22b] 实际 stop_line profile：0~360 色相 + 实测黄线颜色")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    profile = load_profile(os.path.join(
        root, "assets", "stop_line_profile.json"
    ))
    lo, hi = profile["ranges"][0][:2]
    # 实拍 201~207 的 H 中位数 31~37、p99<=43（0~360）。
    check("profile 色相覆盖实测黄线 31~43°",
          lo <= 31.0 and hi >= 43.0, f"{lo}~{hi}")
    check("profile 没有误用 0~180 单位（下限应 <31）",
          lo < 31.0, f"ranges={profile['ranges']}")
    check("profile 声明 0~360 色相空间",
          profile.get("hsv_space") == "degrees_0_360_s01_v01",
          f"{profile.get('hsv_space')}")

    img = np.full((480, 640, 3), 0.2, np.float32)
    img[380:390, :, :] = (0.80, 0.55, 0.26)     # H≈32°，实测黄线中位数
    line = detect_stop_line(img, profile)
    check("实际 profile 能看到合成橙黄线",
          line.visible and line.area_ratio > profile["min_area_ratio"],
          f"{line}")
    check("合成线行号 ≈ 0.8125",
          line.visible and abs(line.row - 0.8125) < 0.01,
          f"row={line.row:.4f}")

    off = np.full((480, 640, 3), 0.2, np.float32)
    none = detect_stop_line(off, profile)
    check("实际 profile 灰底不可见", not none.visible, f"{none}")

    # 保护：不要再把实测黄线误当成 50~84° 的黄绿线（0~180 单位乘 2 的错误）。
    wrong = dict(profile, ranges=[[50.0, 84.0, 0.35, 0.45]])
    miss = detect_stop_line(img, wrong)
    check("错误的 50~84° 范围看不到实测黄线", not miss.visible, f"{miss}")


def test_traffic_sequence():
    print("\n[23] 停止线序列：接近 → 停线 → 红灯等 → 绿灯过 → 交回避障")
    cfg = fast_cfg(
        traffic_vx=0.15,
        traffic_forward_s=0.5,
        stop_line_trigger_row=0.83,
        stop_settle_s=0.25,
        traffic_state_stale_s=0.5,
        stop_line_stale_s=0.5,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=5.0,
    )
    sm = AvoidStateMachine(cfg)
    sm.reset(0.0)
    sm.phase = GO
    active = threading.Event()
    seq = TrafficSequence(cfg, active, sm)
    base = 100.0

    def perception(t):
        return Perception(1.5, True, t)

    def ok_result(step, t):
        if step.request is None:
            return None
        return CommandResult(0, step.request.cmd.tag, "ok", ts=t)

    line_far = StopLineState(True, 0.70, base)
    check("停止线可见且新鲜时启动",
          seq.maybe_start(base, line_far) and seq.active)
    step = seq.step(base, line_far, None, executor_busy=False,
                    perception=perception(base), perception_ts=base)
    check("接近时低速前进", step.request is not None
          and step.request.kind == "forward"
          and abs(step.request.cmd.vx - cfg.traffic_vx) < 1e-9,
          f"{step.request}")
    seq.note_sent(step.request, base)

    t = base + 0.05
    step = seq.step(t, line_far, None, executor_busy=False,
                    perception=perception(t), perception_ts=t,
                    result=ok_result(step, t))
    check("前进结果成功后继续等待", step.request is None,
          f"{step.request}")

    line_stop = StopLineState(True, cfg.stop_line_trigger_row + 0.01,
                              base + 0.1)
    t = base + 0.1
    step = seq.step(t, line_stop, None, executor_busy=False,
                    perception=perception(t), perception_ts=t)
    check("到线立刻停车", step.request is not None
          and step.request.kind == "stop" and step.interrupt,
          f"{step.request}")
    seq.note_sent(step.request, t)

    t = base + 0.11
    step = seq.step(t, line_stop, None, executor_busy=False,
                    perception=perception(t), perception_ts=t,
                    result=ok_result(step, t))
    check("停车结果成功后进入停稳计时", step.request is None,
          f"{step.request}")

    t = base + 0.1 + cfg.stop_settle_s + 0.01
    seq.step(t, line_stop, None, executor_busy=False,
             perception=perception(t), perception_ts=t)
    check("停稳后开启红绿灯处理", active.is_set() and seq.phase == "SIGNAL",
          f"phase={seq.phase}, active={active.is_set()}")

    red = TrafficLightState("red", True, t, (0, 0, 1, 1), 0.01)
    step = seq.step(t + 0.1, line_stop, red, executor_busy=False,
                    perception=perception(t + 0.1),
                    perception_ts=t + 0.1)
    check("红灯保持停车",
          step.request is None or step.request.kind == "stop",
          f"{step.request}")

    t = t + 0.2
    green = TrafficLightState("green", True, t, (0, 0, 1, 1), 0.01)
    step = seq.step(t, line_stop, green, executor_busy=False,
                    perception=perception(t), perception_ts=t)
    check("绿灯后开始通过",
          step.request is not None and step.request.kind == "forward"
          and seq.phase == "PASS",
          f"phase={seq.phase}")
    check("通过动作使用配置距离",
          step.request.cmd.motion_id == "line_walk"
          and dict(step.request.cmd.params)["distance"] == cfg.traffic_pass_m,
          f"{step.request.cmd}")
    check("通过期间保留红绿灯检测（红灯复检）", active.is_set())
    seq.note_sent(step.request, t)

    t += 0.05
    seq.step(t, line_stop, green, executor_busy=False,
             perception=perception(t), perception_ts=t,
             result=ok_result(step, t))
    check("配置距离完成后交回避障",
          not seq.active and seq.done and sm.phase == HOLD,
          f"phase={seq.phase}, sm={sm.phase}")
    check("通过完成后关闭红绿灯处理", not active.is_set())


def test_traffic_stall():
    print("\n[24] 停止线无进展 → 锁存停车（不交回避障）")
    cfg = fast_cfg(
        traffic_vx=0.3,
        traffic_forward_s=0.5,
        stop_line_trigger_row=0.83,
        stop_settle_s=0.25,
        traffic_stall_s=0.2,
        traffic_stall_row=0.02,
        traffic_state_stale_s=0.5,
        stop_line_stale_s=0.5,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=5.0,
    )
    sm = AvoidStateMachine(cfg)
    sm.reset(0.0)
    sm.phase = GO
    seq = TrafficSequence(cfg, threading.Event(), sm)
    base = 100.0
    line = StopLineState(True, 0.60, base)
    perception = Perception(1.5, True, base)
    check("启动", seq.maybe_start(base, line))
    step = seq.step(base, line, None, executor_busy=False,
                    perception=perception, perception_ts=base)
    seq.note_sent(step.request, base)

    # 先消化前进结果；下一拍再检查“行号没进展”。
    t = base + 0.1
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t,
                    result=CommandResult(0, step.request.cmd.tag, "ok", ts=t))
    check("前进结果成功后仍在接近", step.request is None, f"{step.request}")

    t = base + 0.31
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("行号长时间不增 → 停车", step.request is not None
          and step.request.kind == "stop" and step.interrupt,
          f"{step.request}")
    check("锁存为 WAIT_STOP_LINE，不交回避障",
          seq.active and not seq.done and seq.phase == WAIT_STOP_LINE
          and sm.phase == GO,
          f"phase={seq.phase}, active={seq.active}, sm={sm.phase}")
    check("锁存时播报停止原因", step.voice is not None
          and step.voice.kind == "wait"
          and step.voice.reason == "stop_line_stall_wait",
          f"{step.voice}")

    # 锁存后即使深度持续失效，也只能保持停车，不能退回避障恢复前进。
    t = base + 0.31 + cfg.traffic_stall_s + 0.1
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, False, t), perception_ts=t)
    check("锁存后深度失效仍保持停车",
          seq.active and seq.phase == WAIT_STOP_LINE
          and (step.request is None or step.request.kind == "stop"),
          f"phase={seq.phase}, {step.request}")
    check("锁存后不回普通避障", sm.phase == GO, f"sm={sm.phase}")
    check("锁存原因不重复播报", step.voice is None, f"{step.voice}")


def test_traffic_line_lost_latch():
    print("\n[24b] 停止线丢失 → 锁存停车（不交回避障）")
    cfg = fast_cfg(
        traffic_stall_s=0.4,
        stop_line_stale_s=0.2,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=5.0,
    )
    base = 100.0
    sm, seq, line = _start_seq(cfg)
    check("启动", seq.maybe_start(base, line))
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    check("正常前进", step.request is not None
          and step.request.kind == "forward", f"{step.request}")
    seq.note_sent(step.request, base)

    # 刚过期还没到 stall：只停车等待，仍由序列接管。
    t = base + cfg.stop_line_stale_s + 0.01
    step = seq.step(t, line, None, executor_busy=True,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("短暂丢失只停车等待",
          seq.active and seq.phase != WAIT_STOP_LINE
          and step.request is not None and step.request.kind == "stop"
          and step.interrupt,
          f"phase={seq.phase}, {step.request}")

    # 超过 stall：锁存停车，不交回避障。
    t = base + cfg.traffic_stall_s + 0.01
    step = seq.step(t, line, None, executor_busy=True,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("持续丢失 → 锁存停车",
          seq.active and not seq.done and seq.phase == WAIT_STOP_LINE
          and sm.phase == GO,
          f"phase={seq.phase}, active={seq.active}, sm={sm.phase}")
    check("锁存播报停止线丢失", step.voice is not None
          and step.voice.kind == "wait"
          and step.voice.reason == "stop_line_lost_wait",
          f"{step.voice}")


def _start_seq(cfg, active=None):
    sm = AvoidStateMachine(cfg)
    sm.reset(0.0)
    sm.phase = GO
    seq = TrafficSequence(
        cfg, active if active is not None else threading.Event(), sm
    )
    line = StopLineState(True, 0.60, 100.0)
    return sm, seq, line


def test_traffic_safety():
    print("\n[25] 停止线序列安全：深度/停止线/语音/动作结果")
    cfg = fast_cfg(
        traffic_vx=0.3,
        traffic_forward_s=0.5,
        stop_line_trigger_row=0.83,
        traffic_stall_s=0.2,
        stop_line_stale_s=0.5,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=5.0,
    )
    base = 100.0

    # 停止线过期：不能启动
    sm, seq, line = _start_seq(cfg)
    stale_line = StopLineState(True, 0.60, base - cfg.stop_line_stale_s - 0.1)
    check("停止线时间戳过期 → 不启动",
          not seq.maybe_start(base, stale_line))
    check("停止线时间戳为 0 → 不启动",
          not seq.maybe_start(base, StopLineState(True, 0.60, 0.0)))

    # 语音播报中：不能启动
    sm, seq, line = _start_seq(cfg)
    sm.voice.wait_for_voice = True
    sm.voice.pending = True
    sm.voice.started_ts = base
    check("语音播报中 → 不启动",
          not seq.maybe_start(base, line))
    sm.voice.pending = False

    # 启动后深度无效：立即打断前进
    sm, seq, line = _start_seq(cfg)
    check("正常启动", seq.maybe_start(base, line))
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    check("正常前进", step.request is not None
          and step.request.kind == "forward", f"{step.request}")
    seq.note_sent(step.request, base)
    t = base + 0.05
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, False, t),
                    perception_ts=t)
    check("深度无效 → 打断前进并停车",
          step.request is not None and step.request.kind == "stop"
          and step.interrupt, f"{step.request}")
    check("深度无效清掉待执行前进", not seq.has_pending_action)

    # 前进动作还在跑时到线：必须立刻打断，不能等 0.5s 前进结束
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    seq.note_sent(step.request, base)
    stop_line = StopLineState(True, cfg.stop_line_trigger_row + 0.01,
                              base + 0.05)
    step = seq.step(base + 0.05, stop_line, None, executor_busy=True,
                    perception=Perception(1.5, True, base + 0.05),
                    perception_ts=base + 0.05)
    check("前进中到线 → 立即打断停车",
          step.request is not None and step.request.kind == "stop"
          and step.interrupt and not seq.has_pending_action,
          f"{step.request}")

    # 前进动作还在跑时停止线过期：也必须立刻停车
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    seq.note_sent(step.request, base)
    t = base + cfg.stop_line_stale_s + 0.05
    step = seq.step(t, line, None, executor_busy=True,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("前进中停止线过期 → 立即停车",
          step.request is not None and step.request.kind == "stop"
          and step.interrupt and not seq.has_pending_action,
          f"{step.request}")

    # 深度过期同样停车
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    t = base + cfg.stale_s + 0.1
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    check("深度时间戳过期 → 停车",
          step.request is not None and step.request.kind == "stop",
          f"{step.request}")

    # 深度持续失效超过 stall → 退出序列
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    seq.step(base, line, None, executor_busy=False,
             perception=Perception(1.5, False, base),
             perception_ts=base)
    step = seq.step(base + cfg.traffic_stall_s + 0.01, line, None,
                    executor_busy=False,
                    perception=Perception(1.5, False, base + 0.3),
                    perception_ts=base + 0.3)
    check("深度持续失效 → 交回避障",
          not seq.active and seq.done and sm.phase == HOLD,
          f"phase={seq.phase}, sm={sm.phase}")

    # 序列自己打开的红绿灯处理，异常退出时要关掉
    active = threading.Event()
    sm, seq, line = _start_seq(cfg, active)
    seq.active = True
    seq._traffic_opened = True
    active.set()
    seq._abort(base, "测试异常")
    check("异常退出关闭 traffic_active", not active.is_set())

    # 序列中的前进失败：停车并退出
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    seq.note_sent(step.request, base)
    t = base + 0.05
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t,
                    result=CommandResult(0, step.request.cmd.tag,
                                         "failed", "boom", ts=t))
    check("前进动作失败 → 停车交回避障",
          step.request is not None and step.request.kind == "stop"
          and step.interrupt and not seq.active and seq.done,
          f"{step.request}, active={seq.active}")

    # 序列中的停车失败：不能继续走，要重新停车
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    stop_line = StopLineState(True, cfg.stop_line_trigger_row + 0.01, base)
    step = seq.step(base, stop_line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    check("到线发停车", step.request is not None
          and step.request.kind == "stop")
    seq.note_sent(step.request, base)
    t = base + 0.05
    step = seq.step(t, stop_line, None, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t,
                    result=CommandResult(0, step.request.cmd.tag,
                                         "failed", "boom", ts=t))
    check("停车动作失败 → 继续停车",
          step.request is not None and step.request.kind == "stop"
          and step.interrupt and seq.active,
          f"{step.request}, active={seq.active}")

    # 启动后语音播报：必须打断运动
    sm, seq, line = _start_seq(cfg)
    seq.maybe_start(base, line)
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    seq.note_sent(step.request, base)
    sm.voice.wait_for_voice = True
    sm.voice.pending = True
    sm.voice.started_ts = base + 0.05
    t = base + 0.05
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("序列中语音播报 → 打断前进停车",
          step.request is not None and step.request.kind == "stop"
          and step.interrupt and not seq.has_pending_action,
          f"{step.request}")


def test_traffic_voice_and_recheck():
    print("\n[25b] 红绿灯语音 + 通过中红灯复检 + 黄灯停车")
    cfg = fast_cfg(
        traffic_vx=0.2,
        traffic_forward_s=0.5,
        stop_settle_s=0.0,
        traffic_state_stale_s=0.5,
        traffic_signal_timeout_s=0.3,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=5.0,
    )
    base = 100.0
    sm = AvoidStateMachine(cfg, wait_for_voice=True)
    sm.reset(0.0)
    sm.phase = GO
    active = threading.Event()
    seq = TrafficSequence(cfg, active, sm)
    line = StopLineState(True, 0.60, base)

    seq.active = True
    seq.phase = AT_LINE
    seq.stop_ts = base - 1.0
    step = seq.step(base, line, None, executor_busy=False,
                    perception=Perception(1.5, True, base),
                    perception_ts=base)
    check("停稳后开启红绿灯处理", active.is_set() and seq.phase == "SIGNAL",
          f"phase={seq.phase}, active={active.is_set()}")

    red = TrafficLightState("red", True, base + 0.01, (0, 0, 1, 1), 0.01)
    t = base + 0.01
    step = seq.step(t, line, red, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("红灯播报一次", step.voice is not None
          and step.voice.kind == "traffic_red", f"{step.voice}")
    check("红灯播报期间保持停车", sm.voice_pending
          and (step.request is None or step.request.kind == "stop"),
          f"pending={sm.voice_pending}, {step.request}")

    t += 0.02
    step = seq.step(t, line, red, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("红灯未播完不重复播报", step.voice is None and sm.voice_pending,
          f"{step.voice}")

    t += 0.02
    step = seq.step(t, line, red, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t,
                    voice_done=True)
    check("红灯播完继续停车等待", seq.phase == "SIGNAL"
          and (step.request is None or step.request.kind == "stop"),
          f"phase={seq.phase}, {step.request}")

    green = TrafficLightState("green", True, t + 0.01, (0, 0, 1, 1), 0.01)
    t += 0.01
    step = seq.step(t, line, green, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("绿灯立即进入 PASS（不等播报）", seq.phase == "PASS"
          and not sm.voice_pending
          and step.request is not None
          and step.request.kind == "forward",
          f"phase={seq.phase}, pending={sm.voice_pending}, {step.request}")
    check("绿灯仍然下发播报", step.voice is not None
          and step.voice.kind == "traffic_green", f"voice={step.voice}")
    check("通过中保持红绿灯检测", active.is_set())

    # 通过途中红灯重新稳定：立刻停车并退回 SIGNAL，不盲走完通过时间。
    seq.note_sent(step.request, t)
    t += 0.02
    red2 = TrafficLightState("red", True, t, (0, 0, 1, 1), 0.01)
    step = seq.step(t, line, red2, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("通过中红灯 → 立刻停车",
          seq.phase == "SIGNAL" and step.request is not None
          and step.request.kind == "stop" and step.interrupt,
          f"phase={seq.phase}, {step.request}")
    check("通过中红灯 → 播报红灯", step.voice is not None
          and step.voice.kind == "traffic_red", f"{step.voice}")
    check("通过中红灯仍保持检测", active.is_set())

    # 清掉红灯语音，再看黄灯：黄灯只停不走，并且只播一次。
    t += 0.02
    seq.step(t, line, red2, executor_busy=False,
             perception=Perception(1.5, True, t), perception_ts=t,
             voice_done=True)
    yellow = TrafficLightState("yellow", True, t + 0.01, (0, 0, 1, 1), 0.01)
    t += 0.01
    step = seq.step(t, line, yellow, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("黄灯停车并播报", step.voice is not None
          and step.voice.kind == "traffic_yellow"
          and (step.request is None or step.request.kind == "stop"),
          f"{step.voice}, {step.request}")
    t += 0.02
    step = seq.step(t, line, yellow, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t,
                    voice_done=True)
    check("黄灯不重复播报也不通过", step.voice is None
          and seq.phase == "SIGNAL"
          and (step.request is None or step.request.kind == "stop"),
          f"phase={seq.phase}, voice={step.voice}")

    # 红绿灯数据长期缺失：保持停车，不把控制权交回避障。
    t += 0.5
    step = seq.step(t, line, None, executor_busy=False,
                    perception=Perception(1.5, True, t), perception_ts=t)
    check("信号丢失只停车不交回避障",
          seq.active and seq.phase == "SIGNAL"
          and (step.request is None or step.request.kind == "stop"),
          f"active={seq.active}, phase={seq.phase}")


def test_stop_line_voter_history():
    print("\n[26] 停止线投票：过期后清空历史")
    profile = {"confirm_frames": 3, "stale_s": 0.3}
    line = StopLineState(True, 0.70, 0.0)
    voter = StopLineVoter(profile)
    voter.update(line, 0.0)
    voter.update(line, 0.1)
    # 中间长时间没有线，再只来一帧：旧的两帧不能用来说明“连续可见”
    voter.update(StopLineState(False, 0.0, 0.0), 1.0)
    state = voter.update(line, 1.1)
    check("间隔超过 stale 后仍需重新累计",
          not state.visible, f"{state}")

    voter2 = StopLineVoter(profile)
    for i in range(3):
        state = voter2.update(line, i * 0.1)
    check("连续 3 帧先确认", state.visible, f"{state}")
    state = voter2.update(line, 2.0)      # 大间隔后的第一帧
    check("已确认后大间隔也不能沿用旧历史",
          not state.visible, f"{state}")
    state = voter2.update(StopLineState(False, 0.0, 0.0), 2.1)
    check("大间隔后旧确认不再保持可见",
          not state.visible, f"{state}")


class _FakeResp:
    def __init__(self, success=True, message="", **fields):
        self.success = success
        self.message = message
        for key, value in fields.items():
            setattr(self, key, value)


class _FakeChannel:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class _FakeRobotClient:
    def __init__(self, state_ok=True, oa=True, speed=40,
                 oa_set_ok=True, speed_set_ok=True,
                 stand_ok=True, walk_ok=True):
        self.state_ok = state_ok
        self.oa = oa
        self.speed = speed
        self.oa_set_ok = oa_set_ok
        self.speed_set_ok = speed_set_ok
        self.stand_ok = stand_ok
        self.walk_ok = walk_ok
        self.calls = []
        self.channel = _FakeChannel()

    def get_state(self):
        return _FakeResp(
            self.state_ok,
            obstacle_avoidance_enabled=self.oa,
            current_speed_ratio=self.speed,
        )

    def set_obstacle_avoidance(self, enable):
        self.calls.append(("oa", bool(enable)))
        if self.oa_set_ok:
            self.oa = bool(enable)
        return _FakeResp(self.oa_set_ok, current_enabled=self.oa)

    def set_speed_ratio(self, ratio):
        self.calls.append(("speed", int(ratio)))
        if self.speed_set_ok:
            self.speed = int(ratio)
        return _FakeResp(self.speed_set_ok, current_speed_ratio=self.speed)

    def balance_stand(self):
        return _FakeResp(self.stand_ok)

    def walk(self):
        return _FakeResp(self.walk_ok)


class _TestRobotLink(RobotLink):
    def _open_channel(self):
        self._channel = None
        self._stub = None


def test_robot_link_safety():
    print("\n[27] RobotLink：状态读取/动作返回值/退出恢复")
    fake = _FakeRobotClient(state_ok=False)
    link = _TestRobotLink("fake", client_factory=lambda _ip: fake)
    try:
        link.connect()
        raised = False
    except RuntimeError:
        raised = True
    check("读不到机器人状态 → connect 失败", raised)
    check("状态未知时不关闭内置避障",
          not any(call[0] == "oa" for call in fake.calls), f"{fake.calls}")

    fake = _FakeRobotClient(oa_set_ok=False)
    link = _TestRobotLink("fake", client_factory=lambda _ip: fake)
    try:
        link.connect()
        raised = False
    except RuntimeError:
        raised = True
    check("关闭内置避障失败 → connect 失败", raised)
    link.close()
    check("失败退出仍恢复原内置避障", fake.oa is True, f"oa={fake.oa}")

    fake = _FakeRobotClient(oa=True, speed=40)
    link = _TestRobotLink("fake", speed_ratio=70,
                          client_factory=lambda _ip: fake)
    link.connect()
    check("运行期间关闭内置避障", fake.oa is False, f"oa={fake.oa}")
    check("运行期间设置速度档位", fake.speed == 70, f"speed={fake.speed}")
    link.stand()
    link.walk()
    link.close()
    check("退出恢复速度档位", fake.speed == 40, f"speed={fake.speed}")
    check("退出恢复内置避障", fake.oa is True, f"oa={fake.oa}")
    check("退出关闭高层通道", fake.channel.closed)

    fake = _FakeRobotClient(stand_ok=False)
    link = _TestRobotLink("fake", client_factory=lambda _ip: fake)
    link.connect()
    try:
        link.stand()
        raised = False
    except RuntimeError:
        raised = True
    check("起立失败 → 抛出异常", raised)
    link.close()

    fake = _FakeRobotClient(walk_ok=False)
    link = _TestRobotLink("fake", client_factory=lambda _ip: fake)
    link.connect()
    try:
        link.walk()
        raised = False
    except RuntimeError:
        raised = True
    check("切换行走失败 → 抛出异常", raised)
    link.close()


def test_traffic_decision_integration():
    print("\n[28] 联调：停止线序列动作结果失败 → 不继续前进")
    cfg = fast_cfg(
        traffic_vx=0.15,
        traffic_forward_s=0.2,
        stop_line_trigger_row=0.84,
        stop_line_stale_s=0.5,
        traffic_stall_s=2.0,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=0.2,
    )
    perception = Latest()
    stop_line = Latest()
    traffic = Latest()
    active = threading.Event()
    robot = RecordingRobot(cap_duration=0.05, fail_tags={"traffic_forward"})
    feeder_stop = threading.Event()

    def feeder():
        while not feeder_stop.is_set():
            now = time.time()
            perception.set(Perception(1.5, True, now))
            stop_line.set(StopLineState(True, 0.60, now))
            traffic.set(TrafficLightState("unknown", False, now))
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop,
        stop_line=stop_line, traffic=traffic, traffic_active=active,
        traffic_sequence=True,
    )

    try:
        deadline = time.time() + 3.0
        while robot.count("traffic_forward") < 1 and time.time() < deadline:
            time.sleep(0.02)
        time.sleep(0.5)               # 给失败结果和退出序列留时间
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)

    check("序列启动后发出低速前进",
          robot.count("traffic_forward") >= 1,
          f"{robot.count('traffic_forward')}")
    check("前进失败后序列退出",
          decision.engine.traffic is not None
          and not decision.engine.traffic.active
          and decision.engine.traffic.done,
          f"active={getattr(decision.engine.traffic, 'active', None)}, "
          f"done={getattr(decision.engine.traffic, 'done', None)}")
    check("前进失败后补发停车",
          robot.count("stop") >= 1, f"{robot.count('stop')}")
    check("前进失败后不重复前进",
          robot.count("traffic_forward") == 1,
          f"{robot.count('traffic_forward')}")


def test_traffic_speed_floor():
    print("\n[29] 停止线速度下限：0.4 m/s（0.3 实测只原地踏步）")
    check("MIN_TRAFFIC_VX = 0.4", abs(MIN_TRAFFIC_VX - 0.4) < 1e-9,
          f"{MIN_TRAFFIC_VX}")
    check("默认 traffic_vx >= 0.4", CFG.traffic_vx >= MIN_TRAFFIC_VX,
          f"{CFG.traffic_vx}")
    check("traffic_vx 不超过 0.5", CFG.traffic_vx <= 0.5, f"{CFG.traffic_vx}")
    check("traffic_forward_s >= 1.0（避免起停加减速吃掉位移）",
          CFG.traffic_forward_s >= 1.0, f"{CFG.traffic_forward_s}")
    check("动作超时 >= forward_s + 1.0",
          CFG.traffic_action_timeout_s >= CFG.traffic_forward_s + 1.0,
          f"{CFG.traffic_action_timeout_s}")

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def run_vx(value):
        try:
            proc = subprocess.run(
                [sys.executable, "main.py", "--traffic-vx", value, "--dry-run"],
                cwd=root, capture_output=True, text=True, timeout=20,
            )
        except subprocess.TimeoutExpired:
            return None
        return proc

    for value in ("0.3", "0.15", "0.6"):
        proc = run_vx(value)
        check(f"--traffic-vx {value} 被拒绝",
              proc is not None and proc.returncode == 2
              and "--traffic-vx" in proc.stderr,
              "进程未退出（参数校验失效）" if proc is None
              else f"code={proc.returncode}")

    # --help 提前退出，不连接硬件；确认帮助文本把下限写清楚了。
    proc = subprocess.run(
        [sys.executable, "main.py", "--traffic-vx", "0.4", "--help"],
        cwd=root, capture_output=True, text=True, timeout=20,
    )
    check("帮助文本说明最低 0.4",
          proc.returncode == 0 and "最低 0.4" in proc.stdout,
          f"code={proc.returncode}")


def test_traffic_sensor_guard():
    print("\n[30] 停止线检测器守卫：无数据/过期停车，健康可见/不可见都放行")
    cfg = fast_cfg(
        stop_line_stale_s=0.5,
        traffic_stall_s=0.4,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=5.0,
    )
    base = 100.0
    sm, seq, _ = _start_seq(cfg)

    hidden = StopLineState(False, 0.0, base)
    check("健康但没看到线 → 不误停",
          seq.guard_sensor(base, hidden, base) is None,
          f"{seq.guard_sensor(base + 0.001, hidden, base)}")

    step = seq.guard_sensor(base + 0.1, None, 0.0)
    check("从没出数据 → 立即打断停车",
          step is not None and step.request is not None
          and step.request.kind == "stop" and step.interrupt,
          f"{step}")
    check("首次失效播报 stop_line_sensor_wait",
          step.voice is not None and step.voice.kind == "wait"
          and step.voice.reason == "stop_line_sensor_wait",
          f"{step.voice}")

    step = seq.guard_sensor(base + 0.2, None, 0.0)
    check("短暂失效保持停车",
          step is not None and step.request is None and not step.interrupt,
          f"{step}")
    check("短暂失效不重复播报", step.voice is None, f"{step.voice}")

    t = base + 0.3
    check("恢复（visible=False 也算健康）→ 放行",
          seq.guard_sensor(t, StopLineState(False, 0.0, t), t) is None,
          f"{seq.guard_sensor(t, StopLineState(False, 0.0, t), t)}")

    # 再次失效，超过 traffic_stall_s → 进入 WAIT_STOP_LINE 锁存。
    t = base + 10.0
    seq.guard_sensor(t, None, 0.0)
    t += cfg.traffic_stall_s + 0.01
    step = seq.guard_sensor(t, None, 0.0)
    check("持续失效 → 锁存 WAIT_STOP_LINE",
          seq.active and seq.phase == WAIT_STOP_LINE,
          f"active={seq.active}, phase={seq.phase}")
    check("锁存时仍发停车",
          step is not None and step.request is not None
          and step.request.kind == "stop",
          f"{step}")

    # 锁存后即使检测器恢复，也要求人工重启，不能自动交回普通避障。
    t += 1.0
    step = seq.guard_sensor(t, StopLineState(True, 0.5, t), t)
    check("锁存后恢复也不放行",
          seq.active and seq.phase == WAIT_STOP_LINE and step is None,
          f"active={seq.active}, phase={seq.phase}")

    check("语音文案包含“停止线检测失效”",
          "停止线检测失效" in render_text(
              VoiceCue("wait", reason="stop_line_sensor_wait")),
          render_text(VoiceCue("wait", reason="stop_line_sensor_wait")))
    check("检测器失效用警告优先级",
          priority_name(
              VoiceCue("wait", reason="stop_line_sensor_wait")) == "kWarning",
          priority_name(VoiceCue("wait", reason="stop_line_sensor_wait")))


def test_traffic_sensor_dead_integration():
    print("\n[31] 联调：检测器无数据不前进；恢复后接近；再失效立即停")
    cfg = fast_cfg(
        stop_line_stale_s=0.3,
        traffic_stall_s=5.0,
        traffic_action_timeout_s=1.0,
        stop_refresh_s=0.2,
        traffic_forward_s=0.2,
        confirm_s=0.2,
    )
    perception = Latest()
    stop_line = Latest()          # 不写 = 模拟子进程卡死
    traffic = Latest()
    active = threading.Event()
    robot = RecordingRobot(cap_duration=0.4)
    feeder_stop = threading.Event()
    mode = {"line": "dead"}

    def feeder():
        while not feeder_stop.is_set():
            now = time.time()
            perception.set(Perception(1.5, True, now))
            traffic.set(TrafficLightState("unknown", False, now))
            if mode["line"] == "healthy":
                # 检测器活着，只是线还没进入视野
                stop_line.set(StopLineState(False, 0.0, now))
            time.sleep(0.02)

    executor, decision, feeder_thread = _start_stack(
        cfg, robot, perception, feeder, feeder_stop,
        stop_line=stop_line, traffic=traffic, traffic_active=active,
        traffic_sequence=True,
    )
    try:
        time.sleep(0.5)
        check("检测器无数据时不发普通前进",
              robot.count("forward") == 0, f"{robot.count('forward')}")
        check("检测器无数据时不发序列前进",
              robot.count("traffic_forward") == 0,
              f"{robot.count('traffic_forward')}")
        check("检测器无数据时发出停车",
              robot.count("stop") >= 1, f"{robot.count('stop')}")

        # 检测器恢复但没看到线：应回到普通避障接近。
        mode["line"] = "healthy"
        deadline = time.time() + 2.0
        while robot.count("forward") < 1 and time.time() < deadline:
            time.sleep(0.02)
        check("检测器健康（visible=False）→ 恢复普通前进",
              robot.count("forward") >= 1, f"{robot.count('forward')}")
        check("健康但没看到线时序列不接管",
              decision.engine.traffic is not None
              and not decision.engine.traffic.active,
              f"active={decision.engine.traffic.active if decision.engine.traffic else None}")

        # 再次失效：超过 stale 后必须立刻停车，并且不再补发前进。
        mode["line"] = "dead"
        time.sleep(0.6)               # stale + 打断 + 停车
        forward_settled = robot.count("forward")
        stop_settled = robot.count("stop")
        time.sleep(0.6)
        check("再次失效后不再前进",
              robot.count("forward") == forward_settled,
              f"{forward_settled} → {robot.count('forward')}")
        check("再次失效后持续停车",
              robot.count("stop") > stop_settled,
              f"{stop_settled} → {robot.count('stop')}")
    finally:
        _stop_stack(executor, decision, feeder_thread, feeder_stop)


def test_lane_follow():
    print("\n[25] 单线跟踪/双黄线盲道：检测 + Motion 动作 + 丢失停车")
    profile = load_lane_profile("assets/lane_profile.json")
    profile["mode"] = "dual"
    h, w = 480, 640
    img = np.full((h, w, 3), 0.5, np.float32)
    for y in range(int(0.55 * h), int(0.96 * h)):
        t = (y - 0.65 * h) / (0.90 * h - 0.65 * h)
        xl = int((0.30 - 0.12 * t) * w)
        xr = int((0.70 + 0.12 * t) * w)
        img[y, max(0, xl - 3):min(w, xl + 3)] = (1.0, 0.55, 0.0)
        img[y, max(0, xr - 3):min(w, xr + 3)] = (1.0, 0.55, 0.0)

    lane = detect_lane(img, profile, now=0.0)
    off = detect_lane(np.full((h, w, 3), 0.5, np.float32), profile, now=0.0)
    missing = img.copy()
    missing[:, int(0.6 * w):] = 0.5
    miss = detect_lane(missing, profile, now=0.0)
    check("双黄线可见", lane.visible and abs(lane.center - 0.5) < 0.03,
          f"{lane}")
    check("灰底不可见", not off.visible, f"{off}")
    check("缺一条线仍可单线回正", miss.visible and miss.one_line, f"{miss}")

    cfg = replace(
        CFG,
        lane_follow_enabled=True,
        lane_steer_sign=-1.0,
        lane_stale_s=0.4,
        lane_forward_vx=0.4,
        lane_control_duration_s=1.2,
        stop_refresh_s=5.0,
    )
    per = Perception(2.0, True, 0.9)
    pose = Pose(0.0, 0.9)
    # 取消双线后，单黄线就是车道中心线
    single_profile = dict(profile, mode="single")
    simg = np.full((h, w, 3), 0.5, np.float32)
    for y in range(int(0.55 * h), int(0.96 * h)):
        x = int(0.50 * w)
        simg[y, max(0, x - 3):min(w, x + 3)] = (1.0, 0.55, 0.0)
    single = detect_lane(simg, single_profile, now=0.9)
    check("单线目标=线中心",
          single.visible and single.single
          and abs(single.center - 0.5) < 0.03,
          f"{single}")
    sf = LaneFollower(cfg)
    ss = sf.step(1.0, single, per, 0.9, pose, None, False)
    check("单线居中 -> Velocity 前进",
          ss.request is not None and ss.request.kind == "lane_velocity"
          and isinstance(ss.request.cmd, VelocityPlan)
          and len(ss.request.cmd.steps) == 2
          and abs(ss.request.cmd.steps[0].vx - 0.4) < 1e-6,
          f"{ss.request}")
    check("前进动作已登记 pending", sf.has_pending)
    res = CommandResult(1, ss.request.cmd.tag, "ok", ts=1.1)
    done = sf.step(1.1, single, per, 0.9, pose, res, False)
    check("速度片段结果 ok -> LANE_UPDATE 并清 pending",
          done.state == "LANE_UPDATE" and not sf.has_pending, f"{done.state}")

    right = LaneState(
        True, 0.62, 0.62, 0.0, 0.0, ts=0.9, confidence=0.8, single=True
    )
    left = LaneState(
        True, 0.38, 0.38, 0.0, 0.0, ts=0.9, confidence=0.8, single=True
    )
    f1 = LaneFollower(cfg)
    s1 = f1.step(1.0, right, per, 0.9, pose, None, False)
    check("中心偏右 -> 输出有界 yaw 速度",
          s1.request is not None and s1.request.kind == "lane_velocity"
          and isinstance(s1.request.cmd, VelocityPlan)
          and abs(s1.request.cmd.steps[0].vyaw) <= cfg.lane_max_yaw_rate,
          f"{s1.request}")
    check("速度指令已登记 pending", f1.has_pending)
    res1 = CommandResult(1, s1.request.cmd.tag, "ok", ts=1.1)
    done1 = f1.step(1.1, right, per, 0.9, pose, res1, False)
    check("速度片段完成 -> LANE_UPDATE", done1.state == "LANE_UPDATE", done1.state)
    f2 = LaneFollower(cfg)
    s2 = f2.step(1.0, left, per, 0.9, pose, None, False)
    check("中心偏左 -> 输出有界 yaw 速度",
          s2.request is not None and s2.request.kind == "lane_velocity"
          and isinstance(s2.request.cmd, VelocityPlan)
          and abs(s2.request.cmd.steps[0].vyaw) <= cfg.lane_max_yaw_rate,
          f"{s2.request}")
    lost = f1.step(1.3, None, per, 0.9, pose, None, False)
    check("已完成动作后车道丢失 -> 停车，无需取消已完成动作",
          lost.request is not None and lost.request.kind == "stop"
          and not lost.interrupt, f"{lost.request}")
    # f2的速度序列尚未完成，丢线必须打断，不能只发送排队的零速。
    moving_lost = f2.step(1.3, None, per, 0.9, pose, None, True)
    check("运动中车道丢失 -> 停车打断",
          moving_lost.request is not None and moving_lost.request.kind == "stop"
          and moving_lost.interrupt, f"{moving_lost.request}")


def main():
    test_profile()
    test_actions()
    test_state_machine_success()
    test_startup_near_obstacle_bypass()
    test_second_entry_yaw()
    test_failed_turn_blocks()
    test_yaw_mismatch_blocks()
    test_action_timeout_blocks()
    test_action_perception_loss_blocks()
    test_dynamic_obstacle_waits()
    test_too_close_blocks()
    test_voice_cues()
    test_voice_publish()
    test_voice_pack()
    test_voice_gate()
    test_executor_results()
    test_workers()
    test_voice_routing_integration()
    test_voice_ack_integration()
    test_avoid_cancelled()
    test_rgb_capture()
    test_traffic_light_detection()
    test_rgb_detect_gate()
    test_stop_line_detection()
    test_stop_line_profile_file()
    test_traffic_sequence()
    test_traffic_stall()
    test_lane_follow()
    test_traffic_line_lost_latch()
    test_traffic_safety()
    test_traffic_voice_and_recheck()
    test_stop_line_voter_history()
    test_robot_link_safety()
    test_traffic_decision_integration()
    test_traffic_speed_floor()
    test_traffic_sensor_guard()
    test_traffic_sensor_dead_integration()
    print("\n" + ("全部通过 ✅" if not FAILS else f"失败 {len(FAILS)} 项 ❌: {FAILS}"))
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
