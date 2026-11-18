"""盲道跟线闭环仿真：真实 LaneFollower + 运动学植物，离线量化"走斜"。

用法：python3 tools/lane_sim.py [--old-tuning]
  默认跑当前 config.py；--old-tuning 只把 5 个跟线参数改回 0.12rad/s / 2.5s / 死区0.018
  那一套（控制逻辑不变），用来证明"走斜"的主因是参数而不是别的。
没有机器人也能验证：改一个参数就能看出航向偏差、横偏和丢线率的变化。
"""
import argparse
import math
import random
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from decisions.lane import LaneFollower
from messages import LaneState, Perception

W, H = 640, 480
K = 1.166          # 2*tan(hfov/2)，camera2 约 60° 水平视场
Y_NEAR, Y_FAR = 0.90, 0.65
D_NEAR, D_FAR = 0.60, 1.40
L_CAM = 0.15
NOISE = [1.0]      # 检测噪声倍率：1.0≈±5px/±1.2°


class Path:
    """恒定曲率路径，s 为弧长；k>0 左弯。"""

    def __init__(self, k=0.0):
        self.k = k

    def point(self, s):
        if abs(self.k) < 1e-9:
            return (s, 0.0)
        return (math.sin(self.k * s) / self.k, (1.0 - math.cos(self.k * s)) / self.k)

    def tangent(self, s):
        return (math.cos(self.k * s), math.sin(self.k * s))

    def solve(self, cam, psi, d, anchor):
        """路径上"机体前向坐标 = d"的点，只在 anchor 附近搜索。"""
        ux, uy = math.cos(psi), math.sin(psi)

        def f(s):
            px, py = self.point(s)
            return (px - cam[0]) * ux + (py - cam[1]) * uy

        lo, hi = anchor - 2.0, anchor + 8.0
        step = (hi - lo) / 200.0
        a, fa = lo, f(lo)
        for i in range(1, 201):
            b = lo + i * step
            if fa - d <= 0.0 <= f(b) - d:
                for _ in range(50):
                    m = (a + b) / 2
                    if f(m) < d:
                        a = m
                    else:
                        b = m
                s = (a + b) / 2
                px, py = self.point(s)
                return s, (px - cam[0]) * -uy + (py - cam[1]) * ux
            a, fa = b, f(b)
        return None


def _frame(path, x, y, psi, anchor, rng, ts):
    """一帧检测结果：与 workers/lane.py 一致，拟合点越界视为没检出。"""
    cam = (x + L_CAM * math.cos(psi), y + L_CAM * math.sin(psi))
    pn = path.solve(cam, psi, D_NEAR, anchor)
    pf = path.solve(cam, psi, D_FAR, anchor)
    if pn is None or pf is None:
        return LaneState(False, ts=ts), anchor
    xn = 0.5 + (-pn[1] / D_NEAR) / K
    xf = 0.5 + (-pf[1] / D_FAR) / K
    if not (-0.35 <= xn <= 1.35 and -0.35 <= xf <= 1.35):
        return LaneState(False, ts=ts), anchor
    heading = math.degrees(math.atan2((xf - xn) * W, (Y_NEAR - Y_FAR) * H))
    c = xn
    c += rng.uniform(-0.008 * NOISE[0], 0.008 * NOISE[0])
    heading += rng.uniform(-1.2 * NOISE[0], 1.2 * NOISE[0])
    return LaneState(True, center=c, center_far=xf, heading_deg=heading,
                     width=0.0, left_x=c, right_x=c, ts=ts, confidence=0.9,
                     single=True), pn[0]


def _true_err(path, x, y, psi, anchor):
    cam = (x + L_CAM * math.cos(psi), y + L_CAM * math.sin(psi))
    p = path.solve(cam, psi, 0.0, anchor)
    if p is None:
        return None
    tx, ty = path.tangent(p[0])
    ang = math.atan2(ty, tx)
    rel = math.degrees(math.atan2(math.sin(psi - ang), math.cos(psi - ang)))
    return p[1], rel


def run(cfg, curv=0.0, y0=0.12, psi0=math.radians(6.0), seconds=30.0, seed=1,
        tau_yaw=0.12, disturb=0.0, framerate=8.0):
    """返回横偏/航向统计；tau_yaw 是底盘的角速度一阶响应时间常数。"""
    rng = random.Random(seed)
    path = Path(curv)
    follower = LaneFollower(cfg)
    x, y, psi = 0.0, y0, psi0
    anchor, cmd, vyaw_act = 0.0, 0.0, 0.0
    t, dt, t0 = 0.0, 0.02, 1000.0
    frame_dt, next_frame = 1.0 / framerate, 0.0
    frame = LaneState(False, ts=t0)
    cross, headings = [], []
    lost = frames = 0
    while t < seconds:
        now = t0 + t
        if t >= next_frame - 1e-9:
            frame, anchor = _frame(path, x, y, psi, anchor, rng, now)
            next_frame += frame_dt
            frames += 1
            lost += 0 if frame.visible else 1
        follower.step(now, frame, Perception(5.0, True, now), now, None, None, True)
        cmd = follower._active_vyaw
        vyaw_act += (cmd + disturb - vyaw_act) * (dt / tau_yaw)
        psi += vyaw_act * dt
        x += cfg.lane_forward_vx * math.cos(psi) * dt
        y += cfg.lane_forward_vx * math.sin(psi) * dt
        err = _true_err(path, x, y, psi, anchor)
        if err is not None and t > 2.0:
            cross.append(err[0])
            headings.append(err[1])
        t += dt
    n = len(cross)
    return {
        "rms_cm": math.sqrt(sum(c * c for c in cross) / n) * 100,
        "max_cm": max(abs(c) for c in cross) * 100,
        "rms_deg": math.sqrt(sum(h * h for h in headings) / n),
        "max_deg": max(abs(h) for h in headings),
        "gt5": sum(1 for h in headings if abs(h) > 5.0) / n,
        "lost": lost / max(frames, 1),
    }


CASES = (
    ("直线 12cm/6deg", dict(y0=0.12, psi0=math.radians(6.0), seconds=30)),
    ("直线 -25cm/-12deg", dict(y0=-0.25, psi0=math.radians(-12.0), seconds=35)),
    ("常值偏航 0.6deg/s", dict(y0=0.0, psi0=0.0, seconds=40, disturb=math.radians(0.6))),
    ("弯道 R=5m", dict(y0=0.0, psi0=0.0, seconds=40, curv=0.2)),
)


def report(name, cfg):
    print(f"--- {name}")
    for label, kw in CASES:
        r = run(cfg, **kw)
        print(f"    {label:18s} rms横偏={r['rms_cm']:5.2f}cm 最大={r['max_cm']:6.2f}cm "
              f"rms航向={r['rms_deg']:5.2f}° 最大={r['max_deg']:6.2f}° "
              f">5°占比={r['gt5'] * 100:3.0f}% 丢线={r['lost'] * 100:3.0f}%")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--old-tuning", action="store_true",
                    help="只把跟线参数改回 0.12rad/s / 2.5s / 死区0.018 对比")
    args = ap.parse_args()
    if args.old_tuning:
        report("旧参数 0.12rad/s / 2.5s / 死区0.018",
               replace(CFG, lane_loop_update_threshold=0.12, lane_loop_min_update_s=2.5,
                       lane_cross_deadband=0.018, lane_filter_alpha=0.25,
                       lane_yaw_accel_limit=0.12))
    report(f"当前 config.py（重发 {CFG.lane_loop_update_threshold}rad/s / "
           f"{CFG.lane_loop_min_update_s}s）", CFG)
