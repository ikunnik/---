"""整段距离运动、位姿与预算守卫；所有重规划保留同一条原路线。"""
import math

from actions import MAX_DISTANCE
from dataclasses import replace

from decisions.types import ADVANCE, CHECK, HOLD, RETURN, TURN, PendingAction, StepResult


class NavigationMixin:
    def _reset_navigation(self):
        self.bypass_started_ts = None
        self.bypass_side = None
        self.alternate_side_available = False
        self.probing_side = False
        self.replans = 0
        self.diagonal_remaining = 0.0
        self.parallel_remaining = 0.0
        self.bypass_lateral_target = 0.0
        self.bypass_lateral_sign = 0.0
        self.lateral_retries = 0
        self.replan_after_realign = False
        self.return_blocked = False
        self.blocked_latched = False
        self.last_history_frame_ts = None
        self.return_clear_origin = None

    # ------------------------------------------------------------ 侧向偏移几何
    def _diagonal_from_lateral(self, lateral: float,
                               angle_deg: float | None = None) -> float:
        """由横向偏移目标反算斜向路径长度：lateral / sin(转角)。"""
        angle = float(self.cfg.bypass_angle_deg if angle_deg is None else angle_deg)
        sin_a = max(0.1, math.sin(math.radians(angle)))
        return max(float(self.cfg.bypass_step_min_m), float(lateral) / sin_a)

    def _bypass_lateral_target(self, perception, side: str) -> float:
        """绕行需要达到的横向偏移：覆盖机身、障碍与净空，并受该侧实测空间约束。

        旧实现把斜向长度固定为 0.7m，横向只有 0.495m；对 0.5m 宽的障碍，
        扣掉机身半宽后余量不足 0.07m，只能贴着擦过去，障碍再宽就会撞上。
        """
        cfg = self.cfg
        needed = (float(cfg.body_half_width_m)
                  + float(cfg.obstacle_half_width_m)
                  + float(cfg.bypass_side_clearance_m))
        if perception is not None and perception.near_ratio >= cfg.wall_near_ratio:
            needed += float(cfg.wall_extra_m)
        floor = float(cfg.bypass_lateral_min_m)
        if perception is None:
            return max(needed, floor)
        space = perception.left_m if side == "left" else perception.right_m
        if not math.isfinite(space):
            return max(needed, floor)
        usable = float(space) - float(cfg.body_half_width_m) - float(cfg.bypass_side_clearance_m)
        if usable <= floor:
            # 侧向空间只够勉强通过：按实测空间给，不再加码，避免越到另一侧。
            return max(float(cfg.bypass_step_min_m), usable)
        return min(needed, usable)

    def _lateral_shortfall(self, pose) -> float | None:
        """实测横向偏移与目标的差额；换不到位姿或方向不符时返回 None。"""
        target = float(getattr(self, "bypass_lateral_target", 0.0) or 0.0)
        sign = float(getattr(self, "bypass_lateral_sign", 0.0) or 0.0)
        if target <= 0.0 or sign == 0.0:
            return None
        lateral = self._path_lateral(pose)
        if lateral is None or lateral * sign < 0:
            return None
        return max(0.0, target - abs(lateral))

    def _pose_valid(self, pose, now):
        return (pose is not None
                and all(math.isfinite(v) for v in (pose.ts, pose.yaw_rad, pose.x, pose.y))
                and pose.ts > 0 and 0 <= now - pose.ts <= self.cfg.bypass_pose_stale_s)

    def _safe_step(self, perception, wanted):
        if (perception is None or not perception.valid
                or not math.isfinite(perception.distance_m)):
            return 0.0
        reserve = max(self.cfg.danger_m, self.cfg.emergency_m) + self.cfg.safety_buffer_m
        # 各阶段一次执行当前目标；保留前向净空和底层单次距离上限。
        return max(0.0, min(wanted, MAX_DISTANCE, perception.distance_m - reserve))

    def _rotation_clear(self, perception):
        # 缺少侧区数据不能证明扫过的空间安全。
        return (perception is not None and perception.valid
                and perception.left_valid and perception.right_valid
                and all(math.isfinite(v) and v >= self.cfg.rotate_clear_m
                        for v in (perception.distance_m, perception.left_m,
                                  perception.right_m, perception.min_m)))

    def _start_segment(self, ctx, wanted, stage, *, returning=False):
        if not ctx.valid or not self._pose_valid(ctx.pose, ctx.now):
            return self._block(ctx.now, "距离运动缺少新鲜感知/位置", True, "no_pose")
        step = self._safe_step(ctx.perception, wanted)
        minimum = max(self.cfg.bypass_step_min_m,
                      self.cfg.path_return_min_step_m if returning else 0.0)
        if step < minimum - 1e-6:
            if returning:
                return self._defer_return(ctx)
            return self._replan(ctx, "当前方向净空不足，重新规划")
        if (self.avoid_cycles >= self.cfg.max_avoid_cycles
                or self.bypass_distance_m + step > self.cfg.max_bypass_distance_m):
            return self._block(ctx.now, "绕障/回线累计路程或动作次数达到上限", True, "max_cycles")
        note = "回原路线" if returning else "斜向绕行" if stage == "diagonal" else "沿原方向绕出"
        request = self.actions.advance(step, note)
        self.pending = PendingAction(
            kind="return_advance" if returning else "advance",
            tag=request.cmd.tag, started_ts=ctx.now, timeout_s=request.timeout_s,
            distance_m=step, start_x=ctx.pose.x, start_y=ctx.pose.y,
            start_yaw_rad=ctx.pose.yaw_rad,
        )
        self.avoid_cycles += 1
        self._bypass_stage = stage
        self.phase = RETURN if returning else ADVANCE
        return StepResult(self.phase, request,
                          log=f"{note} {step:.2f}m；累计实测 {self.bypass_distance_m:.2f}m")

    def _record_progress(self, ctx, p, *, completed):
        if not self._pose_valid(ctx.pose, ctx.now):
            return self._block(ctx.now, "运动中位置失效", True, "no_pose")
        dx, dy = ctx.pose.x - p.start_x, ctx.pose.y - p.start_y
        moved = math.hypot(dx, dy)
        forward = dx * math.cos(p.start_yaw_rad) + dy * math.sin(p.start_yaw_rad)
        if not math.isfinite(moved):
            return self._block(ctx.now, "无法测量绕障位移", True, "no_pose")
        if completed and forward < p.distance_m * self.cfg.bypass_min_progress_ratio:
            return self._block(ctx.now, "动作报告成功但实际前进不足", True, "advance_failed")
        if moved > p.distance_m + max(0.20, self.cfg.safety_buffer_m * 2):
            return self._block(ctx.now, "位置跳变或运动明显超出目标", True, "advance_failed")
        self.bypass_distance_m += moved
        if self.bypass_distance_m > self.cfg.max_bypass_distance_m:
            return self._block(ctx.now, "累计实测路程超过绕障预算", True, "max_cycles")
        if p.kind == "advance":
            if self._bypass_stage == "diagonal":
                self.diagonal_remaining = max(0.0, self.diagonal_remaining - forward)
            else:
                self.parallel_remaining = max(0.0, self.parallel_remaining - forward)
        if moved >= self.cfg.bypass_step_min_m:
            self.tried_angles.clear()  # 换了观察位置，旧方向失败记录不再适用。
        return None

    def _stop_for_replan(self, ctx, p):
        # 取消后先等待停车成功，不能把旧动作的 cancelled 当作新计划的失败。
        failed = self._record_progress(ctx, p, completed=False)
        if failed is not None:
            return failed
        self.return_clear_origin = None
        self.return_blocked = p.kind == "return_advance"
        request = self.actions.stop(timeout_s=self.cfg.controller_switch_timeout_s)
        request = replace(request, cmd=replace(request.cmd, tag=f"avoid_stop_{self.avoid_cycles}"))
        self.pending = PendingAction(
            "avoid_stop", request.cmd.tag, ctx.now, request.timeout_s,
            start_x=ctx.pose.x, start_y=ctx.pose.y, start_yaw_rad=ctx.pose.yaw_rad,
        )
        self.phase = CHECK
        return StepResult(CHECK, request, interrupt=True, log="路径出现障碍，停车后重新观察")

    def _escalate_bypass_angle(self, ctx):
        """斜向起点净空不足：同侧加大转角再看一次，而不是立刻放弃这一侧。

        障碍近、张角大时（0.45m 牌在 0.7m 处约 35°），固定 40° 转完后中心锥体
        仍被它的边缘切到，_safe_step 永远算出 0；旧逻辑直接 realign 并把该侧写进
        tried_angles，两侧都试完就锁定停车——明明另一侧是空的。
        """
        cfg = self.cfg
        if self._bypass_stage != "diagonal" or self.bypass_distance_m > 0.0:
            return None
        if self.bypass_lateral_sign == 0.0 or not self._pose_valid(ctx.pose, ctx.now):
            return None
        sign = 1.0 if self.bypass_lateral_sign > 0 else -1.0
        step = sign * float(cfg.bypass_angle_step_deg)
        angle = abs(self.heading_dev_deg)
        total = angle + abs(step)
        if abs(step) < 1e-6 or total > float(cfg.bypass_angle_max_deg):
            return None
        if abs(self.heading_dev_deg + step) > float(cfg.max_heading_dev):
            return None
        # 横向目标不变，转角越大斜向段越短（前进量变小，离障碍前角更远）。
        lateral = self.diagonal_remaining * math.sin(math.radians(angle))
        request = self.actions.turn(
            step, purpose=f"斜向净空不足，同侧追加 {abs(step):.0f}°"
        )
        self.pending = PendingAction(
            kind="turn",
            tag=request.cmd.tag,
            started_ts=ctx.now,
            timeout_s=cfg.turn_timeout_s,
            expected_yaw_delta_deg=step,
            start_yaw_rad=ctx.pose.yaw_rad,
        )
        self.diagonal_remaining = self._diagonal_from_lateral(lateral, total)
        self.phase = TURN
        return StepResult(TURN, request, log=(
            f"斜向净空只有 {ctx.perception.distance_m:.2f}m，同侧追加 "
            f"{abs(step):.0f}°（累计 {total:.0f}°）"
        ))

    def _replan(self, ctx, reason):
        self.return_clear_origin = None
        escalated = self._escalate_bypass_angle(ctx)
        if escalated is not None:
            return escalated
        if self.replans >= self.cfg.bypass_max_replans:
            return self._block(ctx.now, "绕障重规划次数达到上限", True, "max_cycles")
        self.replans += 1
        if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            if self._bypass_stage == "diagonal" and self.diagonal_remaining > 0:
                self.tried_angles.add(1 if self.heading_dev_deg > 0 else -1)
            self.replan_after_realign = True
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        self.phase = HOLD
        self.history.clear()
        self.last_history_frame_ts = None
        return self._stop_result(HOLD, ctx.now, log=reason)

    def _defer_return(self, ctx):
        self.return_clear_origin = None
        self.return_blocked = True
        self._bypass_stage = "parallel"
        # 回切被遮挡，先回到原航向，沿障碍外侧继续观察。
        if abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        self.return_blocked = False
        self.parallel_remaining = self.cfg.min_bypass_advance_m
        if self._safe_step(ctx.perception, self.parallel_remaining) < self.cfg.bypass_step_min_m:
            return self._replan(ctx, "回切及前方都受阻，重新选择绕行侧")
        return self._start_segment(ctx, self.parallel_remaining, "parallel")
