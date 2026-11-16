"""盲道行为：数据守卫、无盲道直行、测试锁停和方案 4 纠偏。"""
import math

from actions import Actions
from decisions.safety import depth_valid, lane_fresh
from messages import CommandResult, LaneState, Perception, Pose
from decisions.types import PendingAction, StepResult


class LaneFollower:
    """稳定循环纠偏：持续计算PD目标，达到阈值才取消并重发循环序列。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.actions = Actions(cfg)
        self._pending: PendingAction | None = None
        self._center = None
        self._heading = 0.0
        self._filtered_error = None
        self._prev_error = None
        self._prev_ts = None
        self._last_vyaw = 0.0
        self._last_stop_ts = -1e9
        self._last_log_ts = -1e9
        self._one_line_side = None
        self._cmd_seq = 0
        self._trial_started = None
        self._trial_finished = False
        self._active_vyaw = 0.0
        self._pending_replan_since = None
        self._no_line_mode = False
        self._no_line_since = None

    @property
    def trial_finished(self):
        return self._trial_finished

    @property
    def has_pending(self) -> bool:
        return self._pending is not None

    @property
    def active(self) -> bool:
        return True

    def reset(self):
        """切到避障时清空控制器历史，防止恢复后沿用旧转向量。"""
        self._pending = None
        self._center = None
        self._heading = 0.0
        self._filtered_error = None
        self._prev_error = None
        self._prev_ts = None
        self._last_vyaw = 0.0
        self._one_line_side = None
        self._active_vyaw = 0.0
        self._pending_replan_since = None
        self._no_line_mode = False
        self._no_line_since = None

    def _straight_without_line(self, now, result, interrupt=False):
        """图像健康但没有盲道：零横移、零转向，沿当前朝向前进。"""
        if self._pending is not None:
            return self._service(now, result)
        vyaw = 0.0
        if self.cfg.lane_reacquire_enabled and self._center is not None:
            if self._no_line_since is None:
                self._no_line_since = now
            if now - self._no_line_since <= self.cfg.lane_reacquire_seconds:
                error = self._center - float(self.cfg.lane_center_ref)
                if abs(error) > self.cfg.lane_cross_deadband:
                    vyaw = -math.copysign(
                        self.cfg.lane_reacquire_yaw_rate, error
                    )
        else:
            self._no_line_since = None

        self._cmd_seq += 1
        request = self.actions.lane(
            self.cfg.lane_forward_vx, vyaw, self._cmd_seq,
            purpose="无盲道搜索" if vyaw else "无盲道直行",
        )
        cmd, timeout = request.cmd, request.timeout_s
        self._active_vyaw = vyaw
        self._pending = PendingAction(kind="velocity", tag=cmd.tag,
                                      started_ts=now, timeout_s=timeout)
        return StepResult(
            "LANE_NO_LINE_STRAIGHT",
            request,
            interrupt=interrupt,
            log=(f"未检出盲道，搜索 vx={self.cfg.lane_forward_vx:.2f}m/s "
                 f"vyaw={vyaw:+.3f}" if vyaw else
                 f"图像正常但未检出盲道，直行 vx={self.cfg.lane_forward_vx:.2f}m/s"),
        )

    def _maybe_log(self, now, message):
        if now - self._last_log_ts >= 1.0:
            self._last_log_ts = now
            return message
        return None

    def _stop(self, now, interrupt=False, log=None) -> StepResult:
        self._pending = None
        self._last_vyaw = 0.0
        self._pending_replan_since = None
        self._active_vyaw = 0.0
        if interrupt or (now - self._last_stop_ts) >= self.cfg.stop_refresh_s:
            self._last_stop_ts = now
            return StepResult(
                "LANE_STOP", self.actions.stop(), interrupt=interrupt, log=log
            )
        return StepResult("LANE_STOP", None, log=log)

    def _service(self, now, result) -> StepResult | None:
        p = self._pending
        if p is None:
            return None
        if result is not None and result.tag == p.tag:
            self._pending = None
            if result.status != "ok":
                return self._stop(
                    now, interrupt=True,
                    log=f"盲道速度指令失败 {result.status}: {result.message}",
                )
            return StepResult("LANE_UPDATE")
        if p.timeout_s > 0 and now - p.started_ts >= p.timeout_s:
            self._pending = None
            return self._stop(now, interrupt=True, log="盲道速度指令超时")
        return StepResult("LANE_WAIT")

    def _control(self, now, center, heading, confidence):
        """返回平滑限幅后的 yaw 角速度（rad/s）。"""
        center_ref = float(getattr(self.cfg, "lane_center_ref", 0.5))
        cross_error = center - center_ref

        # 低置信度帧不参与纠偏；由外层 freshness/置信度门槛决定停车。
        heading_weight = float(self.cfg.lane_heading_weight)
        error = cross_error + heading_weight * (heading / 30.0)

        alpha = float(self.cfg.lane_filter_alpha)
        if self._filtered_error is None:
            filtered = error
        else:
            filtered = alpha * error + (1.0 - alpha) * self._filtered_error
        self._filtered_error = filtered

        dt = 0.0 if self._prev_ts is None else max(1e-3, min(0.25, now - self._prev_ts))
        derivative = 0.0
        if self._prev_error is not None and dt > 0:
            derivative = (filtered - self._prev_error) / dt
        self._prev_error = filtered
        self._prev_ts = now

        deadband = float(self.cfg.lane_cross_deadband)
        if abs(filtered) <= deadband:
            proportional = 0.0
        else:
            proportional = math.copysign(abs(filtered) - deadband, filtered)

        target = (
            float(self.cfg.lane_kp) * proportional
            + float(self.cfg.lane_kd) * derivative
        )
        target *= float(self.cfg.lane_steer_sign)
        limit = float(self.cfg.lane_max_yaw_rate)
        target = max(-limit, min(limit, target))

        # 角速度变化率限制，避免相邻控制段突然反向。
        max_delta = float(self.cfg.lane_yaw_accel_limit) * max(
            dt, float(self.cfg.lane_control_duration_s)
        )
        delta = max(-max_delta, min(max_delta, target - self._last_vyaw))
        output = self._last_vyaw + delta
        self._last_vyaw = output
        return output, cross_error

    def step(self, now: float, lane: LaneState | None,
             perception: Perception | None, perception_ts: float,
             pose: Pose | None, result: CommandResult | None,
             executor_busy: bool) -> StepResult:
        fresh = lane_fresh(self.cfg, lane, now)
        if not fresh:
            return self._stop(
                now, interrupt=self._pending is not None,
                log=self._maybe_log(now, "盲道相机无数据/过期，停车"),
            )

        if not lane.visible and not self.cfg.lane_no_line_straight:
            return self._stop(
                now, interrupt=self._pending is not None,
                log=self._maybe_log(now, "未检出盲道，按配置停车"),
            )

        if lane.visible:
            self._no_line_since = None
        if lane.visible and (
            not all(math.isfinite(v) for v in (lane.center, lane.heading_deg, lane.confidence))
            or lane.confidence < float(self.cfg.lane_min_confidence)
        ):
            return self._stop(
                now, interrupt=self._pending is not None,
                log=self._maybe_log(
                    now, f"盲道置信度不足 {lane.confidence:.2f}，停车"
                ),
            )

        if not depth_valid(self.cfg, perception, perception_ts, now):
            return self._stop(
                now, interrupt=self._pending is not None,
                log=self._maybe_log(now, "盲道模式深度失效，停车"),
            )
        if perception.distance_m < self.cfg.danger_m:
            return self._stop(
                now, interrupt=self._pending is not None,
                log=self._maybe_log(
                    now, f"盲道模式前方 {perception.distance_m:.2f}m，停车"
                ),
            )

        # 计时的测试模式：超过最大时间后，持续停车锁定，不因状态机 reset 重置。
        if self._trial_finished:
            return self._stop(now, interrupt=self._pending is not None,
                              log=self._maybe_log(now, "测试时间结束，停车锁定；重启程序才能再次测试"))
        if self._trial_started is None:
            self._trial_started = now
        trial_s = float(getattr(self.cfg, "lane_trial_seconds", 0.0))
        if trial_s > 0 and now - self._trial_started >= trial_s:
            self._trial_finished = True
            return self._stop(now, interrupt=self._pending is not None,
                              log=f"达到限时 {trial_s:.1f}s，执行中断停车并锁定")
        # 模式转换必须立即清除旧转向，不能等loop的2.5秒/0.12阈值。
        # 先消费当前动作的失败或超时，不能通过切模式吞掉失败结果。
        no_line = not lane.visible
        mode_interrupt = False
        if no_line != self._no_line_mode:
            if self._pending is not None:
                handled = self._service(now, result)
                if handled is not None and handled.state == "LANE_STOP":
                    return handled
            mode_interrupt = self._pending is not None
            last_center = (self._center
                           if self.cfg.lane_reacquire_enabled else None)
            self.reset()  # 保留测试计时/锁定，清除旧滤波和转向历史。
            # 丢线后保留最后一条线中心，供 _straight_without_line 朝线侧搜索。
            self._center = last_center
            self._no_line_mode = no_line
        if no_line:
            return self._straight_without_line(now, result, interrupt=mode_interrupt)

        if self._pending is not None:
            handled = self._service(now, result)
            if handled is not None and handled.state != "LANE_WAIT":
                return handled
            # 同一条RPC循环行走；仅显著且持续的偏差才取消重发。

        # 单线模式下 center 就是黄线本身；双线/单边兼容仍使用检测中心。
        target_center = lane.center
        target_heading = lane.heading_deg

        # 对感知量做限跳 + EMA，降低光照、反光和聚类变化造成的摆动。
        if self._center is None:
            self._center = target_center
            self._heading = target_heading
        else:
            dc = target_center - self._center
            max_c = float(self.cfg.lane_max_center_jump)
            target_center = self._center + max(-max_c, min(max_c, dc))
            self._center = 0.75 * self._center + 0.25 * target_center

            dh = target_heading - self._heading
            max_h = float(self.cfg.lane_max_heading_jump_deg)
            target_heading = self._heading + max(-max_h, min(max_h, dh))
            self._heading = 0.75 * self._heading + 0.25 * target_heading

        vyaw, cross_error = self._control(
            now, self._center, self._heading, lane.confidence
        )
        vx = float(self.cfg.lane_forward_vx)
        if self._pending is not None:
            diff = abs(vyaw - self._active_vyaw)
            threshold = float(self.cfg.lane_loop_update_threshold)
            if diff < threshold:
                self._pending_replan_since = None
                return StepResult("LANE_WAIT")
            if self._pending_replan_since is None:
                self._pending_replan_since = now
                return StepResult("LANE_WAIT")
            if (now - self._pending_replan_since < 0.25 or
                now - self._pending.started_ts < self.cfg.lane_loop_min_update_s):
                return StepResult("LANE_WAIT")
            # 重新下发一次 loop 计划，必须通过单一执行器先取消旧流。
            replace_running = True
        else:
            replace_running = False
        self._cmd_seq += 1
        request = self.actions.lane(vx, vyaw, self._cmd_seq)
        cmd, timeout = request.cmd, request.timeout_s
        self._active_vyaw = vyaw
        self._pending_replan_since = None
        self._pending = PendingAction(
            kind="velocity",
            tag=cmd.tag,
            started_ts=now,
            timeout_s=timeout,
        )
        return StepResult(
            "LANE_VELOCITY",
            request,
            interrupt=replace_running or mode_interrupt,
            log=self._maybe_log(
                now,
                f"盲道速度 vx={vx:.2f} vyaw={vyaw:+.3f}rad/s "
                f"center={self._center:.3f} err={cross_error:+.3f} "
                f"heading={self._heading:+.1f}° conf={lane.confidence:.2f}",
            ),
        )


