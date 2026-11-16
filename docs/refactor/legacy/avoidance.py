"""
避障决策核心：纯状态机，不碰线程、网络、时间。

设计目标：
    1. 动作失败/取消/超时，绝不继续前进
    2. 避障后回到“本次避障开始前”的航向 entry_yaw（允许横向偏移）
    3. 动态障碍只停车等待，不绕行
    4. 左右都能绕，候选角度 ±30°/±60°，累计偏离不超过 ±90°
    5. 单帧多区域剖面：center / left / right / min / near_ratio

状态：
    GO ──危险/失效──► HOLD ──稳定障碍──► PLAN ──► TURN ──► CHECK
     ▲                  │                                   │
     │                  │ 安全                               │ 步长足够
     │                  ▼                                   ▼
     └──────────────── GO ◄──clear── CHECK ◄── REALIGN ◄── ADVANCE
                                         ▲
                                         └── 回正后仍被挡 ──► PLAN
    任何失败/超时/航向校验失败 ──► BLOCKED
    语音播报期间 ──► ANNOUNCE（停车等待 VoiceDone）

外部（DecisionEngine）只做三件事：
    - 读感知、姿态、执行器结果
    - 调 step()
    - 把 step() 返回的 ActionRequest 交给执行器
"""
import math
from collections import deque

from actions import Actions
from decisions.safety import depth_fresh
from messages import ActionRequest, CommandResult, Perception, Pose, VoiceCue
from decisions.rules import (
    compute_step,
    estimate_step_m,
    fmt_m,
    make_candidates,
    wrap_pi,
)
from decisions.types import (
    ACTION_STATES,
    ADVANCE,
    ANNOUNCE,
    BLOCKED,
    CHECK,
    GO,
    HOLD,
    PLAN,
    REALIGN,
    RETURN,
    TURN,
    Candidate,
    DeferredAction,
    PendingAction,
    StepContext,
    StepResult,
)
from decisions.voice import TIMEOUT, WAITING, VoiceGate

__all__ = [
    "GO", "HOLD", "PLAN", "TURN", "CHECK", "ADVANCE", "REALIGN",
    "BLOCKED", "ANNOUNCE", "RETURN", "ACTION_STATES",
    "Candidate", "ActionRequest", "StepResult",
    "wrap_pi", "estimate_step_m", "compute_step", "make_candidates",
    "AvoidStateMachine",
]


class AvoidStateMachine:
    """有状态的避障决策机。所有时间都由外部传入，方便离线测试。"""

    def __init__(self, cfg, wait_for_voice: bool = False):
        self.cfg = cfg
        self.actions = Actions(cfg)
        self.wait_for_voice = wait_for_voice
        self.voice = VoiceGate(cfg, wait_for_voice)

        self.phase = HOLD
        self.entry_yaw_rad: float | None = None
        self.entry_x = float("nan")
        self.entry_y = float("nan")
        self.path_return_attempts = 0
        self.path_return_step_m = 0.0
        self.heading_dev_deg = 0.0
        self.bypass_active = False
        self.bypass_distance_m = 0.0
        self.avoid_cycles = 0
        self.realign_attempts = 0
        self.tried_angles: set[int] = set()
        self.pending: PendingAction | None = None
        self.settle_until = 0.0
        self.action_finished_ts = 0.0
        self.check_mode = ""
        self.last_stop_cmd_ts = -1e9
        self.last_dynamic_warn_ts = -1e9
        self.last_pose_warn_ts = -1e9
        self.blocked_since = 0.0
        self.history = deque()
        self.started_at = 0.0

    # ------------------------------------------------------------------
    # 给 DecisionEngine 用的只读属性
    # ------------------------------------------------------------------
    @property
    def has_pending_action(self) -> bool:
        return self.pending is not None

    @property
    def voice_pending(self) -> bool:
        """兼容旧接口：当前是否正在停车等语音播完。"""
        return self.voice.pending

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def reset(self, now: float, keep_voice: bool = False):
        self.phase = HOLD
        self.entry_yaw_rad = None
        self.entry_x = float("nan")
        self.entry_y = float("nan")
        self.path_return_attempts = 0
        self.path_return_step_m = 0.0
        self.heading_dev_deg = 0.0
        self.bypass_active = False
        self.bypass_distance_m = 0.0
        self.avoid_cycles = 0
        self.realign_attempts = 0
        self.tried_angles.clear()
        self.pending = None
        self.settle_until = 0.0
        self.action_finished_ts = 0.0
        self.check_mode = ""
        self.last_stop_cmd_ts = -1e9
        self.last_dynamic_warn_ts = -1e9
        self.last_pose_warn_ts = -1e9
        self.blocked_since = 0.0
        self.history.clear()
        self.started_at = now
        if not keep_voice:
            self.voice.reset()

    def step(self, now: float, perception: Perception | None, perception_ts: float,
             pose: Pose | None, result: CommandResult | None,
             executor_busy: bool, voice_done: bool = False) -> StepResult:
        # 只要 entry_yaw 还在，就持续从姿态更新当前偏离角
        if self.entry_yaw_rad is not None and pose is not None:
            self.heading_dev_deg = self._heading_dev(pose.yaw_rad)

        # 语音播报期间：不执行任何运动指令。只有收到本轮播报的完成回执，
        # 或等待超时（语音进程异常）后，才继续后面的状态。
        voice_state, deferred = self.voice.poll(now, voice_done)
        if voice_state == WAITING:
            return self._stop_result(ANNOUNCE, now)
        if deferred is not None:
            return self._apply_deferred(deferred, now)
        if voice_state == TIMEOUT:
            return self._stop_result(
                ANNOUNCE, now, log="语音等待超时，继续执行"
            )

        ctx = self._context(
            now, perception, perception_ts, pose, result, executor_busy, voice_done
        )

        # 动作执行期间：只等结果/超时，不同时做别的事
        if self.pending is not None:
            return self._service_pending(ctx)

        if self.phase == GO:
            return self._step_go(ctx)
        if self.phase == HOLD:
            return self._step_hold(ctx)
        if self.phase == PLAN:
            if not ctx.valid:
                return self._enter_hold(
                    ctx.now, "规划前感知失效", interrupt=False, keep_entry=True
                )
            return self._step_plan(ctx)
        if self.phase in (CHECK, RETURN):
            return self._step_check(ctx)
        if self.phase == BLOCKED:
            return self._step_blocked(ctx)

        # TURN/ADVANCE/REALIGN 如果没有 pending，说明状态坏了，保守停车
        return self._block(
            ctx.now, f"{self.phase} 没有待执行动作", interrupt=True, reason="internal"
        )

    def _context(self, now, perception, perception_ts, pose, result,
                 executor_busy, voice_done) -> StepContext:
        fresh = depth_fresh(self.cfg, perception, perception_ts, now)
        valid = fresh and perception.valid
        return StepContext(
            now=now,
            perception=perception,
            perception_ts=perception_ts,
            pose=pose,
            result=result,
            executor_busy=executor_busy,
            fresh=fresh,
            valid=valid,
            voice_done=voice_done,
        )

    # ------------------------------------------------------------------
    # GO：安全前进；需要回正时先回正
    # ------------------------------------------------------------------
    def _step_go(self, ctx: StepContext) -> StepResult:
        if not ctx.valid:
            why = "还没有感知数据" if ctx.perception is None else "感知失效/超时"
            return self._enter_hold(
                ctx.now, why, interrupt=True, keep_entry=False
            )

        if ctx.perception.distance_m < self.cfg.danger_m:
            return self._enter_hold(
                ctx.now,
                f"障碍物太近 {ctx.perception.distance_m:.2f}m",
                interrupt=True,
                keep_entry=False,
            )

        # 万一在 GO 里还带着未回正的航向（例如从 HOLD 恢复），先回正
        if self.bypass_active and abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg:
            if ctx.pose is None:
                return self._enter_hold(
                    ctx.now, "需要回正但没有姿态数据",
                    interrupt=True, keep_entry=True,
                    voice=self.voice.announce(
                        VoiceCue("wait", reason="no_pose_wait"), ctx.now
                    ),
                )
            if ctx.perception.min_m >= self.cfg.rotate_clear_m:
                return self._start_realign(ctx.now, ctx.perception, ctx.pose)
            return self._enter_hold(
                ctx.now, "需要回正但净空不足",
                interrupt=True, keep_entry=True,
                voice=self.voice.announce(
                    VoiceCue("wait", reason="realign_space_wait"), ctx.now
                ),
            )

        if ctx.executor_busy:
            return StepResult(GO)

        return StepResult(GO, self.actions.forward())

    # ------------------------------------------------------------------
    # HOLD：记录 entry_yaw、等待障碍稳定、动态障碍等待、恢复 GO
    # ------------------------------------------------------------------
    def _step_hold(self, ctx: StepContext) -> StepResult:
        if not ctx.valid:
            self.history.clear()
            return self._stop_result(HOLD, ctx.now)

        self._history_append(ctx.now, ctx.perception)

        captured = self._hold_capture_entry_yaw(ctx)
        if captured is not None:
            return captured

        if ctx.perception.distance_m >= self.cfg.clear_m:
            return self._hold_when_clear(ctx)
        if ctx.perception.distance_m < self.cfg.danger_m:
            return self._hold_when_dangerous(ctx)
        return self._hold_in_buffer_zone(ctx)

    def _hold_capture_entry_yaw(self, ctx: StepContext) -> StepResult | None:
        """启动时 0.8~1.0m 就挡路：先记录 entry_yaw，不要永远停着等。"""
        if not (
            ctx.perception.distance_m < self.cfg.clear_m
            and self.entry_yaw_rad is None
        ):
            return None

        if ctx.pose is None:
            return self._stop_result(
                HOLD, ctx.now, self._warn(ctx.now, "没有姿态数据，暂不绕障"),
                voice=self.voice.announce(
                    VoiceCue("wait", reason="no_pose_wait"), ctx.now
                ),
            )

        self.entry_yaw_rad = ctx.pose.yaw_rad
        self.entry_x = float(getattr(ctx.pose, "x", float("nan")))
        self.entry_y = float(getattr(ctx.pose, "y", float("nan")))
        self.path_return_attempts = 0
        self.heading_dev_deg = 0.0
        self.bypass_active = True
        self.bypass_distance_m = 0.0
        self.avoid_cycles = 0
        self.realign_attempts = 0
        self.tried_angles.clear()
        return self._stop_result(
            HOLD,
            ctx.now,
            f"记录避障前航向 {math.degrees(ctx.pose.yaw_rad):.1f}°，等待障碍稳定",
        )

    def _hold_when_clear(self, ctx: StepContext) -> StepResult:
        """前方 >= clear_m：可能需要回正，安全后恢复 GO。"""
        if (
            self.entry_yaw_rad is not None
            and abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg
        ):
            if (
                self._stable_clear(ctx.now)
                and ctx.perception.min_m >= self.cfg.rotate_clear_m
            ):
                if ctx.pose is None:
                    return self._stop_result(
                        HOLD, ctx.now,
                        self._warn(ctx.now, "没有姿态数据，无法回正"),
                        voice=self.voice.announce(
                            VoiceCue("wait", reason="no_pose_wait"), ctx.now
                        ),
                    )
                return self._start_realign(
                    ctx.now, ctx.perception, ctx.pose
                )

            if self._stable_clear(ctx.now):
                return self._stop_result(
                    HOLD, ctx.now,
                    voice=self.voice.announce(
                        VoiceCue("wait", reason="realign_space_wait"), ctx.now
                    ),
                )
            return self._stop_result(HOLD, ctx.now)

        if self._stable_clear(ctx.now):
            path_step = self._maybe_start_path_return(ctx)
            if path_step is not None:
                return path_step
            return self._complete_bypass(
                ctx, stop_state=HOLD, log="路径安全，恢复前进"
            )
        return self._stop_result(HOLD, ctx.now)

    def _hold_when_dangerous(self, ctx: StepContext) -> StepResult:
        """中心 < danger_m：动态等待，稳定且安全后进入 PLAN。"""
        if self._is_approaching(ctx.now):
            return self._dynamic_wait(ctx)

        if not self._stable_obstacle(ctx.now):
            return self._stop_result(HOLD, ctx.now)

        if self.entry_yaw_rad is None:
            return self._stop_result(
                HOLD, ctx.now, self._warn(ctx.now, "没有姿态数据，暂不绕障"),
                voice=self.voice.announce(
                    VoiceCue("wait", reason="no_pose_wait"), ctx.now
                ),
            )

        if ctx.perception.min_m < self.cfg.rotate_clear_m:
            return self._block(
                ctx.now,
                f"离障碍太近，无法原地转身 min={ctx.perception.min_m:.2f}m",
                interrupt=True,
                reason="too_close",
            )

        if self.avoid_cycles >= self.cfg.max_avoid_cycles:
            return self._block(
                ctx.now, "绕障次数达到上限", interrupt=True, reason="max_cycles"
            )

        self.phase = PLAN
        return StepResult(PLAN, log="障碍稳定，开始规划绕行")

    def _hold_in_buffer_zone(self, ctx: StepContext) -> StepResult:
        """danger_m <= 中心 < clear_m：避障未结束时继续规划，否则等待。"""
        if not (self.bypass_active and self.entry_yaw_rad is not None):
            return self._stop_result(HOLD, ctx.now)

        if self._is_approaching(ctx.now):
            return self._dynamic_wait(ctx)

        if self._stable_obstacle(ctx.now):
            if self.avoid_cycles >= self.cfg.max_avoid_cycles:
                return self._block(
                    ctx.now, "绕障次数达到上限", interrupt=True,
                    reason="max_cycles",
                )
            self.phase = PLAN
            return StepResult(PLAN, log="仍在障碍附近，继续规划")

        return self._stop_result(HOLD, ctx.now)

    def _dynamic_wait(self, ctx: StepContext) -> StepResult:
        return self._stop_result(
            HOLD, ctx.now,
            self._warn(ctx.now, "障碍物正在接近，停车等待"),
            voice=self.voice.announce(VoiceCue("dynamic"), ctx.now),
        )

    def _complete_bypass(self, ctx: StepContext, stop_state: str,
                         log: str) -> StepResult:
        """绕行完成：先播完成提示并停车，收到回执后再恢复 GO。"""
        done_voice = None
        if (
            self.bypass_active
            and self.bypass_distance_m >= self.cfg.min_bypass_advance_m
        ):
            done_voice = self.voice.announce(
                VoiceCue("bypass_done"),
                ctx.now,
                DeferredAction("go", log=log),
            )

        if done_voice is not None and self.voice.pending:
            return self._stop_result(
                stop_state, ctx.now,
                log="播报完成提示，停车等待",
                voice=done_voice,
            )

        self.phase = GO
        self._reset_bypass(keep_entry=False)
        return StepResult(GO, log=log, voice=done_voice)

    # ------------------------------------------------------------------
    # PLAN：选方向并转向
    # ------------------------------------------------------------------
    def _step_plan(self, ctx: StepContext) -> StepResult:
        if ctx.pose is None:
            return self._block(
                ctx.now, "没有姿态数据，无法规划绕行",
                interrupt=True, reason="no_pose",
            )
        if self.entry_yaw_rad is None:
            self.entry_yaw_rad = ctx.pose.yaw_rad
            self.entry_x = float(getattr(ctx.pose, "x", float("nan")))
            self.entry_y = float(getattr(ctx.pose, "y", float("nan")))
            self.path_return_attempts = 0
            self.bypass_active = True
        self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)

        if ctx.perception.min_m < self.cfg.rotate_clear_m:
            return self._block(
                ctx.now,
                f"净空不足，不能转身 min={ctx.perception.min_m:.2f}m",
                interrupt=True,
                reason="rotate_space",
            )

        return self._plan_turn(ctx)

    def _plan_turn(self, ctx: StepContext) -> StepResult:
        candidates = make_candidates(
            self.cfg, ctx.perception, self.heading_dev_deg, self.tried_angles
        )
        if not candidates:
            return self._block(
                ctx.now, "左右都没有足够空间，停车锁定",
                interrupt=True, reason="no_space",
            )

        chosen = candidates[0]
        self.tried_angles.add(round(chosen.absolute_deg))
        if chosen.is_return:
            return self._start_return_turn(ctx, chosen)
        return self._start_announced_turn(ctx, chosen)

    def _build_turn(self, ctx: StepContext, chosen: Candidate):
        """构造转向 cmd / ActionRequest / PendingAction。"""
        delta = chosen.delta_deg
        purpose = (
            f"回退到避障前航向 {delta:+.0f}°"
            if chosen.is_return
            else f"绕障转向 {delta:+.0f}°"
        )
        request = self.actions.turn(delta, purpose=purpose)
        cmd = request.cmd
        pending = PendingAction(
            kind="turn",
            tag=cmd.tag,
            started_ts=ctx.now,
            timeout_s=self.cfg.turn_timeout_s,
            expected_yaw_delta_deg=delta,
            start_yaw_rad=ctx.pose.yaw_rad,
        )
        return delta, request, pending

    def _start_return_turn(self, ctx: StepContext,
                           chosen: Candidate) -> StepResult:
        """所有方向都试过后的回退：不播提示，直接转回 entry 方向。"""
        delta, request, pending = self._build_turn(ctx, chosen)
        log = (
            f"所有可行方向都已试过，回退 {abs(delta):.0f}°"
            f"（当前偏离 {self.heading_dev_deg:+.0f}°）"
        )
        self.pending = pending
        self.phase = TURN
        return StepResult(TURN, request, log=log)

    def _start_announced_turn(self, ctx: StepContext,
                              chosen: Candidate) -> StepResult:
        delta, request, pending = self._build_turn(ctx, chosen)
        log = (
            f"选择{'左' if delta > 0 else '右'}转 {abs(delta):.0f}°"
            f"（左 {fmt_m(ctx.perception.left_m)} / "
            f"右 {fmt_m(ctx.perception.right_m)}，"
            f"near={ctx.perception.near_ratio:.2f}）"
        )
        cue = self.voice.announce(
            VoiceCue("bypass", side=chosen.side, angle_deg=abs(delta)),
            ctx.now,
            DeferredAction("turn", request=request, pending=pending, log=log),
        )
        if cue is not None and self.voice.pending:
            return self._stop_result(
                ANNOUNCE, ctx.now,
                log="播报绕行提示，停车等待",
                voice=cue,
            )

        self.pending = pending
        self.phase = TURN
        return StepResult(TURN, request, log=log, voice=cue)

    # ------------------------------------------------------------------
    # CHECK：动作后等新帧，再决定前进/回正/重规划
    # ------------------------------------------------------------------
    def _step_check(self, ctx: StepContext) -> StepResult:
        if ctx.now < self.settle_until:
            return StepResult(CHECK)

        if not ctx.fresh:
            return self._enter_hold(
                ctx.now, "动作后感知失效", interrupt=False, keep_entry=True
            )

        if ctx.perception_ts < self.action_finished_ts + self.cfg.settle_s:
            return StepResult(CHECK)

        if not ctx.valid:
            return self._enter_hold(
                ctx.now, "动作后深度无效", interrupt=False, keep_entry=True
            )

        if self.entry_yaw_rad is not None:
            if ctx.pose is None:
                return self._block(
                    ctx.now, "没有姿态数据，无法校验航向",
                    interrupt=True, reason="no_pose",
                )
            self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)

        mode = self.check_mode
        step_m = compute_step(ctx.perception, self.cfg)

        if mode == "turn":
            return self._check_after_turn(ctx, step_m)
        if mode == "advance":
            return self._check_after_advance(ctx, step_m)
        if mode == "realign":
            return self._check_after_realign(ctx)
        if mode == "return_turn":
            return self._check_after_return_turn(ctx)
        if mode == "return_advance":
            return self._check_after_return_advance(ctx)

        return self._block(
            ctx.now, f"未知检查模式 {mode}", interrupt=True, reason="internal"
        )

    def _check_after_turn(self, ctx: StepContext, step_m: float) -> StepResult:
        if (
            step_m >= self.cfg.min_step_m
            and self.avoid_cycles < self.cfg.max_avoid_cycles
            and self.bypass_distance_m + step_m <= self.cfg.max_bypass_distance_m
        ):
            return self._start_advance(ctx.now, step_m, ctx.perception)

        if (
            ctx.perception.distance_m >= self.cfg.clear_m
            and ctx.perception.min_m >= self.cfg.rotate_clear_m
            and self.bypass_distance_m >= self.cfg.min_bypass_advance_m
        ):
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)

        # 转过去之后仍然没有前进空间：先回到 entry_yaw，再从正前方重新规划。
        # 左右侧向 ROI 是相对机身的；如果带着右转/左转的偏航直接换边扫描，
        # 会出现“右转失败后立刻左转”的不合理动作，也会让回正逻辑失去意义。
        if (
            self.entry_yaw_rad is not None
            and abs(self.heading_dev_deg) > self.cfg.yaw_tol_deg
        ):
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)

        self.phase = PLAN
        return StepResult(PLAN, log="转向前方空间仍不足，重新规划")

    def _check_after_advance(self, ctx: StepContext, step_m: float) -> StepResult:
        if (
            ctx.perception.distance_m >= self.cfg.clear_m
            and ctx.perception.min_m >= self.cfg.rotate_clear_m
            and self.bypass_distance_m >= self.cfg.min_bypass_advance_m
        ):
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)

        if ctx.perception.distance_m < self.cfg.danger_m:
            if self.avoid_cycles >= self.cfg.max_avoid_cycles:
                return self._block(
                    ctx.now, "绕障前进次数达到上限",
                    interrupt=True, reason="max_cycles",
                )
            self.phase = PLAN
            return StepResult(PLAN, log="前进后仍危险，重新规划")

        if (
            self.bypass_distance_m >= self.cfg.min_bypass_advance_m
            and ctx.perception.min_m >= self.cfg.rotate_clear_m
        ):
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)

        # 还没走够目标距离，但前方仍安全：先补一步，不要马上重新转向。
        # 步长仍由 compute_step() 限制，并且总位移不超过 max_bypass_distance_m。
        if (
            self.bypass_distance_m < self.cfg.min_bypass_advance_m
            and self.avoid_cycles < self.cfg.max_avoid_cycles
            and step_m >= self.cfg.min_step_m
            and self.bypass_distance_m + step_m <= self.cfg.max_bypass_distance_m
        ):
            return self._start_advance(
                ctx.now, step_m, ctx.perception, top_up=True
            )

        self.phase = PLAN
        return StepResult(PLAN, log="重新规划绕行方向")

    def _path_lateral(self, pose: Pose | None) -> float | None:
        """当前点到“绕障前航向直线”的横向偏差；左正右负。"""
        if pose is None or self.entry_yaw_rad is None:
            return None
        ex, ey = self.entry_x, self.entry_y
        if not (math.isfinite(ex) and math.isfinite(ey)
                and math.isfinite(pose.x) and math.isfinite(pose.y)):
            return None
        dx = float(pose.x) - ex
        dy = float(pose.y) - ey
        return -dx * math.sin(self.entry_yaw_rad) + dy * math.cos(self.entry_yaw_rad)

    def _maybe_start_path_return(self, ctx: StepContext) -> StepResult | None:
        """绕障回正后，如果横向偏离原路线，切一个角回到原线。"""
        if not self.cfg.path_return_enabled or self.pending is not None:
            return None
        if self.path_return_attempts >= self.cfg.path_return_max_attempts:
            return None
        lateral = self._path_lateral(ctx.pose)
        if lateral is None or abs(lateral) <= self.cfg.path_return_tolerance_m:
            return None
        if (ctx.perception is None
                or ctx.perception.min_m < self.cfg.rotate_clear_m):
            return None
        if ctx.pose is None:
            return None

        phi = math.radians(max(5.0, min(80.0, float(self.cfg.path_return_angle_deg))))
        step = abs(lateral) / max(math.sin(phi), 0.20)
        step = max(float(self.cfg.path_return_min_step_m),
                   min(float(self.cfg.path_return_max_step_m), step))
        # lateral>0 表示偏在线左侧，要向右转回来（负角度）。
        delta = -math.copysign(self.cfg.path_return_angle_deg, lateral)

        request = self.actions.turn(delta, purpose="绕障后回到原路线")
        self.path_return_step_m = step
        self.path_return_attempts += 1
        self.pending = PendingAction(
            kind="return_turn",
            tag=request.cmd.tag,
            started_ts=ctx.now,
            timeout_s=self.cfg.turn_timeout_s,
            expected_yaw_delta_deg=delta,
            start_yaw_rad=ctx.pose.yaw_rad,
        )
        self.phase = RETURN
        return StepResult(
            RETURN,
            request,
            log=(f"横向偏离原路线 {lateral:+.2f}m，"
                 f"切 {abs(delta):.0f}° 绕回（横移 {step:.2f}m）"),
        )

    def _check_after_return_turn(self, ctx: StepContext) -> StepResult:
        if ctx.pose is None:
            return self._block(
                ctx.now, "绕回时没有姿态数据", interrupt=True, reason="no_pose"
            )
        step = max(0.0, self.path_return_step_m)
        if step <= 0.0:
            return self._start_realign(ctx.now, ctx.perception, ctx.pose)
        request = self.actions.advance(step, "绕障后回到原路线")
        self.pending = PendingAction(
            kind="return_advance",
            tag=request.cmd.tag,
            started_ts=ctx.now,
            timeout_s=self.cfg.step_timeout_s,
            distance_m=step,
        )
        self.phase = RETURN
        return StepResult(
            RETURN, request, log=f"绕回横移 {step:.2f}m"
        )

    def _check_after_return_advance(self, ctx: StepContext) -> StepResult:
        if ctx.pose is None:
            return self._block(
                ctx.now, "绕回后没有姿态数据", interrupt=True, reason="no_pose"
            )
        lateral = self._path_lateral(ctx.pose)
        if (lateral is not None
                and abs(lateral) <= self.cfg.path_return_tolerance_m
                and abs(self.heading_dev_deg) <= self.cfg.yaw_tol_deg):
            return self._complete_bypass(
                ctx, stop_state=RETURN, log="已绕回原路线，恢复前进"
            )
        # 先回正到绕障前航向；_check_after_realign 会再判断是否还需一次绕回。
        return self._start_realign(ctx.now, ctx.perception, ctx.pose)

    def _check_after_realign(self, ctx: StepContext) -> StepResult:
        path_step = self._maybe_start_path_return(ctx)
        if path_step is not None:
            return path_step

        if ctx.perception.distance_m >= self.cfg.clear_m:
            return self._complete_bypass(
                ctx, stop_state=CHECK, log="回正完成，恢复前进"
            )

        if ctx.perception.distance_m < self.cfg.danger_m:
            if self.realign_attempts >= self.cfg.max_realign_attempts:
                return self._block(
                    ctx.now, "回正后仍被挡，停车锁定",
                    interrupt=True, reason="realign_blocked",
                )
            self.phase = PLAN
            return StepResult(PLAN, log="回正后仍被挡，重新规划")

        self.phase = HOLD
        return StepResult(
            HOLD,
            log="回正后距离不足，停车等待",
            voice=self.voice.announce(
                VoiceCue("wait", reason="clear_wait"), ctx.now
            ),
        )

    # ------------------------------------------------------------------
    # BLOCKED：锁定，安全一段时间后恢复检查
    # ------------------------------------------------------------------
    def _step_blocked(self, ctx: StepContext) -> StepResult:
        if ctx.valid:
            self._history_append(ctx.now, ctx.perception)
            if (
                self._stable_clear_for(ctx.now, self.cfg.blocked_recheck_s)
                and ctx.perception.min_m >= self.cfg.rotate_clear_m
            ):
                self.phase = HOLD
                self.history.clear()
                self.voice.clear_keys()
                self.last_stop_cmd_ts = ctx.now - self.cfg.stop_refresh_s
                return StepResult(HOLD, log="障碍解除，恢复检查")
        else:
            self.history.clear()

        return self._stop_result(BLOCKED, ctx.now)

    # ------------------------------------------------------------------
    # 动作执行：等待结果、校验、设置下一状态
    # ------------------------------------------------------------------
    def _service_pending(self, ctx: StepContext) -> StepResult:
        p = self.pending

        veto = self._pending_safety_veto(ctx, p)
        if veto is not None:
            return veto

        if (
            ctx.result is None
            or ctx.result.status == "running"
            or ctx.result.tag != p.tag
        ):
            return self._pending_wait_or_timeout(ctx, p)

        if ctx.result.status != "ok":
            return self._fail_pending(
                ctx.now,
                p,
                f"{p.kind} {ctx.result.status}: {ctx.result.message}",
                f"{p.kind}_failed",
            )

        actual_delta, failed = self._validate_action_result(ctx, p)
        if failed is not None:
            return failed

        if p.kind == "advance":
            drift = self._apply_advance_progress(ctx, p)
            if drift is not None:
                return drift

        return self._finish_pending(ctx.now, p, actual_delta)

    def _pending_safety_veto(self, ctx: StepContext,
                             p: PendingAction) -> StepResult | None:
        """动作执行期间也必须盯住感知，危险时立即打断。"""
        if not ctx.fresh or not ctx.valid:
            return self._fail_pending(
                ctx.now, p, f"{p.kind} 途中感知失效", "perception_lost"
            )

        # 前进途中如果前方突然进入紧急距离，立即打断
        if p.kind == "advance" and ctx.perception.distance_m < self.cfg.emergency_m:
            return self._fail_pending(
                ctx.now,
                p,
                f"前进中前方过近 {ctx.perception.distance_m:.2f}m",
                "emergency",
            )
        return None

    def _apply_advance_progress(self, ctx: StepContext,
                                p: PendingAction) -> StepResult | None:
        """累计前进距离/次数，并检查前进过程中是否航向漂移。"""
        self.bypass_distance_m += p.distance_m
        self.avoid_cycles += 1
        if ctx.pose is None or self.entry_yaw_rad is None:
            return None

        self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)
        if (
            abs(self.heading_dev_deg)
            > self.cfg.max_heading_dev + self.cfg.yaw_tol_deg
        ):
            return self._fail_pending(
                ctx.now,
                p,
                f"前进中航向漂移 {self.heading_dev_deg:+.1f}°",
                "heading_drift",
            )
        return None

    def _pending_wait_or_timeout(self, ctx: StepContext,
                                 p: PendingAction) -> StepResult:
        if ctx.now - p.started_ts >= p.timeout_s:
            return self._fail_pending(
                ctx.now,
                p,
                f"{p.kind} 超时（{p.timeout_s:.1f}s）",
                f"{p.kind}_timeout",
            )
        return StepResult(self.phase)

    def _validate_action_result(
        self, ctx: StepContext, p: PendingAction
    ) -> tuple[float | None, StepResult | None]:
        """校验转向/回正的 yaw；返回 (实际角度, 失败结果或 None)。"""
        actual_delta = None
        if p.kind not in ("turn", "realign", "return_turn"):
            return actual_delta, None

        if ctx.pose is None or p.start_yaw_rad is None:
            return actual_delta, self._fail_pending(
                ctx.now,
                p,
                "动作完成但无法读取姿态，航向不可信",
                "pose_missing",
            )

        actual_delta = math.degrees(
            wrap_pi(ctx.pose.yaw_rad - p.start_yaw_rad)
        )
        expected = p.expected_yaw_delta_deg or 0.0
        tolerance = max(self.cfg.yaw_tol_deg, abs(expected) * 0.10)
        if abs(actual_delta - expected) > tolerance:
            return actual_delta, self._fail_pending(
                ctx.now,
                p,
                f"航向校验失败：期望 {expected:+.1f}°，实际 {actual_delta:+.1f}°",
                "yaw_mismatch",
            )

        self.heading_dev_deg = self._heading_dev(ctx.pose.yaw_rad)
        return actual_delta, None

    def _finish_pending(self, now: float, p: PendingAction,
                        actual_delta: float | None) -> StepResult:
        self.pending = None
        self.action_finished_ts = now
        self.settle_until = now + self.cfg.settle_s
        if p.kind in ("return_turn", "return_advance"):
            self.check_mode = p.kind
            self.phase = RETURN
            if p.kind == "return_turn":
                log = f"绕回切入完成（实际 {actual_delta:+.1f}°），准备横移"
            else:
                log = "绕回横移完成，准备回正到原航向"
            return StepResult(RETURN, log=log)

        self.check_mode = (
            "turn" if p.kind == "turn"
            else "advance" if p.kind == "advance"
            else "realign"
        )
        self.phase = CHECK

        if p.kind == "turn":
            log = f"转向完成（实际 {actual_delta:+.1f}°），等待新帧"
        elif p.kind == "advance":
            log = f"前进完成，累计 {self.bypass_distance_m:.2f}m"
        else:
            log = f"回正完成（实际 {actual_delta:+.1f}°），等待新帧"
        return StepResult(CHECK, log=log)

    def _fail_pending(self, now: float, p: PendingAction,
                      why: str, reason: str) -> StepResult:
        self.pending = None
        return self._block(now, why, interrupt=True, reason=reason)

    # ------------------------------------------------------------------
    # 动作构造
    # ------------------------------------------------------------------
    def _start_advance(self, now: float, step_m: float,
                       perception: Perception | None = None,
                       top_up: bool = False) -> StepResult:
        note = "补足前进" if top_up else "绕障前进"
        request = self.actions.advance(step_m, note)
        cmd = request.cmd
        self.pending = PendingAction(
            kind="advance",
            tag=cmd.tag,
            started_ts=now,
            timeout_s=self.cfg.step_timeout_s,
            distance_m=step_m,
        )
        self.phase = ADVANCE
        total_m = self.bypass_distance_m + step_m
        center = fmt_m(perception.distance_m) if perception is not None else "?"
        return StepResult(
            ADVANCE,
            request,
            log=(
                f"{note} {step_m:.2f}m"
                f"（中心 {center}，累计 {total_m:.2f}/"
                f"{self.cfg.min_bypass_advance_m:.2f}m）"
            ),
        )

    def _start_realign(self, now: float, perception: Perception,
                       pose: Pose | None) -> StepResult:
        if pose is None:
            return self._block(
                now, "没有姿态数据，无法回正",
                interrupt=True, reason="no_pose",
            )
        if self.entry_yaw_rad is None:
            return self._block(
                now, "没有避障前航向，无法回正",
                interrupt=True, reason="no_entry_yaw",
            )

        self.heading_dev_deg = self._heading_dev(pose.yaw_rad)
        if abs(self.heading_dev_deg) <= self.cfg.yaw_tol_deg:
            self.action_finished_ts = now
            self.settle_until = now + self.cfg.settle_s
            self.check_mode = "realign"
            self.phase = CHECK
            return StepResult(CHECK, log="航向已经对齐")

        if perception.min_m < self.cfg.rotate_clear_m:
            return self._block(
                now,
                f"回正净空不足 min={perception.min_m:.2f}m",
                interrupt=True,
                reason="realign_space",
            )

        self.realign_attempts += 1
        if self.realign_attempts > self.cfg.max_realign_attempts:
            return self._block(
                now, "回正次数达到上限", interrupt=True, reason="realign_limit"
            )

        delta = -self.heading_dev_deg
        request = self.actions.realign(delta)
        cmd = request.cmd
        self.pending = PendingAction(
            kind="realign",
            tag=cmd.tag,
            started_ts=now,
            timeout_s=self.cfg.realign_timeout_s,
            expected_yaw_delta_deg=delta,
            start_yaw_rad=pose.yaw_rad,
        )
        self.phase = REALIGN
        return StepResult(
            REALIGN,
            request,
            log=(
                f"回正 {delta:+.0f}°"
                f"（目标 {math.degrees(self.entry_yaw_rad):.1f}°）"
            ),
        )

    # ------------------------------------------------------------------
    # 通用辅助
    # ------------------------------------------------------------------
    def _heading_dev(self, yaw_rad: float) -> float:
        if self.entry_yaw_rad is None:
            return 0.0
        return math.degrees(wrap_pi(yaw_rad - self.entry_yaw_rad))

    def _reset_bypass(self, keep_entry: bool):
        self.bypass_active = False
        self.bypass_distance_m = 0.0
        self.avoid_cycles = 0
        self.realign_attempts = 0
        self.tried_angles.clear()
        self.settle_until = 0.0
        self.action_finished_ts = 0.0
        self.check_mode = ""
        self.voice.clear_keys()
        if not keep_entry:
            self.entry_yaw_rad = None
            self.entry_x = float("nan")
            self.entry_y = float("nan")
            self.path_return_attempts = 0
            self.heading_dev_deg = 0.0

    def _enter_hold(self, now: float, why: str, interrupt: bool,
                    keep_entry: bool, voice: VoiceCue | None = None) -> StepResult:
        was = self.phase
        self.phase = HOLD
        self.pending = None
        if not keep_entry:
            self.entry_yaw_rad = None
            self.entry_x = float("nan")
            self.entry_y = float("nan")
            self.path_return_attempts = 0
            self.bypass_active = False
            self.bypass_distance_m = 0.0
            self.avoid_cycles = 0
            self.realign_attempts = 0
            self.tried_angles.clear()
            self.heading_dev_deg = 0.0
            self.voice.clear_keys()
        self.history.clear()

        if interrupt:
            self.last_stop_cmd_ts = now
            return StepResult(
                HOLD,
                self._stop_request(),
                interrupt=True,
                log=(f"停车：{why}" if was != HOLD else None),
                voice=voice,
            )

        return self._stop_result(
            HOLD, now, (f"停车：{why}" if was != HOLD else None), voice=voice
        )

    def _block(self, now: float, why: str, interrupt: bool,
               reason: str = "internal") -> StepResult:
        was = self.phase
        self.phase = BLOCKED
        self.pending = None
        voice = None
        if was != BLOCKED:
            self.blocked_since = now
            self.history.clear()
            voice = self.voice.announce(
                VoiceCue("blocked", reason=reason), now
            )
        self.last_stop_cmd_ts = now
        return StepResult(
            BLOCKED,
            self._stop_request(),
            interrupt=interrupt,
            log=(f"锁定：{why}" if was != BLOCKED else None),
            voice=voice,
        )

    def _stop_result(self, state: str, now: float, log: str | None = None,
                     voice: VoiceCue | None = None) -> StepResult:
        if now - self.last_stop_cmd_ts >= self.cfg.stop_refresh_s:
            self.last_stop_cmd_ts = now
            return StepResult(state, self._stop_request(), log=log, voice=voice)
        return StepResult(state, None, log=log, voice=voice)

    def _apply_deferred(self, deferred: DeferredAction,
                        now: float) -> StepResult:
        """语音播完后，执行被推迟的转向或恢复前进。"""
        if deferred.kind == "turn" and deferred.request is not None:
            if deferred.pending is not None:
                deferred.pending.started_ts = now
                self.pending = deferred.pending
            self.phase = TURN
            return StepResult(TURN, deferred.request, log=deferred.log)

        if deferred.kind == "go":
            self.phase = GO
            self._reset_bypass(keep_entry=False)
            return StepResult(GO, log=deferred.log)

        return StepResult(self.phase)

    def _stop_request(self) -> ActionRequest:
        return self.actions.stop()

    def _history_append(self, now: float, perception: Perception):
        self.history.append(
            (now, perception.distance_m, perception.near_ratio, perception.valid)
        )
        cutoff = now - max(self.cfg.confirm_s, self.cfg.blocked_recheck_s) * 2.0 - 1.0
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

    def _window(self, now: float, duration: float):
        # 多留一点历史再过滤：如果只取 <= duration，采样点会永远差一个周期，
        # 导致 now - window[0][0] >= duration 永远不成立。
        grace = max(0.20, 2.0 / max(self.cfg.decision_hz, 1.0))
        return [h for h in self.history if now - h[0] <= duration + grace]

    def _stable_clear(self, now: float) -> bool:
        return self._stable_clear_for(now, self.cfg.confirm_s)

    def _stable_clear_for(self, now: float, duration: float) -> bool:
        window = self._window(now, duration)
        if len(window) < 3:
            return False
        if now - window[0][0] < duration:
            return False
        return all(valid and center >= self.cfg.clear_m
                   for _, center, _, valid in window)

    def _stable_obstacle(self, now: float) -> bool:
        window = self._window(now, self.cfg.confirm_s)
        if len(window) < 3:
            return False
        if now - window[0][0] < self.cfg.confirm_s:
            return False
        centers = [h[1] for h in window if h[3]]
        if not centers:
            return False
        nears = [h[2] for h in window if h[3]]
        if max(centers) - min(centers) > self.cfg.stable_delta_m:
            return False
        if max(nears) - min(nears) > self.cfg.stable_near_delta:
            return False
        return True

    def _is_approaching(self, now: float) -> bool:
        window = self._window(now, self.cfg.confirm_s)
        if len(window) < 3:
            return False
        first = window[0][1]
        last = window[-1][1]
        return last < self.cfg.approach_m and (first - last) > 0.15

    def _warn(self, now: float, message: str) -> str | None:
        if now - self.last_dynamic_warn_ts < self.cfg.warn_log_interval_s:
            return None
        self.last_dynamic_warn_ts = now
        return message
