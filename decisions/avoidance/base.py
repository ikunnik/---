"""避障状态机基类：初始化、reset、step 分发和 StepContext。"""
import math
from collections import deque

from decisions.types import ANNOUNCE, BLOCKED, CHECK, GO, HOLD, PLAN, RETURN, PendingAction, StepContext, StepResult
from messages import CommandResult, Perception, Pose
from actions import Actions
from decisions.safety import depth_fresh
from decisions.voice import TIMEOUT, WAITING, VoiceGate


class AvoidBase:
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
        self._bypass_stage = "diagonal"
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
        self.dynamic_hits = 0
        self.dynamic_until = 0.0
        self.last_approach_info = (0.0, 0.0)
        self.hold_clean_ts = 0.0
        self.invalid_since = 0.0
        self.recover_attempts = 0
        self.last_valid_m = float("inf")
        self.last_pose_warn_ts = -1e9
        self.blocked_since = 0.0
        self.history = deque()
        self.started_at = 0.0
        self._reset_navigation()


    @property
    def has_pending_action(self) -> bool:
        return self.pending is not None


    @property
    def voice_pending(self) -> bool:
        """兼容旧接口：当前是否正在停车等语音播完。"""
        return self.voice.pending


    def reset(self, now: float, keep_voice: bool = False):
        self.phase = HOLD
        self.entry_yaw_rad = None
        self.entry_x = float("nan")
        self.entry_y = float("nan")
        self.path_return_attempts = 0
        self.path_return_step_m = 0.0
        self.heading_dev_deg = 0.0
        self.bypass_active = False
        self._bypass_stage = "diagonal"
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
        self.dynamic_hits = 0
        self.dynamic_until = 0.0
        self.last_approach_info = (0.0, 0.0)
        self.hold_clean_ts = 0.0
        self.invalid_since = 0.0
        self.recover_attempts = 0
        self.last_valid_m = float("inf")
        self.last_pose_warn_ts = -1e9
        self.blocked_since = 0.0
        self.history.clear()
        self.started_at = now
        self._reset_navigation()
        if not keep_voice:
            self.voice.reset()


    def step(self, now: float, perception: Perception | None, perception_ts: float,
             pose: Pose | None, result: CommandResult | None,
             executor_busy: bool, voice_done: bool = False) -> StepResult:
        # 只要 entry_yaw 还在，就持续从姿态更新当前偏离角
        if self.entry_yaw_rad is not None and pose is not None:
            self.heading_dev_deg = self._heading_dev(pose.yaw_rad)

        ctx = self._context(now, perception, perception_ts, pose, result, executor_busy, voice_done)
        if self.bypass_started_ts is not None and self.phase != BLOCKED:
            if now - self.bypass_started_ts > self.cfg.bypass_timeout_s:
                return self._block(now, "绕障/回线总时间达到上限", True, "max_cycles")
            if not self._pose_valid(pose, now):
                return self._block(now, "绕障/回线位置失效或过期", True, "no_pose")

        # 语音播报期间：不执行任何运动指令。只有收到本轮播报的完成回执，
        # 或等待超时（语音进程异常）后，才继续后面的状态。
        voice_state, deferred = self.voice.poll(now, voice_done)
        if voice_state == WAITING:
            return self._stop_result(ANNOUNCE, now)
        if deferred is not None:
            # 播报耗时期间场景可能已经变化，不能直接执行保存的旧动作。
            if not ctx.valid:
                return self._block(now, "播报后感知失效", True, "perception_lost")
            if deferred.kind == "turn":
                self.phase = PLAN
                return self._step_plan(ctx)
            return self._complete_bypass(ctx, HOLD, "已回到原路线，恢复前进")
        if voice_state == TIMEOUT:
            return self._stop_result(
                ANNOUNCE, now, log="语音等待超时，继续执行"
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
        fresh = (depth_fresh(self.cfg, perception, perception_ts, now)
                 and math.isfinite(perception.ts) and perception.ts > 0
                 and 0 <= now - perception.ts < self.cfg.stale_s)
        valid = fresh and perception.valid and math.isfinite(perception.distance_m)
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

