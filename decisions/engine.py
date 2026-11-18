"""统一行为判断入口：读入一帧快照，返回动作、打断和播报意图。

不读取相机队列，不调用机器人或执行器；时间和执行结果由调用方传入。
判断分类：公共有效性守卫 → 当前模式/控制权 → 对应状态机 → 动作跟踪。
"""
from dataclasses import dataclass, replace

from messages import (
    ActionRequest, CommandResult, LaneState, Perception, Pose,
    StopLineState, TrafficLightState, VoiceCue,
)
from decisions.avoidance import AvoidStateMachine
from decisions.lane import LaneFollower
from decisions.safety import lane_seen, obstacle_ahead
from decisions.traffic import TrafficSequence, WAIT_STOP_LINE
from decisions.types import GO, StepResult
from decisions.uturn import UTurnController

LANE_HELP_DELAY_S = 10.0  # 识别盲道播报后多久停车呼救


@dataclass(frozen=True)
class DecisionInput:
    """一轮判断使用同一份感知、姿态、执行结果和时间。"""
    now: float
    perception: Perception | None = None
    perception_ts: float = 0.0
    pose: Pose | None = None
    result: CommandResult | None = None
    executor_busy: bool = False
    voice_done: bool = False
    lane: LaneState | None = None
    line: StopLineState | None = None
    line_ts: float = 0.0
    traffic: TrafficLightState | None = None


@dataclass(frozen=True)
class DecisionOutput:
    """统一判断结果；线程只负责落实这些意图。"""
    step: StepResult
    category: str  # avoidance / lane / traffic / safety
    clear_pending: bool = False
    track_result: bool = False
    notices: tuple[VoiceCue, ...] = ()  # 提示播报，不要求停车等回执


class DecisionEngine:
    def __init__(self, cfg, *, lane_follow=False, traffic_sequence=False,
                 traffic_active=None, wait_for_voice=False, uturn=False):
        self.cfg = cfg
        self.uturn = UTurnController(cfg) if uturn else None
        self.avoidance = AvoidStateMachine(cfg, wait_for_voice=wait_for_voice)
        self.lane = LaneFollower(cfg) if lane_follow else None
        self.traffic = (
            TrafficSequence(cfg, traffic_active, self.avoidance)
            if traffic_sequence else None
        )
        self._lane_avoiding = False
        self._lane_announced = False
        self._lane_help_due = None

    def start(self, now):
        """决策线程启动时初始化避障状态；不重置盲道测试时钟。"""
        self.avoidance.reset(now)

    def evaluate(self, data: DecisionInput) -> DecisionOutput:
        """唯一行为判断入口；盲道模式和停止线模式保持独立。"""
        if self.uturn is not None:
            output = DecisionOutput(self.uturn.step(data), 'uturn')
        elif self.lane is not None:
            output = self._evaluate_lane(data)
        elif self.traffic is not None:
            output = self._evaluate_traffic(data)
        else:
            output = DecisionOutput(self._avoid(data), 'avoidance')

        request = output.step.request
        track = False
        if request is not None:
            if self.traffic is not None and self.traffic.active:
                track = request.kind in ('forward', 'stop')
            else:
                track = request.kind in ('turn', 'advance', 'realign', 'lane_velocity')
                if request.kind == 'stop' and self.avoidance.has_pending_action:
                    track = True
        return replace(output, track_result=track)

    def _avoid(self, data):
        return self.avoidance.step(
            data.now, data.perception, data.perception_ts, data.pose,
            data.result, data.executor_busy, voice_done=data.voice_done,
        )

    # 盲道类：识别提示 → 已接管的避障 → 新障碍接管 → 跟线/无盲道直行。
    def _evaluate_lane(self, data):
        notices = ()
        if lane_seen(self.cfg, data.lane, data.now) and not self._lane_announced:
            self._lane_announced = True
            self._lane_help_due = data.now + LANE_HELP_DELAY_S
            notices = (VoiceCue(kind='lane', reason='lane_recognized'),)
        if self._lane_help_due is not None and data.now >= self._lane_help_due:
            self._lane_help_due = None
            notices += (VoiceCue(kind='help'),)
            # 停车锁存并呼救；不再进入避障。
            return DecisionOutput(
                self.lane.halt(data.now), 'lane',
                clear_pending=True, notices=notices,
            )
        clear_pending = False
        if self._lane_avoiding:
            step = self._avoid(data)
            if self.avoidance.phase == GO:
                self._lane_avoiding = False
            category = 'avoidance'
        elif obstacle_ahead(self.cfg, data.perception, data.perception_ts, data.now):
            self._lane_avoiding = True
            self.lane.reset()
            clear_pending = True
            # 切换后的状态机不消费上一个控制器的执行回执。
            step = self._avoid(replace(data, result=None))
            step = replace(step, interrupt=True)
            category = 'avoidance'
        else:
            step = self.lane.step(
                data.now, data.lane, data.perception, data.perception_ts,
                data.pose, data.result, data.executor_busy,
            )
            category = 'lane'
        return DecisionOutput(step, category, clear_pending=clear_pending, notices=notices)

    # 停止线类：检测器守卫 → 序列接管/运行 → 普通避障。
    def _evaluate_traffic(self, data):
        guard = self.traffic.guard_sensor(
            data.now, data.line, data.line_ts, voice_done=data.voice_done,
        )
        if guard is not None:
            return DecisionOutput(guard, 'safety', clear_pending=True)

        takeover = not self.traffic.active and self.traffic.maybe_start(data.now, data.line)
        if self.traffic.active:
            step = self.traffic.step(
                data.now, data.line, data.traffic, data.executor_busy,
                data.perception, perception_ts=data.perception_ts,
                result=None if takeover else data.result, voice_done=data.voice_done,
            )
            if takeover:
                step = replace(step, interrupt=True)
            return DecisionOutput(step, 'traffic', clear_pending=takeover)
        return DecisionOutput(self._avoid(data), 'avoidance')

    # 回执类：决定哪些动作需要跟踪，以及状态机是否仍等待结果。
    def note_sent(self, request: ActionRequest, now):
        if self.traffic is not None and self.traffic.active:
            self.traffic.note_sent(request, now)

    @property
    def has_pending_action(self):
        if self.uturn is not None:
            return self.uturn.has_pending
        if self.traffic is not None and self.traffic.active:
            return self.traffic.has_pending_action
        if self.lane is not None:
            if self._lane_avoiding:
                return self.avoidance.has_pending_action
            return self.lane.has_pending
        return self.avoidance.has_pending_action


    @property
    def terminal_status(self):
        """给调度线程的结果；正常通过与异常结束严格区分。"""
        if self.uturn is not None:
            if self.uturn.failed:
                return 'failed'
            if self.uturn.completed:
                return 'completed'
        elif self.lane is not None:
            if self.lane.trial_finished:
                return 'completed'
        elif self.traffic is not None:
            if self.traffic.completed:
                return 'completed'
            if self.traffic.done or self.traffic.phase == WAIT_STOP_LINE:
                return 'failed'
        return None
