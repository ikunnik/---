"""
决策层共享数据结构。

这里有状态名、候选方向、状态机输出和内部待执行动作，全部是纯数据，
不依赖线程、网络、DDS 或时间源，方便状态机单独测试。
"""
from dataclasses import dataclass

from messages import ActionRequest, CommandResult, Perception, Pose, VoiceCue

# 对外暴露的状态名
GO = "GO"
HOLD = "HOLD"
PLAN = "PLAN"
TURN = "TURN"
CHECK = "CHECK"
ADVANCE = "ADVANCE"
REALIGN = "REALIGN"
RETURN = "RETURN"   # 绕障后横向回到原路线
BLOCKED = "BLOCKED"
ANNOUNCE = "ANNOUNCE"   # 语音播报中：只保持停车，不产生运动

ACTION_STATES = (TURN, ADVANCE, REALIGN, RETURN)


@dataclass(frozen=True)
class StepResult:
    """一次 step() 的返回值。"""

    state: str
    request: ActionRequest | None = None
    interrupt: bool = False
    log: str | None = None
    voice: VoiceCue | None = None


@dataclass
class PendingAction:
    """已经交给执行器、正在等待 CommandResult 的动作。"""

    kind: str
    tag: str
    started_ts: float
    timeout_s: float
    expected_yaw_delta_deg: float | None = None
    start_yaw_rad: float | None = None
    distance_m: float = 0.0
    start_x: float = float("nan")
    start_y: float = float("nan")


@dataclass
class DeferredAction:
    """等语音播完后才能执行的动作。"""

    kind: str                       # turn / go
    request: ActionRequest | None = None
    pending: PendingAction | None = None
    log: str | None = None


@dataclass(frozen=True)
class StepContext:
    """一次 step() 的只读输入快照，避免每个状态方法重复算 fresh/valid。"""

    now: float
    perception: Perception | None
    perception_ts: float
    pose: Pose | None
    result: CommandResult | None
    executor_busy: bool
    fresh: bool
    valid: bool
    voice_done: bool
