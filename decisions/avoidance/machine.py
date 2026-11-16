"""AvoidStateMachine 门面：把各阶段 mixin 组合成单一状态机。"""
from decisions.avoidance.navigation import NavigationMixin
from decisions.avoidance.base import AvoidBase
from decisions.avoidance.bypass import BypassMixin
from decisions.avoidance.go import GoMixin
from decisions.avoidance.hold import HoldMixin
from decisions.avoidance.path_return import PathReturnMixin
from decisions.avoidance.pending import PendingMixin
from decisions.avoidance.plan import PlanMixin
from decisions.avoidance.transitions import TransitionMixin


class AvoidStateMachine(
    NavigationMixin,
    GoMixin,
    HoldMixin,
    PlanMixin,
    BypassMixin,
    PathReturnMixin,
    PendingMixin,
    TransitionMixin,
    AvoidBase,
):
    """有状态的避障决策机；对外只暴露 step()。"""
