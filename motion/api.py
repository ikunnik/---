"""运动后端的公共协议：执行器只依赖这里，不依赖 gRPC/ROS2。"""
from dataclasses import dataclass
from typing import Iterator, Protocol


class MotionCancelled(Exception):
    """动作被外部取消；执行器据此上报 cancelled，而不是 failed。"""


@dataclass(frozen=True)
class MotionProgress:
    is_final: bool
    success: bool
    message: str = ""


class MotionCall(Protocol):
    def cancel(self) -> None: ...

    def __iter__(self) -> Iterator[MotionProgress]: ...


class MotionBackend(Protocol):
    def connect(self) -> None: ...
    def stand(self) -> None: ...
    def walk(self) -> None: ...
    def execute_stream(self, cmd) -> MotionCall: ...
    def get_pose(self, refresh: bool = False): ...
    def brake(self) -> None: ...
    def close(self) -> None: ...
