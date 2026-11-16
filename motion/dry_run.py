"""假机器人：没有硬件/ROS2 时跑通完整决策链。"""
import math
import threading
import time

from messages import Motion, Pose, VelocityPlan
from motion.api import MotionCancelled, MotionProgress


class _FakeCall:
    """模拟一次速度流：睡够 duration 秒，期间可以被 cancel。"""

    def __init__(self, duration: float, on_done=None):
        self._duration = duration
        self._cancelled = threading.Event()
        self._on_done = on_done

    def cancel(self):
        self._cancelled.set()

    def __iter__(self):
        deadline = time.time() + self._duration
        while time.time() < deadline and not self._cancelled.is_set():
            time.sleep(0.02)
        if self._cancelled.is_set():
            raise MotionCancelled("dry-run cancelled")
        if self._on_done is not None:
            self._on_done()
        yield MotionProgress(True, True, "")


class DryRunRobot:
    """假机器人：记录 yaw，旋转时按动作更新，便于 dry-run 验证回正。"""

    def __init__(self):
        self._yaw = 0.0
        self._x = self._y = 0.0
        self._lock = threading.Lock()

    def connect(self):
        print("[dry-run] 不连接机器人")

    def stand(self):
        pass

    def walk(self):
        pass

    def get_pose(self, refresh: bool = False):
        with self._lock:
            return Pose(yaw_rad=self._yaw, ts=time.time(), x=self._x, y=self._y)

    def execute_stream(self, cmd):
        print(f"[dry-run] 执行 {cmd}")
        if isinstance(cmd, VelocityPlan):
            duration = sum(step.duration for step in cmd.steps)
            return _FakeCall(float("inf") if cmd.loop else duration)

        on_done = None
        if isinstance(cmd, Motion) and cmd.motion_id == "rotation":
            params = dict(cmd.params)
            direction = int(params.get("direction", 0))
            angle = float(params.get("angle", 0.0))

            def _apply_rotation():
                with self._lock:
                    delta = angle if direction == 0 else -angle
                    self._yaw = _wrap_pi(self._yaw + math.radians(delta))

            on_done = _apply_rotation
        elif isinstance(cmd, Motion) and cmd.motion_id == "line_walk":
            params = dict(cmd.params)
            distance = float(params.get("distance", 0.0))
            direction = int(params.get("direction", 0))
            offset = {0: 0.0, 1: math.pi, 2: math.pi / 2, 3: -math.pi / 2}[direction]

            def _apply_move():
                with self._lock:
                    self._x += distance * math.cos(self._yaw + offset)
                    self._y += distance * math.sin(self._yaw + offset)

            on_done = _apply_move

        return _FakeCall(getattr(cmd, "duration", 1.0), on_done=on_done)

    def brake(self):
        print("[dry-run] 刹车")

    def close(self):
        pass


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
