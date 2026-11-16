"""
统一动作目录：速度、距离、转向、停车；Actions 生成统一 ActionRequest。

所有业务模式通过 Actions 构造请求，再交给 MotionExecutor.submit()。
底层函数保留用于构造 SDK 指令；不碰网络、不启动线程。

对应 SDK 的 rotation 动作：
    direction: 0=左转，1=右转
    angle:     0~3600 度

对应 SDK 的 line_walk 动作：
    direction: 0=前，1=后，2=左，3=右
    distance:  0~3 米

circle(direction, turns) 本质就是 angle = turns * 360，因此不需要新的 motion_id。
执行时由 robot_link.build_request() 转成 gRPC 参数，executor 仍可 cancel()。
"""
from messages import ActionRequest, Motion, Velocity, VelocityPlan

MAX_ANGLE = 3600.0
MAX_TURNS = 10
MAX_DISTANCE = 3.0


def _resolve_direction(direction) -> int:
    """把 "left"/"right" 或 0/1 统一成 SDK 的 0/1。"""
    if isinstance(direction, str):
        name = direction.strip().lower()
        if name == "left":
            return 0
        if name == "right":
            return 1
        raise ValueError("direction 必须是 'left'/'right' 或 0/1")

    try:
        value = int(direction)
    except (TypeError, ValueError):
        raise ValueError("direction 必须是 'left'/'right' 或 0/1")
    if value not in (0, 1):
        raise ValueError("direction 必须是 'left'/'right' 或 0/1")
    return value


def _direction_name(direction: int) -> str:
    return "left" if direction == 0 else "right"


def rotate(direction="left", angle=90.0) -> Motion:
    """原地旋转指定角度。angle 会被限制在 [0, 3600]。"""
    direction = _resolve_direction(direction)
    angle = min(max(float(angle), 0.0), MAX_ANGLE)
    name = _direction_name(direction)
    return Motion(
        "rotation",
        (("direction", direction), ("angle", angle)),
        tag=f"rotate_{name} {angle:g}deg",
        duration=max(0.5, min(5.0, angle / 90.0)),
    )


def rotate_left(angle=90.0) -> Motion:
    return rotate("left", angle)


def rotate_right(angle=90.0) -> Motion:
    return rotate("right", angle)


def circle(direction="left", turns=1) -> Motion:
    """旋转指定圈数，turns 会被限制在 [1, 10]。"""
    direction = _resolve_direction(direction)
    turns = min(max(int(turns), 1), MAX_TURNS)
    angle = float(turns * 360)
    name = _direction_name(direction)
    return Motion(
        "rotation",
        (("direction", direction), ("angle", angle)),
        tag=f"circle_{name} {turns}turn",
        duration=max(0.5, min(10.0, turns * 2.0)),
    )


def _resolve_move_direction(direction) -> int:
    """把 forward/backward/left/right 或 0~3 统一成 SDK 的方向码。"""
    if isinstance(direction, str):
        name = direction.strip().lower()
        mapping = {"forward": 0, "backward": 1, "left": 2, "right": 3}
        if name not in mapping:
            raise ValueError(
                "direction 必须是 'forward'/'backward'/'left'/'right' 或 0~3"
            )
        return mapping[name]

    try:
        value = int(direction)
    except (TypeError, ValueError):
        raise ValueError(
            "direction 必须是 'forward'/'backward'/'left'/'right' 或 0~3"
        )
    if value not in (0, 1, 2, 3):
        raise ValueError(
            "direction 必须是 'forward'/'backward'/'left'/'right' 或 0~3"
        )
    return value


def move(direction=0, distance=0.2) -> Motion:
    """沿指定方向前进/横移指定距离，distance 会被限制在 [0, 3]。"""
    direction = _resolve_move_direction(direction)
    distance = min(max(float(distance), 0.0), MAX_DISTANCE)
    names = {0: "forward", 1: "backward", 2: "left", 3: "right"}
    return Motion(
        "line_walk",
        (("direction", direction), ("distance", distance)),
        tag=f"move_{names[direction]} {distance:g}m",
        duration=max(0.5, min(5.0, distance / 0.3)),
    )


def move_forward(distance=0.2) -> Motion:
    """沿当前朝向前进指定距离，distance 会被限制在 [0, 3]。"""
    return move("forward", distance)


def move_backward(distance=0.2) -> Motion:
    return move("backward", distance)


def move_left(distance=0.2) -> Motion:
    return move("left", distance)


def move_right(distance=0.2) -> Motion:
    return move("right", distance)


# ---------------- 速度与停车 ----------------
def stop(duration=0.3, tag="stop") -> Velocity:
    """统一零速指令；用于业务停车、看门狗和退出刹车。"""
    return Velocity(0, 0, 0, duration, tag)


class Actions:
    """统一动作入口。只构造请求，安全判断仍由各状态机负责。

    单位：速度 m/s，角速度 rad/s，距离 m，转角 deg，时间 s。
    三类 SDK 指令由 robot_link 适配，业务模块不再直接拼接它们。
    """

    def __init__(self, cfg):
        self.cfg = cfg

    # 速度控制：直行、横移、弧线纠偏都用同一方法。
    def velocity(self, vx, vy=0.0, vyaw=0.0, *, duration=None,
                 loop=False, steps=1, tag="forward", kind="forward",
                 timeout_s=0.0, purpose="速度行走") -> ActionRequest:
        duration = self.cfg.forward_s if duration is None else duration
        cmd = Velocity(vx, vy, vyaw, duration, "" if loop or steps != 1 else tag)
        if loop or steps != 1:
            if not isinstance(steps, int) or not 1 <= steps <= 8:
                raise ValueError("速度序列步数必须是 1~8 的整数")
            cmd = VelocityPlan(tuple(cmd for _ in range(steps)), loop=loop, tag=tag)
        return ActionRequest(cmd, kind, timeout_s=timeout_s, purpose=purpose)

    def forward(self) -> ActionRequest:
        return self.velocity(self.cfg.forward_vx, purpose="前进")

    def stop(self, *, timeout_s=0.0) -> ActionRequest:
        return ActionRequest(stop(self.cfg.stop_s), "stop",
                             timeout_s=timeout_s, purpose="停车")

    # 距离控制：前后移动和左右横移保留 SDK 的距离语义。
    def move(self, direction="forward", distance=0.2, *,
             purpose="距离移动") -> ActionRequest:
        timeout = self.cfg.step_timeout_s
        if getattr(self.cfg, "motion_backend", "ros2") == "ros2":
            # ROS2 按里程计闭环，起步/减速/发现延迟都比 SDK 长；
            # 1m@0.4m/s 实测刚超过 3s，按距离给足余量。
            speed = max(0.3, float(getattr(self.cfg, "ros2_line_vx", 0.4)))
            timeout = max(timeout, min(10.0, float(distance) / speed + 2.0))
        return ActionRequest(move(direction, distance), "advance",
                             timeout_s=timeout, purpose=purpose)

    def advance(self, distance, note="绕障前进") -> ActionRequest:
        return self.move("forward", distance, purpose=f"{note} {distance:.2f}m")

    # 角度控制：正数左转，负数右转；回正复用同一个方法。
    def turn(self, delta, *, realign=False, purpose="转向") -> ActionRequest:
        cmd = rotate_left(delta) if delta > 0 else rotate_right(-delta)
        actual = dict(cmd.params)["angle"] * (1 if delta > 0 else -1)
        return ActionRequest(
            cmd, "realign" if realign else "turn",
            timeout_s=self.cfg.realign_timeout_s if realign else self.cfg.turn_timeout_s,
            expected_yaw_delta_deg=actual, purpose=purpose,
        )

    def realign(self, delta) -> ActionRequest:
        return self.turn(delta, realign=True, purpose="回正到避障前航向")

    def circle(self, direction="left", turns=1) -> ActionRequest:
        cmd = circle(direction, turns)
        params = dict(cmd.params)
        delta = params["angle"] * (1 if params["direction"] == 0 else -1)
        return ActionRequest(cmd, "turn", timeout_s=self.cfg.turn_timeout_s,
                             expected_yaw_delta_deg=delta, purpose="转圈")

    # 模式预设：只选参数，不复制控制和执行逻辑。
    def lane(self, vx, vyaw, sequence, *, purpose="盲道稳定循环纠偏") -> ActionRequest:
        return self.velocity(
            vx, vyaw=vyaw, duration=float(self.cfg.lane_control_duration_s),
            loop=True, steps=max(1, min(8, int(self.cfg.lane_batch_steps))),
            tag=f"lane_plan_{sequence}", kind="lane_velocity", purpose=purpose,
        )

    def traffic_forward(self, purpose) -> ActionRequest:
        return self.velocity(
            self.cfg.traffic_vx, duration=self.cfg.traffic_forward_s,
            tag="traffic_forward", timeout_s=self.cfg.traffic_action_timeout_s,
            purpose=purpose,
        )
