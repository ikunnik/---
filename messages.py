"""
线程/进程之间传递的消息。

只用 dataclass（纯数据），好处：
  1. 能直接跨进程传递（可 pickle）
  2. 字段名一眼看懂，不用记 dict 的 key

【流向】
    Perception（感知结果）：深度进程 -> 感知线程 -> 决策线程
    Velocity（速度指令）：  决策线程 -> 执行器线程 -> robot_link
    Motion（动作指令）：    显式调用方/决策线程 -> 执行器线程 -> robot_link
    CommandResult：         执行器线程 -> 决策线程（动作是否真的成功）
    Pose：                  robot_link -> 决策线程（用于航向校验/回正）
    VoiceCue：              决策线程 -> 语音进程（播报方向/完成/停车原因）
    VoiceDone：             语音进程 -> 决策线程（播完/失败回执）
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Perception:
    """深度相机算出来的一帧多区域感知结果。"""

    distance_m: float   # 正前方（中心 ROI）最近障碍物距离，无效时为 inf
    valid: bool         # 中心 ROI 是否可信
    ts: float           # 产生时间（time.time()）

    # 多区域剖面：左/右区域距离，用来判断往哪边绕
    left_m: float = float("inf")
    right_m: float = float("inf")
    left_valid: bool = False
    right_valid: bool = False

    # 三个区域里最近的一个；转身净空检查用
    min_m: float = float("inf")

    # 中心 ROI 内“近处像素”的占比：区分细杆和整面墙
    near_ratio: float = 0.0

    def __str__(self):
        dist = "inf" if self.distance_m == float("inf") else f"{self.distance_m:.2f}m"
        left = "inf" if self.left_m == float("inf") else f"{self.left_m:.2f}m"
        right = "inf" if self.right_m == float("inf") else f"{self.right_m:.2f}m"
        return (
            f"dist={dist} L={left} R={right} "
            f"near={self.near_ratio:.2f} valid={self.valid}"
        )


@dataclass(frozen=True)
class RgbFrame:
    """一帧压缩 RGB 图像（camera0 的 CompressedImage，通常是 JPEG）。

    只传压缩字节，不在采集/通信链路里解码；预览或检测按需解码。
    """

    data: bytes           # 压缩图像字节（JPEG/PNG...）
    ts: float             # 产生时间（优先 header stamp，单位秒）
    format: str = "jpeg"  # CompressedImage.format()
    seq: int = 0          # 进程内递增序号
    frame_id: str = ""    # header.frame_id()，例如 camera0_optical_frame

    def __str__(self):
        return f"{self.format} {len(self.data) / 1024.0:.1f}KB seq={self.seq}"


@dataclass(frozen=True)
class TrafficLightState:
    """红绿灯检测结果。color=red/green/yellow/unknown；stable 表示连续多帧确认。"""

    color: str = "unknown"
    stable: bool = False
    ts: float = 0.0
    bbox: tuple[int, int, int, int] = ()   # ROI 内的 (x, y, w, h)
    area_ratio: float = 0.0                # 颜色像素 / ROI 面积

    def __str__(self):
        return (
            f"{self.color} stable={self.stable} "
            f"area={self.area_ratio:.3%} bbox={self.bbox}"
        )


@dataclass(frozen=True)
class StopLineState:
    """黄线停止线检测结果（camera2 RGB）。row 是黄线底边在整帧中的归一化行号。"""

    visible: bool = False
    row: float = 0.0          # 0（画面顶部）~ 1（底部）；越近越大
    ts: float = 0.0
    area_ratio: float = 0.0
    bbox: tuple[int, int, int, int] = ()   # ROI 内的 (x, y, w, h)
    angle_deg: float = 0.0    # 黄线在图像中的倾角；0=水平，正=右端向下

    def __str__(self):
        return (
            f"visible={self.visible} row={self.row:.3f} "
            f"area={self.area_ratio:.3%} bbox={self.bbox} "
            f"angle={self.angle_deg:+.1f}°"
        )


@dataclass(frozen=True)
class LaneState:
    """双黄线盲道检测结果（camera2 RGB）。center/heading 按整帧归一化。"""

    visible: bool = False
    center: float = 0.0       # near_row 处左右线中心 x（0~1）
    center_far: float = 0.0   # far_row 处左右线中心 x
    heading_deg: float = 0.0  # 正值表示远端中心在右侧（车道向右弯）
    width: float = 0.0        # near_row 处两线间距（归一化）
    left_x: float = 0.0
    right_x: float = 0.0
    ts: float = 0.0
    confidence: float = 0.0
    one_line: bool = False    # 双线模式只看到一条线，用另一条线+默认宽度估计中心
    one_line_side: str = ""   # left/right；双线单线回正时锁定方向
    single: bool = False      # 单线跟踪模式：center 就是黄线本身

    def __str__(self):
        return (
            f"visible={self.visible} center={self.center:.3f} "
            f"heading={self.heading_deg:+.1f}° width={self.width:.3f} "
            f"L={self.left_x:.3f} R={self.right_x:.3f}"
            f"{' single' if self.single else ''}"
            f"{' one_line' if self.one_line else ''}"
            f"{'(' + self.one_line_side + ')' if self.one_line_side else ''}"
        )


@dataclass(frozen=True)
class Velocity:
    """一条速度指令，对应 SDK 的 'vx,vy,vyaw,dur' 字符串。"""
    vx: float
    vy: float
    vyaw: float
    duration: float
    tag: str = ""       # 只用于日志，例如 "forward" / "stop"

    def to_string(self) -> str:
        return f"{self.vx:g},{self.vy:g},{self.vyaw:g},{self.duration:g}"

    def __str__(self):
        return f"{self.to_string()} ({self.tag})"


@dataclass(frozen=True)
class VelocityPlan:
    """一个 RPC 内部的多步速度计划；循环计划由执行器心跳看门狗保护。"""
    steps: tuple[Velocity, ...]
    loop: bool = False
    tag: str = ""

    def to_string(self) -> str:
        if not self.steps:
            raise ValueError("速度计划不能为空")
        return ";".join(step.to_string() for step in self.steps)

    def __str__(self):
        return f"plan(loop={self.loop}, steps={len(self.steps)}, {self.to_string()}, tag={self.tag})"


@dataclass(frozen=True)
class Motion:
    """一条 SDK 动作，例如 rotation / line_walk。params 顺序固定，便于测试和日志。"""
    motion_id: str
    params: tuple[tuple[str, object], ...] = ()
    tag: str = ""
    duration: float = 1.0   # 仅 DryRun 用来模拟耗时

    def __str__(self):
        params = ", ".join(f"{key}={value}" for key, value in self.params)
        return f"{self.motion_id}({params}) ({self.tag})"


MotionCommand = Velocity | Motion | VelocityPlan


@dataclass(frozen=True)
class ActionRequest:
    """所有模式共用的动作请求；包含指令、用途、超时和航向预期。"""

    cmd: MotionCommand
    kind: str                 # forward / stop / turn / advance / realign / lane_velocity
    timeout_s: float = 0.0
    expected_yaw_delta_deg: float | None = None
    purpose: str = ""


@dataclass(frozen=True)
class CommandResult:
    """执行器对一条指令的最终结果。

    状态：
        running   已开始执行（决策层不应把它当成完成）
        ok        动作流正常结束，且 final.success=True
        failed    动作流失败 / 没有收到 final / gRPC 错误
        cancelled 被 interrupt 取消，或指令在排队时已过期
    """

    generation: int
    tag: str
    status: str
    message: str = ""
    ts: float = 0.0


@dataclass(frozen=True)
class Pose:
    """机器人机身姿态。yaw 单位是弧度（SDK ori_body[2]）。

    x/y 是可选的世界坐标（m）：ROS2 后端来自 /foot_odom/data，
    SDK 后端来自 robot_state.pos_body。缺失时为 nan，避障不会做“回原路线”。
    """

    yaw_rad: float
    ts: float
    x: float = float("nan")
    y: float = float("nan")

    @property
    def yaw_deg(self) -> float:
        import math
        return math.degrees(self.yaw_rad)


@dataclass(frozen=True)
class VoiceCue:
    """一条待播报的语义事件，不直接包含长段中文。

    状态机只产生 kind/side/reason 这些纯数据；workers/voice.py 负责
    把它们翻译成中文并合成音频。这样状态机测试不需要音频依赖。
    """

    kind: str          # bypass / bypass_done / dynamic / blocked / wait /
                       # traffic_red / traffic_yellow / traffic_green / lane
    side: str = ""     # left / right
    angle_deg: float = 0.0
    reason: str = ""   # 具体失败/等待原因，见 workers/voice.py
    seq: int = 0       # 决策线程分配的播报序号，用于完成回执匹配

    @property
    def key(self) -> str:
        """同一障碍事件内的去重键。"""
        return f"{self.kind}|{self.side}|{round(self.angle_deg)}|{self.reason}"


@dataclass(frozen=True)
class VoiceDone:
    """语音进程回执：一句话已经播完（或合成失败，不再等待）。"""

    seq: int
    ok: bool = True
    key: str = ""
    ts: float = 0.0
