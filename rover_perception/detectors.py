"""Rover X1 感知检测内核：进程内相机管线与独立检测节点共用同一实现。

阶段3 的约束：
    - 每个角色只有一份检测逻辑，节点外壳只负责订阅/参数/发布/健康状态；
    - 不 import rclpy，离线单测与相机进程都能直接用；
    - 既可处理 ROS 消息（detect），也可处理已解码图像（detect_from_image），
      后者让共享相机入口只解码一次，避免 camera2 在两个检测器里各解一遍。

角色与既有纯函数的对应关系（不改算法，只加外壳）：
    depth     -> workers.depth_camera.compute_profile
    lane      -> workers.lane.detect_lane
    stop_line -> workers.stop_line.detect_stop_line + StopLineVoter
    traffic   -> workers.traffic_light.detect_hsv + TrafficLightVoter
"""
from messages import LaneState, Perception, StopLineState, TrafficLightState
from rover_x1.camera_input import decode_depth, decode_rgb_sizes
from workers.depth_camera import compute_profile
from workers.lane import detect_lane, load_profile as load_lane_profile
from workers.stop_line import (
    StopLineVoter, detect_stop_line, load_profile as load_stop_profile,
)
from workers.traffic_light import (
    TrafficLightVoter, detect_hsv, load_profile as load_traffic_profile,
)

DEPTH, LANE, STOP_LINE, TRAFFIC = "depth", "lane", "stop_line", "traffic"
ROLES = (DEPTH, LANE, STOP_LINE, TRAFFIC)

# 各角色在 config.py 中对应的图像话题字段（split 节点据此决定订阅哪个话题）
TOPIC_FIELDS = {
    DEPTH: "ros2_depth_topic",
    LANE: "ros2_camera2_topic",
    STOP_LINE: "ros2_camera2_topic",
    TRAFFIC: "ros2_rgb_topic",
}

# 各角色在 config.py 中对应的标定文件字段；深度不需要 profile
PROFILE_FIELDS = {
    LANE: "lane_profile",
    STOP_LINE: "stop_line_profile",
    TRAFFIC: "traffic_light_profile",
}


class Detector:
    """一个角色一帧的检测器；有状态的部分只有时序投票器。"""

    role = ""
    image_kind = ""       # depth / rgb
    state_type = None     # 输出的 legacy dataclass

    def __init__(self, cfg):
        self.cfg = cfg

    def detect(self, msg, ts):
        """ROS 消息 -> 状态。ts 必须是采集时间，不能是本地接收时间。"""
        raise NotImplementedError

    def detect_from_image(self, image, ts):
        """已解码图像 -> 状态；共享相机入口用它避免重复解码。"""
        raise NotImplementedError

    def reset(self):
        """会话/启停切换时重建时序状态，避免旧票数算进新一段。"""


class DepthDetector(Detector):
    role = DEPTH
    image_kind = "depth"
    state_type = Perception

    def detect(self, msg, ts):
        return self.detect_from_image(decode_depth(msg), ts)

    def detect_from_image(self, millimetres, ts):
        return compute_profile(millimetres, self.cfg, ts=ts)


class _RgbDetector(Detector):
    image_kind = "rgb"
    default_width = 640

    def __init__(self, cfg):
        super().__init__(cfg)
        self.profile = self.load_profile(cfg)
        self.width = int(self.profile.get("camera_width", self.default_width))
        self.voter = self.make_voter(self.profile)

    def load_profile(self, cfg):
        raise NotImplementedError

    def make_voter(self, profile):
        return None

    def reset(self):
        self.voter = self.make_voter(self.profile)

    def detect(self, msg, ts):
        return self.detect_from_image(self.decode(msg), ts)

    def decode(self, msg):
        data = bytes(msg.data)
        return decode_rgb_sizes(data, [self.width])[self.width]


class LaneDetector(_RgbDetector):
    role = LANE
    state_type = LaneState

    def load_profile(self, cfg):
        return load_lane_profile(cfg.lane_profile)

    def detect_from_image(self, image, ts):
        return detect_lane(image, self.profile, now=ts)


class StopLineDetector(_RgbDetector):
    role = STOP_LINE
    state_type = StopLineState

    def load_profile(self, cfg):
        return load_stop_profile(cfg.stop_line_profile)

    def make_voter(self, profile):
        return StopLineVoter(profile)

    def detect_from_image(self, image, ts):
        return self.voter.update(detect_stop_line(image, self.profile, now=ts), ts)


class TrafficDetector(_RgbDetector):
    role = TRAFFIC
    default_width = 960
    state_type = TrafficLightState

    def load_profile(self, cfg):
        return load_traffic_profile(cfg.traffic_light_profile)

    def make_voter(self, profile):
        return TrafficLightVoter(profile)

    def detect_from_image(self, image, ts):
        return self.voter.update(detect_hsv(image, self.profile, now=ts), ts)


DETECTORS = {
    DEPTH: DepthDetector, LANE: LaneDetector,
    STOP_LINE: StopLineDetector, TRAFFIC: TrafficDetector,
}


def build_detector(cfg, role):
    try:
        factory = DETECTORS[role]
    except KeyError:
        raise ValueError(
            f"未知检测角色: {role}（可选 {', '.join(ROLES)}）"
        ) from None
    return factory(cfg)


def empty_state(role, ts):
    """检测失败时的占位输出：形状合法但明确标为不可用。

    接收端据此清空白板。失败不能伪装成“没检测到目标”，
    否则停止线/红绿灯会把坏帧当成一次正常的“未检出”。
    """
    if role == DEPTH:
        return Perception(float("inf"), False, ts)
    if role == LANE:
        return LaneState(ts=ts)
    if role == STOP_LINE:
        return StopLineState(ts=ts)
    if role == TRAFFIC:
        return TrafficLightState(ts=ts)
    raise ValueError(f"未知检测角色: {role}")
