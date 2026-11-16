"""ROS image wire validation and clock checks; no ROS or hardware imports."""
import math
import re


def validate_camera_config(cfg):
    if cfg.camera_backend not in ('dds', 'ros2'):
        raise ValueError('camera_backend must be dds or ros2')
    if cfg.perception_backend not in ('queue', 'ros2'):
        raise ValueError('perception_backend must be queue or ros2')
    if cfg.perception_backend == 'ros2':
        if cfg.camera_backend != 'ros2':
            raise ValueError('ROS perception requires camera_backend=ros2')
        ns = cfg.ros2_perception_namespace
        if (not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*', ns)
                or '__' in ns):
            raise ValueError('ros2_perception_namespace requires an absolute namespace')
    if cfg.camera_backend != 'ros2':
        return
    if not math.isfinite(cfg.ros2_camera_startup_timeout_s) or cfg.ros2_camera_startup_timeout_s <= 0:
        raise ValueError('ros2_camera_startup_timeout_s must be positive')
    needed = ['ros2_depth_topic']
    if cfg.rgb_enabled:
        needed.append('ros2_rgb_topic')
    if cfg.stop_line_enabled or cfg.lane_follow_enabled or cfg.mission_enabled:
        needed.append('ros2_camera2_topic')
    for name in needed:
        topic = getattr(cfg, name)
        if (not isinstance(topic, str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*', topic)
                or '__' in topic):
            raise ValueError(f'{name} requires an explicit absolute ROS topic')
    if any(getattr(cfg, n) == cfg.ros2_depth_topic for n in needed if n != 'ros2_depth_topic'):
        raise ValueError('depth Image and RGB CompressedImage must use different topics')


def decode_depth(msg):
    """Return millimetres for compute_profile; respect byte order and row padding."""
    import numpy as np
    formats = {'16UC1': ('u2', 2, 1.), '32FC1': ('f4', 4, 1000.)}
    if msg.encoding not in formats:
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')
    code, size, scale = formats[msg.encoding]
    w, h, step = msg.width, msg.height, msg.step
    if w <= 0 or h <= 0 or step < w * size or step % size:
        raise ValueError('invalid depth dimensions or step')
    raw = bytes(msg.data)
    if len(raw) != h * step:
        raise ValueError('depth data length does not match height * step')
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + code)
    frame = np.frombuffer(raw, dtype=dtype).reshape(h, step // size)[:, :w]
    if scale == 1.:
        return frame.astype(np.uint16)
    frame = frame.astype(np.float64) * scale
    return np.where(np.isfinite(frame) & (frame > 0), frame, 0.)


def decode_rgb_sizes(data, widths):
    """Decode once; preserve the existing decode_jpeg resize/color operations."""
    import cv2
    import numpy as np
    bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('invalid compressed RGB image')
    h, w = bgr.shape[:2]
    images = {}
    for width in set(widths):
        if width <= 0:
            raise ValueError('camera_width must be positive')
        nh = max(2, int(round(h * width / w / 2.0)) * 2)
        sized = cv2.resize(bgr, (width, nh), interpolation=cv2.INTER_AREA)
        images[width] = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return images


class SourceStampGate:
    """One ROS stream, system-clock capture stamps only during legacy migration.

    ROS Image has no frame sequence. Missing/zero stamps cannot establish source
    freshness and are rejected, as are repeats and time resets. Restart after
    correcting a source clock reset; never silently turn source time into now.
    """
    def __init__(self):
        self.last_stamp_ns = 0

    def accept(self, msg, *, now, max_age_s):
        stamp = msg.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError('invalid capture stamp')
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        ts = ns / 1_000_000_000
        if ns <= self.last_stamp_ns:
            raise ValueError('missing, repeated or reordered capture stamp')
        if not math.isfinite(now) or not 0 <= now - ts < max_age_s:
            raise ValueError('capture stamp is stale, future or not system time')
        self.last_stamp_ns = ns
        return ts
