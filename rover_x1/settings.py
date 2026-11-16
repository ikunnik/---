"""ROS startup settings; pure configuration logic, no hardware imports."""
import math
from dataclasses import fields, replace
from pathlib import Path

from app.cli import build_cfg, parse_args
from config import CFG

MODES = ("mission", "avoidance", "traffic", "lane")
ASSET_FIELDS = ("voice_pack_dir", "lane_profile", "stop_line_profile", "traffic_light_profile")


def validate_config(cfg):
    for field in fields(cfg):
        value = getattr(cfg, field.name)
        expected = type(getattr(CFG, field.name))
        if type(value) is not expected:
            raise ValueError(f"{field.name} must be {expected.__name__}")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"{field.name} must be finite")
    if cfg.camera_backend not in ("dds", "ros2"):
        raise ValueError("camera_backend must be dds or ros2")
    if cfg.ros2_camera_startup_timeout_s <= 0:
        raise ValueError("ros2_camera_startup_timeout_s must be positive")
    if cfg.motion_backend not in ("ros2", "sdk"):
        raise ValueError("motion_backend must be ros2 or sdk")
    if not 0 < cfg.forward_vx <= cfg.max_forward_vx:
        raise ValueError("forward_vx must be in (0, max_forward_vx]")
    if not 0 < cfg.emergency_m <= cfg.danger_m < cfg.clear_m:
        raise ValueError("require 0 < emergency_m <= danger_m < clear_m")
    if cfg.safety_buffer_m < 0:
        raise ValueError("safety_buffer_m must be nonnegative")
    for name in ("decision_hz", "ros2_rate_hz", "ros2_line_vx", "ros2_yaw_rate",
                 "stale_s", "bypass_pose_stale_s", "ros2_odom_stale_s", "rotate_clear_m",
                 "min_passage_m", "min_bypass_advance_m", "bypass_step_min_m",
                 "path_return_min_step_m", "path_return_max_step_m", "path_return_tolerance_m",
                 "max_bypass_distance_m", "bypass_timeout_s", "step_timeout_s",
                 "turn_timeout_s", "realign_timeout_s"):
        if getattr(cfg, name) <= 0:
            raise ValueError(f"{name} must be positive")
    if cfg.path_return_min_step_m > cfg.path_return_max_step_m:
        raise ValueError("path_return_min_step_m exceeds path_return_max_step_m")


def resolve_assets(cfg, share_dir):
    """Resolve installed assets against package share, never against launch cwd."""
    changes = {}
    for name in ASSET_FIELDS:
        path = Path(getattr(cfg, name)).expanduser()
        if not path.is_absolute():
            path = Path(share_dir) / path
        if not (path.is_dir() if name == "voice_pack_dir" else path.is_file()):
            raise ValueError(f"{name} does not exist: {path}")
        changes[name] = str(path.resolve())
    for name in ("dds_config", "dds_uri_file"):
        changes[name] = str(Path(getattr(cfg, name)).expanduser())
    return replace(cfg, **changes)


def build_settings(values, *, mode, dry_run, traffic_light_now, share_dir):
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    unknown = set(values) - {f.name for f in fields(CFG)}
    if unknown:
        raise ValueError(f"Unknown config parameters: {sorted(unknown)}")
    cfg = replace(CFG, **values)
    validate_config(cfg)
    # mode determines feature ownership; dependent detector flags are derived by CLI.
    cfg = replace(cfg, mission_enabled=False, lane_follow_enabled=False,
                  traffic_sequence_enabled=False, traffic_light_enabled=False,
                  stop_line_enabled=False)
    options = {
        "mission": ["--mission"],
        "avoidance": ["--no-mission", "--no-traffic-sequence"],
        "traffic": ["--no-mission", "--traffic-sequence"],
        "lane": ["--mangdao"],
    }[mode][:]
    if dry_run:
        options.append("--dry-run")
    if traffic_light_now:
        if mode not in ("mission", "traffic"):
            raise ValueError("traffic_light_now requires mission or traffic mode")
        options.append("--traffic-light-now")
    args = parse_args(cfg, argv=options)
    cfg = resolve_assets(build_cfg(args, cfg), share_dir)
    from rover_x1.camera_input import validate_camera_config
    validate_camera_config(cfg)
    return cfg, args
