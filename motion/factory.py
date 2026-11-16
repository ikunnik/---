"""运动后端工厂：主程序与工具共用。"""
from motion.dry_run import DryRunRobot


def build_robot(dry_run: bool, cfg):
    if dry_run:
        return DryRunRobot()
    if getattr(cfg, "motion_backend", "ros2") == "ros2":
        from motion.ros2_backend import Ros2MotionBackend
        return Ros2MotionBackend(
            cfg.robot_ip,
            cfg.speed_ratio,
            pose_timeout_s=cfg.pose_timeout_s,
            pose_cache_s=cfg.pose_cache_s,
            cfg=cfg,
        )
    from robot_link import RobotLink
    return RobotLink(
        cfg.robot_ip,
        cfg.speed_ratio,
        pose_timeout_s=cfg.pose_timeout_s,
        pose_cache_s=cfg.pose_cache_s,
    )
