"""兼容旧导入：主程序/工具统一从 motion.factory 取后端。"""
from motion.factory import build_robot

__all__ = ["build_robot"]
