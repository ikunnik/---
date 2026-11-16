"""避障通用小工具（固定角度/固定距离后只保留这里）。"""
import math


def wrap_pi(angle: float) -> float:
    """把弧度折到 (-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def fmt_m(value: float) -> str:
    return "inf" if not math.isfinite(value) else f"{value:.2f}m"
