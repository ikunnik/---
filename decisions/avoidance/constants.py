"""绕障几何：转角与并肩目标以 config.Config 为准，这里保留旧导入名。

斜向长度不再固定：由横向偏移目标反算（lateral / sin 转角），见
NavigationMixin._diagonal_from_lateral。旧固定值 0.7m 在 45° 下只能给出
0.495m 横向，扣掉机身半宽后对 0.5m 宽障碍几乎没有余量。
"""
from config import CFG

BYPASS_ANGLE_DEG = CFG.bypass_angle_deg          # 默认 60°
BYPASS_PARALLEL_M = CFG.min_bypass_advance_m     # 旧导入兼容
