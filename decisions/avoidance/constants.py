"""绕障转角与斜向总目标；各阶段一次执行目标距离，受净空限制。"""

BYPASS_ANGLE_DEG = 45.0
BYPASS_DIAGONAL_M = 0.7
BYPASS_PARALLEL_M = 1.0  # 旧导入兼容；实际使用 cfg.min_bypass_advance_m
