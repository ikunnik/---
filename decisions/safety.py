"""公共感知判断。保留现有各模式的时间边界和阈值语义。"""
import math


def depth_fresh(cfg, perception, timestamp, now):
    return (perception is not None and timestamp > 0.0
            and (now - timestamp) < cfg.stale_s)


def depth_valid(cfg, perception, timestamp, now):
    return depth_fresh(cfg, perception, timestamp, now) and perception.valid


def obstacle_ahead(cfg, perception, timestamp, now):
    return (depth_valid(cfg, perception, timestamp, now)
            and perception.distance_m < cfg.danger_m)


def lane_fresh(cfg, lane, now):
    return (lane is not None and math.isfinite(lane.ts) and lane.ts > 0.0
            and 0.0 <= now - lane.ts <= cfg.lane_stale_s)


def lane_seen(cfg, lane, now):
    """识别播报条件；运动许可仍由 lane_fresh 和盲道控制器判断。"""
    return (lane is not None and lane.visible and lane.ts > 0.0
            and now - lane.ts <= cfg.lane_stale_s)


def stop_line_alive(cfg, line, timestamp, now):
    """图像正常但没有线也算健康；不等同于允许接管。"""
    return (line is not None and timestamp > 0.0
            and now - timestamp <= cfg.stop_line_stale_s)


def stop_line_visible(cfg, line, now):
    return line is not None and line.visible and stop_line_alive(cfg, line, line.ts, now)
