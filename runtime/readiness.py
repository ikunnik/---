"""启动握手：检测器订阅/第一帧就绪。"""
import logging
import math
import time


def _wait_detector_ready(proc, ready, first_frame, states, *, label, topic,
                         startup_timeout_s, frame_timeout_s, max_age_s=None, require_valid=False) -> bool:
    """等待检测器完成相机订阅、产出第一帧并进入 Latest。"""
    def dead() -> bool:
        if proc.is_alive():
            return False
        logging.error("[主进程] %s进程已退出（exitcode=%s）", label, proc.exitcode)
        return True

    for event, timeout, stage in ((ready, startup_timeout_s, "相机订阅"),
                                  (first_frame, frame_timeout_s, "第一帧")):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if dead():
                return False
            if event.wait(0.1):
                break
        else:
            logging.error(
                "[主进程] %s等待%s超时（%.1fs，%s）", label, stage, timeout, topic
            )
            return False

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if dead():
            return False
        value, _ = states.get()
        fresh = (value is not None and (max_age_s is None or (
            math.isfinite(value.ts) and 0 <= time.time() - value.ts < max_age_s)))
        if fresh and (not require_valid or value.valid):
            logging.info("[主进程] %s检测第一帧已就绪", label)
            return True
        time.sleep(0.02)
    logging.error("[主进程] %s第一帧未进入决策层", label)
    return False


def _wait_ready(rt):
    """启动握手：检测器没就绪就拒绝启动运动。"""
    cfg = rt.cfg
    ros = cfg.camera_backend == "ros2"
    if cfg.perception_backend == "ros2":
        deadline = time.monotonic() + cfg.ros2_camera_startup_timeout_s
        while not rt.ros_perceiver.ready.wait(.05):
            if not rt.ros_perceiver.is_alive() or time.monotonic() >= deadline:
                raise SystemExit("ROS感知订阅未就绪，拒绝启动运动序列")
    if ros and not _wait_detector_ready(
        rt.camera, rt.camera_ready, rt.camera_first_frame, rt.perception,
        label="ROS深度", topic=cfg.ros2_depth_topic,
        startup_timeout_s=cfg.ros2_camera_startup_timeout_s,
        frame_timeout_s=cfg.ros2_camera_startup_timeout_s, max_age_s=cfg.stale_s,
        require_valid=True,
    ):
        raise SystemExit("ROS深度图像未就绪，拒绝启动运动序列")
    if cfg.lane_follow_enabled:
        if not _wait_detector_ready(
            rt.camera if ros else rt.lane_camera, rt.lane_ready, rt.lane_first_frame, rt.lane_states,
            label="盲道", topic=cfg.ros2_camera2_topic if ros else cfg.lane_topic,
            startup_timeout_s=cfg.lane_startup_timeout_s,
            frame_timeout_s=cfg.lane_frame_timeout_s,
            max_age_s=cfg.lane_stale_s if ros else None,
        ):
            raise SystemExit("盲道检测未就绪，拒绝启动运动序列")
    elif cfg.traffic_sequence_enabled:
        if not _wait_detector_ready(
            rt.camera if ros else rt.stop_line, rt.stop_line_ready, rt.stop_line_first_frame,
            rt.stop_line_states,
            label="停止线", topic=cfg.ros2_camera2_topic if ros else cfg.stop_line_topic,
            startup_timeout_s=cfg.stop_line_startup_timeout_s,
            frame_timeout_s=cfg.stop_line_frame_timeout_s,
            max_age_s=cfg.stop_line_stale_s if ros else None,
        ):
            raise SystemExit("停止线检测未就绪，拒绝启动运动序列")

    if cfg.mission_enabled:
        if not _wait_detector_ready(
            rt.camera if ros else rt.lane_camera, rt.lane_ready, rt.lane_first_frame, rt.lane_states,
            label="盲道", topic=cfg.ros2_camera2_topic if ros else cfg.lane_topic,
            startup_timeout_s=cfg.lane_startup_timeout_s,
            frame_timeout_s=cfg.lane_frame_timeout_s,
            max_age_s=cfg.lane_stale_s if ros else None,
        ):
            raise SystemExit("盲道检测未就绪，拒绝启动串联任务")



wait = _wait_ready


wait_detector_ready = _wait_detector_ready
