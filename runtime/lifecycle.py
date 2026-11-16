"""运行时生命周期：连接、启动、监视、逆序停止。"""
import logging
import signal
import threading

PROCESS_STOP_TIMEOUT = 2
THREAD_STOP_TIMEOUT = 2
EXECUTOR_STOP_TIMEOUT = 3

BANNER = """
╔══════════════════════════════════════════════════════╗
║  Rover X1 深度避障                                    ║
║    安全 → 前进 ({vx:.2f} m/s，5 秒一条自动补发)        ║
║    危险 → 停车，左右 ±45° 整段绕行               ║
║    绕完 → 先绕出 ≥{bypass_m:.1f}m，再按位置回到原路线            ║
║    动态障碍 → 停车等待；动作失败/超时 → 锁定          ║
║    感知失效 → 立即停车                                ║
║    语音 → 自然女声；播报期间停车，播完再继续          ║
║    Ctrl+C 退出                                        ║
╚══════════════════════════════════════════════════════╝
"""


def _connect_robot(rt):
    cfg = rt.cfg
    logging.info("[主进程] 运动后端: %s；相机输入: %s", cfg.motion_backend, cfg.camera_backend)
    logging.info("[主进程] 前进速度 %.2f m/s", cfg.forward_vx)
    if cfg.rgb_enabled:
        logging.info("[主进程] RGB 采集: %s", cfg.rgb_topic)
    if cfg.traffic_light_enabled:
        state = (
            "立即处理（--traffic-light-now）"
            if rt.traffic_active is not None and rt.traffic_active.is_set()
            else "等待停止线触发"
        )
        logging.info(
            "[主进程] 红绿灯检测（%s）: %s", state, cfg.traffic_light_profile
        )
    if cfg.stop_line_enabled:
        logging.info("[主进程] 停止线检测: %s", cfg.stop_line_profile)
    if cfg.traffic_sequence_enabled:
        logging.info("[主进程] 停止线序列已启用（会控制运动）")
    if cfg.lane_follow_enabled:
        logging.info(
            "[主进程] 独立盲道模式已启用（稳定循环 vx=%.2fm/s，片段=%.2fs）",
            cfg.lane_forward_vx, cfg.lane_control_duration_s,
        )

    logging.info("[主进程] 连接机器人...")
    rt.robot.connect()
    logging.info("[主进程] 起立...")
    rt.robot.stand()
    logging.info("[主进程] 切换 WALK 模式...")
    rt.robot.walk()


def _start_workers(rt):
    """启动顺序：相机 → 语音 → RGB → 搬运线程 → 检测 → 执行器/调度/决策。"""
    for worker in (
        rt.ros_perceiver, rt.camera, rt.voice, rt.rgb_camera, rt.perceiver, rt.rgb_worker,
        rt.traffic_light, rt.traffic_worker, rt.stop_line,
        rt.stop_line_worker, rt.lane_camera, rt.lane_worker,
    ):
        if worker is not None:
            worker.start()


def _print_status(rt):
    cfg = rt.cfg
    if cfg.mission_enabled:
        lane_limit = (
            "持续运行" if cfg.lane_trial_seconds <= 0
            else f"限时{cfg.lane_trial_seconds:.0f}s"
        )
        print(
            f"串联任务：通过红绿灯 → 左转90° → "
            f"直行{cfg.uturn_forward_m:.2f}m → 左转90° → 盲道"
            f"（{lane_limit}）",
            flush=True,
        )
        if cfg.uturn_ignore_obstacles:
            print(
                "警告：掉头阶段已跳过深度净空检查，仅保留姿态/角度校验",
                flush=True,
            )
        elif not cfg.uturn_check_forward_space:
            print(
                "提示：掉头直行不检查前方 1.9m 空间；"
                "--uturn-check-forward-space 可恢复",
                flush=True,
            )
    elif cfg.lane_follow_enabled:
        lane_limit = (
            "持续运行（0=不限时）" if cfg.lane_trial_seconds <= 0
            else f"限时{cfg.lane_trial_seconds:.0f}s"
        )
        print(
            f"盲道模式：稳定循环纠偏；无盲道直行；"
            f"{cfg.lane_forward_vx:.2f}m/s；{lane_limit}；Ctrl+C退出",
            flush=True,
        )
    else:
        print(
            BANNER.format(
                vx=cfg.forward_vx, bypass_m=cfg.min_bypass_advance_m
            ),
            flush=True,
        )


def _install_signals(stop: threading.Event):
    def _on_signal(signum, _frame):
        logging.warning("[主进程] 收到退出信号 %s", signum)
        stop.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)


def _close_stop_line(rt):
    """进入掉头/盲道后不再需要停止线：释放 camera2 reader 和进程/线程。"""
    logging.info(
        "[主进程] 已进入 %s，关闭停止线检测",
        rt.supervisor.snapshot()[0].mode,
    )
    if rt.cfg.camera_backend == "ros2":
        rt.camera.disable_stop_line()
    _stop_thread(rt.stop_line_worker)
    _stop_process(rt.stop_line, "停止线")
    if rt.traffic_active is not None:
        rt.traffic_active.clear()


def _stop_thread(worker, timeout=THREAD_STOP_TIMEOUT):
    if worker is None:
        return
    worker.stop()
    if worker.is_alive():
        worker.join(timeout)


def _stop_process(proc, name, timeout=PROCESS_STOP_TIMEOUT):
    if proc is None:
        return
    proc.stop()
    if proc.is_alive():
        proc.join(timeout)
    if proc.is_alive():
        logging.warning("[主进程] %s进程无响应，强制结束", name)
        # CycloneDDS 在 SIGTERM 清理回调时可能触发断言；退出阶段直接 SIGKILL，
        # 避免卡住的相机进程把退出流程弄脏。
        try:
            proc.kill()
        except AttributeError:
            proc.terminate()
        proc.join(timeout)


def _watch(rt, stop: threading.Event):
    """主线程只做监视：关停止线、发现控制线程/检测进程异常就收尾。"""
    stop_line_closed = False
    stop_line_dead_logged = False
    lane_dead_logged = False

    while not stop.wait(0.2):
        if (
            not stop_line_closed
            and rt.supervisor.snapshot()[0].mode in ('uturn', 'lane')
        ):
            stop_line_closed = True
            stop_line_dead_logged = True
            _close_stop_line(rt)

        if (
            not rt.supervisor.is_alive()
            or not rt.decision.is_alive()
            or not rt.executor.is_alive()
        ):
            logging.error("[主进程] 控制线程退出，停车并结束")
            stop.set()
            continue

        if (rt.cfg.perception_backend == "ros2" and not rt.ros_perceiver.is_alive()):
            rt.ros_perceiver.receiver.clear_all()
            logging.error("[主进程] ROS感知接收线程退出，停止运动并退出")
            stop.set()
            continue

        if rt.cfg.camera_backend == "ros2" and not rt.camera.is_alive():
            rt.perception.set(None)
            rt.lane_states.set(None)
            rt.stop_line_states.set(None)
            logging.error("[主进程] ROS相机进程退出，停止运动并退出")
            stop.set()
            continue

        if (
            rt.lane_camera is not None
            and not lane_dead_logged
            and not rt.lane_camera.is_alive()
        ):
            lane_dead_logged = True
            rt.lane_states.set(None)
            logging.error("[主进程] 盲道检测进程已退出，停止运动并退出")
            stop.set()
            continue

        if (
            rt.stop_line is not None
            and not stop_line_dead_logged
            and not rt.stop_line.is_alive()
        ):
            stop_line_dead_logged = True
            logging.error(
                "[主进程] 停止线进程已退出（exitcode=%s），"
                "检测器失效；序列将保持停车直到重启",
                rt.stop_line.exitcode,
            )


def _shutdown(rt):
    """按启动的相反顺序停止；只启动了一半也要能安全收尾。"""
    logging.info("[主进程] 正在停止...")

    _stop_thread(rt.supervisor)
    _stop_thread(rt.decision)
    _stop_thread(rt.executor, EXECUTOR_STOP_TIMEOUT)
    for worker in (
        rt.perceiver, rt.rgb_worker, rt.traffic_worker,
        rt.stop_line_worker, rt.lane_worker, rt.ros_perceiver,
    ):
        _stop_thread(worker)

    for proc, name in (
        (rt.traffic_light, "红绿灯"),
        (rt.stop_line, "停止线"),
        (rt.lane_camera, "盲道"),
        (rt.rgb_camera, "RGB"),
        (rt.voice, "语音"),
        (rt.camera, "相机"),
    ):
        _stop_process(proc, name)

    try:
        rt.robot.brake()
    finally:
        rt.robot.close()



connect = _connect_robot


start_workers = _start_workers


print_status = _print_status


install_signals = _install_signals


watch = _watch


shutdown = _shutdown
