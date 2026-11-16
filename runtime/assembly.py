"""装配：队列、事件、worker、supervisor、decision。"""
import multiprocessing
from types import SimpleNamespace

from comm import Latest, Mailbox
from workers.decision import DecisionWorker
from workers.depth_camera import DepthCameraProcess
from workers.executor import MotionExecutor
from workers.lane import LaneWorker
from workers.lane_camera import LaneCameraProcess
from workers.perception import PerceptionWorker
from workers.rgb_camera import RgbCameraProcess
from workers.rgb_perception import RgbPerceptionWorker
from workers.stop_line import StopLineProcess, StopLineWorker
from workers.supervisor import ControllerSupervisor
from workers.traffic_light import TrafficLightProcess, TrafficLightWorker
from workers.voice import VoiceProcess


def _new_runtime(cfg, robot) -> SimpleNamespace:
    """一次运行用到的所有队列、线程和进程；先全部置空，装配时再填。"""
    return SimpleNamespace(
        cfg=cfg, robot=robot, traffic_active=None,
        perception=None, depth_queue=None, rgb_frames=None,
        rgb_queue=None, rgb_detect_queue=None,
        traffic_queue=None, traffic_states=None,
        stop_line_queue=None, stop_line_states=None,
        stop_line_ready=None, stop_line_first_frame=None,
        lane_queue=None, lane_states=None,
        lane_ready=None, lane_first_frame=None,
        mailbox=None, voice_queue=None, voice_done_queue=None,
        camera=None, voice=None, rgb_camera=None,
        perceiver=None, rgb_worker=None,
        traffic_light=None, traffic_worker=None,
        stop_line=None, stop_line_worker=None,
        lane_camera=None, lane_worker=None,
        executor=None, supervisor=None, decision=None,
        lane_camera_enabled=False, camera_ready=None, camera_first_frame=None,
        ros_perceiver=None,
    )


def _build_runtime(cfg, args, robot) -> SimpleNamespace:
    """建队列和 worker；此时线程/进程都还没启动。"""
    from rover_x1.camera_input import validate_camera_config
    validate_camera_config(cfg)
    rt = _new_runtime(cfg, robot)
    ros_output = cfg.perception_backend == "ros2"

    rt.traffic_active = (
        multiprocessing.Event() if cfg.traffic_light_enabled else None
    )
    if args.traffic_light_now and rt.traffic_active is not None:
        rt.traffic_active.set()

    rt.perception = Latest()
    rt.depth_queue = multiprocessing.Queue(maxsize=1)
    rt.rgb_frames = Latest()
    rt.rgb_queue = (
        multiprocessing.Queue(maxsize=1) if cfg.rgb_enabled else None
    )
    rt.rgb_detect_queue = (
        multiprocessing.Queue(maxsize=1) if cfg.traffic_light_enabled else None
    )
    rt.traffic_queue = (
        multiprocessing.Queue(maxsize=1) if cfg.traffic_light_enabled else None
    )
    rt.traffic_states = Latest()
    rt.stop_line_queue = (
        multiprocessing.Queue(maxsize=1) if cfg.stop_line_enabled else None
    )
    rt.stop_line_states = Latest()
    rt.lane_states = Latest()
    rt.stop_line_ready = (
        multiprocessing.Event() if cfg.stop_line_enabled else None
    )
    rt.stop_line_first_frame = (
        multiprocessing.Event() if cfg.stop_line_enabled else None
    )
    rt.lane_camera_enabled = cfg.lane_follow_enabled or cfg.mission_enabled
    if rt.lane_camera_enabled:
        rt.lane_queue = multiprocessing.Queue(maxsize=1)
        rt.lane_ready = multiprocessing.Event()
        rt.lane_first_frame = multiprocessing.Event()
    rt.mailbox = Mailbox()
    rt.voice_queue = (
        multiprocessing.Queue(maxsize=1) if cfg.voice_enabled else None
    )
    rt.voice_done_queue = (
        multiprocessing.Queue(maxsize=8) if cfg.voice_enabled else None
    )

    if cfg.camera_backend == "dds":
        rt.camera = DepthCameraProcess(cfg, rt.depth_queue)
    rt.voice = (
        VoiceProcess(cfg, rt.voice_queue, rt.voice_done_queue)
        if cfg.voice_enabled else None
    )
    rt.rgb_camera = (
        RgbCameraProcess(cfg, rt.rgb_queue) if cfg.rgb_enabled and cfg.camera_backend == "dds" else None
    )
    rt.perceiver = PerceptionWorker(cfg, rt.depth_queue, rt.perception)
    rt.rgb_worker = (
        RgbPerceptionWorker(
            cfg, rt.rgb_queue, rt.rgb_frames,
            detect_queue=None if ros_output else rt.rgb_detect_queue,
            detect_active=rt.traffic_active,
        )
        if cfg.rgb_enabled else None
    )
    rt.traffic_light = (
        TrafficLightProcess(
            cfg, rt.rgb_detect_queue, rt.traffic_queue,
            active=rt.traffic_active,
        )
        if cfg.traffic_light_enabled and not ros_output else None
    )
    rt.traffic_worker = (
        TrafficLightWorker(cfg, rt.traffic_queue, rt.traffic_states)
        if cfg.traffic_light_enabled else None
    )
    rt.stop_line = (
        StopLineProcess(
            cfg, rt.stop_line_queue,
            ready=rt.stop_line_ready,
            first_frame=rt.stop_line_first_frame,
        )
        if cfg.stop_line_enabled and cfg.camera_backend == "dds" else None
    )
    rt.stop_line_worker = (
        StopLineWorker(cfg, rt.stop_line_queue, rt.stop_line_states)
        if cfg.stop_line_enabled else None
    )
    rt.lane_camera = (
        LaneCameraProcess(
            cfg, rt.lane_queue,
            ready=rt.lane_ready, first_frame=rt.lane_first_frame,
        )
        if rt.lane_camera_enabled and cfg.camera_backend == "dds" else None
    )
    rt.lane_worker = (
        LaneWorker(cfg, rt.lane_queue, rt.lane_states)
        if rt.lane_camera_enabled else None
    )
    if cfg.camera_backend == "ros2":
        from workers.ros_camera import RosCameraProcess
        rt.camera_ready = multiprocessing.Event()
        rt.camera_first_frame = multiprocessing.Event()
        rt.camera = RosCameraProcess(
            cfg, {"depth": rt.depth_queue, "rgb": rt.rgb_queue,
                  "lane": rt.lane_queue, "stop_line": rt.stop_line_queue,
                  "traffic": rt.traffic_queue if ros_output else None},
            ready=rt.camera_ready,
            first_frames={} if ros_output else {"depth": rt.camera_first_frame, "lane": rt.lane_first_frame,
                          "stop_line": rt.stop_line_first_frame},
            detector_ready={"lane": rt.lane_ready, "stop_line": rt.stop_line_ready},
            traffic_active=rt.traffic_active,
        )
    if ros_output:
        from rover_x1.perception_io import RosPerceptionWorker, roles_for
        states = {"depth": rt.perception, "lane": rt.lane_states,
                  "stop_line": rt.stop_line_states, "traffic": rt.traffic_states}
        rt.ros_perceiver = RosPerceptionWorker(
            cfg, {role: states[role] for role in roles_for(cfg)}, rt.camera.stream_ids,
            first_frames={"depth": rt.camera_first_frame, "lane": rt.lane_first_frame,
                          "stop_line": rt.stop_line_first_frame},
            controls={"stop_line": rt.camera.stop_line_active, "traffic": rt.traffic_active},
        )
        # ROS subscriber owns these whiteboards; no parallel queue writers.
        rt.perceiver = rt.traffic_worker = rt.stop_line_worker = rt.lane_worker = None
    rt.executor = MotionExecutor(cfg, robot, rt.mailbox)

    initial_mode = (
        'lane' if cfg.lane_follow_enabled
        else 'traffic' if cfg.traffic_sequence_enabled
        else 'avoidance'
    )
    available_modes = {'avoidance'}
    if cfg.traffic_sequence_enabled:
        available_modes.add('traffic')
    if rt.lane_camera_enabled:
        available_modes.add('lane')
    if cfg.mission_enabled:
        available_modes.add('uturn')
    rt.supervisor = ControllerSupervisor(
        cfg, initial_mode,
        mission=cfg.mission_enabled,
        available_modes=available_modes,
    )
    rt.decision = DecisionWorker(
        cfg, rt.perception, rt.executor, robot,
        voice_queue=rt.voice_queue,
        voice_done_queue=rt.voice_done_queue,
        stop_line=rt.stop_line_states,
        traffic=rt.traffic_states,
        traffic_active=rt.traffic_active,
        traffic_sequence=cfg.traffic_sequence_enabled,
        lane=rt.lane_states,
        lane_follow=cfg.lane_follow_enabled,
        supervisor=rt.supervisor,
    )
    return rt



build = _build_runtime
