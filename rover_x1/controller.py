"""ROS parameter facade and executable; existing workers retain their lifecycle."""
import json
from copy import deepcopy
import logging
import multiprocessing
import threading
from dataclasses import asdict, fields

from config import CFG
from rover_x1.settings import build_settings


def declare_settings(node, share_dir):
    defaults = {f.name: getattr(CFG, f.name) for f in fields(CFG)}
    defaults.update(mode="mission", dry_run=False, check_config=False, traffic_light_now=False)
    unknown = set(node.get_parameters_by_prefix("")) - set(defaults) - {"use_sim_time"}
    if unknown:
        raise ValueError(f"Unknown ROS parameters: {sorted(unknown)}")
    if node.get_parameter("use_sim_time").value:
        raise ValueError("use_sim_time is not supported by the hardware runtime")
    values = {}
    for name, default in defaults.items():
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        value = node.get_parameter(name).value
        if type(value) is not type(default):
            raise ValueError(f"{name} must be {type(default).__name__}")
        values[name] = value
    mode = values.pop("mode")
    dry_run = values.pop("dry_run")
    check_config = values.pop("check_config")
    traffic_now = values.pop("traffic_light_now")
    cfg, args = build_settings(values, mode=mode, dry_run=dry_run,
                              traffic_light_now=traffic_now, share_dir=share_dir)
    from rclpy.parameter import Parameter
    result = node.set_parameters_atomically([
        Parameter(name, value=value) for name, value in asdict(cfg).items()
    ])
    if not result.successful:
        raise ValueError(result.reason)
    for name in (*defaults, "use_sim_time"):
        descriptor = deepcopy(node.describe_parameter(name))
        descriptor.read_only = True
        descriptor.description = "Startup-only parameter; restart after changing YAML."
        node.set_descriptor(name, descriptor)
    return cfg, args, mode, check_config


def main(args=None):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from motion.ros2_backend import _ensure_ros2_dds

    multiprocessing.set_start_method("spawn", force=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(threadName)s] %(message)s")
    _ensure_ros2_dds()
    context = Context()
    rclpy.init(args=args, context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = thread = None
    stop = threading.Event()
    try:
        node = Node("rover_x1_controller", context=context,
                    automatically_declare_parameters_from_overrides=True)
        cfg, runtime_args, mode, check_config = declare_settings(
            node, get_package_share_directory("rover_x1"))
        if check_config:
            print(json.dumps({"mode": mode, "dry_run": runtime_args.dry_run,
                              "config": asdict(cfg)}, ensure_ascii=False, indent=2), flush=True)
            return 0
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        def spin():
            try:
                executor.spin()
            except Exception:
                logging.exception("[ROS2] parameter executor failed")
            finally:
                stop.set()
        thread = threading.Thread(target=spin, name="ros2_parameters", daemon=True)
        thread.start()
        # Delayed imports keep check_config free of SDK, camera and robot initialization.
        from motion.factory import build_robot
        from runtime.app import run
        robot = build_robot(runtime_args.dry_run, cfg)
        node.get_logger().info(f"Starting mode={mode}, motion_backend={cfg.motion_backend}")
        run(cfg, runtime_args, robot, stop_event=stop)
        return 0
    finally:
        stop.set()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
