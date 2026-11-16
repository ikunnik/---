"""Typed ROS perception transport and guarded legacy decision input adapter."""
import logging
import math
import threading
import time
from dataclasses import fields
from functools import partial

from messages import Perception, LaneState, StopLineState, TrafficLightState
from rover_x1.contracts import CLOCK_SYSTEM, FrameFreshness
from rover_x1.conversions import state_from_ros, state_to_ros, STATE_TYPES

ROLE_TYPES = {'depth': Perception, 'lane': LaneState,
              'stop_line': StopLineState, 'traffic': TrafficLightState}
SUFFIXES = {'depth': 'obstacles', 'lane': 'lane',
            'stop_line': 'stop_line', 'traffic': 'traffic_light'}


def roles_for(cfg):
    roles = ['depth']
    if cfg.lane_follow_enabled or cfg.mission_enabled:
        roles.append('lane')
    if cfg.stop_line_enabled:
        roles.append('stop_line')
    if cfg.traffic_light_enabled:
        roles.append('traffic')
    return roles


def limits_for(cfg):
    return dict(depth=cfg.stale_s, lane=cfg.lane_stale_s,
                stop_line=cfg.stop_line_stale_s, traffic=cfg.traffic_state_stale_s)


def topic_for(cfg, role):
    return cfg.ros2_perception_namespace + '/' + SUFFIXES[role]


def source_topic(cfg, role):
    return (cfg.ros2_depth_topic if role == 'depth' else
            cfg.ros2_rgb_topic if role == 'traffic' else cfg.ros2_camera2_topic)


def perception_qos():
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def create_output_sink(node, cfg):
    from rover_x1_interfaces import msg
    publishers = {role: node.create_publisher(getattr(msg, STATE_TYPES[ROLE_TYPES[role]]),
                                            topic_for(cfg, role), perception_qos())
                  for role in roles_for(cfg)}

    def publish(role, state, frame):
        publishers[role].publish(state_to_ros(state, frame))
    return publish


class PerceptionReceiver:
    """One writer per Latest; pin source sessions and expire on monotonic deadlines."""
    def __init__(self, cfg, outputs, stream_ids, *, first_frames=None, controls=None,
                 clock=time.time, monotonic=time.monotonic):
        self.cfg, self.outputs = cfg, outputs
        self.clock, self.monotonic = clock, monotonic
        self.first_frames, self.controls = first_frames or {}, controls or {}
        self.gates = {role: FrameFreshness(stream_ids[source_topic(cfg, role)]) for role in outputs}
        self.limits = limits_for(cfg)
        self.accepted = {role: 0 for role in outputs}
        self.rejected = {role: 0 for role in outputs}
        self.last_warning = {}

    def enabled(self, role):
        control = self.controls.get(role)
        return control is None or control.is_set()

    def receive(self, role, msg):
        if not self.enabled(role):
            self.clear(role)
            return False
        try:
            state, frame = state_from_ros(msg)
            if type(state) is not ROLE_TYPES[role]:
                raise ValueError('wrong observation type for topic')
            if frame.clock_domain != CLOCK_SYSTEM or frame.source_stamp_ns is None:
                raise ValueError('system capture stamp required')
            source_ts = frame.source_stamp_ns / 1e9
            if not math.isclose(state.ts, source_ts, rel_tol=0., abs_tol=1e-6):
                raise ValueError('payload time differs from capture time')
            if not 0 <= self.clock() - source_ts < self.limits[role]:
                raise ValueError('stale or future capture stamp')
            # Invalid depth may carry inf; other detector outputs must be finite.
            for field in fields(state):
                value = getattr(state, field.name)
                if isinstance(value, float) and (math.isnan(value) or
                        (role != 'depth' and not math.isfinite(value))):
                    raise ValueError('non-finite detector output')
            gate = self.gates[role]
            if not gate.observe(frame, now_monotonic=self.monotonic()):
                raise ValueError('wrong session, repeated or reordered observation')
            if not frame.detector_ok:
                self.clear(role)
                return False
            self.outputs[role].set(state)
            self.accepted[role] += 1
            event = self.first_frames.get(role)
            if event is not None and (role != 'depth' or state.valid):
                event.set()
            return True
        except (ValueError, TypeError, AttributeError) as exc:
            self.rejected[role] += 1
            now = self.monotonic()
            if now - self.last_warning.get(role, float('-inf')) >= self.cfg.warn_log_interval_s:
                logging.warning('[ROS感知输入] 拒绝 %s: %s', role, exc)
                self.last_warning[role] = now
            return False

    def clear(self, role):
        if self.outputs[role].get()[0] is not None:
            self.outputs[role].set(None)

    def expire(self):
        now = self.monotonic()
        for role, gate in self.gates.items():
            value, _ = self.outputs[role].get()
            if value is not None and (not self.enabled(role)
                    or not gate.fresh(now_monotonic=now, max_age_s=self.limits[role])
                    or not 0 <= self.clock() - value.ts < self.limits[role]):
                self.clear(role)

    def clear_all(self):
        for role in self.outputs:
            self.clear(role)


def create_input_node(receiver, *, context):
    from rclpy.node import Node
    from rover_x1_interfaces import msg
    node = Node('rover_x1_perception_input', context=context, use_global_arguments=False)
    try:
        for role in receiver.outputs:
            node.create_subscription(getattr(msg, STATE_TYPES[ROLE_TYPES[role]]),
                                     topic_for(receiver.cfg, role),
                                     partial(receiver.receive, role), perception_qos())
        node.create_timer(.05, receiver.expire)
    except BaseException:
        node.destroy_node()
        raise
    return node


class RosPerceptionWorker(threading.Thread):
    def __init__(self, cfg, outputs, stream_ids, *, first_frames=None, controls=None):
        super().__init__(name='ros_perception_input', daemon=True)
        self.receiver = PerceptionReceiver(cfg, outputs, stream_ids,
                                          first_frames=first_frames, controls=controls)
        self.ready = threading.Event()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        context = Context()
        node = executor = None
        try:
            rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = create_input_node(self.receiver, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.ready.set()
            while not self._stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=.05)
        except Exception:
            logging.exception('[ROS感知输入] 接收线程异常退出')
        finally:
            self.receiver.clear_all()
            if executor is not None:
                executor.shutdown(timeout_sec=1.)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
