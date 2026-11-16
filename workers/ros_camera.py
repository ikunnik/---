"""Optional shared ROS camera ingress; produces the existing worker queue types."""
import logging
import multiprocessing
import signal
import time
from functools import partial
from dataclasses import replace
from uuid import uuid4

from comm import put_latest, setup_process_logging
from messages import RgbFrame, Perception, LaneState, StopLineState, TrafficLightState
from rover_x1.contracts import FrameContext, CLOCK_SYSTEM
from rover_x1.camera_input import (SourceStampGate, decode_depth, decode_rgb_sizes,
                                   validate_camera_config)
from workers.depth_camera import compute_profile
from workers.lane import detect_lane, load_profile as load_lane_profile
from workers.stop_line import StopLineVoter, detect_stop_line, load_profile as load_stop_profile
from workers.traffic_light import TrafficLightVoter, detect_hsv, load_profile as load_traffic_profile


def input_topics(cfg, outputs):
    """One subscription per topic, even when camera0 and camera2 are shared."""
    validate_camera_config(cfg)
    topics = {cfg.ros2_depth_topic: ('depth', ['depth'])}
    for role, topic in (('rgb', cfg.ros2_rgb_topic), ('traffic', cfg.ros2_rgb_topic),
                        ('lane', cfg.ros2_camera2_topic),
                        ('stop_line', cfg.ros2_camera2_topic)):
        if outputs.get(role) is not None:
            kind, roles = topics.setdefault(topic, ('rgb', []))
            if kind != 'rgb':
                raise ValueError('image type conflict')
            roles.append(role)
    return topics


class CameraPipeline:
    """Reuse detector functions; no robot, SDK, ROS graph or motion output."""
    def __init__(self, cfg, outputs, *, first_frames=None, stop_line_active=None,
                 clock=time.time, monotonic=time.monotonic, stream_ids=None,
                 output_sink=None, traffic_active=None):
        self.cfg = cfg
        self.outputs = outputs
        self.first_frames = first_frames or {}
        self.stop_line_active = stop_line_active
        self.traffic_active = traffic_active
        self.output_sink = output_sink
        self.current_frame = None
        self.clock, self.monotonic = clock, monotonic
        self.topics = input_topics(cfg, outputs)
        self.stream_ids = stream_ids or {topic: str(uuid4()) for topic in self.topics}
        self.gates = {topic: SourceStampGate() for topic in self.topics}
        self.sequences = {topic: 0 for topic in self.topics}
        self.accepted = {topic: 0 for topic in self.topics}
        self.rejected = {topic: 0 for topic in self.topics}
        self.last_warning = {}
        self.lane_profile = (load_lane_profile(cfg.lane_profile)
                             if outputs.get('lane') is not None else None)
        self.stop_profile = (load_stop_profile(cfg.stop_line_profile)
                             if outputs.get('stop_line') is not None else None)
        self.stop_voter = StopLineVoter(self.stop_profile) if self.stop_profile else None
        self.limits = {'depth': cfg.stale_s, 'rgb': cfg.traffic_state_stale_s,
                       'lane': cfg.lane_stale_s, 'stop_line': cfg.stop_line_stale_s,
                       'traffic': cfg.traffic_state_stale_s}
        self.traffic_profile = (load_traffic_profile(cfg.traffic_light_profile)
                                if outputs.get('traffic') is not None else None)
        self.traffic_voter = TrafficLightVoter(self.traffic_profile) if self.traffic_profile else None
        self.traffic_was_active = False
        self.traffic_enabled_since = float('inf')
        self.sync_controls()

    def sync_controls(self):
        active = self._enabled('traffic') and self.traffic_profile is not None
        if active != self.traffic_was_active:
            if active:
                self.traffic_voter = TrafficLightVoter(self.traffic_profile)
                self.traffic_enabled_since = self.clock()
            self.traffic_was_active = active

    def _enabled(self, role):
        if role == 'traffic':
            return self.traffic_active is None or self.traffic_active.is_set()
        return (role != 'stop_line' or self.stop_line_active is None
                or self.stop_line_active.is_set())

    def _fresh(self, role, ts, started):
        limit = self.limits[role]
        return (0 <= self.clock() - ts < limit
                and 0 <= self.monotonic() - started < limit)

    def _emit(self, role, value, started):
        if not self._enabled(role) or not self._fresh(role, value.ts, started):
            return False
        if self.output_sink is not None and role != "rgb":
            self.output_sink(role, value, self.current_frame)
        else:
            put_latest(self.outputs[role], value)
        event = self.first_frames.get(role)
        if event is not None and (role != 'depth' or value.valid):
            event.set()
        return True

    def receive(self, topic, msg):
        """Bounded DDS history + single-thread callbacks; reject instead of restamp."""
        started = self.monotonic()
        self.current_frame = None
        self.sync_controls()
        kind, configured_roles = self.topics[topic]
        roles = [role for role in configured_roles if self._enabled(role)]
        if not roles:
            return False
        try:
            ts = self.gates[topic].accept(msg, now=self.clock(),
                                          max_age_s=max(self.limits[r] for r in roles))
            self.sequences[topic] += 1
            self.current_frame = FrameContext(
                self.stream_ids[topic], self.sequences[topic], msg.header.frame_id,
                self.gates[topic].last_stamp_ns, CLOCK_SYSTEM)
            if kind == 'depth':
                result = compute_profile(decode_depth(msg), self.cfg, ts=ts)
                sent = self._emit('depth', result, started)
            else:
                data = bytes(msg.data)
                if not data or 'compresseddepth' in msg.format.lower():
                    raise ValueError('expected compressed RGB, not empty/depth data')
                roles = [r for r in roles if self._fresh(r, ts, started)
                         and (r != 'traffic' or ts >= self.traffic_enabled_since)]
                widths = []
                if 'lane' in roles:
                    widths.append(int(self.lane_profile.get('camera_width', 640)))
                if 'stop_line' in roles:
                    widths.append(int(self.stop_profile.get('camera_width', 640)))
                if 'traffic' in roles:
                    widths.append(int(self.traffic_profile.get('camera_width', 960)))
                # Decode once even for camera0-only input, so corrupt frames cannot
                # set health or escape into the traffic voter as fresh images.
                images = decode_rgb_sizes(data, widths)
                sent = False
                if 'rgb' in roles:
                    frame = RgbFrame(data, ts, msg.format, self.sequences[topic], msg.header.frame_id)
                    sent |= self._emit('rgb', frame, started)
                if 'lane' in roles:
                    width = int(self.lane_profile.get('camera_width', 640))
                    state = detect_lane(images[width], self.lane_profile, now=ts)
                    sent |= self._emit('lane', state, started)
                if 'stop_line' in roles and self._fresh('stop_line', ts, started):
                    width = int(self.stop_profile.get('camera_width', 640))
                    raw = detect_stop_line(images[width], self.stop_profile, now=ts)
                    if self._fresh('stop_line', ts, started):
                        state = self.stop_voter.update(raw, ts)
                        sent |= self._emit('stop_line', state, started)
                if 'traffic' in roles and self._fresh('traffic', ts, started):
                    width = int(self.traffic_profile.get('camera_width', 960))
                    raw = detect_hsv(images[width], self.traffic_profile, now=ts)
                    if self._fresh('traffic', ts, started) and self._enabled('traffic'):
                        state = self.traffic_voter.update(raw, ts)
                        sent |= self._emit('traffic', state, started)
            if sent:
                self.accepted[topic] += 1
            else:
                self.rejected[topic] += 1
            return sent
        except (ValueError, TypeError, RuntimeError, OverflowError) as exc:
            self.rejected[topic] += 1
            # Only a newly accepted capture stamp may carry detector failure.
            # Duplicate/stale input is not republished as a fresh failure frame.
            if self.output_sink is not None and self.current_frame is not None:
                failed_frame = replace(self.current_frame, detector_ok=False)
                ts = failed_frame.source_stamp_ns / 1e9
                empty = {'depth': Perception(float('inf'), False, ts),
                         'lane': LaneState(ts=ts), 'stop_line': StopLineState(ts=ts),
                         'traffic': TrafficLightState(ts=ts)}
                for role in roles:
                    if role in empty and self._fresh(role, ts, started) and self._enabled(role):
                        self.output_sink(role, empty[role], failed_frame)
            now = self.monotonic()
            if now - self.last_warning.get(topic, float('-inf')) >= self.cfg.warn_log_interval_s:
                logging.warning('[ROS相机] 丢弃 %s: %s', topic, exc)
                self.last_warning[topic] = now
            return False


def create_camera_node(pipeline, *, context):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage
    node_name = 'rover_x1_perception' if pipeline.cfg.perception_backend == 'ros2' else 'rover_x1_cameras'
    node = Node(node_name, context=context, use_global_arguments=False)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    try:
        if pipeline.cfg.perception_backend == 'ros2':
            from rover_x1.perception_io import create_output_sink
            pipeline.output_sink = create_output_sink(node, pipeline.cfg)
        for topic, (kind, _) in pipeline.topics.items():
            node.create_subscription(Image if kind == 'depth' else CompressedImage,
                                     topic, partial(pipeline.receive, topic), qos)
            logging.info('[ROS相机] 已订阅 %s (%s, best_effort, depth=1)', topic, kind)
    except BaseException:
        node.destroy_node()
        raise
    return node


class RosCameraProcess(multiprocessing.Process):
    def __init__(self, cfg, outputs, *, ready=None, first_frames=None,
                 detector_ready=None, traffic_active=None, name='ros_cameras'):
        super().__init__(name=name, daemon=True)
        self.cfg, self.outputs = cfg, outputs
        self.ready = ready
        self.traffic_active = traffic_active
        self.stream_ids = {topic: str(uuid4()) for topic in input_topics(cfg, outputs)}
        self.first_frames = first_frames or {}
        self.detector_ready = detector_ready or {}
        self._stop_event = multiprocessing.Event()
        self.stop_line_active = multiprocessing.Event()
        self.stop_line_active.set()

    def stop(self):
        self._stop_event.set()

    def disable_stop_line(self):
        # Keep the shared camera alive for depth and lane detection.
        self.stop_line_active.clear()

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        from rclpy.executors import SingleThreadedExecutor
        context = Context()
        node = executor = None
        try:
            pipeline = CameraPipeline(self.cfg, self.outputs, first_frames=self.first_frames,
                                      stop_line_active=self.stop_line_active,
                                      stream_ids=self.stream_ids, traffic_active=self.traffic_active)
            rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = create_camera_node(pipeline, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            for event in [self.ready, *self.detector_ready.values()]:
                if event is not None:
                    event.set()
            while not self._stop_event.is_set() and context.ok():
                pipeline.sync_controls()
                executor.spin_once(timeout_sec=.05)
        except BaseException:
            logging.exception('[ROS相机] 进程异常退出')
            raise
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1.)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
