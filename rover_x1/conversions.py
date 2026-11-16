"""Lossless legacy/ROS conversion. Import generated messages only on demand.

No subscriptions or output commands; callers must apply health/freshness gates.
"""
import math
from dataclasses import fields

from messages import (Perception, LaneState, StopLineState, TrafficLightState,
                      CommandResult, VoiceCue, VoiceDone)
from rover_x1.contracts import FrameContext, TaskKey

STATE_TYPES = {
    Perception: 'ObstacleProfile', LaneState: 'LaneObservation',
    StopLineState: 'StopLineObservation', TrafficLightState: 'TrafficLightObservation',
}
EVENT_TYPES = {CommandResult: 'MotionResult', VoiceCue: 'VoiceEvent', VoiceDone: 'VoiceResult'}


def _messages():
    from rover_x1_interfaces import msg
    return msg


def _validate(value):
    if hasattr(value, 'ts') and (not math.isfinite(value.ts) or value.ts < 0):
        raise ValueError('legacy ts must be finite nonnegative wall-clock seconds')
    if hasattr(value, 'bbox') and (len(value.bbox) not in (0, 4)
                                   or any(type(n) is not int or n < 0 for n in value.bbox)):
        raise ValueError('bbox must be empty or nonnegative ROI x/y/w/h pixels')
    if isinstance(value, TrafficLightState) and value.color not in ('red', 'green', 'yellow', 'unknown'):
        raise ValueError('unknown traffic light color')
    if isinstance(value, CommandResult) and value.status not in ('running', 'ok', 'failed', 'cancelled'):
        raise ValueError('unknown motion result status')
    if isinstance(value, Perception):
        for distance, valid in ((value.distance_m, value.valid),
                                (value.left_m, value.left_valid), (value.right_m, value.right_valid)):
            if valid and (not math.isfinite(distance) or distance < 0):
                raise ValueError('valid depth must be finite and nonnegative')


def _copy_to(value, target, *, omit=()):
    _validate(value)
    for field in fields(value):
        if field.name in omit:
            continue
        item = getattr(value, field.name)
        if isinstance(item, tuple):
            item = list(item)
        # Legacy constructors sometimes supply int values for float fields.
        if field.type is float:
            item = float(item)
        setattr(target, 'legacy_ts' if field.name == 'ts' else field.name, item)
    return target


def _copy_from(source, cls, **overrides):
    values = {}
    for field in fields(cls):
        if field.name in overrides:
            values[field.name] = overrides[field.name]
        else:
            item = getattr(source, 'legacy_ts' if field.name == 'ts' else field.name)
            values[field.name] = tuple(item) if field.name == 'bbox' else item
    value = cls(**values)
    _validate(value)
    return value


def _encode_frame(frame):
    msg = _messages().ObservationHeader()
    msg.stream_id, msg.sequence = frame.stream_id, frame.sequence
    msg.source.frame_id = frame.frame_id
    msg.source_stamp_valid = frame.source_stamp_ns is not None
    if msg.source_stamp_valid:
        msg.source.stamp.sec, msg.source.stamp.nanosec = divmod(frame.source_stamp_ns, 1_000_000_000)
    msg.clock_domain, msg.detector_ok = frame.clock_domain, frame.detector_ok
    return msg


def _decode_frame(msg):
    sec, ns = msg.source.stamp.sec, msg.source.stamp.nanosec
    if not 0 <= ns < 1_000_000_000:
        raise ValueError('invalid nanoseconds')
    if not msg.source_stamp_valid and (sec or ns):
        raise ValueError('missing source stamp must be zero')
    stamp = sec * 1_000_000_000 + ns if msg.source_stamp_valid else None
    return FrameContext(msg.stream_id, msg.sequence, msg.source.frame_id,
                        stamp, msg.clock_domain, msg.detector_ok)


def state_to_ros(value, frame):
    """frame must come from acquisition; legacy ts never becomes a capture stamp."""
    cls = STATE_TYPES.get(type(value))
    if cls is None:
        raise TypeError(f'unsupported state: {type(value)}')
    msg = _copy_to(value, getattr(_messages(), cls)())
    msg.observation = _encode_frame(frame)
    return msg


def state_from_ros(msg):
    for cls, name in STATE_TYPES.items():
        if isinstance(msg, getattr(_messages(), name)):
            return _copy_from(msg, cls), _decode_frame(msg.observation)
    raise TypeError(f'unsupported observation: {type(msg)}')


def _encode_task(key):
    return _messages().TaskIdentity(session_id=key.session_id,
                                   generation=key.generation, goal_id=key.goal_id)


def event_to_ros(value, task):
    cls = EVENT_TYPES.get(type(value))
    if cls is None:
        raise TypeError(f'unsupported event: {type(value)}')
    if isinstance(value, CommandResult) and value.generation != task.generation:
        raise ValueError('result generation does not match its task')
    msg = _copy_to(value, getattr(_messages(), cls)(), omit=('generation',))
    msg.task = _encode_task(task)
    return msg


def event_from_ros(msg, *, expected_task):
    """Reject a late result from a previous session, generation, or goal."""
    for cls, name in EVENT_TYPES.items():
        if isinstance(msg, getattr(_messages(), name)):
            key = TaskKey(msg.task.session_id, msg.task.generation, msg.task.goal_id)
            if key != expected_task:
                raise ValueError('event does not belong to the active task')
            overrides = {'generation': key.generation} if cls is CommandResult else {}
            return _copy_from(msg, cls, **overrides)
    raise TypeError(f'unsupported event: {type(msg)}')
