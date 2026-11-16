"""Version 1 ROS boundary contracts. No ROS or hardware imports here."""
import math
from dataclasses import dataclass
from uuid import UUID, uuid4

UINT64_MAX = (1 << 64) - 1
CLOCK_UNKNOWN, CLOCK_SYSTEM, CLOCK_ROS, CLOCK_DEVICE = range(4)


def _uuid(value):
    if not isinstance(value, str):
        raise ValueError('identity must be a UUID string')
    try:
        parsed = UUID(value)
    except (ValueError, AttributeError) as exc:
        raise ValueError('identity must be a UUID string') from exc
    if not parsed.int or str(parsed) != value:
        raise ValueError('identity must be a canonical nonzero UUID')


def _uint64(value):
    if type(value) is not int or not 0 <= value <= UINT64_MAX:
        raise ValueError('sequence/generation must be uint64')


@dataclass(frozen=True)
class TaskKey:
    session_id: str
    generation: int
    goal_id: str

    def __post_init__(self):
        _uuid(self.session_id)
        _uuid(self.goal_id)
        _uint64(self.generation)

    @classmethod
    def new(cls, session_id, generation):
        return cls(session_id, generation, str(uuid4()))


@dataclass(frozen=True)
class FrameContext:
    stream_id: str
    sequence: int
    frame_id: str = ''
    source_stamp_ns: int | None = None
    clock_domain: int = CLOCK_UNKNOWN
    detector_ok: bool = True

    def __post_init__(self):
        _uuid(self.stream_id)
        _uint64(self.sequence)
        if not isinstance(self.frame_id, str) or type(self.detector_ok) is not bool:
            raise ValueError('frame_id must be a string and detector_ok a bool')
        if type(self.clock_domain) is not int or self.clock_domain not in range(4):
            raise ValueError('unknown source clock domain')
        if self.source_stamp_ns is None:
            if self.clock_domain != CLOCK_UNKNOWN:
                raise ValueError('missing stamp must have CLOCK_UNKNOWN')
        elif (type(self.source_stamp_ns) is not int
              or not 0 <= self.source_stamp_ns < (1 << 31) * 1_000_000_000
              or self.clock_domain == CLOCK_UNKNOWN):
            raise ValueError('capture stamp requires a known clock and ROS Time range')


class FrameFreshness:
    """Consumer-local progress gate for ONE acquisition session.

    A new session requires explicit reset (new instance); never auto-accept a
    replayed stream. Source stamps are compared only within the same clock.
    This detects progress, not the transport age of the very first frame.
    """
    def __init__(self, stream_id):
        _uuid(stream_id)
        self.stream_id = stream_id
        self.last_sequence = -1
        self.last_source_ns = None
        self.clock_domain = CLOCK_UNKNOWN
        self.received_monotonic = None
        self.healthy = False

    def observe(self, frame, *, now_monotonic):
        if not math.isfinite(now_monotonic):
            raise ValueError('receive time must be finite monotonic seconds')
        if (frame.stream_id != self.stream_id or frame.sequence <= self.last_sequence
                or (self.received_monotonic is not None
                    and now_monotonic < self.received_monotonic)):
            return False
        if self.last_source_ns is not None:
            if (frame.source_stamp_ns is None or frame.clock_domain != self.clock_domain
                    or frame.source_stamp_ns <= self.last_source_ns):
                return False
        self.last_sequence = frame.sequence
        self.last_source_ns = frame.source_stamp_ns
        self.clock_domain = frame.clock_domain
        self.received_monotonic = now_monotonic
        self.healthy = frame.detector_ok
        return True

    def fresh(self, *, now_monotonic, max_age_s):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('max_age_s must be positive finite seconds')
        return (self.healthy and self.received_monotonic is not None
                and math.isfinite(now_monotonic)
                and 0 <= now_monotonic - self.received_monotonic < max_age_s)
