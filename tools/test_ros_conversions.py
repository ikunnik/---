"""Real ROS serialization round trips. No nodes, DDS, SDK or hardware."""
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rclpy.serialization import serialize_message, deserialize_message
from messages import (Perception, LaneState, StopLineState, TrafficLightState,
                      CommandResult, VoiceCue, VoiceDone)
from rover_x1.contracts import FrameContext, TaskKey, CLOCK_SYSTEM
from rover_x1.conversions import (state_to_ros, state_from_ros, event_to_ros, event_from_ros)


def wire(msg):
    return deserialize_message(serialize_message(msg), type(msg))


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.frame = FrameContext(str(uuid4()), 10, 'camera2_optical_frame',
                                  1_700_000_000_123_456_789, CLOCK_SYSTEM)
        self.task = TaskKey.new(str(uuid4()), 7)

    def test_all_observations_preserve_every_field_over_ros_serialization(self):
        states = [
            Perception(.7, True, 123., .8, float('inf'), True, False, .7, .25),
            Perception(float('inf'), False, 123.),
            LaneState(True, .4, .3, -12., .2, .3, .5, 123., .8, True, 'left', False),
            StopLineState(True, .83, 123., .1, (1, 2, 3, 4), -15.),
            TrafficLightState('green', True, 123., (5, 6, 7, 8), .1),
            TrafficLightState(), LaneState(), StopLineState(),
        ]
        for state in states:
            with self.subTest(state=state):
                decoded, frame = state_from_ros(wire(state_to_ros(state, self.frame)))
                self.assertEqual(decoded, state)
                self.assertEqual(frame, self.frame)

    def test_missing_source_is_not_replaced_by_legacy_time(self):
        frame = FrameContext(self.frame.stream_id, 1, detector_ok=False)
        msg = wire(state_to_ros(LaneState(False, ts=123.), frame))
        self.assertEqual(msg.observation.source.stamp.sec, 0)
        self.assertFalse(msg.observation.source_stamp_valid)
        decoded, recovered = state_from_ros(msg)
        self.assertEqual(decoded.ts, 123.)
        self.assertEqual(recovered, frame)

    def test_all_events_preserve_fields_and_match_task(self):
        for value in (CommandResult(7, 'advance', 'failed', 'timeout', 123.),
                      VoiceCue('bypass', 'left', 45., 'obstacle', 9),
                      VoiceDone(9, False, 'missing_file', 123.)):
            with self.subTest(value=value):
                self.assertEqual(event_from_ros(wire(event_to_ros(value, self.task)),
                                                expected_task=self.task), value)

    def test_late_results_rejected_even_with_same_tag_or_voice_sequence(self):
        for value in (CommandResult(7, 'advance', 'ok', ts=123.), VoiceDone(9)):
            msg = wire(event_to_ros(value, self.task))
            for changes in ({'session_id': str(uuid4())}, {'generation': 8},
                            {'goal_id': str(uuid4())}):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    event_from_ros(msg, expected_task=replace(self.task, **changes))

    def test_malformed_state_and_result_are_rejected(self):
        for value in (Perception(float('inf'), True, 1.),
                      StopLineState(bbox=(1, 2, 3)), TrafficLightState('purple'),
                      LaneState(ts=float('nan'))):
            with self.subTest(value=value), self.assertRaises(ValueError):
                state_to_ros(value, self.frame)
        with self.assertRaises(ValueError):
            event_to_ros(CommandResult(6, 'advance', 'ok'), self.task)
        msg = state_to_ros(LaneState(), self.frame)
        msg.observation.source_stamp_valid = False
        with self.assertRaises(ValueError):
            state_from_ros(msg)
        msg = event_to_ros(CommandResult(7, 'advance', 'ok'), self.task)
        msg.status = 'unrecognised'
        with self.assertRaises(ValueError):
            event_from_ros(msg, expected_task=self.task)


if __name__ == '__main__':
    unittest.main(verbosity=2)
