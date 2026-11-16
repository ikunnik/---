"""ROS perception provenance, fault handling and end-to-end synthetic transport."""
import multiprocessing
import os
import queue
import sys
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from comm import Latest
from messages import Perception, LaneState, StopLineState, TrafficLightState
from rover_x1.contracts import FrameContext, CLOCK_SYSTEM
from rover_x1.conversions import state_to_ros
from rover_x1.perception_io import PerceptionReceiver, roles_for, source_topic, topic_for
from runtime import assembly, lifecycle
from workers.ros_camera import CameraPipeline
from workers.traffic_light import TrafficLightVoter, detect_hsv
from workers.depth_camera import compute_profile
from rover_x1.camera_input import decode_depth, decode_rgb_sizes
from tools.test_ros_camera import ros_cfg, depth_msg, rgb_msg


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ros_cfg(perception_backend='ros2')
        self.wall, self.mono = 100.1, 10.
        self.streams = {name: str(uuid4()) for name in ('/test/depth', '/test/rgb', '/test/camera2')}
        self.outputs = {role: Latest() for role in roles_for(self.cfg)}
        self.first = {role: threading.Event() for role in self.outputs}
        self.traffic = threading.Event(); self.traffic.set()
        self.receiver = PerceptionReceiver(self.cfg, self.outputs, self.streams,
            first_frames=self.first, controls={'traffic': self.traffic},
            clock=lambda: self.wall, monotonic=lambda: self.mono)

    def frame(self, role='depth', sequence=1, ts=100., **changes):
        return replace(FrameContext(self.streams[source_topic(self.cfg, role)], sequence,
                                    'test_optical', round(ts*1e9), CLOCK_SYSTEM), **changes)

    def test_all_states_round_trip_to_decision_whiteboards(self):
        values = dict(depth=Perception(1.5, True, 100.), lane=LaneState(False, ts=100.),
                      stop_line=StopLineState(False, ts=100.), traffic=TrafficLightState('red', True, 100.))
        for role, state in values.items():
            self.assertTrue(self.receiver.receive(role, state_to_ros(state, self.frame(role))))
            self.assertEqual(self.outputs[role].get()[0], state)
            self.assertTrue(self.first[role].is_set())

    def test_wrong_session_repeated_reordered_old_and_conflicting_times_rejected(self):
        state = Perception(1.5, True, 100.)
        frame = self.frame()
        self.assertTrue(self.receiver.receive('depth', state_to_ros(state, frame)))
        samples = [(state, frame), (state, replace(frame, sequence=0)),
                   (replace(state, ts=100.05), self.frame(sequence=2, ts=100.05, stream_id=str(uuid4()))),
                   (replace(state, ts=99.), self.frame(sequence=2, ts=99.)),
                   (replace(state, ts=100.05), self.frame(sequence=2, ts=100.06)),
                   (replace(state, ts=100.2), self.frame(sequence=2, ts=100.2))]
        for value, provenance in samples:
            with self.subTest(value=value, provenance=provenance):
                self.assertFalse(self.receiver.receive('depth', state_to_ros(value, provenance)))
                self.assertEqual(self.outputs['depth'].get()[0], state)
        self.mono += 1.
        self.receiver.expire()
        self.assertIsNone(self.outputs['depth'].get()[0])

    def test_detection_failure_clears_instead_of_looking_like_free_space(self):
        self.receiver.receive('lane', state_to_ros(LaneState(True, ts=100.), self.frame('lane')))
        bad = state_to_ros(LaneState(False, ts=100.05), self.frame('lane', 2, 100.05, detector_ok=False))
        self.assertFalse(self.receiver.receive('lane', bad))
        self.assertIsNone(self.outputs['lane'].get()[0])
        good = state_to_ros(LaneState(False, ts=100.06), self.frame('lane', 3, 100.06))
        self.assertTrue(self.receiver.receive('lane', good))
        self.assertFalse(self.outputs['lane'].get()[0].visible)

    def test_clock_jumps_and_disabled_branch_clear_latest(self):
        self.receiver.receive('traffic', state_to_ros(TrafficLightState('green', True, 100.), self.frame('traffic')))
        self.traffic.clear(); self.receiver.expire()
        self.assertIsNone(self.outputs['traffic'].get()[0])
        self.assertFalse(self.receiver.receive('traffic', state_to_ros(TrafficLightState(ts=100.05), self.frame('traffic', 2, 100.05))))
        self.receiver.receive('depth', state_to_ros(Perception(1., True, 100.), self.frame()))
        self.wall = 99.; self.receiver.expire()
        self.assertIsNone(self.outputs['depth'].get()[0])

    def test_nan_and_topic_type_mismatch_cannot_reach_decision(self):
        bad = state_to_ros(LaneState(heading_deg=float('nan'), ts=100.), self.frame('lane'))
        self.assertFalse(self.receiver.receive('lane', bad))
        wrong = state_to_ros(StopLineState(ts=100.), self.frame('lane'))
        self.assertFalse(self.receiver.receive('lane', wrong))
        self.assertIsNone(self.outputs['lane'].get()[0])

    def test_pipeline_publishes_exact_capture_identity_and_no_parallel_queue_output(self):
        outputs = {role: queue.Queue(1) for role in ('depth', 'lane', 'stop_line', 'traffic')}
        self.traffic.clear()
        records = []
        pipeline = CameraPipeline(self.cfg, outputs, stream_ids=self.streams,
            output_sink=lambda *args: records.append(args), traffic_active=self.traffic,
            clock=lambda: self.wall, monotonic=lambda: self.mono)
        self.assertTrue(pipeline.receive('/test/depth', depth_msg()))
        self.assertTrue(pipeline.receive('/test/camera2', rgb_msg()))
        self.assertEqual({r[0] for r in records}, {'depth', 'lane', 'stop_line'})
        self.assertEqual(records[0][1], compute_profile(decode_depth(depth_msg()), self.cfg, ts=100.))
        self.assertEqual(records[0][2].source_stamp_ns, 100_000_000_000)
        self.assertEqual(records[0][2].stream_id, self.streams['/test/depth'])
        lane, line = records[1:]
        self.assertEqual(lane[2], line[2])
        self.assertTrue(all(out.empty() for out in outputs.values()))
        records.clear()
        self.assertFalse(pipeline.receive('/test/camera2', rgb_msg()))
        self.assertEqual(records, [])
        broken = rgb_msg(100.05); broken.data = b'broken'
        self.assertFalse(pipeline.receive('/test/camera2', broken))
        self.assertEqual({r[0] for r in records}, {'lane', 'stop_line'})
        self.assertTrue(all(not r[2].detector_ok for r in records))

    def test_traffic_activation_restarts_votes_and_excludes_pre_activation_frames(self):
        outputs = {'depth': queue.Queue(1), 'traffic': queue.Queue(1)}
        self.traffic.clear(); records = []
        pipeline = CameraPipeline(self.cfg, outputs, output_sink=lambda *a: records.append(a),
            traffic_active=self.traffic, clock=lambda: self.wall, monotonic=lambda: self.mono)
        self.assertFalse(pipeline.receive('/test/rgb', rgb_msg()))
        self.traffic.set(); pipeline.sync_controls()
        self.assertFalse(pipeline.receive('/test/rgb', rgb_msg(100.05)))
        self.wall = 100.2
        frame = rgb_msg(100.15)
        self.assertTrue(pipeline.receive('/test/rgb', frame))
        profile = pipeline.traffic_profile
        rgb = decode_rgb_sizes(frame.data, [int(profile.get('camera_width', 960))])
        raw = detect_hsv(next(iter(rgb.values())), profile, now=100.15)
        expected = TrafficLightVoter(profile).update(raw, 100.15)
        self.assertEqual(records[-1][1], expected)
        old_voter = pipeline.traffic_voter
        self.traffic.clear(); pipeline.sync_controls()
        self.traffic.set(); pipeline.sync_controls()
        self.assertIsNot(old_voter, pipeline.traffic_voter)

    def test_runtime_has_single_state_writer_and_preserves_queue_fallback(self):
        for mode in ('mission', 'avoidance', 'traffic', 'lane'):
            rt = assembly.build(ros_cfg(mode, perception_backend='ros2'), NS(traffic_light_now=False), Mock())
            try:
                self.assertIsNotNone(rt.ros_perceiver)
                self.assertTrue(all(w is None for w in (rt.perceiver, rt.lane_worker, rt.stop_line_worker,
                                                       rt.traffic_worker, rt.traffic_light)))
                self.assertEqual(rt.camera.first_frames, {})
                self.assertEqual(set(rt.ros_perceiver.receiver.outputs), set(roles_for(rt.cfg)))
            finally:
                for name, value in vars(rt).items():
                    if name.endswith('_queue') and value is not None: value.close(); value.join_thread()
        with self.assertRaises(ValueError): ros_cfg(camera_backend='dds', perception_backend='ros2')
        with self.assertRaises(ValueError): ros_cfg(perception_backend='invalid')
        with self.assertRaises(ValueError): ros_cfg(perception_backend='ros2', ros2_perception_namespace='relative')

    def test_dead_receiver_causes_runtime_exit(self):
        rt = NS(cfg=self.cfg, supervisor=Mock(), decision=Mock(), executor=Mock(), ros_perceiver=Mock())
        rt.supervisor.snapshot.return_value = (NS(mode='avoidance'),)
        rt.ros_perceiver.is_alive.return_value = False
        stop = Mock(); stop.wait.side_effect = [False, True]
        lifecycle.watch(rt, stop)
        stop.set.assert_called_once()
        rt.ros_perceiver.receiver.clear_all.assert_called_once()


class PerceptionTransportTests(unittest.TestCase):
    def test_images_to_ros_observations_to_legacy_input_and_expiry(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image, CompressedImage
        multiprocessing.set_start_method('spawn', force=True)
        prefix = '/perception_test_' + uuid4().hex
        cfg = ros_cfg(perception_backend='ros2', ros2_perception_namespace=prefix+'/observations',
                      ros2_depth_topic=prefix+'/depth', ros2_rgb_topic=prefix+'/rgb',
                      ros2_camera2_topic=prefix+'/rgb')
        rt = assembly.build(cfg, NS(traffic_light_now=False), Mock())
        context = Context(); node = None; camera_started = reader_started = False
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '233', 'ROS_LOCALHOST_ONLY': '1',
                                     'ROS_LOG_DIR': '/tmp/rover_perception_test_logs', 'CYCLONEDDS_URI': ''}):
            try:
                rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
                node = Node('perception_image_fixture', context=context, use_global_arguments=False)
                depth_pub = node.create_publisher(Image, cfg.ros2_depth_topic, qos_profile_sensor_data)
                rgb_pub = node.create_publisher(CompressedImage, cfg.ros2_rgb_topic, qos_profile_sensor_data)
                d, c = depth_msg(), rgb_msg()
                depth = Image(height=d.height, width=d.width, encoding=d.encoding, step=d.step, data=d.data)
                rgb = CompressedImage(format=c.format, data=c.data)
                def publish():
                    ns = time.time_ns()
                    for msg in (depth, rgb):
                        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
                        msg.header.frame_id = 'synthetic_camera'
                    depth_pub.publish(depth); rgb_pub.publish(rgb)
                def until(condition, timeout=8.):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        publish()
                        if condition(): return True
                        time.sleep(.05)
                    return False
                rt.camera.start(); camera_started = True
                self.assertTrue(rt.camera_ready.wait(10))
                publish(); time.sleep(.15)
                self.assertFalse(rt.camera_first_frame.is_set())  # Publishing != decision readiness.
                rt.ros_perceiver.start(); reader_started = True
                self.assertTrue(rt.ros_perceiver.ready.wait(5))
                self.assertTrue(until(lambda: all(e.is_set() for e in
                    (rt.camera_first_frame, rt.lane_first_frame, rt.stop_line_first_frame))))
                self.assertEqual(rt.perception.get()[0].distance_m, 1.5)
                self.assertIsNotNone(rt.lane_states.get()[0])
                self.assertIsNotNone(rt.stop_line_states.get()[0])
                self.assertIsNone(rt.traffic_states.get()[0])
                self.assertTrue(rt.depth_queue.empty() and rt.lane_queue.empty() and rt.stop_line_queue.empty())
                rt.traffic_active.set()
                self.assertTrue(until(lambda: rt.traffic_states.get()[0] is not None))
                rt.camera.disable_stop_line(); rt.traffic_active.clear()
                self.assertTrue(until(lambda: rt.stop_line_states.get()[0] is None
                                     and rt.traffic_states.get()[0] is None))
                self.assertIsNotNone(rt.lane_states.get()[0])
                deadline = time.monotonic() + 3.
                while time.monotonic() < deadline and any(
                        out.get()[0] is not None for out in (rt.perception, rt.lane_states)):
                    time.sleep(.05)
                self.assertIsNone(rt.perception.get()[0])
                self.assertIsNone(rt.lane_states.get()[0])
                self.assertEqual(depth_pub.get_subscription_count(), 1)
                self.assertEqual(rgb_pub.get_subscription_count(), 1)
                rt.robot.assert_not_called()
            finally:
                if camera_started:
                    rt.camera.stop(); rt.camera.join(5)
                    if rt.camera.is_alive(): rt.camera.kill(); rt.camera.join(2)
                if reader_started:
                    rt.ros_perceiver.stop(); rt.ros_perceiver.join(3)
                if node is not None: node.destroy_node()
                if context.ok(): context.shutdown()
                for name, value in vars(rt).items():
                    if name.endswith('_queue') and value is not None: value.close(); value.join_thread()
            self.assertEqual(rt.camera.exitcode, 0)
            self.assertFalse(rt.ros_perceiver.is_alive())


if __name__ == '__main__':
    unittest.main(verbosity=2)
