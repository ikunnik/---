"""Pure image pipeline and runtime ownership tests; no graph or hardware."""
import queue
import sys
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import CFG
from rover_x1.camera_input import SourceStampGate, decode_depth, decode_rgb_sizes
from rover_x1.settings import build_settings
from workers.depth_camera import compute_profile
from workers.lane import detect_lane
from workers.stop_line import detect_stop_line, StopLineVoter
from workers.traffic_light import decode_jpeg, detect_ros_frame, TrafficLightVoter, load_profile
from workers.ros_camera import CameraPipeline
from messages import RgbFrame
from runtime import assembly, lifecycle, readiness


def ros_cfg(mode='mission', **changes):
    values = dict(camera_backend='ros2', ros2_depth_topic='/test/depth',
                  ros2_rgb_topic='/test/rgb', ros2_camera2_topic='/test/camera2',
                  voice_enabled=False)
    values.update(changes)
    return build_settings(values, mode=mode, dry_run=True, traffic_light_now=False,
                          share_dir=ROOT)[0]


def header(ts=100.):
    ns = round(ts * 1e9)
    sec, nano = divmod(ns, 10**9)
    return NS(stamp=NS(sec=sec, nanosec=nano), frame_id='camera_optical_frame')


def depth_msg(array=None, ts=100., encoding='16UC1', endian=False, padding=0):
    if array is None:
        array = np.full((80, 100), 1500, np.uint16)
    dtype = ('>' if endian else '<') + ('u2' if encoding == '16UC1' else 'f4')
    raw = b''.join(row.astype(dtype).tobytes() + b'\x00' * padding for row in array)
    return NS(header=header(ts), width=array.shape[1], height=array.shape[0], encoding=encoding,
              is_bigendian=int(endian), step=array.shape[1] * np.dtype(dtype).itemsize + padding,
              data=raw)


def rgb_msg(ts=100.):
    bgr = np.zeros((120, 160, 3), np.uint8)
    bgr[80:110, 10:150] = (0, 255, 255)
    ok, data = cv2.imencode('.png', bgr)
    assert ok
    return NS(header=header(ts), format='png', data=data.tobytes())


class RosCameraTests(unittest.TestCase):
    def pipeline(self, **changes):
        cfg = ros_cfg(**changes)
        outputs = {name: queue.Queue(1) for name in ('depth', 'rgb', 'lane', 'stop_line')}
        first = {name: threading.Event() for name in outputs}
        active = threading.Event(); active.set()
        pipeline = CameraPipeline(cfg, outputs, first_frames=first, stop_line_active=active,
                                  clock=lambda: 100.1, monotonic=lambda: 10.)
        return pipeline, outputs, first, active

    def test_depth_units_endian_padding_match_existing_profile(self):
        array = np.full((80, 100), 1250, np.uint16)
        expected = compute_profile(array, CFG, ts=100.)
        for encoding, data, padding in (('16UC1', array, 4), ('32FC1', array/1000., 8)):
            for endian in (False, True):
                with self.subTest(encoding=encoding, endian=endian):
                    decoded = decode_depth(depth_msg(data, encoding=encoding, endian=endian, padding=padding))
                    self.assertEqual(compute_profile(decoded, CFG, ts=100.), expected)
        floats = np.array([[np.nan, np.inf, -1, 0, 1.25]], np.float32)
        np.testing.assert_array_equal(decode_depth(depth_msg(floats, encoding='32FC1')), [[0, 0, 0, 0, 1250]])

    def test_malformed_depth_is_rejected(self):
        for changes in ({'step': 1}, {'data': b'bad'}, {'width': 0}, {'encoding': 'mono8'}, {'step': 201}):
            msg = depth_msg()
            for name, value in changes.items(): setattr(msg, name, value)
            with self.subTest(changes=changes), self.assertRaises(ValueError): decode_depth(msg)

    def test_source_clock_rejects_missing_future_old_repeat_and_time_reset(self):
        gate = SourceStampGate()
        self.assertEqual(gate.accept(depth_msg(), now=100.1, max_age_s=.5), 100.)
        for ts in (0, 99, 100, 100.3, 1.):
            with self.subTest(ts=ts), self.assertRaises(ValueError):
                gate.accept(depth_msg(ts=ts), now=100.2, max_age_s=.5)
        self.assertEqual(gate.accept(depth_msg(ts=100.1), now=100.2, max_age_s=.5), 100.1)
        with self.assertRaises(ValueError):
            SourceStampGate().accept(depth_msg(ts=99.), now=100., max_age_s=.5)

    def test_camera2_decodes_once_and_preserves_detector_outputs_and_capture_time(self):
        pipeline, out, events, _ = self.pipeline()
        msg = rgb_msg()
        with patch('workers.ros_camera.decode_rgb_sizes', wraps=decode_rgb_sizes) as decoder:
            self.assertTrue(pipeline.receive('/test/camera2', msg))
            decoder.assert_called_once()
        rgb = decode_jpeg(msg.data, 640)
        lane = detect_lane(rgb, pipeline.lane_profile, now=100.)
        stop = StopLineVoter(pipeline.stop_profile).update(
            detect_stop_line(rgb, pipeline.stop_profile, now=100.), 100.)
        self.assertEqual(out['lane'].get_nowait(), lane)
        self.assertEqual(out['stop_line'].get_nowait(), stop)
        self.assertTrue(events['lane'].is_set() and events['stop_line'].is_set())
        self.assertFalse(pipeline.receive('/test/camera2', msg))
        self.assertTrue(out['lane'].empty() and out['stop_line'].empty())

    def test_shared_rgb_topic_has_one_subscription_and_resize_matches_legacy(self):
        pipeline, _, _, _ = self.pipeline(ros2_rgb_topic='/test/camera2')
        self.assertEqual(len(pipeline.topics), 2)
        self.assertEqual(set(pipeline.topics['/test/camera2'][1]), {'rgb', 'lane', 'stop_line'})
        msg = rgb_msg()
        for width, rgb in decode_rgb_sizes(msg.data, (320, 640, 320)).items():
            np.testing.assert_array_equal(rgb, decode_jpeg(msg.data, width))

    def test_stopping_stop_line_keeps_lane_and_depth_alive(self):
        pipeline, out, first, active = self.pipeline()
        active.clear()
        self.assertTrue(pipeline.receive('/test/camera2', rgb_msg()))
        self.assertTrue(out['stop_line'].empty())
        self.assertFalse(first['stop_line'].is_set())
        self.assertFalse(out['lane'].empty())
        self.assertTrue(pipeline.receive('/test/depth', depth_msg()))
        self.assertTrue(first['depth'].is_set())

    def test_bad_or_expired_images_do_not_mark_ready(self):
        pipeline, out, first, _ = self.pipeline()
        msg = rgb_msg(); msg.data = b'corrupt'
        self.assertFalse(pipeline.receive('/test/camera2', msg))
        self.assertFalse(pipeline.receive('/test/depth', depth_msg(ts=99.)))
        self.assertFalse(any(e.is_set() for e in first.values()))
        with patch('workers.ros_camera.decode_rgb_sizes', side_effect=lambda *a: (setattr(pipeline, 'clock', lambda: 101.) or decode_rgb_sizes(*a))):
            self.assertFalse(pipeline.receive('/test/camera2', rgb_msg(100.05)))
        self.assertTrue(all(q.empty() for q in out.values()))

    def test_invalid_depth_does_not_satisfy_first_frame(self):
        pipeline, out, first, _ = self.pipeline()
        self.assertTrue(pipeline.receive('/test/depth', depth_msg(np.zeros((80, 100), np.uint16))))
        self.assertFalse(out['depth'].get_nowait().valid)
        self.assertFalse(first['depth'].is_set())

    def test_mode_checks_required_ros_topics_without_changing_dds_defaults(self):
        self.assertEqual(CFG.camera_backend, 'dds')
        for values in ({'ros2_depth_topic': ''}, {'ros2_camera2_topic': ''},
                       {'ros2_rgb_topic': ''}, {'ros2_rgb_topic': '/test/depth'},
                       {'ros2_depth_topic': '/bad topic'}, {'camera_backend': 'invalid'}):
            with self.subTest(values=values), self.assertRaises(ValueError): ros_cfg(**values)
        cfg = ros_cfg('avoidance', ros2_camera2_topic='', ros2_rgb_topic='')
        self.assertFalse(cfg.stop_line_enabled or cfg.rgb_enabled)

    def test_runtime_selects_only_one_camera_owner_in_every_mode(self):
        for mode in ('mission', 'avoidance', 'traffic', 'lane'):
            rt = assembly.build(ros_cfg(mode), NS(traffic_light_now=False), Mock())
            try:
                from workers.ros_camera import RosCameraProcess
                self.assertIsInstance(rt.camera, RosCameraProcess)
                self.assertIsNone(rt.rgb_camera)
                self.assertIsNone(rt.lane_camera)
                self.assertIsNone(rt.stop_line)
                self.assertIsNotNone(rt.camera_ready)
            finally:
                for name, value in vars(rt).items():
                    if name.endswith('_queue') and value is not None:
                        value.close(); value.join_thread()

    def test_lifecycle_closes_stop_line_branch_without_stopping_shared_camera(self):
        rt = NS(cfg=ros_cfg(), camera=Mock(), stop_line_worker=Mock(), stop_line=None,
                traffic_active=threading.Event(), supervisor=Mock())
        rt.supervisor.snapshot.return_value = (NS(mode='lane'),)
        rt.stop_line_worker.is_alive.return_value = False
        lifecycle._close_stop_line(rt)
        rt.camera.disable_stop_line.assert_called_once()
        rt.camera.stop.assert_not_called()

    def test_camera_process_failure_stops_runtime_and_invalidates_states(self):
        rt = NS(cfg=ros_cfg(), supervisor=Mock(), decision=Mock(), executor=Mock(),
                camera=Mock(), perception=Mock(), lane_states=Mock(), stop_line_states=Mock())
        rt.supervisor.snapshot.return_value = (NS(mode='avoidance'),)
        rt.camera.is_alive.return_value = False
        stop = Mock()
        stop.wait.side_effect = [False, True]
        lifecycle.watch(rt, stop)
        stop.set.assert_called_once()
        for state in (rt.perception, rt.lane_states, rt.stop_line_states):
            state.set.assert_called_once_with(None)

    def test_readiness_uses_shared_camera_and_requires_depth(self):
        cfg = ros_cfg()
        rt = NS(cfg=cfg, camera=object(), camera_ready=object(), camera_first_frame=object(),
                perception=object(), lane_ready=object(), lane_first_frame=object(), lane_states=object(),
                stop_line_ready=object(), stop_line_first_frame=object(), stop_line_states=object())
        with patch.object(readiness, '_wait_detector_ready', return_value=True) as wait:
            readiness.wait(rt)
            self.assertEqual(wait.call_count, 3)
            self.assertTrue(all(c.args[0] is rt.camera for c in wait.call_args_list))
            self.assertTrue(wait.call_args_list[0].kwargs['require_valid'])
        with patch.object(readiness, '_wait_detector_ready', return_value=False):
            with self.assertRaises(SystemExit): readiness.wait(rt)

    def test_traffic_detector_retains_source_stamp_and_drops_slow_decode(self):
        profile = dict(load_profile(ROOT/'assets/traffic_light_profile.json'), input_stale_s=.5)
        msg = rgb_msg()
        frame = RgbFrame(msg.data, 100.)
        voter = TrafficLightVoter(profile)
        result = detect_ros_frame(frame, profile, voter, clock=lambda: 100.1)
        self.assertEqual(result.ts, 100.)
        self.assertIsNone(detect_ros_frame(frame, profile, voter, clock=lambda: 101.))
        clock = Mock(side_effect=[100.1, 101.])
        with patch.object(voter, 'update') as vote:
            self.assertIsNone(detect_ros_frame(frame, profile, voter, clock=clock))
            vote.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
