"""ROS camera process integration in a localhost-only domain with synthetic images."""
import multiprocessing
import os
import queue
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rover_x1.settings import build_settings
from workers.ros_camera import RosCameraProcess


class RosCameraTransportTests(unittest.TestCase):
    def test_spawned_process_receives_standard_images_and_shuts_down(self):
        import cv2
        import numpy as np
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image, CompressedImage

        # Match controller's spawn model; this test never constructs the runtime robot.
        multiprocessing.set_start_method('spawn', force=True)
        prefix = '/rover_camera_test_' + uuid4().hex
        cfg, _ = build_settings(dict(
            camera_backend='ros2', ros2_depth_topic=prefix+'/depth',
            ros2_rgb_topic=prefix+'/rgb', ros2_camera2_topic=prefix+'/rgb', voice_enabled=False),
            mode='mission', dry_run=True, traffic_light_now=False, share_dir=ROOT)
        outputs = {r: multiprocessing.Queue(1) for r in ('depth', 'rgb', 'lane', 'stop_line')}
        ready = multiprocessing.Event()
        first = {r: multiprocessing.Event() for r in outputs}
        proc = RosCameraProcess(cfg, outputs, ready=ready, first_frames=first)
        context = Context()
        node = None
        started = False
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '232', 'ROS_LOCALHOST_ONLY': '1',
                                     'ROS_LOG_DIR': '/tmp/rover_camera_test_logs',
                                     'CYCLONEDDS_URI': ''}):
            try:
                rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
                node = Node('camera_test_publisher', context=context, use_global_arguments=False)
                depth_pub = node.create_publisher(Image, cfg.ros2_depth_topic, qos_profile_sensor_data)
                rgb_pub = node.create_publisher(CompressedImage, cfg.ros2_rgb_topic, qos_profile_sensor_data)
                proc.start(); started = True
                self.assertTrue(ready.wait(10), f'camera initialization failed: {proc.exitcode}')
                depth = Image(height=80, width=100, encoding='16UC1', step=200,
                              data=np.full((80, 100), 1500, '<u2').tobytes())
                ok, data = cv2.imencode('.png', np.zeros((120, 160, 3), np.uint8))
                self.assertTrue(ok)
                rgb = CompressedImage(format='png', data=data.tobytes())
                deadline = time.monotonic() + 8.
                while time.monotonic() < deadline:
                    stamp = time.time_ns()
                    for msg in (depth, rgb):
                        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                        msg.header.frame_id = 'synthetic_camera'
                    depth_pub.publish(depth); rgb_pub.publish(rgb)
                    if all(first[r].is_set() for r in ('depth', 'lane', 'stop_line')):
                        break
                    time.sleep(.05)
                self.assertTrue(all(first[r].is_set() for r in ('depth', 'lane', 'stop_line')))
                per = outputs['depth'].get(timeout=2)
                lane = outputs['lane'].get(timeout=2)
                stop = outputs['stop_line'].get(timeout=2)
                self.assertEqual(per.distance_m, 1.5)
                self.assertTrue(per.valid)
                self.assertFalse(lane.visible or stop.visible)
                self.assertAlmostEqual(lane.ts, stop.ts, places=6)
                self.assertLess(time.time()-per.ts, cfg.stale_s)
                # ROS camera process has only two readers for all four output roles.
                self.assertEqual(depth_pub.get_subscription_count(), 1)
                self.assertEqual(rgb_pub.get_subscription_count(), 1)
                proc.disable_stop_line()
                self.assertTrue(proc.is_alive())
                proc.stop(); proc.join(5)
                self.assertFalse(proc.is_alive())
                self.assertEqual(proc.exitcode, 0)
            finally:
                if started and proc.is_alive():
                    proc.stop(); proc.join(3)
                    if proc.is_alive(): proc.kill(); proc.join(2)
                if node is not None: node.destroy_node()
                if context.ok(): context.shutdown()
                for output in outputs.values(): output.close(); output.join_thread()


if __name__ == '__main__':
    unittest.main(verbosity=2)
