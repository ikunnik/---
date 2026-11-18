"""--mangdao 独立模式回归；所有机器人和DDS连接均使用模拟对象。"""
import contextlib
import io
import queue
import sys
import threading
import time
import types
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import main
import runtime.assembly as assembly
import runtime.readiness as readiness
from comm import Latest
from config import CFG
from messages import LaneState, Perception, RgbFrame, CommandResult
from decisions.lane import LaneFollower
from workers.lane_camera import LaneCameraProcess


class Captured(Exception):
    pass


class MangdaoTests(unittest.TestCase):
    def configuration(self, args, cfg=CFG):
        result=[]
        def capture(dry_run, config):
            result.append(config)
            raise Captured
        with patch.object(sys, 'argv', ['main.py']+args), patch.object(main, 'CFG', cfg), patch.object(main, 'build_robot', capture):
            with self.assertRaises(Captured):
                main.main()
        return result[0]

    def test_mangdao_is_exclusive_even_if_config_enables_traffic(self):
        cfg=self.configuration(['--mangdao'], replace(CFG, stop_line_enabled=True,
                                                       traffic_light_enabled=True))
        self.assertTrue(cfg.lane_follow_enabled)
        self.assertFalse(cfg.stop_line_enabled or cfg.traffic_light_enabled or cfg.traffic_sequence_enabled)
        self.assertEqual((cfg.lane_loop_update_threshold,cfg.lane_loop_min_update_s),(.02,.22))
        self.assertTrue(cfg.lane_no_line_straight)

    def test_default_traffic_and_plain_avoidance_are_preserved(self):
        traffic=self.configuration([])
        self.assertTrue(traffic.traffic_sequence_enabled and traffic.stop_line_enabled and traffic.traffic_light_enabled)
        self.assertFalse(traffic.lane_follow_enabled)
        avoid=self.configuration(['--no-traffic-sequence'])
        self.assertFalse(avoid.traffic_sequence_enabled or avoid.stop_line_enabled or avoid.lane_follow_enabled)

    def test_conflicting_modes_and_removed_switches_fail_before_connection(self):
        cases=[['--mangdao',flag] for flag in ('--stop-line','--traffic-light','--traffic-light-now','--traffic-sequence')]
        cases += [['--lane-variant','4'],['--lane-loop-experimental'],
                  ['--lane-batch-steps','4'],['--lane-duration','2.4'],['--lane-accept-loop-risk']]
        for args in cases:
            with self.subTest(args=args), patch.object(sys,'argv',['main.py']+args), patch.object(main,'build_robot') as connect, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as exc:
                    main.main()
                self.assertEqual(exc.exception.code,2)
                connect.assert_not_called()

    def test_process_selection_and_failed_startup_cleanup(self):
        constructors=('DepthCameraProcess','VoiceProcess','RgbCameraProcess','PerceptionWorker',
                      'RgbPerceptionWorker','TrafficLightProcess','TrafficLightWorker',
                      'StopLineProcess','StopLineWorker','LaneCameraProcess','LaneWorker',
                      'MotionExecutor','DecisionWorker','ControllerSupervisor')
        for mangdao in (False,True):
            with self.subTest(mangdao=mangdao), contextlib.ExitStack() as stack:
                stack.enter_context(patch.object(sys,'argv',['main.py','--no-voice']+(['--mangdao'] if mangdao else ['--no-mission'])))
                robot=stack.enter_context(patch.object(main,'build_robot')).return_value
                mocks={name:stack.enter_context(patch.object(assembly,name)) for name in constructors}
                for mock in mocks.values(): mock.return_value.is_alive.return_value=False
                ready=stack.enter_context(patch.object(readiness,'_wait_detector_ready',return_value=False))
                with self.assertRaises(SystemExit): main.main()
                if mangdao:
                    mocks['LaneCameraProcess'].assert_called_once()
                    mocks['LaneWorker'].assert_called_once()
                    for name in ('StopLineProcess','StopLineWorker','TrafficLightProcess','TrafficLightWorker'):
                        mocks[name].assert_not_called()
                    self.assertIs(ready.call_args.args[0],mocks['LaneCameraProcess'].return_value)
                    mocks['LaneCameraProcess'].return_value.stop.assert_called_once()
                else:
                    mocks['StopLineProcess'].assert_called_once()
                    mocks['LaneCameraProcess'].assert_not_called()
                    self.assertIs(ready.call_args.args[0],mocks['StopLineProcess'].return_value)
                mocks['MotionExecutor'].return_value.start.assert_not_called()
                mocks['DecisionWorker'].return_value.start.assert_not_called()
                robot.brake.assert_called_once()
                robot.close.assert_called_once()

    def test_lane_camera_does_not_load_stop_line_profile(self):
        cfg=replace(CFG,stop_line_profile='/nonexistent/stop_line.json')
        out=queue.Queue(maxsize=1)
        ready,first=threading.Event(),threading.Event()
        camera=LaneCameraProcess(cfg,out,ready,first)
        camera._latest=(RgbFrame(b'jpeg',100.0),100.0)
        camera._stop_event=MagicMock()
        camera._stop_event.is_set.side_effect=[False,True]
        middleware=MagicMock()
        fake_dds=types.SimpleNamespace(PyDDSMiddleware=MagicMock(return_value=middleware))
        import numpy as np
        with patch.dict(sys.modules,dds_middleware_python=fake_dds), patch('workers.lane_camera.decode_jpeg',return_value=np.zeros((480,640,3))), patch('workers.stop_line.load_profile',side_effect=AssertionError('must not load stop-line profile')):
            camera._run()
        state=out.get_nowait()
        self.assertIsInstance(state,LaneState)
        self.assertFalse(state.visible)
        self.assertEqual(state.ts,100.0)
        self.assertTrue(ready.is_set() and first.is_set())
        self.assertEqual(middleware.subscribeCompressedImage.call_args.args[0],cfg.lane_topic)

    def test_failed_decode_does_not_mark_first_frame(self):
        camera=LaneCameraProcess(CFG,queue.Queue(),threading.Event(),threading.Event())
        camera._latest=(RgbFrame(b'bad',100),100)
        camera._stop_event=MagicMock()
        camera._stop_event.is_set.side_effect=[False,True]
        fake=types.SimpleNamespace(PyDDSMiddleware=MagicMock())
        with patch.dict(sys.modules,dds_middleware_python=fake), patch('workers.lane_camera.decode_jpeg',side_effect=ValueError):
            camera._run()
        self.assertFalse(camera.first_frame.is_set())
        self.assertTrue(camera.out_queue.empty())

    def test_handshake_uses_lane_states_and_rejects_dead_detector(self):
        ready,first=threading.Event(),threading.Event()
        ready.set(); first.set()
        states=Latest(); states.set(LaneState(False,ts=time.time()))
        proc=MagicMock(); proc.is_alive.return_value=True
        opts=dict(label='盲道',topic=CFG.lane_topic,startup_timeout_s=.2,frame_timeout_s=.2)
        self.assertTrue(main._wait_detector_ready(proc,ready,first,states,**opts))
        proc.is_alive.return_value=False
        self.assertFalse(main._wait_detector_ready(proc,ready,first,states,**opts))

    def test_stable_loop_waits_then_replans_and_handles_cancel(self):
        f=LaneFollower(CFG)
        def tick(t,center=.5,result=None):
            return f.step(t,LaneState(True,center,center,0,0,ts=t,confidence=.9),
                          Perception(2,True,t),t,None,result,True)
        initial=tick(100)
        self.assertTrue(initial.request.cmd.loop)
        self.assertEqual(len(initial.request.cmd.steps),2)
        # 偏差跳变后不能在最小间隔内反复打断执行器
        self.assertTrue(all(tick(100+i*.025,.9).request is None for i in range(1,9)))
        replan=None
        for i in range(1,151):
            t=100+i*.025
            out=tick(t,.9)
            if out.request is not None:
                replan=out
                self.assertGreaterEqual(t-100,CFG.lane_loop_min_update_s)
                break
        self.assertIsNotNone(replan)
        self.assertTrue(replan.interrupt and replan.request.cmd.loop)
        # 重发必须是"从当前执行值渐变"，不是一步跳到目标值
        self.assertLess(abs(replan.request.cmd.steps[0].vyaw-initial.request.cmd.steps[0].vyaw),.12)
        self.assertLess(replan.request.cmd.steps[0].vyaw,0.0)   # center 偏右 -> 右转(负角速度)
        failed=CommandResult(1,replan.request.cmd.tag,'cancelled',ts=t+.01)
        stopped=tick(t+.02,.9,failed)
        self.assertTrue(stopped.interrupt)
        self.assertEqual(stopped.request.cmd.vx,0)


if __name__=='__main__':
    unittest.main(verbosity=2)
