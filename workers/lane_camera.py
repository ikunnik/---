"""独立盲道检测进程：camera2 RGB -> LaneState，不依赖停止线标定和检测。"""
import logging
import multiprocessing
import signal
import time

from comm import put_latest, setup_process_logging
from workers.lane import detect_lane, load_profile
from workers.rgb_camera import decode_compressed
from workers.traffic_light import decode_jpeg


class LaneCameraProcess(multiprocessing.Process):
    def __init__(self, cfg, out_queue, ready=None, first_frame=None, name="lane_camera"):
        super().__init__(name=name, daemon=True)
        self.cfg = cfg
        self.out_queue = out_queue
        self.ready = ready
        self.first_frame = first_frame
        self._stop_event = multiprocessing.Event()
        self._latest = None
        self._seq = 0

    def stop(self):
        self._stop_event.set()

    def _on_image(self, msg):
        try:
            self._seq += 1
            received_ts = time.time()
            frame = decode_compressed(msg, seq=self._seq, now=received_ts)
            if frame.data:
                self._latest = (frame, received_ts)
        except Exception:
            logging.exception("[盲道相机] 解析失败")

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        setup_process_logging()
        try:
            self._run()
        except Exception:
            logging.exception("[盲道相机] 检测进程异常退出")
            raise
        finally:
            logging.info("[盲道相机] 退出")

    def _run(self):
        import dds_middleware_python as dds

        profile = load_profile(self.cfg.lane_profile)
        width = int(profile.get("camera_width", 640))
        middleware = dds.PyDDSMiddleware(self.cfg.dds_config)
        middleware.subscribeCompressedImage(self.cfg.lane_topic, self._on_image)
        logging.info("[盲道相机] 已订阅 %s，标定 %s", self.cfg.lane_topic, self.cfg.lane_profile)
        if self.ready is not None:
            self.ready.set()

        first = True
        fps_ts = last_frame_ts = time.time()
        count = 0
        while not self._stop_event.is_set():
            item = self._latest
            if item is None:
                now = time.time()
                if now - last_frame_ts >= self.cfg.rgb_log_s:
                    logging.warning("[盲道相机] 无新图像：%s", self.cfg.lane_topic)
                    last_frame_ts = now
                time.sleep(.01)
                continue
            self._latest = None
            frame, received_ts = item
            last_frame_ts = time.time()
            try:
                rgb = decode_jpeg(frame.data, width)
            except Exception:
                logging.warning("[盲道相机] JPEG解码失败，等待新帧")
                continue
            state = detect_lane(rgb, profile, now=received_ts)
            put_latest(self.out_queue, state)
            if first:
                first = False
                if self.first_frame is not None:
                    self.first_frame.set()
                logging.info("[盲道相机] 第一帧：%s", state)
            count += 1
            now = time.time()
            if now - fps_ts >= self.cfg.rgb_log_s:
                logging.info("[盲道相机] %.1f FPS", count / (now - fps_ts))
                fps_ts, count = now, 0
