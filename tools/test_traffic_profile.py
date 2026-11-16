"""红绿灯 profile 回归：远端小绿灯光斑必须能识别，旧阈值必须识别不到。

不依赖相机：用 HSV 合成一块 12x16 的小绿斑，位置/大小按实拍
（960x540 下 ROI 内 (188,150)，对应约 5m 外的信号灯）复现。
"""
import json
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workers.traffic_light import detect_hsv, load_profile

W, H = 960, 540
BG = (0.62, 0.72, 0.80)          # 浅蓝墙
GREEN_HSV = (133.0, 0.35, 0.40)  # 实拍绿灯光斑较暗的核心
PATCH_X, PATCH_Y, PATCH_W, PATCH_H = 476, 281, 6, 11  # 实拍旧掩码的 bbox（全局坐标）


def hsv_to_rgb(h_deg, s, v):
    h = h_deg / 60.0
    i = int(h) % 6
    f = h - int(h)
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [
        (v, t, p), (q, v, p), (p, v, t),
        (p, q, v), (t, p, v), (v, p, q),
    ][i]


def make_frame():
    img = np.empty((H, W, 3), dtype=np.float32)
    img[:] = BG
    img[PATCH_Y:PATCH_Y + PATCH_H, PATCH_X:PATCH_X + PATCH_W] = hsv_to_rgb(*GREEN_HSV)
    return img


class TrafficProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile('assets/traffic_light_profile.json')
        self.frame = make_frame()

    def test_small_distant_green_is_detected(self):
        state = detect_hsv(self.frame, self.profile, now=100.0)
        self.assertEqual(state.color, 'green', state)
        self.assertGreaterEqual(state.area_ratio, 0.0003, state)

    def test_old_thresholds_would_miss_it(self):
        old = json.loads(json.dumps(self.profile))
        old['ranges']['green'] = [[125.0, 165.0, 0.3, 0.25]]
        old['min_area_ratio'] = 0.001
        state = detect_hsv(self.frame, old, now=100.0)
        self.assertEqual(state.color, 'unknown', state)

    def test_green_range_covers_measured_hue(self):
        ranges = self.profile['ranges']['green']
        self.assertTrue(any(lo <= GREEN_HSV[0] <= hi for lo, hi, _, _ in ranges))

    def test_red_and_green_together_counts_as_green(self):
        frame = make_frame()
        # 在 ROI 内画一块明显更大的红色区域，仍然应该判绿。
        red = hsv_to_rgb(0.0, 0.80, 0.60)
        frame[140:200, 300:420] = red
        state = detect_hsv(frame, self.profile, now=100.0)
        self.assertEqual(state.color, 'green', state)


if __name__ == '__main__':
    unittest.main(verbosity=2)
