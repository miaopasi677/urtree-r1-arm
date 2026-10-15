# -*- coding: utf-8 -*-
"""轨迹层单元测试：最小 jerk 曲线与速度约束。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from r1arm.trajectory import (  # noqa: E402
    MINJERK_PEAK_FACTOR,
    Segment,
    minjerk,
    peak_speeds,
    stretch_duration,
)


class MinJerkTests(unittest.TestCase):
    def test_endpoints(self):
        self.assertEqual(minjerk(0.0), 0.0)
        self.assertEqual(minjerk(1.0), 1.0)
        self.assertAlmostEqual(minjerk(0.5), 0.5, places=12)

    def test_monotonic(self):
        prev = -1.0
        for i in range(101):
            v = minjerk(i / 100.0)
            self.assertGreaterEqual(v, prev - 1e-12)
            prev = v

    def test_clamp(self):
        self.assertEqual(minjerk(-0.5), 0.0)
        self.assertEqual(minjerk(1.5), 1.0)


class SegmentTests(unittest.TestCase):
    def _seg(self, duration=1.0):
        return Segment(
            start={"J1": 0.0, "J2": 10.0},
            end={"J1": 60.0, "J2": 10.0},
            duration_s=duration,
        )

    def test_setpoint_endpoints(self):
        seg = self._seg()
        self.assertAlmostEqual(seg.setpoint(0.0)["J1"], 0.0)
        self.assertAlmostEqual(seg.setpoint(1.0)["J1"], 60.0)
        self.assertAlmostEqual(seg.setpoint(0.5)["J1"], 30.0)
        self.assertAlmostEqual(seg.setpoint(0.3)["J2"], 10.0)

    def test_peak_speeds(self):
        seg = self._seg(duration=2.0)
        peaks = peak_speeds(seg)
        self.assertAlmostEqual(peaks["J1"], MINJERK_PEAK_FACTOR * 60.0 / 2.0, places=9)
        self.assertEqual(peaks["J2"], 0.0)


class StretchTests(unittest.TestCase):
    def _seg(self, duration=1.0):
        return Segment(
            start={"J1": 0.0, "J2": 10.0},
            end={"J1": 60.0, "J2": 10.0},
            duration_s=duration,
        )

    def test_stretch_when_too_fast(self):
        seg = self._seg(duration=0.5)
        t = stretch_duration(seg, {"J1": 120.0})
        self.assertAlmostEqual(t, MINJERK_PEAK_FACTOR * 60.0 / 120.0, places=9)

    def test_keep_when_slow_enough(self):
        seg = self._seg(duration=3.0)
        t = stretch_duration(seg, {"J1": 120.0})
        self.assertAlmostEqual(t, 3.0, places=9)

    def test_multi_joint_takes_max(self):
        seg = Segment(start={"J1": 0.0, "J2": 0.0}, end={"J1": 30.0, "J2": 90.0}, duration_s=0.5)
        t = stretch_duration(seg, {"J1": 120.0, "J2": 60.0})
        self.assertAlmostEqual(t, max(
            MINJERK_PEAK_FACTOR * 30.0 / 120.0,
            MINJERK_PEAK_FACTOR * 90.0 / 60.0,
        ), places=9)

    def test_min_duration(self):
        seg = Segment(start={"J1": 0.0}, end={"J1": 0.1}, duration_s=0.0)
        t = stretch_duration(seg, {"J1": 120.0})
        self.assertGreater(t, 0.0)


if __name__ == "__main__":
    unittest.main()
