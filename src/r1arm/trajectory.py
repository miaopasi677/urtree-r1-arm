# -*- coding: utf-8 -*-
"""轨迹插值：最小 jerk 曲线 + 关节速度上限自动拉长时长。

最小 jerk 时间规律 s(t) = 10t³ − 15t⁴ + 6t⁵（t∈[0,1]），
端点速度、加速度均为 0；其速度峰值出现在 t=0.5，为 1.875×Δq/T。
因此给定各关节速度上限时，最短时长 T_min = 1.875 × |Δq| / v_max。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

__all__ = ["minjerk", "MINJERK_PEAK_FACTOR", "Segment", "stretch_duration", "peak_speeds"]

MINJERK_PEAK_FACTOR = 1.875
MIN_SEGMENT_DURATION_S = 0.05


def minjerk(t: float) -> float:
    """最小 jerk 时间规律，输入输出均在 [0,1]。"""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return 10.0 * t ** 3 - 15.0 * t ** 4 + 6.0 * t ** 5


@dataclass
class Segment:
    """一段关节空间运动：从 start 位姿到 end 位姿（单位：度）。"""

    start: Dict[str, float]
    end: Dict[str, float]
    duration_s: float
    name: str = ""

    def deltas(self) -> Dict[str, float]:
        return {k: float(self.end[k]) - float(self.start.get(k, 0.0)) for k in self.end}

    def setpoint(self, frac: float) -> Dict[str, float]:
        """按进度 frac∈[0,1] 计算插值位姿。"""
        s = minjerk(frac)
        return {
            k: float(self.start.get(k, 0.0)) + (float(self.end[k]) - float(self.start.get(k, 0.0))) * s
            for k in self.end
        }

    def peak_speeds(self, duration_s: Optional[float] = None) -> Dict[str, float]:
        """返回本段在给定时长下的峰值速度（度/秒）。"""
        t = float(duration_s if duration_s is not None else self.duration_s)
        out: Dict[str, float] = {}
        for k, d in self.deltas().items():
            out[k] = 0.0 if t <= 0 else MINJERK_PEAK_FACTOR * abs(d) / t
        return out


def stretch_duration(
    seg: Segment,
    limits_deg_s: Dict[str, float],
    peak_factor: float = MINJERK_PEAK_FACTOR,
    min_duration_s: float = MIN_SEGMENT_DURATION_S,
) -> float:
    """按各关节速度上限计算本段时长（只拉长、不缩短到超速）。"""
    t = max(float(seg.duration_s), min_duration_s)
    for name, delta in seg.deltas().items():
        limit = float(limits_deg_s.get(name, 0.0) or 0.0)
        if limit > 0.0 and abs(delta) > 1e-9:
            t_min = peak_factor * abs(delta) / limit
            if t_min > t:
                t = t_min
    return t


def peak_speeds(seg: Segment, duration_s: Optional[float] = None) -> Dict[str, float]:
    return seg.peak_speeds(duration_s)
