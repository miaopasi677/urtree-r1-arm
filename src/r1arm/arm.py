# -*- coding: utf-8 -*-
"""关节模型：角度换算（零点 / 方向）、软限位、位姿工具。"""

from __future__ import annotations

import logging
import math
from typing import Dict, List, Optional, Tuple

from .bus import JointFeedback, JointSpec

log = logging.getLogger(__name__)

__all__ = ["ArmModel", "PoseError"]


class PoseError(ValueError):
    """位姿定义错误（缺关节、越限且不允许钳位等）。"""


class ArmModel:
    """把「关节角（度）」与「电机位置（输出端弧度）」互相换算，并管理软限位。

    约定：joint_deg = direction × (motor_output_deg − offset_deg)
    offset_deg 为电机输出 0° 与关节零位的差，现场用 `r1ctl.py teach` 的 zero 命令标定。
    """

    def __init__(self, joints: List[JointSpec], safe_margin_deg: float = 2.0):
        self.joints: Dict[str, JointSpec] = {j.name: j for j in joints}
        self.order: List[str] = [j.name for j in joints]
        self.safe_margin_deg = float(safe_margin_deg)

    # ------------------------------------------------------------ 换算

    def joint_deg(self, spec: JointSpec, fb: JointFeedback) -> float:
        """由反馈电机位置解析关节角（度）。"""
        motor_deg = math.degrees(fb.state.pos_rad)
        return spec.direction * (motor_deg - spec.offset_deg)

    def measured_pose(self, feedback: Dict[str, JointFeedback]) -> Dict[str, float]:
        return {name: self.joint_deg(spec, feedback[name]) for name, spec in self.joints.items()}

    def motor_pos_rad(self, spec: JointSpec, joint_deg: float) -> float:
        """关节角（度）→ 电机输出端位置（弧度），供组指令用。"""
        motor_deg = spec.offset_deg + spec.direction * joint_deg
        return math.radians(motor_deg)

    # ------------------------------------------------------------ 限位

    def limits_deg(self, spec: JointSpec) -> Tuple[float, float]:
        lo = spec.min_deg + self.safe_margin_deg
        hi = spec.max_deg - self.safe_margin_deg
        if lo >= hi:
            raise PoseError(f"{spec.name}: 软限位为空（检查 min/max 与 safe_margin）")
        return lo, hi

    def clamp_deg(self, spec: JointSpec, deg: float) -> Tuple[float, bool]:
        lo, hi = self.limits_deg(spec)
        if deg < lo:
            return lo, True
        if deg > hi:
            return hi, True
        return deg, False

    def clamp_pose(self, pose: Dict[str, float]) -> Tuple[Dict[str, float], List[str]]:
        """按软限位钳位整组位姿；返回（钳位后位姿, 被钳位的关节说明列表）。"""
        out: Dict[str, float] = {}
        clamped: List[str] = []
        for name, spec in self.joints.items():
            if name not in pose:
                raise PoseError(f"位姿缺少关节 {name}")
            v, did = self.clamp_deg(spec, float(pose[name]))
            out[name] = v
            if did:
                clamped.append(f"{name}: {pose[name]:.1f}° → {v:.1f}°")
        return out, clamped

    def validate_pose(self, pose: Dict[str, float], allow_clamp: bool = False) -> Dict[str, float]:
        """校验位姿：关节齐全；越限时按 allow_clamp 决定钳位或报错。"""
        missing = [n for n in self.order if n not in pose]
        if missing:
            raise PoseError(f"位姿缺少关节: {missing}")
        if allow_clamp:
            return self.clamp_pose(pose)[0]
        out: Dict[str, float] = {}
        bad: List[str] = []
        for name, spec in self.joints.items():
            lo, hi = self.limits_deg(spec)
            v = float(pose[name])
            if not lo <= v <= hi:
                bad.append(f"{name}={v:.1f}° (允许 {lo:.1f}~{hi:.1f}°)")
            out[name] = v
        if bad:
            raise PoseError("位姿超出软限位: " + "; ".join(bad))
        return out

    # ------------------------------------------------------------ 位姿工具

    def pose_from_list(self, deg_list: List[float]) -> Dict[str, float]:
        if len(deg_list) != len(self.order):
            raise PoseError(f"位姿向量长度应为 {len(self.order)}，收到 {len(deg_list)}")
        return {name: float(v) for name, v in zip(self.order, deg_list)}

    def pose_to_list(self, pose: Dict[str, float]) -> List[float]:
        return [float(pose[n]) for n in self.order]

    def speed_limits(self) -> Dict[str, float]:
        return {n: float(spec.max_speed_deg_s) for n, spec in self.joints.items()}

    def max_delta_deg(self, a: Dict[str, float], b: Dict[str, float]) -> Tuple[str, float]:
        """两姿态间最大单关节差（度）及其关节名。"""
        best_name, best = "", 0.0
        for n in self.order:
            d = abs(float(b[n]) - float(a[n]))
            if d > best:
                best_name, best = n, d
        return best_name, best

    def summary_lines(self, pose: Dict[str, float]) -> str:
        return "  ".join(f"{n}={pose[n]:.1f}°" for n in self.order)
