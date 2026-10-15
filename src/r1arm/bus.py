# -*- coding: utf-8 -*-
"""电机总线层：多电机轮询、状态缓存、丢线看门狗。

一条 RS-485 上挂全部 7 颗关节电机（ID 0~6）。每个控制周期：
  1) 把 7 帧指令打包成一次写（减少等待）；
  2) 在 read_timeout_ms 窗口内收状态帧（帧头 0xFC 0xEE + CRC 校验）；
  3) 更新各关节反馈缓存；超过 lost_timeout_ms 未更新的关节标记为掉线。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .protocol import (
    FrameScanner,
    MotorCommand,
    MotorState,
    MotorUnits,
    build_command_frame,
    stop_command,
)
from .transport import Transport

log = logging.getLogger(__name__)

__all__ = ["JointSpec", "JointFeedback", "MotorBus", "BusLostError"]


class BusLostError(IOError):
    """总线通信丢失（全部或部分关节长期无反馈）。"""


@dataclass
class JointSpec:
    """单关节硬件配置（对应 arm_config.json 的 joints 数组项）。"""

    name: str
    motor_id: int
    model: str
    min_deg: float
    max_deg: float
    offset_deg: float = 0.0          # 电机输出 0 位到关节零位的偏移（现场标定写入）
    direction: int = 1               # 关节角正方向与电机正方向的关系（±1）
    kp: float = 10.0                 # 输出端位置刚度 N·m/rad
    kd: float = 0.5                  # 输出端速度刚度 N·m·s/rad
    max_speed_deg_s: float = 120.0   # 展示运行速度上限
    tau_guard_nm: Optional[float] = None   # 碰撞/过载力矩阈值（None=关闭）

    def __post_init__(self):
        if self.direction not in (-1, 1):
            raise ValueError(f"{self.name}: direction 只能是 ±1")
        if self.min_deg >= self.max_deg:
            raise ValueError(f"{self.name}: min_deg 必须小于 max_deg")
        self.units: MotorUnits = MotorUnits.of(self.model)

    @staticmethod
    def from_dict(d: dict) -> "JointSpec":
        return JointSpec(
            name=str(d["name"]),
            motor_id=int(d["motor_id"]),
            model=str(d["model"]),
            min_deg=float(d["min_deg"]),
            max_deg=float(d["max_deg"]),
            offset_deg=float(d.get("offset_deg", 0.0)),
            direction=int(d.get("direction", 1)),
            kp=float(d.get("kp", 10.0)),
            kd=float(d.get("kd", 0.5)),
            max_speed_deg_s=float(d.get("max_speed_deg_s", 120.0)),
            tau_guard_nm=(None if d.get("tau_guard_nm") is None else float(d["tau_guard_nm"])),
        )


@dataclass
class JointFeedback:
    """单关节最近一次反馈。"""

    name: str
    motor_id: int
    state: MotorState = field(default_factory=MotorState)
    online: bool = False
    last_rx_t: float = 0.0           # time.monotonic()
    rx_count: int = 0

    def age_s(self, now: Optional[float] = None) -> float:
        now = time.monotonic() if now is None else now
        return now - self.last_rx_t if self.last_rx_t else float("inf")


class MotorBus:
    """多电机总线（单线程使用：由控制环调用 step()）。"""

    def __init__(
        self,
        transport: Transport,
        joints: List[JointSpec],
        *,
        read_timeout_ms: float = 3.0,
        lost_timeout_ms: float = 500.0,
    ):
        if len({j.motor_id for j in joints}) != len(joints):
            raise ValueError("电机 ID 重复，请检查 joints 配置")
        self._tx = transport
        self.joints = list(joints)
        self._by_id = {j.motor_id: j for j in joints}
        self._scanner = FrameScanner()
        self._feedback: Dict[str, JointFeedback] = {
            j.name: JointFeedback(name=j.name, motor_id=j.motor_id) for j in joints
        }
        self._read_timeout_s = max(0.0, read_timeout_ms / 1000.0)
        self.lost_timeout_s = max(0.0, lost_timeout_ms / 1000.0)
        self.stats = {"cycles": 0, "tx_frames": 0, "rx_frames": 0, "crc_errors": 0, "cycles_incomplete": 0}
        self._cyc_rx = 0

    # ------------------------------------------------------------ 核心循环

    def step(self, cmds: Dict[str, MotorCommand]) -> Dict[str, JointFeedback]:
        """发送一轮指令并收集状态帧；返回各关节反馈缓存（引用，勿修改）。"""
        buf = bytearray()
        expected = 0
        for j in self.joints:
            cmd = cmds.get(j.name)
            if cmd is None:
                continue
            buf += build_command_frame(j.motor_id, cmd, j.units)
            expected += 1
        if buf:
            self._tx.write(bytes(buf))
            self.stats["tx_frames"] += expected

        self._cyc_rx = 0
        self.stats["cycles"] += 1
        started = time.monotonic()
        deadline = started + self._read_timeout_s
        hard_deadline = started + 2.0 * self._read_timeout_s
        while True:
            data = self._tx.read_available()
            if data:
                for frame in self._scanner.feed(data):
                    self._rx_frame(frame)
                if self._cyc_rx >= expected:
                    break
                # 有数据流动时，稍作等待再尝试收齐
                deadline = min(time.monotonic() + 0.0005, hard_deadline)
            now = time.monotonic()
            if now >= deadline:
                break
            time.sleep(0.0001)

        if expected and self._cyc_rx < expected:
            self.stats["cycles_incomplete"] += 1

        self.stats["rx_frames"] = self._scanner.frames_ok
        self.stats["crc_errors"] = self._scanner.crc_errors
        self._check_lost()
        return self._feedback

    def _rx_frame(self, frame: bytes) -> None:
        motor_id = frame[2] & 0x0F
        spec = self._by_id.get(motor_id)
        if spec is None:
            return
        try:
            from .protocol import parse_status_frame
            state = parse_status_frame(frame, spec.units)
        except Exception:
            return
        fb = self._feedback[spec.name]
        fb.state = state
        fb.online = True
        fb.last_rx_t = time.monotonic()
        fb.rx_count += 1
        self._cyc_rx += 1

    def _check_lost(self) -> None:
        now = time.monotonic()
        for fb in self._feedback.values():
            if fb.online and fb.age_s(now) > self.lost_timeout_s:
                fb.online = False
                log.warning("关节 %s 超过 %.0f ms 无反馈，标记掉线", fb.name, self.lost_timeout_s * 1000)

    # ------------------------------------------------------------ 辅助

    def request_stop(self, rounds: int = 5, gap_s: float = 0.002) -> None:
        """无条件向所有电机连发停机帧（急停/退出清理用，不读取反馈）。"""
        frame = b"".join(
            build_command_frame(j.motor_id, stop_command(), j.units) for j in self.joints
        )
        for _ in range(max(1, rounds)):
            try:
                self._tx.write(frame)
            except Exception as exc:
                log.error("发送停机帧失败: %s", exc)
                break
            time.sleep(gap_s)

    def poll_for(self, seconds: float, stop_first: bool = True) -> Dict[str, JointFeedback]:
        """周期性轮询采集一段时间的状态（自检/监控用）。

        电机「无命令则不返回」，因此窗口内每 200ms 重发一轮停机帧以保持应答新鲜。
        """
        deadline = time.monotonic() + seconds
        next_poll = 0.0
        if stop_first:
            self.request_stop(rounds=1)
            next_poll = time.monotonic() + 0.2
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            data = self._tx.read_available()
            if data:
                for frame in self._scanner.feed(data):
                    self._rx_frame(frame)
            if now >= next_poll:
                self.request_stop(rounds=1)
                next_poll = now + 0.2
            time.sleep(0.0005)
        self._check_lost()
        return self._feedback

    def missing(self) -> List[str]:
        """返回当前不在线的关节名列表。"""
        return [fb.name for fb in self._feedback.values() if not fb.online]

    def feedback(self, name: str) -> JointFeedback:
        return self._feedback[name]

    def all_feedback(self) -> Dict[str, JointFeedback]:
        return self._feedback

    def describe(self) -> str:
        return self._tx.describe()

    def close(self) -> None:
        self._tx.close()
