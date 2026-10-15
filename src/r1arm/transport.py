# -*- coding: utf-8 -*-
"""数据通道层：真实 RS-485 串口（pyserial）与仿真总线（无硬件联调用）。"""

from __future__ import annotations

import math
import random
import time
from typing import Dict, List, Optional

from .protocol import (
    CMD_FRAME_LEN,
    FB_FRAME_LEN,
    CMD_HEAD,
    MotorUnits,
    build_status_frame,
    crc32_mpeg2,
)

__all__ = ["Transport", "SerialTransport", "MockTransport", "TransportError"]


class TransportError(IOError):
    """通道层错误（串口打开失败、写超时等）。"""


# ---------------------------------------------------------------- 抽象接口

class Transport:
    """字节流通道抽象：写一段字节、读回当前可读的全部字节。"""

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def read_available(self) -> bytes:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return self.__class__.__name__


# ---------------------------------------------------------------- 真实串口

class SerialTransport(Transport):
    """真实 RS-485 串口通道。

    注意：6 Mbps 属高波特率，务必确认 USB-RS485 适配器支持（见宇树文档提示），
    否则会表现为「电机收不到指令 / 完全无返回」。写超时默认 0.2s。
    """

    def __init__(self, port: str, baudrate: int = 6000000, write_timeout_s: float = 0.2):
        try:
            import serial  # 延迟导入：仿真/测试环境无需安装 pyserial
        except ImportError as exc:  # pragma: no cover
            raise TransportError(
                "未安装 pyserial，无法使用真实串口。请先执行: pip install pyserial"
            ) from exc

        self._port = port
        self._baudrate = baudrate
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,                # 非阻塞读
                write_timeout=write_timeout_s,
            )
        except Exception as exc:
            raise TransportError(f"打开串口 {port} 失败: {exc}") from exc
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def write(self, data: bytes) -> None:
        try:
            self._serial.write(data)
        except Exception as exc:
            raise TransportError(f"串口写失败: {exc}") from exc

    def read_available(self) -> bytes:
        n = self._serial.in_waiting
        if not n:
            return b""
        return self._serial.read(n)

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            pass

    def describe(self) -> str:
        return f"串口 {self._port} @ {self._baudrate} bps"


# ---------------------------------------------------------------- 仿真总线

class _SimMotor:
    """仿真电机：简化运动学 + 温度/电压随负载变化。"""

    def __init__(self, motor_id: int, model: str, max_speed_deg_s: float):
        self.motor_id = motor_id
        self.units = MotorUnits.of(model)
        self.max_speed_rad_s = math.radians(max_speed_deg_s)
        self.target_pos = 0.0        # 输出端 rad
        self.pos = 0.0
        self.speed = 0.0
        self.status = 0              # 0 停机 / 1 FOC
        self.kp = 0.0
        self.kd = 0.0
        self.last_cmd_time = 0.0
        self.timeout_flag = False
        self.timeout_armed = False
        self.peak_torque = 0.0

    def step(self, dt: float) -> None:
        if self.status != 1:
            self.speed = 0.0
            return
        err = self.target_pos - self.pos
        max_step = self.max_speed_rad_s * dt
        delta = max(-max_step, min(max_step, err))
        self.pos += delta
        self.speed = delta / dt if dt > 0 else 0.0

    def torque_out(self) -> float:
        """简化的 PD 力矩估计（含少量负载扰动）。"""
        err = self.target_pos - self.pos
        tau = self.kp * err - self.kd * self.speed
        self.peak_torque = max(self.peak_torque, abs(tau))
        return tau


class MockTransport(Transport):
    """仿真总线：模拟一条 RS-485 上的多颗 R1-7A 电机。

    - 未收到过任何指令的电机不回帧（与真机「无命令则不返回」一致）；
    - 收到指令后每轮询回一帧状态（可配置丢帧率模拟通信劣化）；
    - status=0 停机、status=1 按最大速度向目标位置运动；
    - 超过 1s 未收到指令且 timeout 保护开启 → 置 timeout 标志并停机。
    """

    def __init__(
        self,
        motors: List[dict],
        *,
        sim_dt_s: float = 0.005,
        dropout_rate: float = 0.0,
        noise_deg: float = 0.0,
        seed: Optional[int] = 20261015,
    ):
        self._motors: Dict[int, _SimMotor] = {}
        for m in motors:
            self._motors[int(m["motor_id"])] = _SimMotor(
                int(m["motor_id"]), str(m["model"]), float(m.get("max_speed_deg_s", 150.0))
            )
        self._dt = float(sim_dt_s)
        self._dropout = float(dropout_rate)
        self._noise_rad = math.radians(float(noise_deg))
        self._rng = random.Random(seed)
        self._sim_time = 0.0
        self._pending: List[int] = []      # 本轮需要回帧的电机 id
        self._tx_buffer = bytearray()
        self._cmd_history: List[dict] = []   # 供测试检查（记录每次收到的指令）

    # ---- Transport 接口
    def write(self, data: bytes) -> None:
        self._tx_buffer += data
        # 从缓冲中逐帧提取 20 字节指令帧（帧头 0xFE 0xEE）
        while True:
            idx = self._tx_buffer.find(bytes(CMD_HEAD))
            if idx < 0:
                self._tx_buffer.clear()
                break
            if len(self._tx_buffer) - idx < CMD_FRAME_LEN:
                del self._tx_buffer[:idx]
                break
            frame = bytes(self._tx_buffer[idx: idx + CMD_FRAME_LEN])
            del self._tx_buffer[: idx + CMD_FRAME_LEN]
            self._apply_command(frame)

    def read_available(self) -> bytes:
        # 推进仿真时间
        self._sim_time += self._dt
        for m in self._motors.values():
            m.step(self._dt)
            if m.status == 1 and m.timeout_armed and (self._sim_time - m.last_cmd_time) > 1.0:
                m.timeout_flag = True
                m.status = 0

        if not self._pending:
            return b""
        out = bytearray()
        for mid in self._pending:
            m = self._motors.get(mid)
            if m is None:
                continue
            if self._dropout > 0.0 and self._rng.random() < self._dropout:
                continue
            pos = m.pos + (self._rng.gauss(0.0, self._noise_rad) if self._noise_rad else 0.0)
            load = min(1.0, abs(m.torque_out()) / 6.0)
            out += build_status_frame(
                m.motor_id,
                m.units,
                status=m.status,
                timeout_flag=m.timeout_flag,
                temp1_c=int(35 + 10 * load),
                temp2_c=int(40 + 15 * load),
                voltage_v=48.0 - 2.0 * load,
                torque_nm=m.torque_out(),
                speed_rad_s=m.speed,
                pos_rad=pos,
            )
        self._pending = []
        return bytes(out)

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return f"仿真总线（{len(self._motors)} 台电机，dt={self._dt * 1000:.1f} ms）"

    # ---- 仿真内部
    def _apply_command(self, frame: bytes) -> None:
        # 校验 CRC（与真机一致：CRC 不符直接丢弃）
        calc = crc32_mpeg2(frame[:16])
        recv = int.from_bytes(frame[16:20], "little")
        if recv != calc:
            return
        b2 = frame[2]
        mid = b2 & 0x0F
        status = (b2 >> 4) & 0x07
        timeout_en = bool((b2 >> 7) & 1)
        m = self._motors.get(mid)
        if m is None:
            return
        tor_raw, spd_raw = int.from_bytes(frame[4:6], "little", signed=True), int.from_bytes(frame[6:8], "little", signed=True)
        pos_raw = int.from_bytes(frame[8:12], "little", signed=True)
        kp_raw, kd_raw = int.from_bytes(frame[12:14], "little"), int.from_bytes(frame[14:16], "little")

        m.target_pos = m.units.pos_out(pos_raw)
        m.kp = float(kp_raw)
        m.kd = float(kd_raw)
        m.status = 1 if status == 1 else 0
        m.timeout_armed = timeout_en
        if timeout_en:
            m.last_cmd_time = self._sim_time
        m.timeout_flag = False  # 收到有效指令即清除超时标志
        self._pending.append(mid)
        self._cmd_history.append(
            {
                "t": self._sim_time,
                "motor_id": mid,
                "status": status,
                "pos_rad": m.target_pos,
                "kp_raw": kp_raw,
                "kd_raw": kd_raw,
                "tor_raw": tor_raw,
                "spd_raw": spd_raw,
            }
        )

    # ---- 供测试使用
    @property
    def commanded_frames(self) -> List[dict]:
        return self._cmd_history

    def sim_positions_deg(self, model_by_id: Dict[int, str]) -> Dict[int, float]:
        return {mid: math.degrees(m.pos) for mid, m in self._motors.items()}
