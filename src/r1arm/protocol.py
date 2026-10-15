# -*- coding: utf-8 -*-
"""R1-7A 电机通讯协议层（纯字节 ↔ 物理量，不碰串口）。

依据：《Unitree R1-7A 说明文档 · 电机驱动文档》（support.unitree.com/home/zh/R1-7a）
  - RS-485 串口，6.0 Mbps，8 数据位 / 无校验 / 1 停止位；
  - 周期发送 20 字节指令帧，电机返回 26 字节状态帧（无命令则不返回）；
  - 电机 ID 0~14，15 = 广播（无返回）；
  - 多字节变量小端序；CRC32-MPEG2（多项式 0x04C11DB7、初值 0xFFFFFFFF）——
    指令帧覆盖 16 字节（含帧头），状态帧覆盖第 2~21 字节（不含帧头）；
  - 协议中所有物理量均为「转子端」，输出端按各电机减速比换算：
    力矩 输出=转子×ratio；速度 输出=转子÷ratio；位置 输出=转子÷ratio；
    刚度 Kp/Kd 输出=转子×ratio²。
  - 数字量换算（转子端）：力矩 2560=1 N·m；速度 64/2π=1 rad/s；
    位置 32768/2π=1 rad；Kp 12800=1 N·m/rad；Kd 51200=1 N·m·s/rad。

本文件的实现与官方参考代码（C 版 CRC32_MPEG2 与结构体）逐位一致，
详见 docs/PROTOCOL.md 中的核对清单。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import List, Optional

__all__ = [
    "ProtocolError",
    "MotorUnits",
    "MotorCommand",
    "MotorState",
    "FrameScanner",
    "crc32_mpeg2",
    "build_command_frame",
    "build_status_frame",
    "parse_status_frame",
    "is_valid_feedback_frame",
    "CMD_FRAME_LEN",
    "FB_FRAME_LEN",
    "STATUS_STOP",
    "STATUS_FOC",
    "MOTOR_RATIOS",
    "POS_UNITS_PER_RAD",
    "SPD_UNITS_PER_RAD_S",
    "TORQUE_UNITS_PER_NM",
    "KP_UNITS_PER_NM_PER_RAD",
    "KD_UNITS_PER_NM_S_PER_RAD",
]

# ---------------------------------------------------------------- 常量

CMD_HEAD = (0xFE, 0xEE)          # 指令帧帧头（参与 CRC）
FB_HEAD = (0xFC, 0xEE)           # 状态帧帧头（不参与 CRC）
CMD_FRAME_LEN = 20
FB_FRAME_LEN = 26
_CMD_CRC_LEN = 16                # 指令帧 CRC 覆盖前 16 字节
_FB_CRC_START, _FB_CRC_END = 2, 22   # 状态帧 CRC 覆盖 [2, 22) 共 20 字节

STATUS_STOP = 0                  # 停机（上电默认）
STATUS_FOC = 1                   # FOC 运行

# 数字量 ↔ 物理量（转子端）
POS_UNITS_PER_RAD = 32768.0 / (2.0 * math.pi)          # ≈ 5215.189
SPD_UNITS_PER_RAD_S = 64.0 / (2.0 * math.pi)           # ≈ 10.186
TORQUE_UNITS_PER_NM = 2560.0
KP_UNITS_PER_NM_PER_RAD = 12800.0
KD_UNITS_PER_NM_S_PER_RAD = 51200.0

# 三款电机减速比（R1-7A 说明文档第三章）
MOTOR_RATIOS = {
    "6010": 32.0,
    "6014": 12.666,
    "5010": 16.0,
}

_INT16_MIN, _INT16_MAX = -32768, 32767
_INT32_MIN, _INT32_MAX = -2147483648, 2147483647
_KP_KD_RAW_MAX = 32767       # k_pos/k_spd 按有符号 16 位上界保守钳位


class ProtocolError(ValueError):
    """协议层错误（帧格式、CRC、取值范围等）。"""


def _clamp(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else (hi if v > hi else v)


# ---------------------------------------------------------------- CRC

def crc32_mpeg2(data: bytes) -> int:
    """CRC32-MPEG2（与宇树官方 C 参考代码逐位一致）。

    data 必须是 4 字节的整数倍；内部按小端 32 位字逐位处理（自高位到低位），
    与官方代码 `CRC32_MPEG2(uint32_t *ptr, uint32_t len)` 的行为完全一致。
    指令帧传入 16 字节，状态帧传入第 2~21 字节（20 字节）。
    """
    if len(data) % 4 != 0:
        raise ProtocolError("CRC 输入长度必须是 4 字节的整数倍")
    poly = 0x04C11DB7
    crc = 0xFFFFFFFF
    mask = 0xFFFFFFFF
    for i in range(0, len(data), 4):
        # 小端 32 位字（与 C 代码在 x86 上读 uint32_t 数组一致）
        word = data[i] | (data[i + 1] << 8) | (data[i + 2] << 16) | (data[i + 3] << 24)
        xbit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) & mask) ^ poly
            else:
                crc = (crc << 1) & mask
            if word & xbit:
                crc ^= poly
            xbit >>= 1
    return crc & mask


# ---------------------------------------------------------------- 单位换算

@dataclass(frozen=True)
class MotorUnits:
    """某一型号电机的转子端/输出端换算器。"""

    model: str
    ratio: float

    @staticmethod
    def of(model: str) -> "MotorUnits":
        if model not in MOTOR_RATIOS:
            raise ProtocolError(f"未知电机型号: {model!r}（可用: {sorted(MOTOR_RATIOS)}）")
        return MotorUnits(model=model, ratio=MOTOR_RATIOS[model])

    # ---- 输出端物理量 → 转子端原始值（未钳位）
    def pos_raw(self, rad_out: float) -> float:
        return rad_out * self.ratio * POS_UNITS_PER_RAD

    def speed_raw(self, rad_s_out: float) -> float:
        return rad_s_out * self.ratio * SPD_UNITS_PER_RAD_S

    def torque_raw(self, nm_out: float) -> float:
        return nm_out / self.ratio * TORQUE_UNITS_PER_NM

    def kp_raw(self, kp_out: float) -> float:
        return kp_out / (self.ratio ** 2) * KP_UNITS_PER_NM_PER_RAD

    def kd_raw(self, kd_out: float) -> float:
        return kd_out / (self.ratio ** 2) * KD_UNITS_PER_NM_S_PER_RAD

    # ---- 转子端原始值 → 输出端物理量
    def pos_out(self, raw: int) -> float:
        return raw / (self.ratio * POS_UNITS_PER_RAD)

    def speed_out(self, raw: int) -> float:
        return raw / (self.ratio * SPD_UNITS_PER_RAD_S)

    def torque_out(self, raw: int) -> float:
        return raw / TORQUE_UNITS_PER_NM * self.ratio

    def kp_out(self, raw: int) -> float:
        return raw / KP_UNITS_PER_NM_PER_RAD * (self.ratio ** 2)

    def kd_out(self, raw: int) -> float:
        return raw / KD_UNITS_PER_NM_S_PER_RAD * (self.ratio ** 2)


# ---------------------------------------------------------------- 数据结构

@dataclass
class MotorCommand:
    """发往单颗电机的指令（输出端物理量）。"""

    pos_rad: float = 0.0
    speed_rad_s: float = 0.0
    torque_nm: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    status: int = STATUS_STOP
    timeout_enable: bool = True     # 超过 1s 无指令电机自动停机（安全项，默认开启）


def stop_command() -> MotorCommand:
    """停机指令（status=0）。"""
    return MotorCommand(status=STATUS_STOP, timeout_enable=True)


@dataclass
class MotorState:
    """单颗电机的状态帧解析结果（物理量为输出端）。"""

    motor_id: int = -1
    status: int = 0
    timeout_flag: bool = False       # 电机侧超时保护触发（需控制位发 0 清除）
    temp1_c: int = 0                 # 驱动温度
    temp2_c: int = 0                 # 绕组温度
    voltage_v: float = 0.0
    torque_nm: float = 0.0           # 输出端
    speed_rad_s: float = 0.0         # 输出端
    pos_rad: float = 0.0             # 输出端
    pos_raw: int = 0                 # 转子端原始位置
    merror: int = 0                  # 错误码（按位，非 0 即故障，详见宇树文档表 1）
    warning: int = 0                 # 警告码 0~7

    @property
    def pos_deg(self) -> float:
        return math.degrees(self.pos_rad)


# ---------------------------------------------------------------- 组帧 / 解帧

def build_command_frame(motor_id: int, cmd: MotorCommand, units: MotorUnits) -> bytes:
    """组 20 字节指令帧（含 CRC32）。

    超范围数值会被保守钳位（位置按符号 32 位、力矩/速度按符号 16 位、刚度 [0, 32767]）。
    """
    if not 0 <= motor_id <= 14:
        raise ProtocolError(f"电机 ID 必须在 0~14：{motor_id}")

    pos_raw = _clamp(int(round(units.pos_raw(cmd.pos_rad))), _INT32_MIN, _INT32_MAX)
    spd_raw = _clamp(int(round(units.speed_raw(cmd.speed_rad_s))), _INT16_MIN, _INT16_MAX)
    tor_raw = _clamp(int(round(units.torque_raw(cmd.torque_nm))), _INT16_MIN, _INT16_MAX)
    kp_raw = _clamp(int(round(units.kp_raw(cmd.kp))), 0, _KP_KD_RAW_MAX)
    kd_raw = _clamp(int(round(units.kd_raw(cmd.kd))), 0, _KP_KD_RAW_MAX)

    # 字节 2 位域：bit7=timeout, bit6~4=status, bit3~0=id
    b2 = ((1 if cmd.timeout_enable else 0) << 7) | ((cmd.status & 0x07) << 4) | (motor_id & 0x0F)

    body = struct.pack(
        "<BBBBhhihh",
        CMD_HEAD[0], CMD_HEAD[1], b2, 0x00,
        tor_raw, spd_raw, pos_raw, kp_raw, kd_raw,
    )
    assert len(body) == _CMD_CRC_LEN
    crc = crc32_mpeg2(body)
    return body + struct.pack("<I", crc)


def parse_status_frame(frame: bytes, units: MotorUnits) -> MotorState:
    """解析 26 字节状态帧；帧头或 CRC 不符时抛 ProtocolError。"""
    if len(frame) != FB_FRAME_LEN:
        raise ProtocolError(f"状态帧长度应为 {FB_FRAME_LEN}：{len(frame)}")
    if frame[0] != FB_HEAD[0] or frame[1] != FB_HEAD[1]:
        raise ProtocolError("状态帧帧头不符")

    crc_calc = crc32_mpeg2(frame[_FB_CRC_START:_FB_CRC_END])
    (crc_recv,) = struct.unpack_from("<I", frame, 22)
    if crc_calc != crc_recv:
        raise ProtocolError(f"状态帧 CRC 校验失败：计算 0x{crc_calc:08X} / 收到 0x{crc_recv:08X}")

    b2 = frame[2]
    st = MotorState()
    st.motor_id = b2 & 0x0F
    st.status = (b2 >> 4) & 0x07
    st.timeout_flag = bool((b2 >> 7) & 1)
    st.temp1_c = struct.unpack_from("<b", frame, 3)[0]
    st.temp2_c = frame[4]
    st.voltage_v = frame[5] / 2.0
    (tor_raw,) = struct.unpack_from("<h", frame, 6)
    (spd_raw,) = struct.unpack_from("<h", frame, 8)
    st.pos_raw = struct.unpack_from("<i", frame, 10)[0]
    st.merror = struct.unpack_from("<I", frame, 14)[0]
    st.warning = (struct.unpack_from("<H", frame, 18)[0] >> 13) & 0x07
    st.torque_nm = units.torque_out(tor_raw)
    st.speed_rad_s = units.speed_out(spd_raw)
    st.pos_rad = units.pos_out(st.pos_raw)
    return st


def is_valid_feedback_frame(frame: bytes) -> bool:
    """快速校验：帧头 + 长度 + CRC（不解析物理量）。"""
    if len(frame) != FB_FRAME_LEN or frame[0] != FB_HEAD[0] or frame[1] != FB_HEAD[1]:
        return False
    (crc_recv,) = struct.unpack_from("<I", frame, 22)
    return crc32_mpeg2(frame[_FB_CRC_START:_FB_CRC_END]) == crc_recv


def build_status_frame(
    motor_id: int,
    units: MotorUnits,
    *,
    status: int = STATUS_FOC,
    timeout_flag: bool = False,
    temp1_c: int = 35,
    temp2_c: int = 40,
    voltage_v: float = 48.0,
    torque_nm: float = 0.0,
    speed_rad_s: float = 0.0,
    pos_rad: float = 0.0,
    merror: int = 0,
    warning: int = 0,
) -> bytes:
    """组 26 字节状态帧（输出端物理量入参），主要用于仿真与测试。"""
    b2 = ((1 if timeout_flag else 0) << 7) | ((status & 0x07) << 4) | (motor_id & 0x0F)
    tor_raw = _clamp(int(round(units.torque_raw(torque_nm))), _INT16_MIN, _INT16_MAX)
    spd_raw = _clamp(int(round(units.speed_raw(speed_rad_s))), _INT16_MIN, _INT16_MAX)
    pos_raw = _clamp(int(round(units.pos_raw(pos_rad))), _INT32_MIN, _INT32_MAX)
    vol_raw = _clamp(int(round(voltage_v * 2.0)), 0, 255)   # 2 LSB = 1 V
    warn_res1 = (warning & 0x7) << 13
    body = struct.pack(
        "<BBBBBBhhiIH",
        FB_HEAD[0], FB_HEAD[1], b2,
        _clamp(temp1_c, -128, 127) & 0xFF, _clamp(temp2_c, 0, 255), vol_raw,
        tor_raw, spd_raw, pos_raw, merror, warn_res1,
    )
    # 注意：<BBBBBB... 中 temp1 以无符号字节写入，解析时按有符号解释 —— 与 C 位域一致
    body += struct.pack("<H", 0)          # RES2（保留）
    assert len(body) == 22
    crc = crc32_mpeg2(body[_FB_CRC_START:_FB_CRC_END])
    return body + struct.pack("<I", crc)


# ---------------------------------------------------------------- 流式扫描器

_FB_HEAD_BYTES = bytes(FB_HEAD)


class FrameScanner:
    """从连续字节流中提取合法状态帧（自动找帧头、CRC 校验、失步重同步）。"""

    def __init__(self, max_buffer: int = 4096):
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.crc_errors = 0
        self.frames_ok = 0

    def feed(self, data: bytes) -> List[bytes]:
        """喂入字节流，返回其中校验通过的状态帧列表（bytes）。"""
        if data:
            self._buf += data
        if len(self._buf) > self._max_buffer:
            del self._buf[: len(self._buf) - self._max_buffer]

        out: List[bytes] = []
        while True:
            idx = self._buf.find(_FB_HEAD_BYTES)
            if idx < 0:
                # 保留末尾可能的半个帧头
                if self._buf and self._buf[-1] == FB_HEAD[0]:
                    self._buf = bytearray(self._buf[-1:])
                else:
                    self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < FB_FRAME_LEN:
                break
            frame = bytes(self._buf[:FB_FRAME_LEN])
            if is_valid_feedback_frame(frame):
                out.append(frame)
                self.frames_ok += 1
                del self._buf[:FB_FRAME_LEN]
            else:
                self.crc_errors += 1
                del self._buf[:1]        # 失步：前移 1 字节重新找帧头
        return out
