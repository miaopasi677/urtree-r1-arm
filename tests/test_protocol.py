# -*- coding: utf-8 -*-
"""协议层单元测试。

CRC 交叉验证思路：
  宇树参考代码按「小端 32 位字、字内自高位到低位」处理数据，等价于对
  「每 4 字节反转后的字节序列」做标准 CRC-32/MPEG-2（非反射、init=0xFFFFFFFF）。
  标准实现的正确性用公认校验值验证：CRC-32/MPEG-2("123456789") = 0x0376E6E7。
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from r1arm.protocol import (  # noqa: E402
    CMD_FRAME_LEN,
    FB_FRAME_LEN,
    MotorCommand,
    MotorUnits,
    ProtocolError,
    FrameScanner,
    build_command_frame,
    build_status_frame,
    crc32_mpeg2,
    is_valid_feedback_frame,
    parse_status_frame,
    POS_UNITS_PER_RAD,
    SPD_UNITS_PER_RAD_S,
    STATUS_FOC,
    STATUS_STOP,
    TORQUE_UNITS_PER_NM,
    KP_UNITS_PER_NM_PER_RAD,
    KD_UNITS_PER_NM_S_PER_RAD,
)


def _std_crc32_mpeg2(data: bytes) -> int:
    """独立实现：标准 CRC-32/MPEG-2（逐字节、MSB first）。"""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) & 0xFFFFFFFF) ^ 0x04C11DB7
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc & 0xFFFFFFFF


def _reverse_word_chunks(data: bytes) -> bytes:
    """每 4 字节一组反转（小端字 → 逐字节序列）。"""
    out = bytearray()
    for i in range(0, len(data), 4):
        out += data[i:i + 4][::-1]
    return bytes(out)


class CrcTests(unittest.TestCase):
    def test_standard_reference_value(self):
        self.assertEqual(_std_crc32_mpeg2(b"123456789"), 0x0376E6E7)

    def test_unitree_variant_matches_standard_via_word_reversal(self):
        data = bytes((i * 7 + 3) & 0xFF for i in range(256))
        self.assertEqual(crc32_mpeg2(data), _std_crc32_mpeg2(_reverse_word_chunks(data)))

    def test_another_random_buffer(self):
        data = bytes((i * 31 + 11) & 0xFF for i in range(64))
        self.assertEqual(crc32_mpeg2(data), _std_crc32_mpeg2(_reverse_word_chunks(data)))

    def test_length_must_be_multiple_of_four(self):
        with self.assertRaises(ProtocolError):
            crc32_mpeg2(b"\x01\x02\x03")


class CommandFrameTests(unittest.TestCase):
    def setUp(self):
        self.units = MotorUnits.of("6010")

    def _cmd(self, **kw):
        defaults = dict(pos_rad=1.0, speed_rad_s=0.0, torque_nm=0.0, kp=10.0, kd=0.5,
                        status=STATUS_FOC, timeout_enable=True)
        defaults.update(kw)
        return MotorCommand(**defaults)

    def test_frame_layout(self):
        frame = build_command_frame(5, self._cmd(), self.units)
        self.assertEqual(len(frame), CMD_FRAME_LEN)
        self.assertEqual(frame[0], 0xFE)
        self.assertEqual(frame[1], 0xEE)
        # bit7=timeout(1), bit6..4=status(1), bit3..0=id(5)
        self.assertEqual(frame[2], 0b1_001_0101)
        (crc,) = struct.unpack_from("<I", frame, 16)
        self.assertEqual(crc, crc32_mpeg2(frame[:16]))

    def test_field_encoding(self):
        frame = build_command_frame(0, self._cmd(pos_rad=1.0, kp=10.0, kd=0.5), self.units)
        pos_raw = struct.unpack_from("<i", frame, 8)[0]
        self.assertEqual(pos_raw, round(1.0 * 32.0 * POS_UNITS_PER_RAD))
        kp_raw = struct.unpack_from("<H", frame, 12)[0]
        self.assertEqual(kp_raw, round(10.0 / (32.0 ** 2) * KP_UNITS_PER_NM_PER_RAD))
        kd_raw = struct.unpack_from("<H", frame, 14)[0]
        self.assertEqual(kd_raw, round(0.5 / (32.0 ** 2) * KD_UNITS_PER_NM_S_PER_RAD))

    def test_timeout_bit_disabled(self):
        frame = build_command_frame(1, self._cmd(timeout_enable=False), self.units)
        self.assertEqual(frame[2] >> 7, 0)

    def test_stop_command(self):
        frame = build_command_frame(7, MotorCommand(status=STATUS_STOP), self.units)
        self.assertEqual((frame[2] >> 4) & 0x07, STATUS_STOP)

    def test_clamping(self):
        frame = build_command_frame(0, self._cmd(pos_rad=1e12, torque_nm=1e6, kp=-5.0), self.units)
        pos_raw = struct.unpack_from("<i", frame, 8)[0]
        tor_raw = struct.unpack_from("<h", frame, 4)[0]
        kp_raw = struct.unpack_from("<H", frame, 12)[0]
        self.assertEqual(pos_raw, 2147483647)
        self.assertEqual(tor_raw, 32767)
        self.assertEqual(kp_raw, 0)

    def test_motor_id_range(self):
        with self.assertRaises(ProtocolError):
            build_command_frame(15, self._cmd(), self.units)


class StatusFrameTests(unittest.TestCase):
    def test_round_trip(self):
        units = MotorUnits.of("6014")
        frame = build_status_frame(
            3, units, status=STATUS_FOC, timeout_flag=True,
            temp1_c=41, temp2_c=55, voltage_v=51.0,
            torque_nm=2.0, speed_rad_s=0.5, pos_rad=-0.7,
            merror=0x11223344, warning=5,
        )
        self.assertEqual(len(frame), FB_FRAME_LEN)
        st = parse_status_frame(frame, units)
        self.assertEqual(st.motor_id, 3)
        self.assertEqual(st.status, STATUS_FOC)
        self.assertTrue(st.timeout_flag)
        self.assertEqual(st.temp1_c, 41)
        self.assertEqual(st.temp2_c, 55)
        self.assertAlmostEqual(st.voltage_v, 51.0, delta=1.0)
        self.assertAlmostEqual(st.torque_nm, 2.0, places=2)
        self.assertAlmostEqual(st.speed_rad_s, 0.5, places=2)
        self.assertAlmostEqual(st.pos_rad, -0.7, delta=2e-5)   # 容差 = 1 个位置编码量化步长
        self.assertEqual(st.merror, 0x11223344)
        self.assertEqual(st.warning, 5)

    def test_validation(self):
        units = MotorUnits.of("5010")
        frame = build_status_frame(2, units, pos_rad=0.3)
        self.assertTrue(is_valid_feedback_frame(frame))
        bad = bytearray(frame)
        bad[10] ^= 0xFF
        self.assertFalse(is_valid_feedback_frame(bytes(bad)))
        with self.assertRaises(ProtocolError):
            parse_status_frame(frame[:20], units)
        with self.assertRaises(ProtocolError):
            parse_status_frame(b"\xAB" + frame[1:], units)


class UnitsTests(unittest.TestCase):
    def test_ratios(self):
        self.assertEqual(MotorUnits.of("6010").ratio, 32.0)
        self.assertAlmostEqual(MotorUnits.of("6014").ratio, 12.666)
        self.assertEqual(MotorUnits.of("5010").ratio, 16.0)
        with self.assertRaises(ProtocolError):
            MotorUnits.of("9999")

    def test_round_trip(self):
        for model in ("6010", "6014", "5010"):
            u = MotorUnits.of(model)
            self.assertAlmostEqual(u.pos_out(u.pos_raw(1.234)), 1.234, places=9)
            self.assertAlmostEqual(u.speed_out(u.speed_raw(-2.5)), -2.5, places=9)
            self.assertAlmostEqual(u.torque_out(u.torque_raw(3.7)), 3.7, places=9)
            self.assertAlmostEqual(u.kp_out(u.kp_raw(18.0)), 18.0, places=6)
            self.assertAlmostEqual(u.kd_out(u.kd_raw(1.2)), 1.2, places=6)


class ScannerTests(unittest.TestCase):
    def _frames(self, ids, pos=0.1):
        units = MotorUnits.of("6010")
        return [build_status_frame(i, units, pos_rad=pos) for i in ids]

    def test_batch(self):
        frames = self._frames([0, 1, 2])
        sc = FrameScanner()
        out = sc.feed(b"".join(frames))
        self.assertEqual(len(out), 3)
        self.assertEqual(sc.crc_errors, 0)

    def test_byte_by_byte(self):
        frames = self._frames([4, 5])
        sc = FrameScanner()
        out = []
        blob = b"".join(frames)
        for i in range(len(blob)):
            out += sc.feed(blob[i:i + 1])
        self.assertEqual(len(out), 2)

    def test_noise_and_corruption(self):
        frames = self._frames([6, 7])
        bad = bytearray(frames[1])
        bad[12] ^= 0x55
        blob = b"\x00\xff" + bytes(bad) + frames[0]
        sc = FrameScanner()
        out = sc.feed(blob)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0][2] & 0x0F, 6)
        self.assertGreaterEqual(sc.crc_errors, 1)

    def test_split_frame_across_feeds(self):
        frames = self._frames([9])
        sc = FrameScanner()
        blob = b"".join(frames)
        out = sc.feed(blob[:10]) + sc.feed(blob[10:])
        self.assertEqual(len(out), 1)


if __name__ == "__main__":
    unittest.main()
