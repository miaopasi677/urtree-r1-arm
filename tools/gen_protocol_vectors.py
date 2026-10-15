# -*- coding: utf-8 -*-
"""生成协议回归向量（改动 protocol.py 后运行，与 docs/PROTOCOL.md 中的向量对照）。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from r1arm.protocol import (  # noqa: E402
    MotorCommand,
    MotorUnits,
    STATUS_FOC,
    build_command_frame,
    build_status_frame,
    crc32_mpeg2,
)


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def main() -> None:
    u6010 = MotorUnits.of("6010")
    cmd = MotorCommand(
        pos_rad=0.5, speed_rad_s=0.0, torque_nm=0.0,
        kp=10.0, kd=0.5, status=STATUS_FOC, timeout_enable=True,
    )
    print("指令帧（motor_id=0, 6010, pos=+0.5 rad 输出端, kp=10, kd=0.5, FOC, timeout=1）:")
    print("  " + hexs(build_command_frame(0, cmd, u6010)))

    u6014 = MotorUnits.of("6014")
    st = build_status_frame(
        3, u6014, status=STATUS_FOC, temp1_c=41, temp2_c=55, voltage_v=48.0,
        torque_nm=2.0, speed_rad_s=0.5, pos_rad=-0.7, merror=0, warning=0,
    )
    print("状态帧（motor_id=3, 6014, pos=-0.7 rad, tau=2.0 N·m, spd=0.5 rad/s, 48V, 41/55℃）:")
    print("  " + hexs(st))

    print(f"CRC32-MPEG2('12345678' 按字处理) = 0x{crc32_mpeg2(b'12345678'):08X}")
    print("（本实现的等价关系：crc32_mpeg2(data) == 标准 CRC-32/MPEG-2(每 4 字节反转后的 data)，")
    print("  标准校验值 CRC-32/MPEG-2('123456789') = 0x0376E6E7）")


if __name__ == "__main__":
    main()
