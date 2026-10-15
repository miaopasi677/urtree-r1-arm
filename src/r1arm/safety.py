# -*- coding: utf-8 -*-
"""安全工具：异常类型、信号处理、反馈健康检查。"""

from __future__ import annotations

import logging
import signal
from dataclasses import dataclass
from typing import Callable, Dict, Optional

log = logging.getLogger(__name__)

__all__ = [
    "EStopTriggered",
    "FaultError",
    "MoveAborted",
    "TempLimits",
    "check_feedback_health",
    "install_signal_handlers",
]


class EStopTriggered(RuntimeError):
    """急停被触发（软件急停：向所有电机持续发停机帧）。"""


class FaultError(RuntimeError):
    """运行故障：电机错误码 / 超温 / 超时保护 / 掉线等。"""


class MoveAborted(RuntimeError):
    """动作被中断（急停、故障或主动释放）。"""


@dataclass
class TempLimits:
    """温度阈值（摄氏度）：达到 warn 记录警告，达到 stop 触发故障停机。"""

    temp1_warn_c: float = 70.0
    temp1_stop_c: float = 85.0
    temp2_warn_c: float = 90.0
    temp2_stop_c: float = 110.0


def check_feedback_health(
    feedback: Dict[str, object],
    limits: TempLimits,
    warn_cb: Optional[Callable[[str, object], None]] = None,
) -> None:
    """检查所有关节反馈；致命问题直接抛 FaultError。

    feedback: {关节名: JointFeedback}
    """
    for name, fb in feedback.items():
        st = fb.state  # type: ignore[attr-defined]
        if st.merror:
            raise FaultError(
                f"关节 {name} 报告错误码 0x{st.merror:08X}（触发后电机已自动停机），"
                f"请断电排查后重新使能"
            )
        if st.timeout_flag:
            raise FaultError(
                f"关节 {name} 触发电机侧超时保护（超过 1s 未收到指令，已自动停机）"
            )
        if st.temp1_c >= limits.temp1_stop_c:
            raise FaultError(f"关节 {name} 驱动温度过高：{st.temp1_c}℃（上限 {limits.temp1_stop_c}℃）")
        if st.temp2_c >= limits.temp2_stop_c:
            raise FaultError(f"关节 {name} 绕组温度过高：{st.temp2_c}℃（上限 {limits.temp2_stop_c}℃）")
        if warn_cb is not None and (
            st.temp1_c >= limits.temp1_warn_c or st.temp2_c >= limits.temp2_warn_c
        ):
            warn_cb(name, st)


def install_signal_handlers(handler: Callable[[int], None]) -> Callable[[], None]:
    """为 SIGINT/SIGTERM 安装处理函数（仅在主线程调用）。

    返回恢复原处理函数的回调。
    """
    old = {}

    def _wrap(signum, _frame):
        handler(signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            old[sig] = signal.getsignal(sig)
            signal.signal(sig, _wrap)
        except (ValueError, OSError):
            # 非主线程或平台不支持
            pass

    def restore() -> None:
        for sig, fn in old.items():
            try:
                signal.signal(sig, fn)
            except (ValueError, OSError):
                pass

    return restore
