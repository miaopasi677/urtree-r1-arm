# -*- coding: utf-8 -*-
"""Dex1-1 夹爪驱动。

Dex1-1 是宇树的两指平行夹爪，内部为单颗 M4010 电机（串口 6 Mbps）。
官方控制方式：运行 dex1_1_service（github.com/unitreerobotics/dex1_1_service），
服务把串口转成 DDS 话题：
    rt/dex1/{left,right}/cmd    unitree_go::msg::dds_::MotorCmds_
    rt/dex1/{left,right}/state  unitree_go::msg::dds_::MotorStates_
指令字段：mode=1（FOC）、q 目标（rad）、kp/kd 增益。

标定与量程（官方服务标定后）：闭合位 q≈0，全开 q≈0.5 rad；
夹持力由 kp×(q_target − q_actual) 决定，q_close 越小夹得越紧。
现场用 `python r1ctl.py gripper-test` 标定 q_open / q_close / kp。

backend 取值（arm_config.json → gripper.backend）：
    dex1_service  经官方服务（DDS）控制 —— 推荐，默认
    mock          仿真夹爪（无硬件联调/测试）
    none          不使用夹爪（纯臂测试）
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

log = logging.getLogger(__name__)

__all__ = [
    "GripperError",
    "GripperBase",
    "Dex1ServiceGripper",
    "MockGripper",
    "NoneGripper",
    "make_gripper",
]


class GripperError(RuntimeError):
    """夹爪驱动错误。"""


class GripperBase:
    """夹爪接口。"""

    backend = "base"

    def connect(self) -> None:
        """建立连接/启动后台发布。"""

    def open_grasp(self, wait: bool = True) -> None:
        raise NotImplementedError

    def close_grasp(self, wait: bool = True) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        """保持当前位置。"""

    def position(self) -> Optional[float]:
        return None

    def torque(self) -> Optional[float]:
        return None

    def is_grasped(self) -> Optional[bool]:
        """是否已夹到物体；无法判断时返回 None。"""
        return None

    def state(self) -> dict:
        return {"backend": self.backend}

    def shutdown(self) -> None:
        pass

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.shutdown()
        return False


# ---------------------------------------------------------------- 官方服务客户端

class Dex1ServiceGripper(GripperBase):
    """经宇树官方 dex1_1_service（DDS）控制 Dex1-1。

    前置条件：
      1) 控制主机上已编译并运行 dex1_1_gripper_server（见其 README）；
      2) Python 环境安装了 unitree_sdk2py；
      3) dds_network 与服务使用的网络接口一致。
    """

    backend = "dex1_service"

    def __init__(self, cfg: dict):
        self.side = str(cfg.get("side", "right"))
        self.prefix = str(cfg.get("cmd_topic_prefix", "rt/dex1"))
        self.topic_cmd = f"{self.prefix}/{self.side}/cmd"
        self.topic_state = f"{self.prefix}/{self.side}/state"
        self.dds_network = cfg.get("dds_network") or None
        self.kp = float(cfg.get("kp", 5.0))
        self.kd = float(cfg.get("kd", 0.05))
        self.q_open = float(cfg.get("q_open", 0.5))
        self.q_close = float(cfg.get("q_close", 0.02))
        self.rate_hz = float(cfg.get("publish_rate_hz", 100.0))
        self.settle_tol = float(cfg.get("settle_tol", 0.03))
        self.timeout_s = float(cfg.get("timeout_s", 3.0))
        self.grasp_tau = None if cfg.get("grasp_tau_nm") is None else float(cfg["grasp_tau_nm"])

        self._MotorCmd_ = None
        self._MotorCmds_ = None
        self._pub = None
        self._sub = None
        self._lock = threading.Lock()
        self._target: Optional[float] = None
        self._q: Optional[float] = None
        self._dq: Optional[float] = None
        self._tau: Optional[float] = None
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ---- 连接
    def connect(self) -> None:
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_, MotorCmds_, MotorStates_  # noqa: F401
        except ImportError as exc:
            raise GripperError(
                "未安装 unitree_sdk2py，无法连接 dex1_1_service。"
                "安装：git clone https://github.com/unitreerobotics/unitree_sdk2_python"
                " && pip install -e ./unitree_sdk2_python"
            ) from exc

        self._MotorCmd_ = MotorCmd_
        self._MotorCmds_ = MotorCmds_

        try:
            ChannelFactoryInitialize(0, self.dds_network)
        except Exception:
            pass  # 重复初始化等情况忽略

        # 先验证 IDL 构造可用，尽早报错
        try:
            self._make_msg(0.0)
        except Exception as exc:
            raise GripperError(f"构造 MotorCmds_ 失败（unitree_sdk2py IDL 不匹配）: {exc}") from exc

        self._pub = ChannelPublisher(self.topic_cmd, MotorCmds_)
        self._pub.Init()
        self._sub = ChannelSubscriber(self.topic_state, MotorStates_)
        self._sub.Init(self._on_state, 10)

        t0 = time.time()
        while time.time() - t0 < 3.0 and self._q is None:
            time.sleep(0.02)
        if self._q is None:
            raise GripperError(
                f"3s 内未收到夹爪状态（话题 {self.topic_state}）。"
                "请确认 dex1_1_gripper_server 已在本机运行、dds_network 配置正确"
            )
        with self._lock:
            self._target = self._q
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._publish_loop, name="gripper-pub", daemon=True)
        self._thread.start()
        log.info("Dex1-1 已连接：cmd=%s state=%s，q=%.3f", self.topic_cmd, self.topic_state, self._q)

    def _on_state(self, msg) -> None:
        try:
            st = msg.states[0]
            self._q = float(st.q)
            self._dq = float(getattr(st, "dq", 0.0) or 0.0)
            self._tau = float(getattr(st, "tau", 0.0) or 0.0)
        except Exception:
            pass

    def _make_msg(self, q: float):
        cmd = self._MotorCmd_(mode=1, q=float(q), dq=0.0, tau=0.0, kp=self.kp, kd=self.kd)
        return self._MotorCmds_(cmds=[cmd])

    def _publish_loop(self) -> None:
        period = 1.0 / max(1.0, self.rate_hz)
        misses = 0
        while not self._stop_evt.wait(period):
            with self._lock:
                q = self._target
            if q is None or self._pub is None:
                continue
            try:
                self._pub.Write(self._make_msg(q))
                misses = 0
            except Exception as exc:
                misses += 1
                if misses <= 3 or misses % 100 == 0:
                    log.error("夹爪指令发布失败（第 %d 次）：%s", misses, exc)

    # ---- 动作
    def _set(self, q: float) -> None:
        with self._lock:
            self._target = float(q)

    def _wait_until(self, predicate, what: str) -> bool:
        t0 = time.time()
        while time.time() - t0 < self.timeout_s:
            if predicate():
                return True
            time.sleep(0.01)
        log.warning("夹爪%s等待超时（%.1fs，q=%s, tau=%s）", what, self.timeout_s, self._q, self._tau)
        return False

    def open_grasp(self, wait: bool = True) -> None:
        self._set(self.q_open)
        if wait:
            self._wait_until(
                lambda: self._q is not None and abs(self._q - self.q_open) < self.settle_tol, "张开"
            )

    def close_grasp(self, wait: bool = True) -> None:
        self._set(self.q_close)
        if wait:
            def done() -> bool:
                if self._q is None:
                    return False
                if abs(self._q - self.q_close) < self.settle_tol:
                    return True
                if self.grasp_tau is not None and self._tau is not None and abs(self._tau) >= self.grasp_tau:
                    return True
                return False
            self._wait_until(done, "闭合")

    def stop(self) -> None:
        if self._q is not None:
            self._set(self._q)

    # ---- 状态
    def position(self) -> Optional[float]:
        return self._q

    def torque(self) -> Optional[float]:
        return self._tau

    def is_grasped(self) -> Optional[bool]:
        if self._tau is None or self.grasp_tau is None:
            return None
        return abs(self._tau) >= self.grasp_tau

    def state(self) -> dict:
        return {
            "backend": self.backend,
            "topic": self.topic_cmd,
            "q": self._q,
            "tau": self._tau,
            "target": self._target,
            "grasped": self.is_grasped(),
        }

    def shutdown(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None


# ---------------------------------------------------------------- 仿真夹爪

class MockGripper(GripperBase):
    """仿真夹爪：模拟开合时序与「夹到物体」停顿，用于无硬件联调与测试。"""

    backend = "mock"

    def __init__(self, cfg: dict):
        self.q_open = float(cfg.get("q_open", 0.5))
        self.q_close = float(cfg.get("q_close", 0.02))
        self.close_time_s = float(cfg.get("mock_close_time_s", 0.24))
        self.settle_tol = float(cfg.get("settle_tol", 0.03))
        self._q = self.q_open
        self._target = self.q_open
        self._tau = 0.0
        self._grasped = False
        self.events: list = []

    def connect(self) -> None:
        log.info("仿真夹爪就绪（q=%.3f → %.3f，闭合用时 %.2fs）", self.q_close, self.q_open, self.close_time_s)

    def _log(self, what: str) -> None:
        self.events.append((time.time(), what, round(self._q, 4)))
        log.info("[仿真夹爪] %s（q=%.3f）", what, self._q)

    def _move_to(self, q: float) -> None:
        stroke = max(1e-6, abs(self.q_open - self.q_close))
        t = abs(q - self._q) / stroke * self.close_time_s
        steps = max(1, int(t / 0.02))
        for _ in range(steps):
            self._q += (q - self._q) / steps
            time.sleep(t / steps if t > 0 else 0.0)
        self._q = q

    def open_grasp(self, wait: bool = True) -> None:
        self._target = self.q_open
        self._grasped = False
        self._tau = 0.0
        self._log("张开")
        if wait:
            self._move_to(self.q_open)

    def close_grasp(self, wait: bool = True) -> None:
        self._target = self.q_close
        self._log("闭合指令")
        if wait:
            stall = self.q_close + 0.12     # 模拟夹到手机后停住
            self._move_to(stall)
            self._tau = 2.0
            self._grasped = True
            self._log("已夹持（模拟物体）")

    def stop(self) -> None:
        self._target = self._q

    def position(self) -> Optional[float]:
        return self._q

    def torque(self) -> Optional[float]:
        return self._tau

    def is_grasped(self) -> Optional[bool]:
        return self._grasped

    def state(self) -> dict:
        return {
            "backend": self.backend,
            "q": self._q,
            "tau": self._tau,
            "target": self._target,
            "grasped": self._grasped,
        }


class NoneGripper(GripperBase):
    """未启用夹爪：所有动作空操作（纯臂测试用）。"""

    backend = "none"

    def connect(self) -> None:
        log.info("夹爪未启用（backend=none）")

    def open_grasp(self, wait: bool = True) -> None:
        log.info("[无夹爪] 忽略张开")

    def close_grasp(self, wait: bool = True) -> None:
        log.info("[无夹爪] 忽略闭合")

    def state(self) -> dict:
        return {"backend": self.backend}


# ---------------------------------------------------------------- 工厂

def make_gripper(cfg: dict, sim: bool = False) -> GripperBase:
    """按配置构造夹爪驱动；sim=True 时强制使用仿真夹爪（backend=none 除外）。"""
    cfg = dict(cfg or {})
    backend = str(cfg.get("backend", "dex1_service"))
    if sim and backend != "none":
        backend = "mock"
    if backend == "none":
        return NoneGripper()
    if backend == "mock":
        return MockGripper(cfg)
    if backend == "dex1_service":
        return Dex1ServiceGripper(cfg)
    raise GripperError(f"未知夹爪 backend：{backend}（可选 dex1_service / mock / none）")
