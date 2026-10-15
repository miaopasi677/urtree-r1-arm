# -*- coding: utf-8 -*-
"""控制环：后台线程按固定频率刷新指令帧。

运行模式：
    IDLE    未使能 —— 持续发停机帧（status=0）
    HOLD    已使能 —— 保持当前指令位置（FOC，各关节使用配置的 kp/kd）
    TRACK   跟踪轨迹 —— 最小 jerk 插值，可被急停中断
    ESTOP   急停   —— 持续发停机帧，直到 release()
    FAULT   故障   —— 同急停；错误信息保留在 .error，需排查后重启程序

线程模型：控制环在独立线程中运行；enable / move_to / play / estop 等对外
接口线程安全，阻塞接口通过内部事件等待完成。
"""

from __future__ import annotations

import csv
import enum
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .arm import ArmModel
from .bus import BusLostError, MotorBus
from .protocol import MotorCommand, STATUS_FOC, stop_command
from .safety import EStopTriggered, FaultError, MoveAborted, TempLimits, check_feedback_health
from .trajectory import Segment, stretch_duration

log = logging.getLogger(__name__)

__all__ = ["RunMode", "ControllerConfig", "ArmController"]


class RunMode(enum.Enum):
    IDLE = "idle"
    HOLD = "hold"
    TRACK = "track"
    ESTOP = "estop"
    FAULT = "fault"


@dataclass
class ControllerConfig:
    """控制环与安全参数（由 config.py 从 arm_config.json 装载）。"""

    rate_hz: float = 200.0
    read_timeout_ms: float = 3.0
    lost_timeout_ms: float = 500.0
    temp1_warn_c: float = 70.0
    temp1_stop_c: float = 85.0
    temp2_warn_c: float = 90.0
    temp2_stop_c: float = 110.0
    enable_requires_all_online: bool = True
    csv_log: bool = False
    csv_dir: str = "logs"

    def temp_limits(self) -> TempLimits:
        return TempLimits(
            temp1_warn_c=self.temp1_warn_c,
            temp1_stop_c=self.temp1_stop_c,
            temp2_warn_c=self.temp2_warn_c,
            temp2_stop_c=self.temp2_stop_c,
        )


class ArmController:
    """七轴臂控制环。"""

    def __init__(self, bus: MotorBus, model: ArmModel, config: Optional[ControllerConfig] = None):
        self._bus = bus
        self._model = model
        self._cfg = config or ControllerConfig()

        self._lock = threading.RLock()
        self._mode = RunMode.IDLE
        self._setpoint: Dict[str, float] = {}
        self._segments: List[Segment] = []
        self._seg_idx = 0
        self._seg_t0 = 0.0
        self._evt = threading.Event()
        self._ready_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._error: Optional[BaseException] = None

        self.stats: Dict[str, float] = {"cycles": 0, "overruns": 0, "hz": 0.0}
        self._hz_count = 0
        self._hz_t0 = 0.0

        self._csv = None
        self._csv_writer = None
        self._csv_path = ""
        self._csv_every = max(1, int(self._cfg.rate_hz / 20))   # CSV 约 20 Hz
        self._csv_i = 0
        self._note = ""
        self._warn_ts: Dict[str, float] = {}

    # ================================================================ 生命周期

    def start(self, timeout_s: float = 3.0) -> None:
        """启动控制环线程并等待首个控制周期完成。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._ready_evt.clear()
        if self._cfg.csv_log:
            self._open_csv()
        self._hz_t0 = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="arm-control", daemon=True)
        self._thread.start()
        if not self._ready_evt.wait(timeout_s):
            raise RuntimeError("控制环启动超时（首周期未完成）")
        log.info("控制环已启动：目标 %.0f Hz，通道 %s", self._cfg.rate_hz, self._bus.describe())

    def shutdown(self, stop_rounds: int = 5) -> None:
        """停止控制环并向所有电机连发停机帧。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._bus.request_stop(rounds=stop_rounds)
        self._close_csv()
        log.info("控制环已退出，电机已停机")

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def describe(self) -> str:
        return self._bus.describe()

    # ================================================================ 状态查询

    @property
    def mode(self) -> RunMode:
        return self._mode

    @property
    def error(self) -> Optional[BaseException]:
        return self._error

    def measured_pose(self) -> Dict[str, float]:
        return self._model.measured_pose(self._bus.all_feedback())

    def current_setpoint(self) -> Dict[str, float]:
        """当前指令位姿（度）——展示脚本/示教在此基础上叠加新目标。"""
        with self._lock:
            return dict(self._setpoint)

    def rebase_setpoint(self, pose_deg: Dict[str, float]) -> None:
        """零位标定后重置指令位姿（仅 HOLD 模式；仅限示教工具使用）。"""
        with self._lock:
            if self._mode != RunMode.HOLD:
                raise MoveAborted("仅使能保持状态下可重置指令位姿")
            self._setpoint = dict(pose_deg)

    def snapshot(self) -> dict:
        fb = self._bus.all_feedback()
        with self._lock:
            mode = self._mode
            setpoint = dict(self._setpoint)
        measured: Dict[str, Optional[float]] = {}
        for name, spec in self._model.joints.items():
            f = fb[name]
            measured[name] = self._model.joint_deg(spec, f) if f.online else None
        return {
            "mode": mode.value,
            "setpoint": setpoint,
            "measured": measured,
            "online": {n: fb[n].online for n in self._model.order},
            "error": (str(self._error) if self._error else None),
            "bus_stats": dict(self._bus.stats),
            "loop": {"hz": float(self.stats.get("hz", 0.0)), "overruns": int(self.stats.get("overruns", 0))},
        }

    def set_note(self, note: str) -> None:
        """写入运行日志的备注列（展示程序用来标注当前步骤）。"""
        self._note = str(note)

    # ================================================================ 指令接口

    def enable(self, timeout_s: float = 5.0) -> None:
        """使能所有关节（进入 FOC）并保持当前位置。"""
        t0 = time.monotonic()
        while True:
            missing = self._bus.missing()
            if not missing:
                break
            if not self._cfg.enable_requires_all_online:
                log.warning("部分关节不在线（配置允许继续使能）：%s", missing)
                break
            if time.monotonic() - t0 > timeout_s:
                raise BusLostError(f"以下关节无反馈，无法使能：{missing}（请检查接线/供电/波特率）")
            time.sleep(0.05)

        pose = self.measured_pose()
        with self._lock:
            if self._mode == RunMode.FAULT:
                raise (self._error if isinstance(self._error, Exception) else FaultError("故障状态"))
            if self._mode == RunMode.ESTOP:
                raise MoveAborted("当前处于急停状态，请先 release() 解除")
            self._setpoint = dict(pose)
            self._segments = []
            self._mode = RunMode.HOLD
            self._evt.clear()
        log.info("已使能并保持当前位置：%s", self._model.summary_lines(pose))

    def release(self, note: str = "释放使能") -> None:
        """释放使能（持续发停机帧）；同时解除急停状态。"""
        with self._lock:
            if self._mode == RunMode.FAULT:
                raise FaultError("故障状态需重启程序并排查硬件后再使能")
            was = self._mode
            self._mode = RunMode.IDLE
            if isinstance(self._error, EStopTriggered):
                self._error = None
            self._evt.set()
        if was != RunMode.IDLE:
            log.info("%s（电机停机）", note)

    def hold(self) -> None:
        """中止当前动作并保持在当前位置。"""
        with self._lock:
            if self._mode != RunMode.TRACK:
                return
            self._segments = []
            self._mode = RunMode.HOLD
            self._setpoint = dict(self._setpoint)
            self._evt.set()
        log.info("已保持当前位置")

    def estop(self, reason: str = "软件急停") -> None:
        """软件急停：立即切换为持续停机帧。"""
        with self._lock:
            if self._mode == RunMode.FAULT:
                return
            self._mode = RunMode.ESTOP
            if not isinstance(self._error, (EStopTriggered, FaultError)):
                self._error = EStopTriggered(reason)
            self._evt.set()
        log.warning("急停已触发：%s", reason)

    # ================================================================ 动作接口

    def make_segment(self, target_pose_deg: Dict[str, float], duration_s: float, name: str = "") -> Segment:
        """基于当前指令位置构造一段轨迹（自动按速度上限拉长时长）。"""
        with self._lock:
            if self._mode != RunMode.HOLD:
                raise MoveAborted(f"当前模式为 {self._mode.value}，无法下发动作（需先 enable 且不在动作中）")
            start = dict(self._setpoint)
        target = self._model.validate_pose(target_pose_deg, allow_clamp=False)
        seg = Segment(start=start, end=target, duration_s=float(duration_s), name=name)
        seg.duration_s = stretch_duration(seg, self._model.speed_limits())
        return seg

    def move_to(self, target_pose_deg: Dict[str, float], duration_s: float,
                blocking: bool = True, name: str = "") -> None:
        """单段移动到位姿（度）。"""
        seg = self.make_segment(target_pose_deg, duration_s, name=name)
        self.play([seg], blocking=blocking, name=name)

    def play(self, segments: List[Segment], blocking: bool = True, name: str = "") -> None:
        """下发一段或多段轨迹；blocking=True 时等待执行完成。"""
        if not segments:
            return
        with self._lock:
            if self._mode != RunMode.HOLD:
                raise MoveAborted(f"当前模式为 {self._mode.value}，无法下发动作（需先 enable 且不在动作中）")
            # 统一按各关节速度上限拉长时长（只拉长不缩短）
            limits = self._model.speed_limits()
            adjusted: List[Segment] = []
            for seg in segments:
                seg.duration_s = max(float(seg.duration_s), stretch_duration(seg, limits))
                adjusted.append(seg)
            self._segments = adjusted
            self._seg_idx = 0
            self._seg_t0 = 0.0
            self._mode = RunMode.TRACK
            self._evt.clear()
        if name:
            self.set_note(name)
        if blocking:
            self.wait_done()

    def wait_done(self, timeout_s: Optional[float] = None) -> None:
        """等待当前动作结束；故障/急停/释放时抛出相应异常。"""
        t0 = time.monotonic()
        while True:
            if self._evt.wait(0.1):
                break
            if self._mode in (RunMode.FAULT, RunMode.ESTOP, RunMode.IDLE):
                self._evt.set()
                break
            if timeout_s is not None and (time.monotonic() - t0) > timeout_s:
                raise TimeoutError("等待动作完成超时")
        with self._lock:
            if self._mode == RunMode.FAULT:
                raise (self._error if isinstance(self._error, Exception) else FaultError("故障停机"))
            if self._mode == RunMode.ESTOP:
                raise (self._error if isinstance(self._error, EStopTriggered) else EStopTriggered("急停"))
            if self._mode == RunMode.IDLE:
                raise MoveAborted("动作被中断（使能已释放）")

    # ================================================================ 控制环内部

    def _loop(self) -> None:
        period = 1.0 / max(1.0, float(self._cfg.rate_hz))
        next_t = time.monotonic()
        first = True
        while self._running:
            now = time.monotonic()
            try:
                cmds = self._compose(now)
                self._bus.step(cmds)
            except Exception as exc:      # 总线异常等
                self._fault(exc)
            else:
                self._post_checks()

            self.stats["cycles"] = int(self.stats["cycles"]) + 1
            self._hz_count += 1
            if now - self._hz_t0 >= 1.0:
                self.stats["hz"] = self._hz_count / max(1e-6, now - self._hz_t0)
                self._hz_count = 0
                self._hz_t0 = now

            self._csv_tick()
            if first:
                first = False
                self._ready_evt.set()

            # 故障/急停时降频发送停机帧，避免刷屏与总线拥堵
            eff_period = period if self._mode in (RunMode.HOLD, RunMode.TRACK) else max(period, 0.02)
            next_t += eff_period
            delay = next_t - time.monotonic()
            if delay <= 0:
                self.stats["overruns"] = int(self.stats["overruns"]) + 1
                next_t = time.monotonic()
            else:
                time.sleep(delay)

        try:
            self._bus.request_stop(rounds=3)
        except Exception:
            pass

    def _compose(self, now: float) -> Dict[str, MotorCommand]:
        """按当前模式生成一帧指令（在锁内更新插值状态）。"""
        with self._lock:
            mode = self._mode
            if mode in (RunMode.IDLE, RunMode.ESTOP, RunMode.FAULT):
                return {j.name: stop_command() for j in self._bus.joints}

            if mode == RunMode.HOLD:
                pose = dict(self._setpoint)
            else:  # TRACK
                seg = self._segments[self._seg_idx]
                if self._seg_t0 == 0.0:
                    self._seg_t0 = now
                frac = (now - self._seg_t0) / max(1e-6, seg.duration_s)
                if frac >= 1.0:
                    self._seg_idx += 1
                    if self._seg_idx >= len(self._segments):
                        self._mode = RunMode.HOLD
                        self._setpoint = dict(seg.end)
                        self._evt.set()
                        pose = dict(self._setpoint)
                    else:
                        self._seg_t0 = now
                        seg = self._segments[self._seg_idx]
                        pose = seg.setpoint(0.0)
                        self._setpoint = pose
                else:
                    pose = seg.setpoint(frac)
                    self._setpoint = pose

            return {
                j.name: MotorCommand(
                    pos_rad=self._model.motor_pos_rad(j, pose[j.name]),
                    speed_rad_s=0.0,
                    torque_nm=0.0,
                    kp=j.kp,
                    kd=j.kd,
                    status=STATUS_FOC,
                    timeout_enable=True,
                )
                for j in self._bus.joints
            }

    def _post_checks(self) -> None:
        with self._lock:
            mode = self._mode
        if mode not in (RunMode.HOLD, RunMode.TRACK):
            return
        fb = self._bus.all_feedback()
        try:
            check_feedback_health(fb, self._cfg.temp_limits(), warn_cb=self._warn_temp)
        except FaultError as exc:
            self._fault(exc)
            return

        now = time.monotonic()
        lost_s = self._cfg.lost_timeout_ms / 1000.0
        for name, f in fb.items():
            if f.last_rx_t and (now - f.last_rx_t) > lost_s:
                self._fault(BusLostError(f"关节 {name} 通信丢失（超过 {lost_s * 1000:.0f} ms 无反馈）"))
                return

        if mode == RunMode.TRACK:
            for j in self._bus.joints:
                if j.tau_guard_nm is not None:
                    tau = fb[j.name].state.torque_nm
                    if abs(tau) > j.tau_guard_nm:
                        self._fault(
                            FaultError(
                                f"关节 {j.name} 力矩 {tau:.2f} N·m 超过阈值 {j.tau_guard_nm} N·m（疑似碰撞），已停机"
                            )
                        )
                        return

    def _fault(self, exc: BaseException) -> None:
        with self._lock:
            if self._mode == RunMode.FAULT:
                return
            self._mode = RunMode.FAULT
            self._error = exc if isinstance(exc, Exception) else FaultError(str(exc))
            self._evt.set()
        log.error("触发故障停机：%s", exc)

    def _warn_temp(self, name: str, st) -> None:
        now = time.monotonic()
        key = f"temp:{name}"
        if (now - self._warn_ts.get(key, 0.0)) > 30.0:
            self._warn_ts[key] = now
            log.warning("关节 %s 温度偏高：驱动 %d℃ / 绕组 %d℃", name, st.temp1_c, st.temp2_c)

    # ================================================================ CSV 日志

    def _open_csv(self) -> None:
        try:
            os.makedirs(self._cfg.csv_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self._csv_path = os.path.join(self._cfg.csv_dir, f"arm_{stamp}.csv")
            self._csv = open(self._csv_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv)
            header = ["t", "mode", "note", "max_temp1_c", "max_temp2_c"]
            header += [f"{n}_deg" for n in self._model.order]
            header += [f"{n}_tau_nm" for n in self._model.order]
            self._csv_writer.writerow(header)
            log.info("运行日志：%s", self._csv_path)
        except Exception as exc:
            log.warning("无法创建 CSV 运行日志：%s", exc)
            self._csv = None
            self._csv_writer = None

    def _csv_tick(self) -> None:
        if self._csv_writer is None:
            return
        self._csv_i += 1
        if self._csv_i % self._csv_every:
            return
        fb = self._bus.all_feedback()
        deg: List[str] = []
        tau: List[str] = []
        t1 = t2 = 0
        for n in self._model.order:
            f = fb[n]
            t1 = max(t1, int(f.state.temp1_c))
            t2 = max(t2, int(f.state.temp2_c))
            deg.append(f"{self._model.joint_deg(self._model.joints[n], f):.2f}" if f.online else "")
            tau.append(f"{f.state.torque_nm:.2f}" if f.online else "")
        try:
            self._csv_writer.writerow(
                [f"{time.time():.3f}", self._mode.value, self._note, t1, t2] + deg + tau
            )
            self._csv.flush()
        except Exception:
            pass

    def _close_csv(self) -> None:
        if self._csv is not None:
            try:
                self._csv.close()
            except Exception:
                pass
            self._csv = None
            self._csv_writer = None
