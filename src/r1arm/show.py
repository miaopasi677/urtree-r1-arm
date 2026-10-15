# -*- coding: utf-8 -*-
"""展示动作脚本执行器。

对应《展厅机械臂旋转展示方案_V1.0》的 8 步动作路径：
    1 待机 → 2 夹取手机 → 3 抬升 90° → 4 左旋 90° → 5 右旋 180°
    → 6 回中 → 7 末端旋转 3 圈 → 8 放回手机（补充）→ 9 复位循环

动作全部由 config/show_script.json 驱动，节拍 = duration_s / speed_scale，
最终时长还会被控制环按关节速度上限自动拉长（只慢不快，安全兜底）。
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Dict, List, Optional

from .arm import ArmModel
from .controller import ArmController
from .gripper import GripperBase
from .trajectory import Segment

log = logging.getLogger(__name__)

__all__ = ["ShowRunner", "ShowError"]


class ShowError(RuntimeError):
    """展示脚本错误（位姿缺失、未标定、动作类型未知等）。"""


class ShowRunner:
    """把 show_script.json 变成真实动作序列。"""

    def __init__(
        self,
        controller: ArmController,
        model: ArmModel,
        gripper: GripperBase,
        script: dict,
        poses: dict,
        *,
        speed_scale: Optional[float] = None,
        cycles: Optional[int] = None,
        allow_uncalibrated: bool = False,
        say: Optional[Callable[[str], None]] = None,
    ):
        self._ctl = controller
        self._model = model
        self._gripper = gripper
        self.script = script
        self.poses = poses
        self.speed_scale = float(
            speed_scale if speed_scale is not None else script.get("speed_scale", 1.0)
        ) or 1.0
        self.cycles = int(cycles if cycles is not None else script.get("cycles", 0))
        self.allow_uncalibrated = allow_uncalibrated
        self._say: Callable[[str], None] = say or (lambda s: None)
        self._stop = False

    # ---------------------------------------------------------------- 准备

    def _pose(self, name: str) -> Dict[str, float]:
        p = self.poses.get("poses", {}).get(name)
        if p is None:
            raise ShowError(f"位姿 {name} 未在 poses.json 中定义")
        try:
            return self._model.pose_from_list(list(p["deg"]))
        except Exception as exc:
            raise ShowError(f"位姿 {name} 格式错误：{exc}") from exc

    def check_ready(self) -> None:
        """运行前检查：位姿已标定、脚本引用的位姿齐全。"""
        if not self.poses.get("calibrated", False) and not self.allow_uncalibrated:
            raise ShowError(
                "poses.json 尚未标定（calibrated=false）。"
                "请先运行 `python r1ctl.py teach` 在真机上示教各关键位姿并标记 calibrated，"
                "或演练模式加 --force"
            )
        steps = self.script.get("steps") or []
        if not steps:
            raise ShowError("动作脚本没有步骤（show_script.json → steps）")
        for st in steps:
            for key in ("pose", "pre_pose", "retreat_pose"):
                if key in st:
                    self._pose(st[key])
                    if not self.allow_uncalibrated and not self.poses["poses"][st[key]].get("calibrated", False):
                        raise ShowError(f"位姿 {st[key]} 尚未标定，真机禁止运行")
            if st.get("action") == "wrist_spin":
                jname = str(st.get("joint", "J7"))
                if jname not in self._model.joints:
                    raise ShowError(f"wrist_spin 指定的关节 {jname} 不存在")
        log.info(
            "展示脚本检查通过：%s，%d 步，speed_scale=%.2f，cycles=%s",
            self.script.get("name", "?"), len(steps), self.speed_scale,
            "∞" if self.cycles == 0 else self.cycles,
        )

    # ---------------------------------------------------------------- 运行

    def stop(self) -> None:
        """请求停止（当前步执行完后退出循环）。"""
        self._stop = True

    def run(self, dry_run: bool = False) -> int:
        """执行展示循环；返回完成的循环数。"""
        self.check_ready()
        name = self.script.get("name", "show")
        self._say(
            f"开始展示「{name}」：speed_scale={self.speed_scale}，"
            f"cycles={'∞' if self.cycles == 0 else self.cycles}"
            + ("（预览模式，不驱动硬件）" if dry_run else "")
        )
        done = 0
        cycle = 0
        while not self._stop:
            cycle += 1
            if self.cycles and cycle > self.cycles:
                break
            for st in self.script["steps"]:
                if self._stop:
                    break
                self._exec_step(st, cycle, dry_run)
            if not self._stop:
                done = cycle
                self._say(f"—— 第 {cycle} 循环完成 ——")
        self._say(f"展示结束，共完成 {done} 个循环")
        return done

    # ---------------------------------------------------------------- 单步

    def _dur(self, seconds: float) -> float:
        return float(seconds) / self.speed_scale

    def _note(self, text: str) -> None:
        self._ctl.set_note(text)
        self._say(text)

    def _exec_step(self, st: dict, cycle: int, dry_run: bool) -> None:
        action = st.get("action")
        label = f"[步{st.get('step', '?')}] {st.get('desc', '')}"
        if dry_run:
            self._say(f"(预览){label}")
            return
        self._note(label)

        if action == "pose":
            self._ctl.move_to(self._pose(st["pose"]), self._dur(st["duration_s"]), name=label)

        elif action == "pick":
            pre = self._pose(st["pre_pose"])
            pick = self._pose(st["pose"])
            self._ctl.move_to(pre, self._dur(st.get("approach_s", 2.0)), name=label + "·接近")
            self._ctl.move_to(pick, self._dur(st.get("descend_s", 1.0)), name=label + "·下探")
            self._say("  夹爪闭合…")
            self._gripper.close_grasp()
            time.sleep(max(0.0, self._dur(st.get("close_s", 0.3))))
            st_state = self._gripper.state()
            grasped = st_state.get("grasped")
            self._say(
                f"  夹持完成 q={st_state.get('q')}"
                + ("" if grasped is None else ("（已夹到物体）" if grasped else "（⚠ 未检测到夹持力，请检查）"))
            )

        elif action == "place":
            pick = self._pose(st["pose"])
            self._ctl.move_to(pick, self._dur(st.get("approach_s", 2.0)), name=label + "·放回")
            self._say("  夹爪张开…")
            self._gripper.open_grasp()
            time.sleep(max(0.0, self._dur(st.get("open_s", 0.3))))
            retreat = st.get("retreat_pose")
            if retreat:
                self._ctl.move_to(self._pose(retreat), self._dur(st.get("retreat_s", 1.5)), name=label + "·撤离")

        elif action == "reset":
            self._ctl.move_to(
                self._pose(st.get("pose", "HOME")), self._dur(st.get("duration_s", 3.0)), name=label
            )

        elif action == "wrist_spin":
            self._wrist_spin(st, label)

        elif action == "wait":
            time.sleep(max(0.0, self._dur(st.get("duration_s", 1.0))))

        elif action == "gripper":
            to = str(st.get("to", "open"))
            if to == "open":
                self._gripper.open_grasp()
            else:
                self._gripper.close_grasp()

        else:
            raise ShowError(f"未知动作类型：{action!r}（step {st.get('step')}）")

    # ---------------------------------------------------------------- 腕部旋转

    def _wrist_spin(self, st: dict, label: str) -> None:
        """末端旋转 N 圈：按方案「腕部往复旋转」的视觉等效实现。

        以当前 J7（或指定关节）角为中心，±amplitude 往复 2×cycles 次后回到中心；
        若评审后确认改用旋转台等其他形式，直接修改本函数或替换脚本步骤。
        """
        jname = str(st.get("joint", "J7"))
        spec = self._model.joints[jname]
        amp = float(st.get("amplitude_deg", 90.0))
        n = max(1, int(st.get("cycles", 3)))
        total = max(0.1, self._dur(float(st.get("duration_s", 6.0))))

        base = dict(self._ctl.current_setpoint())
        cur = float(base[jname])
        lo, hi = self._model.limits_deg(spec)

        t_each = total / (2 * n + 1)
        segs: List[Segment] = []
        prev = dict(base)
        for i in range(2 * n):
            target = cur + amp if i % 2 == 0 else cur - amp
            if target < lo or target > hi:
                clamped = min(hi, max(lo, target))
                log.warning("腕部目标 %.1f° 超出 %s 软限位 %.1f~%.1f°，已钳位", target, jname, lo, hi)
                target = clamped
            end = dict(prev)
            end[jname] = target
            segs.append(Segment(start=dict(prev), end=end, duration_s=t_each, name=f"{label}·摆动{i + 1}"))
            prev = end
        end = dict(prev)
        end[jname] = cur
        segs.append(Segment(start=dict(prev), end=end, duration_s=t_each, name=f"{label}·回中"))
        self._ctl.play(segs, blocking=True, name=label)
