# -*- coding: utf-8 -*-
"""示教与标定工具（交互式命令行）。

现场标定流程建议：
  1) 上电后运行 `python r1ctl.py teach`（程序会先使能并保持当前位置）；
  2) 用 j / r 微调各关节到约定机械零位，执行 zero 把当前姿态记为关节零位
     （写入 arm_config.json 的 offset_deg，之后所有角度都以该零位为参考）；
  3) 依次把臂移动到 HOME / PRE_PICK / PICK / LIFT90 / ROT_L90 / ROT_R90 / ROT_MID，
     每个点位执行 save <名称>，确认动作正确后执行 calib <名称> 标记已标定；
  4) q 保存退出；`python r1ctl.py demo --cycles 1 --speed 0.5` 低速试运行。
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Dict, Optional

from .arm import ArmModel
from .config import AppConfig, save_arm_config, save_poses
from .controller import ArmController
from .gripper import GripperBase

log = logging.getLogger(__name__)

__all__ = ["TeachSession"]

TEACH_SPEED_DEG_S = 40.0     # 示教点动速度（度/秒，时长按最大关节行程估算）
MIN_MOVE_S = 0.4

HELP = """可用命令：
  help                    显示本帮助
  show                    显示当前关节角 / 在线 / 温度 / 夹爪状态
  j <关节> <角度>         某关节转到绝对角度（度），如：j J2 90
  r <关节> <增量>         某关节点动相对增量，如：r J2 +5
  move <J1>..<J7>         整体移动到 7 个绝对角度（度），空格分隔
  open / close            夹爪张开 / 闭合
  hold                    保持当前位置（中止当前动作）
  save <名称> [说明]      把当前位姿记录到 poses.json（默认未标定）
  calib <名称>            把某位姿标记为「已标定」
  list                    列出已记录位姿
  zero                    把当前物理姿态记为各关节零位（写回 arm_config.json）
  q / quit                保存并退出（电机停机）

说明：move/j/r 的时长按 %d°/s 估算，控制环会按关节速度上限自动兜底。
""" % TEACH_SPEED_DEG_S


class TeachSession:
    """交互式示教会话（阻塞主线程读 stdin，控制环在后台线程保持使能）。"""

    def __init__(
        self,
        controller: ArmController,
        model: ArmModel,
        gripper: Optional[GripperBase],
        app_config: AppConfig,
        poses: dict,
        say: Optional[Callable[[str], None]] = None,
    ):
        self._ctl = controller
        self._model = model
        self._gripper = gripper
        self._cfg = app_config
        self.poses = poses
        self._say: Callable[[str], None] = say or (lambda s: None)

    # ---------------------------------------------------------------- 主循环

    def run(self) -> None:
        self._ctl.enable()
        self._say("进入示教模式（help 查看命令；Ctrl+C 保存退出）")
        while True:
            try:
                line = input("teach> ").strip()
            except (EOFError, KeyboardInterrupt):
                line = "q"
            if not line:
                continue
            try:
                if self._exec(line):
                    break
            except (ValueError, IndexError) as exc:
                self._say(f"参数错误：{exc}（help 查看用法）")
            except Exception as exc:  # 控制环抛出的故障/中断
                self._say(f"执行失败：{exc}")
                if self._ctl.mode.value in ("fault", "estop"):
                    self._say("控制环已停机，退出示教。")
                    self._save()
                    break

    # ---------------------------------------------------------------- 命令实现

    def _exec(self, line: str) -> bool:
        """执行一条命令；返回 True 表示退出。"""
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            self._save()
            return True
        if cmd == "help":
            self._say(HELP)
        elif cmd == "show":
            self._show()
        elif cmd == "j":
            joint = self._joint(parts[1])
            self._goto_one(joint, float(parts[2]))
        elif cmd == "r":
            joint = self._joint(parts[1])
            cur = self._ctl.current_setpoint()
            self._goto_one(joint, cur[joint] + float(parts[2]))
        elif cmd == "move":
            if len(parts) - 1 != len(self._model.order):
                raise ValueError(f"需要 {len(self._model.order)} 个角度值")
            vals = [float(x) for x in parts[1:]]
            self._move(self._model.pose_from_list(vals), "整体移动")
        elif cmd in ("open", "close"):
            if self._gripper is None:
                self._say("未启用夹爪")
            elif cmd == "open":
                self._gripper.open_grasp()
            else:
                self._gripper.close_grasp()
        elif cmd == "hold":
            self._ctl.hold()
        elif cmd == "save":
            if len(parts) < 2:
                raise ValueError("save <名称> [说明]")
            self._save_pose(parts[1], " ".join(parts[2:]))
        elif cmd == "calib":
            if len(parts) < 2:
                raise ValueError("calib <名称>")
            self._mark_calibrated(parts[1])
        elif cmd == "list":
            self._list()
        elif cmd == "zero":
            self._zero()
        else:
            self._say("未知命令（help 查看帮助）")
        return False

    def _joint(self, name: str) -> str:
        name = name.upper().replace("J", "J")
        if name not in self._model.joints:
            raise ValueError(f"关节 {name} 不存在（{','.join(self._model.order)}）")
        return name

    def _move(self, pose: Dict[str, float], what: str) -> None:
        pose, clamped = self._model.clamp_pose(pose)
        for c in clamped:
            self._say(f"  软限位钳位：{c}")
        best, delta = self._model.max_delta_deg(self._ctl.current_setpoint(), pose)
        duration = max(MIN_MOVE_S, delta / TEACH_SPEED_DEG_S)
        self._ctl.move_to(pose, duration, name=f"示教·{what}")
        self._say("  " + self._model.summary_lines(self._ctl.current_setpoint()))

    def _goto_one(self, joint: str, target: float) -> None:
        pose = self._ctl.current_setpoint()
        pose[joint] = target
        self._move(pose, f"{joint} → {target:.1f}°")

    def _show(self) -> None:
        snap = self._ctl.snapshot()
        self._say(
            f"模式={snap['mode']}  环路={snap['loop']['hz']:.0f}Hz  错误={snap['error']}"
        )
        measured = snap["measured"]
        fb = self._ctl._bus.all_feedback()
        for name in self._model.order:
            f = fb[name]
            spec = self._model.joints[name]
            if f.online:
                self._say(
                    f"  {name}: {measured[name]:7.1f}°  [{spec.min_deg:.0f},{spec.max_deg:.0f}]"
                    f"  温度 {f.state.temp1_c}/{f.state.temp2_c}℃  力矩 {f.state.torque_nm:+.2f}N·m"
                    + (f"  ⚠错误码 0x{f.state.merror:08X}" if f.state.merror else "")
                )
            else:
                self._say(f"  {name}: 无反馈")
        if self._gripper is not None:
            self._say(f"  夹爪: {self._gripper.state()}")

    def _list(self) -> None:
        for name, p in self.poses.get("poses", {}).items():
            flag = "✓" if p.get("calibrated") else "未标定"
            self._say(f"  {name:10s} {flag}  {p.get('desc', '')}")
        self._say(f"  整体标定状态: {'已标定' if self.poses.get('calibrated') else '未标定'}")

    def _save_pose(self, name: str, desc: str) -> None:
        measured = self._ctl.measured_pose()
        deg = self._model.pose_to_list(measured)
        self.poses.setdefault("poses", {})[name.upper()] = {
            "deg": deg,
            "calibrated": False,
            "desc": desc or "",
        }
        self.poses["calibrated"] = False
        save_poses(self._cfg.poses_path, self.poses)
        self._say(f"  已记录 {name.upper()}: [{', '.join(f'{v:.1f}' for v in deg)}]（尚未标记已标定）")

    def _mark_calibrated(self, name: str) -> None:
        name = name.upper()
        p = self.poses.get("poses", {}).get(name)
        if p is None:
            raise ValueError(f"位姿 {name} 不存在，先 save 记录")
        p["calibrated"] = True
        all_ok = all(bool(q.get("calibrated")) for q in self.poses["poses"].values())
        self.poses["calibrated"] = all_ok
        save_poses(self._cfg.poses_path, self.poses)
        self._say(f"  {name} 已标记为已标定" + ("；全部位姿已标定 ✅" if all_ok else ""))

    def _zero(self) -> None:
        """把当前姿态记为各关节零位：offset_new = offset + direction × 当前关节角。"""
        measured = self._ctl.measured_pose()
        raw_joints = [dict(j) for j in self._cfg.raw.get("joints", [])]
        for j in raw_joints:
            name = str(j["name"])
            spec = self._model.joints[name]
            new_offset = spec.offset_deg + spec.direction * measured[name]
            j["offset_deg"] = round(float(new_offset), 4)
            spec.offset_deg = float(new_offset)
        save_arm_config(self._cfg, {"joints": raw_joints})
        # 控制环继续运行：把指令位姿重置为全 0（= 当前物理姿态）
        self._ctl.rebase_setpoint({n: 0.0 for n in self._model.order})
        self._say("  零位已写入 arm_config.json（offset_deg），指令位姿已重置为 0")

    def _save(self) -> None:
        try:
            save_poses(self._cfg.poses_path, self.poses)
        except Exception as exc:
            log.error("保存 poses.json 失败：%s", exc)
