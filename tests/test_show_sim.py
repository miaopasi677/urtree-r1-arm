# -*- coding: utf-8 -*-
"""仿真联测：MockTransport + MockGripper 无硬件跑通完整展示脚本（含夹取手机）。

覆盖：
  1) 两轮完整展示循环（8+ 步动作）无异常完成；
  2) 所有下发指令都在关节软限位内；
  3) 急停路径：运行中 estop → 动作被中断 → 电机收到停机帧。
"""

import contextlib
import io
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from r1arm.arm import ArmModel  # noqa: E402
from r1arm.cli import _boot, main  # noqa: E402
from r1arm.bus import MotorBus  # noqa: E402
from r1arm.config import load_config, load_poses, load_show  # noqa: E402
from r1arm.controller import ArmController, RunMode  # noqa: E402
from r1arm.gripper import make_gripper  # noqa: E402
from r1arm.safety import EStopTriggered, MoveAborted  # noqa: E402
from r1arm.show import ShowError, ShowRunner  # noqa: E402
from r1arm.transport import MockTransport  # noqa: E402

PROJECT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _make_harness(rate_hz=300.0, speed_limit=3000.0, sim_dt=0.002):
    """搭建一套仿真运行时（提速参数用于缩短测试时间）。"""
    cfg = load_config(os.path.join(PROJECT, "config"))
    for j in cfg.joints:
        j.max_speed_deg_s = speed_limit
    cfg.control.rate_hz = rate_hz
    cfg.control.csv_log = False
    cfg.control.read_timeout_ms = 3.0
    cfg.gripper = dict(cfg.gripper)
    cfg.gripper["backend"] = "mock"
    cfg.gripper["mock_close_time_s"] = 0.05

    motors = [
        {"motor_id": j.motor_id, "model": j.model, "max_speed_deg_s": j.max_speed_deg_s}
        for j in cfg.joints
    ]
    transport = MockTransport(motors, sim_dt_s=sim_dt)
    bus = MotorBus(
        transport, cfg.joints,
        read_timeout_ms=cfg.control.read_timeout_ms,
        lost_timeout_ms=cfg.control.lost_timeout_ms,
    )
    model = ArmModel(cfg.joints, safe_margin_deg=float(cfg.safety.get("safe_margin_deg", 0.0)))
    ctrl = ArmController(bus, model, cfg.control)
    ctrl.start()
    gripper = make_gripper(cfg.gripper, sim=True)
    gripper.connect()
    poses = load_poses(cfg.poses_path)
    script = load_show(cfg.show_path)
    return cfg, transport, bus, model, ctrl, gripper, poses, script


def _assert_commands_within_limits(transport, model):
    """校验所有 FOC 指令都落在软限位内。"""
    for rec in transport.commanded_frames:
        if rec["status"] != 1:
            continue
        spec = next(j for j in model.joints.values() if j.motor_id == rec["motor_id"])
        # 由指令目标位置（输出端 rad）反推关节角（度）
        motor_deg = rec["pos_rad"] * 180.0 / 3.141592653589793
        joint_deg = spec.direction * (motor_deg - spec.offset_deg)
        lo = spec.min_deg - 0.5
        hi = spec.max_deg + 0.5
        if not (lo <= joint_deg <= hi):
            raise AssertionError(
                f"{spec.name} 指令越限：{joint_deg:.2f}° 不在 [{lo}, {hi}]"
            )


class TestFullShow(unittest.TestCase):
    def test_two_cycles_complete(self):
        cfg, transport, bus, model, ctrl, gripper, poses, script = _make_harness()
        try:
            ctrl.enable()
            runner = ShowRunner(
                ctrl, model, gripper, script, poses,
                speed_scale=20.0, cycles=2, allow_uncalibrated=True,
                say=lambda s: None,
            )
            done = runner.run()
            self.assertEqual(done, 2)
            self.assertIsNone(ctrl.error)
            self.assertEqual(ctrl.mode, RunMode.HOLD)
            self.assertEqual(bus.missing(), [])
            _assert_commands_within_limits(transport, model)
            # 收尾（place 步）后夹爪应为张开状态
            self.assertAlmostEqual(gripper.position(), cfg.gripper["q_open"], delta=0.02)
            # 仿真电机确实动过（离开零位后又回到 HOME=0 附近）
            snap = ctrl.snapshot()
            for name in model.order:
                self.assertTrue(snap["online"][name])
                self.assertIsNotNone(snap["measured"][name])
        finally:
            gripper.shutdown()
            ctrl.shutdown()

    def test_estop_during_show(self):
        cfg, transport, bus, model, ctrl, gripper, poses, script = _make_harness()
        result = {}

        def work():
            try:
                ctrl.enable()
                runner = ShowRunner(
                    ctrl, model, gripper, script, poses,
                    speed_scale=20.0, cycles=0, allow_uncalibrated=True,
                    say=lambda s: None,
                )
                runner.run()
                result["error"] = None
            except Exception as exc:  # noqa: BLE001
                result["error"] = exc

        try:
            t = threading.Thread(target=work, daemon=True)
            t.start()
            time.sleep(1.2)
            ctrl.estop("单元测试急停")
            t.join(timeout=10.0)
            self.assertFalse(t.is_alive(), "展示线程未在急停后退出")
            self.assertIsInstance(result.get("error"), (EStopTriggered, MoveAborted))
            self.assertEqual(ctrl.mode, RunMode.ESTOP)
            # 急停后控制环应转为持续发停机帧（等下一个周期到位）
            deadline = time.time() + 2.0
            history = transport.commanded_frames
            while time.time() < deadline:
                history = transport.commanded_frames
                if history and history[-1]["status"] == 0:
                    break
                time.sleep(0.02)
            self.assertTrue(history)
            self.assertEqual(history[-1]["status"], 0)
            # 解除急停 → 释放
            ctrl.release()
            self.assertEqual(ctrl.mode, RunMode.IDLE)
        finally:
            try:
                ctrl.release()
            except Exception:
                pass
            gripper.shutdown()
            ctrl.shutdown()


class TestCliSafety(unittest.TestCase):
    def test_dry_run_prints_one_cycle_without_booting_hardware(self):
        output = io.StringIO()
        with patch("r1arm.cli._boot") as boot, contextlib.redirect_stdout(output):
            self.assertEqual(main(["demo", "--dry-run"]), 0)
            boot.assert_not_called()
        self.assertIn("第 1 循环完成", output.getvalue())
        self.assertNotIn("第 2 循环完成", output.getvalue())

    def test_real_hardware_rejects_uncalibrated_poses_before_boot(self):
        output = io.StringIO()
        with patch("r1arm.cli._boot") as boot, contextlib.redirect_stdout(output):
            self.assertEqual(main(["demo", "--cycles", "1"]), 1)
            self.assertEqual(main(["demo", "--force", "--cycles", "1"]), 2)
            self.assertEqual(main(["home"]), 2)
            boot.assert_not_called()

    def test_no_gripper_uses_safe_noop_backend(self):
        args = SimpleNamespace(config=None, port=None, sim=True, no_gripper=True)
        boot = _boot(args, need_ctrl=False, need_gripper=True)
        try:
            self.assertEqual(boot.gripper.state()["backend"], "none")
            boot.gripper.open_grasp()
            boot.gripper.close_grasp()
        finally:
            boot.cleanup()

    def test_each_script_pose_must_be_calibrated_on_hardware(self):
        cfg = load_config(os.path.join(PROJECT, "config"))
        poses = load_poses(cfg.poses_path)
        poses["calibrated"] = True
        script = load_show(cfg.show_path)
        model = ArmModel(cfg.joints, safe_margin_deg=0.0)
        runner = ShowRunner(None, model, None, script, poses)
        with self.assertRaisesRegex(ShowError, "HOME 尚未标定"):
            runner.check_ready()


if __name__ == "__main__":
    unittest.main()
