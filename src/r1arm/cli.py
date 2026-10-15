# -*- coding: utf-8 -*-
"""r1ctl 命令行入口。

常用命令：
    python r1ctl.py --sim demo --cycles 2        无硬件仿真跑两轮展示
    python r1ctl.py check                        真机连线自检
    python r1ctl.py teach                        示教标定（点动/记录位姿/零位）
    python r1ctl.py demo                         运行展示循环（含夹取手机）
    python r1ctl.py home                         使能并回待机位
    python r1ctl.py estop                        应急：向所有电机连发停机帧
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Dict, Optional

from . import __version__
from .arm import ArmModel
from .bus import MotorBus
from .config import ConfigError, load_config, load_poses, load_show
from .controller import ArmController
from .gripper import GripperError, NoneGripper, make_gripper
from .safety import EStopTriggered, FaultError, MoveAborted, install_signal_handlers
from .show import ShowError, ShowRunner
from .teach import TeachSession
from .transport import MockTransport, SerialTransport, TransportError

log = logging.getLogger("r1ctl")


# ---------------------------------------------------------------- 启动装配

class _Boot:
    """一次命令运行所需的全部运行时对象。"""

    def __init__(self, cfg, transport, bus, model, ctrl, gripper):
        self.cfg = cfg
        self.transport = transport
        self.bus = bus
        self.model = model
        self.ctrl = ctrl
        self.gripper = gripper

    def cleanup(self, release: bool = True) -> None:
        if self.gripper is not None:
            try:
                self.gripper.shutdown()
            except Exception:
                pass
        if self.ctrl is not None:
            try:
                if release:
                    self.ctrl.release()
            except Exception:
                pass
            try:
                self.ctrl.shutdown()
            except Exception:
                pass
        else:
            try:
                self.bus.request_stop(rounds=3)
            except Exception:
                pass
            try:
                self.transport.close()
            except Exception:
                pass


def _build_transport(args, cfg):
    if getattr(args, "sim", False) or getattr(args, "dry_run", False):
        motors = [
            {"motor_id": j.motor_id, "model": j.model, "max_speed_deg_s": j.max_speed_deg_s}
            for j in cfg.joints
        ]
        return MockTransport(motors)
    return SerialTransport(cfg.port, cfg.baudrate)


def _boot(args, *, need_ctrl: bool = True, need_gripper: bool = False) -> _Boot:
    cfg = load_config(args.config)
    if getattr(args, "port", None):
        cfg.port = args.port
    transport = _build_transport(args, cfg)
    bus = MotorBus(
        transport,
        cfg.joints,
        read_timeout_ms=cfg.control.read_timeout_ms,
        lost_timeout_ms=cfg.control.lost_timeout_ms,
    )
    model = ArmModel(cfg.joints, safe_margin_deg=float(cfg.safety.get("safe_margin_deg", 2.0)))
    ctrl = None
    gripper = None
    try:
        if need_ctrl:
            ctrl = ArmController(bus, model, cfg.control)
            ctrl.start()
        if need_gripper:
            if getattr(args, "no_gripper", False):
                gripper = NoneGripper()
            else:
                gripper = make_gripper(
                    cfg.gripper,
                    sim=bool(getattr(args, "sim", False) or getattr(args, "dry_run", False)),
                )
            gripper.connect()
    except Exception:
        if gripper is not None:
            gripper.shutdown()
        if ctrl is not None:
            ctrl.shutdown()
        transport.close()
        raise
    return _Boot(cfg, transport, bus, model, ctrl, gripper)


# ---------------------------------------------------------------- 子命令

def cmd_check(args) -> int:
    boot = _boot(args, need_ctrl=False)
    try:
        print(f"通道：{boot.transport.describe()}")
        print(f"关节配置：{', '.join(f'{j.name}(ID{j.motor_id},{j.model})' for j in boot.cfg.joints)}")
        print("发送停机帧并采集应答（2s）…")
        fb = boot.bus.poll_for(2.0)
        print()
        print(f"{'关节':<5}{'ID':<4}{'型号':<7}{'应答':<6}{'位置':>9}{'驱动温':>8}{'绕组温':>8}{'电压':>7}  错误码")
        missing = []
        for name in boot.model.order:
            f = fb[name]
            spec = boot.model.joints[name]
            if not f.online:
                missing.append(name)
                print(f"{name:<5}{spec.motor_id:<4}{spec.model:<7}{'✗':<6}")
                continue
            deg = boot.model.joint_deg(spec, f)
            err = f"0x{f.state.merror:08X}" if f.state.merror else "-"
            warn = " ⚠超时保护" if f.state.timeout_flag else ""
            print(
                f"{name:<5}{spec.motor_id:<4}{spec.model:<7}{'✓':<6}{deg:>8.1f}°"
                f"{f.state.temp1_c:>7}℃{f.state.temp2_c:>7}℃{f.state.voltage_v:>6.1f}V  {err}{warn}"
            )
        print()
        if missing:
            print(f"⚠ {len(missing)} 个关节无应答：{missing}")
            print("  排查：供电（24–60V）/ 485 接线 A/B / 电机 ID 与配置一致 / 适配器是否支持 6 Mbps")
            return 1
        print("✓ 全部关节应答正常")
        return 0
    finally:
        try:
            boot.bus.request_stop(rounds=3)
        except Exception:
            pass
        boot.transport.close()


def cmd_monitor(args) -> int:
    boot = _boot(args)
    try:
        print("实时监视（Ctrl+C 退出）…")
        while True:
            snap = boot.ctrl.snapshot()
            meas = "  ".join(
                f"{n}:{(snap['measured'][n] if snap['measured'][n] is not None else float('nan')):7.1f}°"
                for n in boot.model.order
            )
            online = "".join("✓" if snap["online"][n] else "✗" for n in boot.model.order)
            print(
                f"[{snap['mode']:>5}] {snap['loop']['hz']:5.0f}Hz 在线{online}  {meas}"
                + (f"  错误:{snap['error']}" if snap["error"] else "")
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        boot.cleanup(release=False)   # 监视不打断使能状态，退出时保持停机帧


def cmd_home(args) -> int:
    cfg = load_config(args.config)
    poses = load_poses(cfg.poses_path)
    name = str(cfg.raw.get("home_pose", "HOME"))
    p = poses.get("poses", {}).get(name)
    if p is None:
        raise ConfigError(f"poses.json 中没有 {name} 位姿")
    if not getattr(args, "sim", False) and (
        not poses.get("calibrated", False) or not p.get("calibrated", False)
    ):
        raise ConfigError(f"{name} 位姿未标定；真机禁止回 HOME，请先 teach 示教标定")
    boot = _boot(args, need_gripper=False)
    try:
        pose = boot.model.pose_from_list(list(p["deg"]))
        boot.ctrl.enable()
        boot.ctrl.move_to(pose, 3.0, name="回待机位")
        print(f"已在待机位 {name} 并保持使能。按 Ctrl+C 停机退出。")
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print("\n停机…")
        return 0
    except (FaultError, MoveAborted, EStopTriggered, ConfigError) as exc:
        print(f"失败：{exc}")
        return 1
    finally:
        boot.cleanup(release=True)


def cmd_release(args) -> int:
    boot = _boot(args, need_gripper=False)
    try:
        boot.ctrl.release()
        time.sleep(0.2)
        print("已释放使能（电机停机）")
        return 0
    except FaultError as exc:
        print(f"失败：{exc}")
        return 1
    finally:
        boot.cleanup(release=False)


def cmd_teach(args) -> int:
    cfg = load_config(args.config)
    poses = load_poses(cfg.poses_path)
    boot = _boot(args, need_gripper=True)
    try:
        session = TeachSession(boot.ctrl, boot.model, boot.gripper, cfg, poses)
        session.run()
        return 0
    except (FaultError, MoveAborted, EStopTriggered) as exc:
        print(f"示教中断：{exc}")
        return 1
    finally:
        boot.cleanup(release=True)


def cmd_demo(args) -> int:
    cfg = load_config(args.config)
    poses = load_poses(cfg.poses_path)
    script = load_show(cfg.show_path)
    sim = bool(getattr(args, "sim", False))
    if args.force and not (sim or args.dry_run):
        raise ConfigError("--force 仅可用于 --sim 或 --dry-run；真机必须先完成位姿标定")
    model = ArmModel(cfg.joints, safe_margin_deg=float(cfg.safety.get("safe_margin_deg", 2.0)))
    if args.dry_run:
        runner = ShowRunner(
            None, model, None, script, poses,
            speed_scale=args.speed, cycles=1, allow_uncalibrated=True, say=print,
        )
        runner.run(dry_run=True)
        return 0
    # 真机先检查位姿与脚本，未通过时连串口都不打开。
    ShowRunner(
        None, model, None, script, poses,
        speed_scale=args.speed, cycles=args.cycles, allow_uncalibrated=sim,
    ).check_ready()
    boot = _boot(args, need_gripper=True)
    ctrl = boot.ctrl

    def _on_signal(_sig):
        print("\n收到中断信号 → 软件急停")
        ctrl.estop("Ctrl+C 急停")

    restore = install_signal_handlers(_on_signal)
    runner = ShowRunner(
        ctrl,
        boot.model,
        boot.gripper,
        script,
        poses,
        speed_scale=args.speed,
        cycles=args.cycles,
        allow_uncalibrated=sim,
        say=print,
    )
    exit_code = 0
    try:
        ctrl.enable()
        runner.run(dry_run=False)
    except (EStopTriggered, MoveAborted) as exc:
        print(f"已中断：{exc}")
        exit_code = 130
    except (ShowError, ConfigError, FaultError, TransportError) as exc:
        log.error("展示运行失败：%s", exc)
        print(f"失败：{exc}")
        exit_code = 1
    finally:
        restore()
        boot.cleanup(release=True)
    return exit_code


def cmd_gripper_test(args) -> int:
    boot = _boot(args, need_ctrl=False, need_gripper=True)
    g = boot.gripper
    try:
        print(f"夹爪 backend={g.backend}，状态：{g.state()}")
        for i in range(max(1, args.cycles)):
            print(f"--- 第 {i + 1}/{args.cycles} 次：张开 ---")
            g.open_grasp()
            print(f"    q={g.position()} tau={g.torque()}")
            time.sleep(0.5)
            print(f"--- 第 {i + 1}/{args.cycles} 次：闭合 ---")
            g.close_grasp()
            print(f"    q={g.position()} tau={g.torque()} 夹到物体={g.is_grasped()}")
            time.sleep(0.5)
        g.open_grasp()
        print("测试完成，夹爪已张开")
        return 0
    except Exception as exc:
        print(f"夹爪测试失败：{exc}")
        return 1
    finally:
        try:
            g.shutdown()
        except Exception:
            pass
        boot.transport.close()


def cmd_estop(args) -> int:
    cfg = load_config(args.config)
    if getattr(args, "port", None):
        cfg.port = args.port
    transport = _build_transport(args, cfg)
    try:
        bus = MotorBus(transport, cfg.joints, read_timeout_ms=cfg.control.read_timeout_ms)
        bus.request_stop(rounds=150, gap_s=0.003)
        print("已向所有电机连发停机帧（status=0）")
        return 0
    finally:
        transport.close()


def cmd_version(_args) -> int:
    print(f"r1arm {__version__}（宇树 R1-7A + Dex1-1 展示控制程序）")
    return 0


# ---------------------------------------------------------------- 参数解析

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="r1ctl",
        description="宇树 R1-7A 七轴机械臂 + Dex1-1 夹爪 展示控制程序",
    )
    p.add_argument("--config", help="配置目录（含 arm_config.json），默认使用项目内 config/")
    p.add_argument("--port", help="覆盖串口，如 /dev/ttyUSB0 或 COM3")
    p.add_argument("--sim", action="store_true", help="仿真模式：不连接真实硬件")
    p.add_argument("-v", "--verbose", action="store_true", help="输出调试日志")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="连接自检：各电机应答/温度/位置")
    sub.add_parser("monitor", help="实时监视关节状态（Ctrl+C 退出）")
    sub.add_parser("home", help="使能并回到 HOME 待机位（保持使能）")
    sub.add_parser("release", help="释放使能（电机停机）")
    sub.add_parser("teach", help="示教模式：点动、记录位姿、标定零位")

    demo = sub.add_parser("demo", help="运行展示动作循环（含夹取手机）")
    demo.add_argument("--cycles", type=int, help="循环次数（默认取脚本配置；0=无限）")
    demo.add_argument("--speed", type=float, help="节拍倍率：>1 加快，<1 放慢（默认取脚本配置）")
    demo.add_argument("--force", action="store_true", help="仅仿真/预览可用；真机必须先完成标定")
    demo.add_argument("--no-gripper", action="store_true", help="不启用夹爪")
    demo.add_argument("--dry-run", action="store_true", help="只打印动作序列（自动转为仿真通道）")

    gt = sub.add_parser("gripper-test", help="夹爪开合测试（标定 q_open/q_close 用）")
    gt.add_argument("--cycles", type=int, default=2, help="开合次数（默认 2）")

    sub.add_parser("estop", help="应急：向所有电机连发停机帧")
    sub.add_parser("version", help="显示版本")
    return p


_COMMANDS: Dict[str, object] = {
    "check": cmd_check,
    "monitor": cmd_monitor,
    "home": cmd_home,
    "release": cmd_release,
    "teach": cmd_teach,
    "demo": cmd_demo,
    "gripper-test": cmd_gripper_test,
    "estop": cmd_estop,
    "version": cmd_version,
}


def main(argv: Optional[list] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    handler = _COMMANDS.get(args.cmd)
    if handler is None:  # pragma: no cover
        parser.error(f"未知命令 {args.cmd}")
    try:
        return int(handler(args))  # type: ignore[operator]
    except ConfigError as exc:
        print(f"配置错误：{exc}")
        return 2
    except (TransportError, GripperError, ShowError) as exc:
        print(f"运行错误：{exc}")
        return 1
    except KeyboardInterrupt:
        print("\n已中断")
        return 130


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
