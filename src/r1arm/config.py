# -*- coding: utf-8 -*-
"""配置与数据文件加载/保存（arm_config.json / poses.json / show_script.json）。"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

from .bus import JointSpec
from .controller import ControllerConfig

__all__ = [
    "ConfigError",
    "AppConfig",
    "find_config_dir",
    "load_config",
    "load_poses",
    "save_poses",
    "load_show",
    "save_arm_config",
]

DEFAULT_CONFIG_NAME = "arm_config.json"


class ConfigError(ValueError):
    """配置文件错误。"""


@dataclass
class AppConfig:
    base_dir: str                # 配置目录（.../config）
    project_dir: str             # 项目根目录（config 的上一级）
    raw: dict
    port: str
    baudrate: int
    joints: List[JointSpec]
    control: ControllerConfig
    safety: dict
    gripper: dict
    poses_path: str
    show_path: str
    config_path: str


def find_config_dir(explicit: Optional[str] = None) -> str:
    """定位配置目录：优先命令行参数，其次项目内 config/，最后 CWD/config。"""
    if explicit:
        p = os.path.abspath(explicit)
        if not os.path.isfile(os.path.join(p, DEFAULT_CONFIG_NAME)):
            raise ConfigError(f"{p} 下未找到 {DEFAULT_CONFIG_NAME}")
        return p
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.abspath(os.path.join(here, "..", "..", "config"))
    if os.path.isfile(os.path.join(cand, DEFAULT_CONFIG_NAME)):
        return cand
    cand2 = os.path.abspath(os.path.join("config"))
    if os.path.isfile(os.path.join(cand2, DEFAULT_CONFIG_NAME)):
        return cand2
    raise ConfigError("未找到 arm_config.json，请用 --config <目录> 指定")


def load_config(config_dir: Optional[str] = None) -> AppConfig:
    cdir = find_config_dir(config_dir)
    config_path = os.path.join(cdir, DEFAULT_CONFIG_NAME)
    with open(config_path, encoding="utf-8") as f:
        raw = json.load(f)

    joints = [JointSpec.from_dict(d) for d in raw.get("joints", [])]
    if len(joints) != 7:
        raise ConfigError(f"joints 应为 7 个（当前 {len(joints)} 个），请检查 {config_path}")
    ids = [j.motor_id for j in joints]
    if len(set(ids)) != len(ids):
        raise ConfigError("关节电机 ID 重复")

    c = raw.get("control", {}) or {}
    s = raw.get("safety", {}) or {}
    project_dir = os.path.abspath(os.path.join(cdir, ".."))
    csv_dir = str(c.get("csv_dir", "logs"))
    if not os.path.isabs(csv_dir):
        csv_dir = os.path.join(project_dir, csv_dir)

    control = ControllerConfig(
        rate_hz=float(c.get("rate_hz", 200.0)),
        read_timeout_ms=float(c.get("read_timeout_ms", 3.0)),
        lost_timeout_ms=float(s.get("lost_timeout_ms", 500.0)),
        temp1_warn_c=float(s.get("temp1_warn_c", 70.0)),
        temp1_stop_c=float(s.get("temp1_stop_c", 85.0)),
        temp2_warn_c=float(s.get("temp2_warn_c", 90.0)),
        temp2_stop_c=float(s.get("temp2_stop_c", 110.0)),
        enable_requires_all_online=bool(s.get("enable_requires_all_online", True)),
        csv_log=bool(c.get("csv_log", False)),
        csv_dir=csv_dir,
    )

    poses_path = os.path.join(cdir, str(raw.get("poses_file", "poses.json")))
    show_path = os.path.join(cdir, str(raw.get("show_file", "show_script.json")))

    return AppConfig(
        base_dir=cdir,
        project_dir=project_dir,
        raw=raw,
        port=str(raw.get("port", "/dev/ttyUSB0")),
        baudrate=int(raw.get("baudrate", 6000000)),
        joints=joints,
        control=control,
        safety=s,
        gripper=dict(raw.get("gripper", {}) or {}),
        poses_path=poses_path,
        show_path=show_path,
        config_path=config_path,
    )


# ---------------------------------------------------------------- 位姿库

def load_poses(path: str) -> dict:
    if not os.path.isfile(path):
        raise ConfigError(f"位姿文件不存在：{path}")
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if "poses" not in data:
        raise ConfigError(f"位姿文件缺少 poses 字段：{path}")
    return data


def save_poses(path: str, data: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def load_show(path: str) -> dict:
    if not os.path.isfile(path):
        raise ConfigError(f"动作脚本不存在：{path}")
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if "steps" not in data:
        raise ConfigError(f"动作脚本缺少 steps 字段：{path}")
    return data


# ---------------------------------------------------------------- 主配置回写（标定用）

def save_arm_config(config: AppConfig, updates: Dict[str, object]) -> None:
    """把 updates 合并进 arm_config.json 并回写（用于示教标定零位等）。

    updates 示例：{"joints": [...新的 joints 列表...]}
    """
    raw = dict(config.raw)
    raw.update(updates)
    with open(config.config_path, "w", encoding="utf-8") as f:
        json.dump(raw, f, ensure_ascii=False, indent=2)
        f.write("\n")
