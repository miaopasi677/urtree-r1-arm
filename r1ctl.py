#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1-7A 展示机械臂控制程序 · 命令行入口。

用法示例：
    python r1ctl.py --sim demo --cycles 2      # 无硬件仿真跑两轮
    python r1ctl.py check                      # 真机连线自检
    python r1ctl.py teach                      # 示教标定
    python r1ctl.py demo                       # 展示循环（含夹取手机）
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from r1arm.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
