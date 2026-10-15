# -*- coding: utf-8 -*-
"""R1-7A 展示机械臂控制程序（宇树 R1-7A 七轴臂 + Dex1-1 夹爪）。

模块结构：
    protocol    电机通讯协议（帧编解码 / CRC32-MPEG2 / 单位换算）
    transport   数据通道（真实 RS-485 串口 / 仿真总线）
    bus         多电机总线（轮询、状态缓存、看门狗）
    arm         关节模型（限位、增益、坐标换算）
    trajectory  轨迹插值（最小 jerk、速度约束）
    controller  控制环（后台线程、模式管理、安全监控）
    gripper     夹爪驱动（官方 dex1_1_service DDS 客户端 / 仿真）
    show        展示动作脚本执行器（含夹取手机流程）
    teach       示教与标定工具
    safety      安全工具（急停、信号处理、温度/错误检查）
    cli         命令行入口（r1ctl.py）
"""

__version__ = "0.1.0"
