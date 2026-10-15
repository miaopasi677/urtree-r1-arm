# R1-7A 展示机械臂控制程序

宇树 **R1-7A 七轴机械臂 + Dex1-1 夹爪** 的展厅展示控制程序（Python）。

对应《展厅机械臂旋转展示方案_V1.0》的动作路径：

> 夹取手机 → 抬升至 90° → 左旋 90° → 右旋 180° → 回中 → 末端旋转 3 圈 → 放回手机（补充步）→ 复位，自动循环

单循环约 32~35 s，节拍可调；支持一键复位、软件急停、循环次数设置、双工位一致部署。

---

## 1. 硬件与控制链路

| 部件 | 说明 | 控制方式 |
| --- | --- | --- |
| R1-7A 七轴臂 | J1/J2=6010，J3/J4/J5=6014，J6/J7=5010 | RS-485 总线 6 Mbps，20 字节指令帧 / 26 字节状态帧，CRC32-MPEG2（**随臂不带控制软件，本程序即自研的控制软件**） |
| Dex1-1 夹爪 | 单颗 M4010 电机 | 宇树官方 `dex1_1_service`（串口转 DDS），本程序作为 DDS 客户端发指令 |
| 控制主机 | Ubuntu 22.04（臂标称运行系统） | Python 3.10+，仅硬依赖 `pyserial` |

> 说明：R1-7A 官方文档明确「随臂不提供控制软件，客户需依据协议自行开发电机控制程序」，本仓库就是这套程序。官方产品页提到「陆续开源机械臂控制程序」，若后续官方开源了 R1-7A 控制/SDK，可评估替换 `src/r1arm/protocol.py` 与 `bus.py` 这两层（见 docs/DEVELOPMENT.md）。

## 2. 快速开始

```bash
# ① 无硬件仿真：完整跑一轮展示（含夹爪模拟）
python r1ctl.py --sim demo --cycles 1

# ② 无硬件预览动作序列（不驱动任何东西）
python r1ctl.py demo --dry-run

# ③ 跑单元测试（33 个）
python tests/run_tests.py

# ---- 以下为真机命令（先看 docs/HARDWARE_SETUP.md 完成接线）----
python r1ctl.py check            # 连线自检：每个电机应答/温度/位置
python r1ctl.py teach            # 示教标定（必须先做，见 docs/FIELD_CALIBRATION.md）
python r1ctl.py gripper-test     # 夹爪开合测试（标定 q_open/q_close）
python r1ctl.py home             # 使能并回待机位
python r1ctl.py demo             # 正式展示循环（未标定会被拒绝运行）
python r1ctl.py estop            # 应急：向所有电机连发停机帧
```

全局参数：`--config <目录>`（默认项目内 `config/`）、`--port <串口>`、`--sim`、`-v`（调试日志）。

## 3. 命令一览

| 命令 | 作用 | 备注 |
| --- | --- | --- |
| `check` | 总线自检，列出各电机应答/温度/电压/错误码 | 接线后第一步；退出码 0=全部在线 |
| `monitor` | 实时打印关节角/环路频率/在线状态 | Ctrl+C 退出 |
| `home` | 使能并移动到 HOME 位，保持使能 | Ctrl+C 停机退出 |
| `release` | 释放使能（电机停机） | |
| `teach` | 交互式示教：点动、记录位姿、标定零位 | 见 docs/FIELD_CALIBRATION.md |
| `demo` | 运行展示循环 | `--cycles N`（0=无限）、`--speed x`（节拍倍率）、`--force`（仅仿真/预览可用）、`--no-gripper`、`--dry-run`（只预览一轮） |
| `gripper-test` | 夹爪开合 N 次 | 标定夹爪参数用 |
| `estop` | 独立进程连发停机帧 | 程序卡死时使用；物理急停按钮仍是第一优先 |
| `version` | 版本号 | |

展示运行中按 **Ctrl+C = 软件急停**（立即向所有电机发停机帧）。

## 4. 目录结构

```
r1_7a_control/
├─ r1ctl.py                 命令行入口
├─ requirements.txt         依赖（pyserial；可选 unitree_sdk2py）
├─ config/
│  ├─ arm_config.json       主配置：串口/控制环/安全/7关节/夹爪
│  ├─ poses.json            位姿库（现场示教生成；默认为示意占位值）
│  └─ show_script.json      展示动作脚本（步骤/节拍/循环次数）
├─ src/r1arm/
│  ├─ protocol.py           协议层：帧编解码/CRC32-MPEG2/单位换算
│  ├─ transport.py          通道：真实串口 / 仿真总线 MockTransport
│  ├─ bus.py                多电机总线：轮询、状态缓存、掉线看门狗
│  ├─ arm.py                关节模型：角度换算、软限位、位姿工具
│  ├─ trajectory.py         轨迹：最小 jerk 插值、速度约束
│  ├─ controller.py         控制环线程：模式机/安全监控/CSV 日志
│  ├─ gripper.py            夹爪：官方服务 DDS 客户端 / 仿真 / 空实现
│  ├─ show.py               展示脚本执行器（含夹取手机流程）
│  ├─ teach.py              示教与标定工具
│  ├─ safety.py             安全工具：异常类型/信号/温度与错误检查
│  ├─ config.py             配置加载与回写
│  └─ cli.py                子命令实现
├─ tests/                   33 个单元/仿真测试（tests/run_tests.py）
├─ tools/gen_protocol_vectors.py   协议回归向量生成器
├─ logs/                    运行 CSV 日志（arm_config.json 可关）
└─ docs/                    本套文档
```

## 5. 常用修改速查（改哪里）

| 我想改… | 改哪里 | 说明 |
| --- | --- | --- |
| 节拍快慢 | `config/show_script.json` → `speed_scale`（全局）或每步 `duration_s` | 时长 ÷ speed_scale；控制环会按速度上限自动兜底拉长 |
| 循环次数 | `show_script.json` → `cycles`（0=无限）或 `demo --cycles N` | |
| 动作顺序/增删步骤 | `show_script.json` → `steps` | 新动作类型需改 `show.py` 的 `_exec_step`，见 docs/DEVELOPMENT.md |
| 各个位姿（取件位等） | 现场用 `teach` 记录，或手改 `config/poses.json` | 手改后把对应位姿 `calibrated` 置 true（真机才允许跑） |
| 末端旋转形式 | `show_script.json` 第 7 步：`joint`/`amplitude_deg`/`cycles` | 当前为 J7 ±90° 往复 3 次（方案的「视觉等效」实现），评审后可固化 |
| 关节增益 kp/kd | `arm_config.json` → `joints[].kp/kd` | 单位为**输出端** N·m/rad（程序自动换算到转子端），需现场整定 |
| 关节速度上限 | `joints[].max_speed_deg_s` | 标称 >180°/s，展示建议 ≤150 |
| 软限位 | `joints[].min_deg/max_deg` | 默认按官方行程；J4 方向差异见下文「已知事项」 |
| 串口/波特率 | `arm_config.json` → `port/baudrate` | 6 Mbps 需适配器支持 |
| 夹爪开合/夹持力 | `gripper` 节：`q_open/q_close/kp` | 夹持力 ≈ kp×(q_target−q_actual)；先 `gripper-test` 验证 |
| 夹爪后端 | `gripper.backend` | `dex1_service`/`mock`/`none` |
| 安全阈值 | `safety` 节（温度/掉线）+ `joints[].tau_guard_nm`（力矩碰撞，null=关） | |
| CSV 运行日志 | `control.csv_log / csv_dir` | 每步动作会写进 note 列，便于对拍 |

改完代码/配置后：`python tests/run_tests.py` → `python r1ctl.py --sim demo --cycles 1` 验证。

## 6. 安全机制（内置）

- **软限位**：所有目标位姿强制钳位在关节行程内（`safe_margin_deg` 可再加内缩）；
- **速度兜底**：任何一段动作按各关节 `max_speed_deg_s` 自动拉长时长，配置再快也不会超速；
- **电机侧超时保护**：指令帧 timeout 位=1，控制环意外死亡 1 s 内电机自动停机；
- **软件急停**：Ctrl+C 或 `estop` 命令 → 持续发停机帧（status=0）；
- **故障停机**：电机错误码、超温（默认驱动 85 ℃/绕组 110 ℃）、通信丢失（>500 ms）、力矩超限（可选）任一触发即停机并锁定，需排查后重启程序；
- **未标定保护**：`poses.json` 未标定或所引用位姿未逐项标定时拒绝在真机跑 `demo`/`home`（`--force` 仅限仿真/预览；真机上不起作用）。

物理急停按钮、限速运行、软质夹持垫、展台 1 m×1 m 包络等现场措施见《展厅机械臂旋转展示方案》与 docs/HARDWARE_SETUP.md。

## 7. 已知事项 / 待现场确认

1. **J4 行程方向**：R1-7A 开发文档写 `-180°~0°`，官网产品页写 `0°~180°`，两者差一个方向/零位约定。默认按开发文档配置；真机首次点动 J4 时确认，必要时改 `direction` 与 `min/max`。
2. **增益整定**：`kp/kd` 默认值是工程估算起点。官方产品页提示该臂「减速比较低、位置刚度偏低，控制不优化会有较大误差与晃动」——现场从低速开始整定，必要时启用 `tau_guard_nm`。
3. **CRC 首次核对**：协议 CRC 已与标准 CRC-32/MPEG-2 交叉验证（tests/test_protocol.py），但首次连真机时请用 `check` 确认电机会应答（若全部无应答且接线正常，优先怀疑波特率/适配器，其次用 docs/PROTOCOL.md 的回归向量核对）。
4. **夹爪链路**：Dex1-1 走官方 `dex1_1_service`，Python 客户端按其话题与 IDL 编写，需在真机上用 `gripper-test` 联调（含 `dds_network` 网卡名）。
5. **「放回手机」步骤**：方案 8 步表里没有放回动作，但循环演示物理上必须放回再取；已作为补充步骤写入脚本（`show_script.json` 第 8 步），动作评审时确认——若确认「不放下」，删除该步并调整第 2 步逻辑。
6. **直连 M4010**：目前夹爪必须依赖官方服务；若需要单进程直驱，见 docs/DEVELOPMENT.md 的扩展点说明（等宇树 M4010 电机文档）。

## 8. 文档索引

| 文档 | 内容 |
| --- | --- |
| docs/PROTOCOL.md | R1-7A 电机协议逐字段说明、CRC、单位换算、回归向量、改协议层的检查单 |
| docs/HARDWARE_SETUP.md | 接线、Ubuntu 部署、dex1_1_service 安装、双工位、自启动、故障排查、验收清单 |
| docs/FIELD_CALIBRATION.md | 零位/位姿/夹爪/节拍标定流程与验收测试单 |
| docs/DEVELOPMENT.md | 代码架构、模块职责、「改 X 动哪里」详解、扩展指南、测试说明 |
