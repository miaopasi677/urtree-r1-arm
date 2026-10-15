# 硬件接线与部署指南

每个展示工位的交付形态：R1-7A 机械臂本体 + Dex1-1 夹爪 + 控制主机 + 安装固定件与线缆。本文覆盖从接线到「`demo` 能跑」的完整部署流程。

## 1. 硬件清单（单工位）

| 项 | 说明 |
| --- | --- |
| R1-7A 机械臂 | 7 轴，J1~J7 电机已在臂内 RS-485 总线（ID 0~6 出厂即配） |
| Dex1-1 夹爪 | 单颗 M4010 电机，自带串口板（接主机 USB） |
| 控制主机 | Ubuntu 22.04，USB 口 ≥2 个；臂标称「运行系统 Ubuntu」 |
| USB-RS485 适配器 | **必须支持 6 Mbps**（推荐宇树配套 USB 转 RS-485 模块；普通 CH340/FTDI 基础款多数只到 2~3 Mbps） |
| 供电 | 臂：24–60 V（XT30）；夹爪按其串口板要求供电 |
| 安装固定 | 底板通孔 M8 螺钉固定支架，或 ≥2 个 C 型夹固定桌角 |
| 急停 | 建议外接物理急停（断供电回路），本程序的 `estop` 只是软件级 |

## 2. 接线

1. 臂的 RS-485（A/B）→ USB-RS485 适配器 → 主机 USB（对应 `arm_config.json → port`，Linux 下通常是 `/dev/ttyUSB0`）；
2. 夹爪串口板 → 主机另一个 USB 口（官方 `dex1_1_service` 会自动扫描 `/dev/ttyUSB*`）；
3. 分别给臂、夹爪串口板上电；
4. 桌面/支架安装按方案：预留 1 m × 1 m 运动包络，观众动线间隔 ≥0.5 m，夹爪装软质夹持垫。

> Linux 串口权限：`sudo usermod -aG dialout $USER` 后重新登录，否则打开串口报 Permission denied。

## 3. 软件安装（控制主机）

```bash
# Python 3.10+（Ubuntu 22.04 自带）
sudo apt update && sudo apt install -y python3-pip
cd r1_7a_control
pip install -r requirements.txt          # 即 pyserial

# 自检（此时应能看到 7 个电机应答；无应答看第 7 节排查）
python3 r1ctl.py check
```

## 4. Dex1-1 夹爪服务（官方 dex1_1_service）

夹爪不由本程序直接驱动串口，而是走宇树官方服务（串口→DDS 桥）：

```bash
# 依赖
sudo apt install -y libserialport-dev libspdlog-dev libboost-all-dev libyaml-cpp-dev libfmt-dev
# 离线 deb 备选：官方仓库 lib/ 目录（arm64/amd64 都有）

# 编译（需要先编译安装 unitree_sdk2）
git clone https://github.com/unitreerobotics/unitree_sdk2
cd unitree_sdk2 && mkdir build && cd build && cmake .. && sudo make install
cd ~ && git clone https://github.com/unitreerobotics/dex1_1_service
cd dex1_1_service && mkdir build && cd build && cmake .. && make -j6

# 标定（手动把夹爪捏到完全闭合，按提示按 s+回车）
sudo ./dex1_1_gripper_server -c

# 启动服务（-n 指定 DDS 网卡，ifconfig 查 192.168.123.* 网段对应的网卡名）
sudo ./dex1_1_gripper_server --network eth0

# 开机自启（官方脚本）
bash ~/dex1_1_service/setup_autostart.sh
```

然后在 `r1_7a_control/config/arm_config.json` 的 `gripper` 节核对：

```json
"backend": "dex1_service",
"side": "right",            // 官方服务约定：电机 ID 0=右爪，1=左爪
"dds_network": "eth0",      // 与服务启动参数一致
```

Python 侧需要 `unitree_sdk2py`：

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python
pip install -e ./unitree_sdk2_python
```

验证：`python3 r1ctl.py gripper-test --cycles 2`，应看到开合动作与 q 值反馈。

## 5. 标定与试运行

接线、自检通过后，**必须先标定再正式跑**（程序会拒绝未标定的真机 demo）：

1. 零位标定 → 2. 七个位姿示教 → 3. 夹爪参数 → 4. 低速试运行。
   详细步骤见 **docs/FIELD_CALIBRATION.md**。

```bash
python3 r1ctl.py teach                      # 示教
python3 r1ctl.py demo --cycles 1 --speed 0.5   # 半速试运行一轮
python3 r1ctl.py demo                       # 正式运行（默认取 show_script.json 的 cycles）
```

## 6. 双工位部署（两台 R1-7A）

两工位各一套「主机 + 臂 + 夹爪」，本程序**每个工位跑一份**（同一套代码）：

1. 两台主机安装步骤相同；`arm_config.json` 按各自主机改 `port`（可能与 `/dev/ttyUSB1` 等）与 `gripper.side`；
2. **位姿一致性**：以 1 号工位为基准完成标定后，把它的 `config/poses.json` 和 `arm_config.json` 里 7 个 `offset_deg` 拷到 2 号工位；若机械安装角度有差，只微调 2 号的 `offset_deg`（J1 的 offset 加装角差），位姿其余数值不动；
3. 一致性校准：两工位同时 `demo --cycles 3`，对比 `logs/arm_*.csv` 的 J1~J7 曲线，节拍差应 <0.2 s；
4. `show_script.json` 两边必须同版本（建议纳入 git 或用同一份分发）。

## 7. 故障排查

| 现象 | 可能原因 / 处理 |
| --- | --- |
| `check` 全部无应答 | 供电未上 / A-B 接反 / 适配器不支持 6 Mbps / 串口不对（`ls /dev/ttyUSB*`）/ 波特率不一致 |
| 单关节无应答 | 该电机 ID 与配置不符（官方上电默认或用官方工具改 ID）/ 该节点线缆松动 |
| 应答时断时续、环路 Hz 低 | 485 总线终端电阻/干扰；`read_timeout_ms` 提到 4~5 ms；换屏蔽线 |
| CRC 错误持续增长（日志） | 波特率不匹配或线缆干扰；核对 docs/PROTOCOL.md 回归向量 |
| 电机报错误码 | 状态帧 MError 十六进制查官方文档表 1；排除断电重启 |
| 「触发超时保护」 | 程序曾 >1 s 未发指令（卡顿/被挂起）；恢复后会自动清除，频繁出现查主机负载 |
| 温度报警 | 停机降温；检查关节是否长时间堵转/夹爪夹持力过大 |
| 夹爪无反馈 | `dex1_1_gripper_server` 未启动 / `dds_network` 网卡名不对 / 串口板未上电 |
| J4 方向与预期相反 | 改 `arm_config.json` 该关节 `direction: -1` 并核对 `min_deg/max_deg`（见 README 已知事项 1） |

## 8. 开机自启（展示值守）

官方夹爪服务用 `setup_autostart.sh`。本程序建议用 systemd：

```ini
# /etc/systemd/system/r1show.service
[Unit]
Description=R1-7A Show Demo
After=network.target

[Service]
User=show
WorkingDirectory=/opt/r1_7a_control
ExecStart=/usr/bin/python3 r1ctl.py demo --cycles 0
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now r1show
sudo journalctl -u r1show -f        # 看日志
sudo systemctl stop r1show          # 停止（会触发优雅停机+电机停机帧）
```

> 注意：`demo` 进程退出/被杀后，电机靠 1 s 超时保护自动停机（安全默认），但**不会自动重新使能**；重启服务即可恢复。

## 9. 验收清单（单工位）

- [ ] `check` 全绿，7 关节温度/电压正常
- [ ] `teach` 完成零位与 7 个位姿，`poses.json` 全部 ✓
- [ ] `gripper-test` 开合顺畅，夹持手机不划伤、不滑落
- [ ] `demo --cycles 1 --speed 0.5` 半速通过，无越限告警
- [ ] `demo --cycles 0` 连续运行 ≥2 h：无故障停机、温升正常、节拍稳定（对比 CSV）
- [ ] Ctrl+C 急停演练：立即停机、手机不脱落
- [ ] 物理急停按钮演练有效
- [ ] 断电重启后全流程可重复
- [ ] 双工位动作一致（对比 CSV，节拍差 <0.2 s）
