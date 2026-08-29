# GRIT

[English](README.md)

本仓库是面向GRIT算法的Unitree G1 摇操部署代码。策略推理进程通过统一的 UDP 接口，使用PICO即可实现机器人的全身摇操运动控制跟踪。

本仓库包含：

- GRIT ONNX 策略及其配置文件
- 50 Hz Python 推理与控制环路
- 支持无界面和交互界面的 MuJoCo sim2sim
- 基于 PICO/XRoboToolkit 的实时动作重定向
- 基于 Unitree SDK2 的原生硬件桥接，包含指令超时阻尼保护

训练和数据生成代码不在本部署仓库的范围内。

## 遥操作演示

[![GRIT 遥操作演示](docs/assets/teleop.gif)](docs/assets/teleop.mp4)

[观看遥操作演示视频（MP4）](docs/assets/teleop.mp4)

## TODO List

1. **8 月 29 日：GRIT v0.0.1**：开源首版策略 checkpoint，同时发布
   sim2sim、本地 sim2real 部署和 VR 遥操作的完整链路。
2. **9 月：GRIT v0.0.2**：开源新版策略 checkpoint 和配套技术文档，同时发布
   端侧部署链路、头部/手部/相机的硬件改装方案，以及 VLA 数据采集链路。
3. **10 月：GRIT v1.0.0**：正式发布通用运动基础模型，包括技术报告（论文）、
   功能整合版 checkpoint、训练数据集、训练框架及代码，并提供 VLA 数据采集与
   训练接入教程。

## 仓库结构

```text
.
├── sim2real/
│   ├── checkpoints/                 # GRIT ONNX 模型及配置文件
│   ├── config/g1/
│   │   ├── motions/                 # 示例参考动作
│   │   ├── tracking.yaml            # 本地参考动作模式
│   │   ├── tracking_vr.yaml         # PICO 实时控制模式
│   │   ├── controller.yaml          # 策略频率、增益和关节顺序
│   │   ├── bridge.yaml              # MuJoCo UDP 桥接配置
│   │   └── retarget/teleop.yaml     # PICO 重定向与浏览器可视化配置
│   ├── src/
│   │   ├── deploy.py                # GRIT 推理与控制入口
│   │   ├── sim2sim.py               # MuJoCo 机器人进程
│   │   └── runtime/
│   │       ├── grit_policy.py       # GRIT ONNX 和动作契约
│   │       └── grit_observation.py  # GRIT 观测构造
│   ├── teleop/                      # XR 数据流、G1 重定向和 8080 Web viewer
│   └── tests/test_grit_contract.py
├── g1_sim2real/                     # Unitree SDK2 原生硬件桥接器
├── THIRD_PARTY.md
└── LICENSE
```

## 环境要求

- `x86_64` 或 `aarch64` 架构的 Ubuntu 22.04/24.04
- Python 3.10 和 [uv](https://docs.astral.sh/uv/)
- CMake 3.16+、支持 C++17 的编译器、`make` 和 `flock`
- 运行交互式仿真所需的 MuJoCo 图形环境
- 使用 PICO 时需要 XRoboToolkit、Python SDK，以及与工作站同一局域网的低延迟 Wi-Fi
- 真机部署需要 Unitree G1 和本地电脑通过网线连接

在仓库根目录安装 Python 环境：

```bash
cd sim2real
uv sync
cd ..
```

## 快速开始：MuJoCo

在终端 1 启动仿真器：

```bash
cd sim2real
uv run src/sim2sim.py --robot g1
```

在终端 2 启动 GRIT：

```bash
cd sim2real
uv run src/deploy.py \
  --robot g1 \
  --tracking-config tracking.yaml \
  --policy-path checkpoints/policy.onnx
```

在 GRIT 终端按 `s`，让机器人移动到默认姿态；随后在仿真器终端按 `a`
启用策略控制。按 `x` 停止仿真。

`tracking.yaml` 默认循环播放 `config/g1/motions/walk_turn.npz`。可以通过
以下参数换成其他符合 GRIT 契约的参考动作：

```bash
cd sim2real
uv run src/deploy.py \
  --robot g1 \
  --motion-file /path/to/reference.npz \
  --policy-path checkpoints/policy.onnx
```

如需运行有限时长的无界面冒烟测试，分别启动以下两个进程：

```bash
# 终端 1
cd sim2real
uv run src/sim2sim.py \
  --robot g1 --headless --auto-start --max-control-seconds 10
```

```bash
# 终端 2
cd sim2real
uv run src/deploy.py \
  --robot g1 --auto-start --max-policy-steps 500
```

## PICO 环境配置

未配置过PICO软件的，请先按照该文档配置：[PICO 硬件、XRoboToolkit 安装、标定与网络配置指南](docs/pico_setup_zh.md)

先将 XRoboToolkit PC Service 安装到 `/opt/apps/roboticsservice`，然后把
Python 绑定安装到 GRIT 环境中：

```bash
cd sim2real
bash install_xrobottoolkit_sdk.sh
```

该 Python 绑定由安装脚本在 `uv` 环境之外单独安装。完成安装后，如需再次同步
环境，请使用 `uv sync --inexact`，避免清理该绑定。

本仓库仅支持通过 Wi-Fi 传输 PICO 遥操作数据。启动 XRoboToolkit PC Service：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

查询工作站的 Wi-Fi IPv4 地址，并将其填入头显端 XRoboToolkit 的
`PC Service`。不要填写 `127.0.0.1`、容器地址或 G1 专用网口地址。连接成功后，
头显端状态必须显示 `WORKING`，并启用 `Head`、`Controller`、`Send` 和
`Full body`。

启动动作重定向进程：

```bash
cd sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

动作重定向服务默认同时启动浏览器可视化。在工作站打开
<http://localhost:8080>，可以查看人体姿态和重定向后的 G1。局域网中的其他
设备可打开 `http://<工作站Wi-Fi-IP>:8080`。

实时控制链路使用 TCP 端口 `28701`、`28702` 和 `28703`，浏览器可视化使用
TCP 端口 `8080`。

如果工作站没有对应的 CPU 编号，可以去掉启动命令中的 `taskset -c ...`，不影响
功能，只是不再固定进程使用的 CPU 核心。

## PICO sim2sim

依次启动以下进程：

```bash
# 终端 1：PICO 动作重定向
cd sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

```bash
# 终端 2：MuJoCo
cd sim2real
uv run src/sim2sim.py --robot g1
```

```bash
# 终端 3：GRIT 控制
cd sim2real
taskset -c 4-7 uv run src/deploy.py \
  --robot g1 \
  --tracking-config tracking_vr.yaml \
  --policy-path checkpoints/policy.onnx
```

启用实时控制前，先在终端 3 按 `s`。

| PICO 按键 | 行为 |
| --- | --- |
| 左手 `X` | 激活 GRIT；激活后回到默认站立姿态 |
| 右手 `A` | 开始或继续实时动作 |
| 左手 `Y` | 停止控制器 |

## G1 真机部署

必须先在 MuJoCo 中验证完全相同的模型、配置和 PICO 控制流程。首次使用时
编译原生桥接：

```bash
cd g1_sim2real
bash scripts/build.sh
cd ..
```

固定或吊装机器人，并安排操作人员随时控制急停。将下方的 `enp129s0`
替换为实际连接 G1 的网络接口。

```bash
# 终端 1：PICO 动作重定向
cd sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

```bash
# 终端 2：Unitree 桥接器
cd g1_sim2real
G1_NET=enp129s0 taskset -c 2-3 bash scripts/run_bridge.sh
```

```bash
# 终端 3：GRIT 控制
cd sim2real
taskset -c 4-7 uv run src/deploy.py \
  --robot g1 \
  --tracking-config tracking_vr.yaml \
  --policy-path checkpoints/policy.onnx
```

只有在 GRIT 进程报告已收到有效机器人状态后，才能按 `s`。解除物理支撑
前，必须检查关节顺序、控制增益、默认姿态、网络接口和模型哈希。

原生桥接收到有效 Python 指令前，不会释放 Unitree 运动服务的控制权。
完成控制权交接后，如果指令流丢失或超时，桥接器默认在 `0.2 s` 后切换到
持续阻尼模式。该看门狗不能替代物理急停和经过验证的故障恢复流程。

## GRIT 模型

仓库附带以下文件：

```text
sim2real/checkpoints/policy.onnx
sim2real/checkpoints/policy.json
```

ONNX 计算图必须提供完全一致的接口：

| 名称 | 方向 | 形状 |
| --- | --- | --- |
| `reference_context` | 输入 | `[batch, 9, 70]` |
| `proprio_history` | 输入 | `[batch, 990]` |
| `actions` | 输出 | `[batch, 29]` |

如果输入或输出名称不兼容，运行时会直接拒绝加载模型。模型校验和及替换说明
见 `sim2real/checkpoints/README.md`。

参考动作 NPZ 文件使用 GRIT 动作格式，必须包含 `fps`、`joint_pos`、
`joint_vel`、`body_pos_w`、`body_quat_w`、`body_lin_vel_w` 和
`body_ang_vel_w`。

## 验证

运行 GRIT 部署测试：

```bash
cd sim2real
uv run python tests/test_grit_contract.py -q
```

编译检查原生桥接器：

```bash
cd g1_sim2real
bash scripts/build.sh
```

## 许可证

GRIT 部署代码采用 MIT License。仓库内的第三方依赖、机器人资产和
XRoboToolkit 组件保留各自的许可条款，详见 `THIRD_PARTY.md`。
