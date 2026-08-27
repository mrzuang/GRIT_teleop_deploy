# GRIT

[English](README.md)

GRIT 是面向 29 自由度 Unitree G1 全身动作跟踪策略的部署运行时。同一个
GRIT 推理进程通过统一的 UDP 接口，既可以控制 MuJoCo 仿真机器人，也可以
控制真实机器人。

本仓库包含：

- GRIT ONNX 策略及其部署契约
- 50 Hz Python 推理与控制运行时
- 支持无界面和交互界面的 MuJoCo sim2sim
- 基于 PICO/XRoboToolkit 的实时动作重定向
- 基于 Unitree SDK2 的原生硬件桥接器，包含指令超时阻尼保护

训练和数据生成代码不在本部署仓库的范围内。

## 仓库结构

```text
.
├── sim2real/
│   ├── checkpoints/                 # GRIT ONNX 模型及运行时契约
│   ├── config/g1/
│   │   ├── motions/                 # 示例参考动作
│   │   ├── tracking.yaml            # 本地参考动作模式
│   │   ├── tracking_vr.yaml         # PICO 实时控制模式
│   │   ├── controller.yaml          # 策略频率、增益和关节顺序
│   │   └── bridge.yaml              # MuJoCo UDP 桥接配置
│   ├── src/
│   │   ├── deploy.py                # GRIT 推理与控制入口
│   │   ├── sim2sim.py               # MuJoCo 机器人进程
│   │   └── runtime/
│   │       ├── grit_policy.py       # GRIT ONNX 和动作契约
│   │       └── grit_observation.py  # GRIT 观测构造
│   ├── teleop/                      # XR 数据流和 G1 动作重定向
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
- 使用 PICO 时需要 XRoboToolkit、ADB 和 Python SDK
- 真机部署需要 Unitree G1 和独立的机器人侧网络接口

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

先将 XRoboToolkit PC Service 安装到 `/opt/apps/roboticsservice`，然后把
Python 绑定安装到 GRIT 环境中：

```bash
cd sim2real
bash install_xrobottoolkit_sdk.sh
```

PICO 通过 USB 连接时，运行以下脚本配置 ADB 隧道并重启头显端
XRoboToolkit：

```bash
cd sim2real
bash scripts/setup_xrobotoolkit_usb.sh
```

通过以太网连接时，需要手动启动 XRoboToolkit PC Service，并在头显客户端
中填入主机 IP。

启动动作重定向进程：

```bash
cd sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

实时链路使用 TCP 端口 `28701`、`28702` 和 `28703`。

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
| 左手 `X` | 激活 GRIT；激活后用于暂停实时动作 |
| 右手 `A` | 开始或继续实时动作 |
| 左手 `Y` | 停止控制器 |

## G1 真机部署

必须先在 MuJoCo 中验证完全相同的模型、配置和 PICO 控制流程。首次使用时
编译原生桥接器：

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

原生桥接器收到有效 Python 指令前，不会释放 Unitree 运动服务的控制权。
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

运行 GRIT 部署契约测试：

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
