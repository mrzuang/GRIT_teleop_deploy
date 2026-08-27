# VR 遥操作配置与使用（PICO）

[English](pico_setup.md)

本文介绍使用 PICO VR 进行 GRIT 全身遥操作前所需的一次性硬件和软件配置，
包括 PICO 硬件准备、XRoboToolkit 安装、体感追踪器标定和网络连接。

完成本文步骤后，可返回主文档继续运行：

- [PICO sim2sim](../README_ZH.md#pico-sim2sim)
- [G1 真机部署](../README_ZH.md#g1-真机部署)

## 所需硬件

- PICO 4 或 PICO 4 Pro 头显
- 2 个 PICO 手柄
- 2 个 PICO Motion Tracker，分别绑在左右脚踝
- 高速、低延迟的 Wi-Fi 网络
- 一台运行 Ubuntu 22.04 的 x86_64 笔记本电脑或工作站

> [!IMPORTANT]
> 遥操作的稳定性和延迟很大程度上取决于无线网络质量。建议工作站使用有线
> 网络连接路由器，并让 PICO 连接同一路由器的 5 GHz Wi-Fi。避免使用拥挤的
> 公共网络或信号较弱的热点。

## 步骤一：安装 XRoboToolkit

XRoboToolkit 由 PC Service 和 PICO 应用组成。PC Service 运行在工作站上，
PICO 应用运行在头显中，两者配合传输人体跟踪数据。

### 1. 安装 PC Service

PICO 建立连接前，必须先在工作站上安装并运行 PC Service。

Ubuntu 22.04 x86_64 工作站执行：

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

如需适配其他平台或使用更新版本，请查看
[XRoboToolkit PC Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)。

安装完成后，PC Service 默认位于 `/opt/apps/roboticsservice`。可以从桌面应用
列表启动，也可以在终端运行：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

GRIT 还需要 XRoboToolkit Python 绑定。在仓库根目录执行：

```bash
cd sim2real
bash install_xrobottoolkit_sdk.sh
```

### 2. 安装 PICO 应用

1. 戴上 PICO 头显，完成 PICO 快速设置，并确认头显已连接 Wi-Fi。
2. 打开 PICO 开发者选项，在 USB 连接设置中选择“传输文件”。
3. 在本地电脑下载
   [XRoboToolkit-PICO-1.1.1.apk](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk)。
   其他版本可在
   [XRoboToolkit Unity Client Releases](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)
   页面获取。
4. 将 APK 拷贝到移动存储设备并连接 PICO，或者通过 USB 将 APK 拷贝到头显。
5. 在 PICO 文件管理器中选择 `XRoboToolkit-PICO-1.1.1.apk`，然后点击安装。
6. 安装完成后，在应用库的 `Unknown`（未知来源）分区打开 XRoboToolkit。

如果电脑已安装 ADB，也可以通过 USB 安装：

```bash
adb devices
adb install -g XRoboToolkit-PICO-1.1.1.apk
```

运行 `adb devices` 后，头显中可能会弹出 USB 调试授权提示。授权成功后，设备
状态应显示为 `device`，而不是 `unauthorized`。

## 步骤二：配置 PICO 遥操作环境

### 1. 标定体感追踪器

1. 将两个 PICO Motion Tracker 分别绑在左右脚踝，确认方向和左右位置正确。
2. 长按每个追踪器的按钮约 6 秒，直到指示灯亮起。
3. 在 PICO 中打开“体感追踪器”应用，确认两个追踪器均已正确配对。
4. 按照头显中的图示完成体感追踪器标定。
5. 检查虚拟角色的脚是否接触地面。如果悬空或陷入地面，点击画面左上角的
   “校准地面”，直到角色双脚正常落在地面上。

标定时保持自然直立，双脚与肩同宽，并确保头显、手柄和脚踝追踪器都有足够
电量。更换追踪器位置、使用者或场地后，应重新标定。

### 2. 将 PICO 连接到工作站

1. 打开工作站和 PICO 的 Wi-Fi 设置，确认二者连接到同一个局域网。
2. 查询并记下工作站在该网络中的 Wi-Fi IPv4 地址。Ubuntu 可运行：

   ```bash
   hostname -I
   ```

   如果显示多个地址，请选择与 PICO 处于同一网段的局域网地址，不要使用
   `127.0.0.1`、容器地址或机器人专用网口地址。

3. 启动工作站上的 XRoboToolkit PC Service。
4. 在 PICO 中打开 XRoboToolkit。在 `PC Service:` 旁点击 `Enter`，输入工作站
   的 IPv4 地址。
5. 连接成功后，`Status:` 旁应显示 `WORKING`。如果应用中已保存 IP 地址，
   请在 `Network` 区域的 `Status:` 一栏点击 `Reconnect`。

### 3. 检查数据发送选项

在 XRoboToolkit PICO 应用中确认以下设置：

| 区域 | 设置 |
| --- | --- |
| `Tracking` | 勾选 `Head` 和 `Controller` |
| `Data/Control` | 选择 `Send` |
| `Pico Motion Tracker` | 选择 `Full body` |

配置完成后，在工作站启动 GRIT 动作重定向进程：

```bash
cd sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

实时链路使用 TCP 端口 `28701`、`28702` 和 `28703`。启动后确认终端没有连接
错误，再继续运行 MuJoCo 或真机控制流程。

## 连接检查

继续部署前，逐项确认：

- 头显、两个手柄和两个脚踝追踪器均已连接且电量充足
- PICO 中虚拟角色的双脚正确落在地面上
- PICO 与工作站位于同一局域网
- XRoboToolkit 显示 `WORKING`
- `Head`、`Controller`、`Send` 和 `Full body` 已按要求设置
- PC Service 与 GRIT 动作重定向进程均在运行
- 防火墙没有阻止 TCP 端口 `28701`、`28702` 和 `28703`

## 常见问题

### XRoboToolkit 一直无法显示 `WORKING`

- 确认 PC Service 正在工作站上运行。
- 确认输入的是工作站在 PICO 所连接局域网中的 IPv4 地址。
- 确认 PICO 与工作站可以互相访问，且没有启用客户端隔离。
- IP 地址变化后，在 PICO 应用中更新地址并点击 `Reconnect`。

### 无法获取完整身体姿态

- 确认两个脚踝追踪器均已开机、配对并完成标定。
- 确认 `Pico Motion Tracker` 设置为 `Full body`。
- 重新执行地面校准，并确认左右追踪器没有绑反。

### ADB 显示 `unauthorized`

重新插拔 USB 线，在头显中接受 USB 调试授权，然后再次运行 `adb devices`。

### 网络延迟或动作抖动明显

优先使用 5 GHz Wi-Fi，缩短 PICO 与接入点之间的距离，并减少同一无线网络中
的视频流和大文件传输。真机部署时，工作站连接 G1 的专用网口与 PICO 使用的
局域网接口应分别配置，避免路由冲突。
