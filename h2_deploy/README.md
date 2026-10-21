# H2 部署包：sim2sim + sim2real

H2 全身运动跟踪策略的部署程序。运行时只需要 Python ≥ 3.10、NumPy、MuJoCo、ONNX Runtime（CPU 即可）。

```
config/h2.json              关节顺序、DDS 电机顺序、默认姿态、PD 增益、动作缩放、编码器偏置
config/deploy.json          真机运行时：UDP 端口、准备时长、阻尼、安全阈值
policy/h2_policy.onnx       策略（输入 reference [1,9,74]、state [1,1050]，输出 action [1,31]）
motions/
  walk_stand_start_stop_12s.npz   站立 ~1.4 s → 直线行走 ~9 s → 停下站立（约 11 m，峰值 ~1.5 m/s）
  stand_hold_10s.npz              原地站立 10 s（真机首测用）
src/h2_deploy/              策略推理、参考轨迹、sim2sim、真机控制程序、MuJoCo 替身机器人
native_bridge/              H2 原生 bridge（unitree_sdk2 hg DDS ⇄ UDP），C++，依赖源码已包含
scripts/                    环境安装、启动脚本、自检、录制、bridge 回环测试
reports/                    自检结果与预览视频
```

## 环境

```bash
./scripts/setup_env.sh                                # 创建 ./.venv（Python ≥ 3.10）并安装依赖
./scripts/python.sh scripts/check_package.py         # 自检，约 1 分钟
```

`setup_env.sh` 优先使用 [uv](https://docs.astral.sh/uv/)（没有合适的 Python 时会自动下载）；
没有 uv 时使用 PATH 中的 python3.10 及以上版本。已有的 `.venv` 若低于 3.10 会被重建。
也可以用 conda：`conda create -n h2 python=3.10 && conda activate h2 && ./scripts/setup_env.sh`。
所有脚本默认使用 `./.venv`；用其他解释器时设置 `H2_PYTHON=/path/to/python`。

真机电脑上还需要编译 bridge（x86_64 / aarch64，需要 cmake 和 C++17 编译器）：

```bash
./scripts/build_bridge.sh
./scripts/python.sh scripts/check_package.py --bridge   # 额外做 DDS 回环映射测试
```

## 1. sim2sim

机器人从参考轨迹第一帧开始：

```bash
./scripts/run_sim2sim.sh                                     # 行走轨迹，带窗口
./scripts/run_sim2sim.sh --motion motions/stand_hold_10s.npz
./scripts/run_sim2sim.sh --headless --report reports/walk.json
./scripts/run_sim2sim.sh --action-delay 2                    # 测试 2 ms 动作延迟
./scripts/python.sh scripts/record_video.py --output reports/walk.mp4
```

窗口按键：空格 暂停/继续，R 重置，G 显示/隐藏参考（绿色），C 碰撞体，F 跟随相机。

## 2. sim2real

由原生 bridge（DDS ⇄ UDP）和策略运行时（`h2_deploy.controller`）组成。运行时每 20 ms 收到一帧状态，
推理一次，发送 31 个关节的 `q_des / Kp / Kd`，PD 在电机端执行。

### 2.1 无硬件：用 MuJoCo 替身机器人跑完整真机流程

替身与原生 bridge 使用相同的 UDP 协议、接管规则和超时阻尼；初始朝向默认 0.7 rad，用于检验航向对齐。

```bash
./scripts/run_standin_robot.sh --viewer      # 终端 1，在这个终端按 s / a / x
./scripts/run_robot_policy.sh                # 终端 2，与真机命令相同
```

### 2.2 真机

```bash
H2_NET=enp3s0 ./scripts/run_bridge.sh                                  # 终端 1，H2_NET 为连接机器人的网卡
./scripts/run_robot_policy.sh --motion motions/stand_hold_10s.npz      # 终端 2，首测：原地站立
./scripts/run_robot_policy.sh                                          # 行走轨迹
```

按键在 **bridge 终端** 输入（遥控器 A / stop 也有效；进入控制必须在 bridge 终端按 s）：

| 阶段 | 行为 | 下一步 |
|---|---|---|
| zero | Kp=Kd=0，机器人无力；第一条此类命令触发 bridge 接管底层 | 机器人挂在龙门架上，按 **s** |
| prepare | 3 s 内从当前姿态插值到默认姿态 | 自动 |
| hold | 保持默认姿态 | 把机器人放到脚着地，按 **A** |
| track | 参考轨迹绕 z 轴旋转到机器人当前 IMU 朝向，50 Hz 跟踪；结束后保持最后的站立帧 | **x** 停止 |
| damping | Kp=0、Kd=8，0.5 s 后退出 | — |

自动进入阻尼：x/stop、骨盆倾斜 >45°、关节超出硬限位 0.05 rad、0.25 s 无状态、状态非有限值、Ctrl+C。
bridge 自身：0.2 s 无命令即持续阻尼；退出时发送阻尼。每次运行记录在 `reports/robot_run_*.npz`。

## 上机前必须逐项确认（仿真无法验证）

1. **DDS 电机顺序**：`config/h2.json` 的 `dds_joint_names`（0–11 腿，脚踝 roll 在前、pitch 在后；12–14 腰；15–28 双臂；29–30 头）。
   bridge 的映射已用回环测试验证，但这份顺序本身要用实机核对：挂架、小 Kp 单独动一个关节，确认关节和方向。
2. **IMU**：策略使用 **骨盆** IMU（四元数 + 角速度）。确认 `rt/lowstate` 的 `imu_state` 是骨盆 IMU。
3. **踝关节模式**：`mode_pr: 0`（PR，直接给 pitch/roll 关节角），确认 H2 并联踝在该模式下由固件换算。
4. **运动服务释放**：bridge 通过 MotionSwitcherClient 释放原厂运控（`release_required: true`，失败即退出）。
5. **增益**：髋/膝 200/4，踝 40/2，腰 150/3，头/肩/肘/腕 roll 40/2，腕 pitch/yaw 20/1（Kp/Kd），确认与驱动单位和限幅一致。
6. **编码器偏置**：`encoder_offset` 目前全 0。观测使用 `q_测量 + offset`，下发 `目标 − offset`；编码器读数比真实角度大 b 时填 `-b`。
7. **头部关节**：DDS 槽位 29/30 为 head_pitch/yaw，确认实机有对应电机；槽位 31–34 恒为 0。
8. **场地**：行走轨迹向前约 11 m，需要 ≥13 m 直线空场、保护绳和急停。
   建议顺序：sim2sim → 替身闭环 → 挂架站立 → 落地站立（stand_hold）→ 行走。

## 已完成的验证（`reports/check_package.json`）

- 策略/场景校验和一致；`h2.json`、策略元数据、bridge 配置三处关节顺序与 UDP 端口一致。
- sim2sim：站立、行走两段均无跟踪失败（连续 15 帧根误差 >0.6 m）。
- 替身闭环：zero → prepare → hold → track（全部参考帧）→ 停止阻尼，航向对齐正确，跟踪中机器人保持直立。
- DDS 回环：`h2_udp_bridge` 在两个方向上 31 个关节映射正确，未用槽位为 0。
- `reports/preview_walk_stand_start_stop_12s.mp4`：行走轨迹 sim2sim 预览。
