# Physical AI Runtime — Embodiment Bringup Stack

This directory contains the physical robot hardware bringup packages, dividing responsibilities strictly into **RT Host Realtime Controllers** and **Workstation Peripherals & Perception Services**.

---

## 1. Directory Structure & Architecture Overview

Each embodiment bringup package (e.g. `piper_manipulation_controller_bringup`) isolates hardware execution into two distinct launch environments:

```text
src/bringup/piper_manipulation_controller_bringup/
├── config/
│   ├── controller/                    # ros2_control Controller configurations (JTC, JSPC, TSKPC)
│   ├── recording/
│   │   └── rmi_piper_bimanual.yaml    # Topic stream contract for C++ MCAP recorder
│   └── teleop/
│       └── piper_leaders.yaml         # Dual-arm Piper Master Leader teleop parameters
├── launch/
│   ├── rt_launch/                     # 👈 [RT Host 实时机专属]
│   │   ├── controller_bringup.launch.py   # ros2_control & hardware interfaces
│   │   └── rt_stack.launch.py             # RT Host aggregate launch
│   │
│   └── workstation_launch/            # 👈 [Workstation 算法/示教工作站专属]
│       ├── orbbec_camera_bringup.launch.py        # Static Orbbec RGBD camera
│       ├── realsense_camera_bringup.launch.py     # Left/Right wrist RealSense D435
│       ├── piper_teleop_leader_bringup.launch.py  # Dual-arm Piper Master Leader teleop
│       ├── recorder_bringup.launch.py             # C++ MCAP Episode Recorder server
│       └── workstation_stack.launch.py            # Workstation aggregate launch
├── test/
│   └── test_bringup_contract.py       # Contract validation tests
└── package.xml
```

---

## 2. Distributed Execution Model

```text
┌────────────────────────────────────────────────────────┐
│  RT Host (机器人本体工控机 / 边缘计算节点)             │
│  • ros2_control + Piper CAN 驱动 (piper0, piper1)      │
│  • 500Hz/1000Hz 实时关节伺服                           │
│  👉 $ ros2 launch ... rt_launch/rt_stack.launch.py     │
└───────────────────────────┬────────────────────────────┘
                            │ ROS 2 DDS 网络总线
                            ▼
┌────────────────────────────────────────────────────────┐
│  Workstation (算法/示教工作站，配备 GPU)               │
│  • 3路相机驱动 (1x Orbbec + 2x RealSense)              │
│  • 2个示教主臂驱动 (can0, can1)                        │
│  • C++ 高吞吐 MCAP 录制服务 (episode_recorder)         │
│  👉 $ ros2 launch ... workstation_stack.launch.py      │
└───────────────────────────┬────────────────────────────┘
                            │ Topic Ingress & Service Contracts
                            ▼
┌────────────────────────────────────────────────────────┐
│  应用层客户端 (Pure Client Apps via RMI SDK)           │
│  • pixi run record   : 数据集录制状态机 + 预热验证     │
│  • pixi run teleop   : 纯 Client 遥控会话接管          │
│  • pixi run replay   : MCAP 动作轨迹平滑 1:1 回放      │
│  • pixi run eval     : VLM / Policy 闭环推理           │
└────────────────────────────────────────────────────────┘
```

---

## 3. Standard Launch Commands

First build the bringup package once in this workspace. Every command below
sources the resulting colcon overlay, so it works from a fresh terminal without
requiring Direnv.

```bash
pixi run colcon build --packages-select piper_manipulation_controller_bringup --symlink-install
```

### 3.1 On RT Host (Physical Realtime Machine)
```bash
# Start dual-arm follower controllers on physical robot via SocketCAN interfaces:
pixi run bash -lc 'source install/setup.bash && ros2 launch piper_manipulation_controller_bringup rt_stack.launch.py left_can_interface:=piper0 right_can_interface:=piper1 use_fake_hardware:=false'
```

### 3.2 On Workstation (Peripherals & Perception Services)

Workstation peripherals can be launched using either of the two standard workflows:

#### 方式 1: 一键聚合启动（3路本地相机 + 2个示教主臂 + MCAP录制服务）
```bash
# 启动静态 Orbbec、左右手腕 D435i、双示教主臂和 MCAP 录制服务。
# 将“新任务名”替换为本次采集的任务名，例如 pick_and_place：
pixi run bash -lc 'source install/setup.bash && ros2 launch \
  piper_manipulation_controller_bringup workstation_stack.launch.py \
  recording_experiment_name:=新任务名 \
  recording_task:=新任务名'
```

`recording_experiment_name` 决定数据目录名，`recording_task` 保存为 episode
元数据中的任务标签。
所有相机均在本机启动，并发布到 `/observation/...`，供 RViz、RMI 和录制服务订阅。

#### 方式 2: 分别独立启动常驻服务（适用于分终端或容器化部署）
```bash
# 1. 静态 Orbbec 顶视/前视相机服务 -> /observation/static_orbbec/...
pixi run bash -lc 'source install/setup.bash && ros2 launch piper_manipulation_controller_bringup orbbec_camera_bringup.launch.py'

# 2. 本机启动左右手腕双 RealSense D435i -> /observation/{left,right}_hand_realsense/...
pixi run bash -lc 'source install/setup.bash && ros2 launch piper_manipulation_controller_bringup realsense_camera_bringup.launch.py'

# 3. 左右双臂 Piper 示教主臂服务 (can0, can1) -> /action_sources/piper_leader_{left,right}/...
pixi run bash -lc 'source install/setup.bash && ros2 launch piper_manipulation_controller_bringup piper_teleop_leader_bringup.launch.py'

# 4. C++ MCAP 数据集录制 Server (自动加载 rmi_piper_bimanual.yaml 契约)
pixi run bash -lc 'source install/setup.bash && ros2 launch piper_manipulation_controller_bringup recorder_bringup.launch.py'
```

---

## 4. 三相机帧率与延迟诊断

先按方式 1 或方式 2 启动三台相机，然后在另一个终端运行 30 秒采样：

```bash
pixi run bash -lc 'source install/setup.bash && python3 "$(ros2 pkg prefix piper_manipulation_controller_bringup)/share/piper_manipulation_controller_bringup/scripts/camera_stream_diagnostics.py" --duration 30'
```

脚本会同时订阅以下三路彩色图像，并在结束时分别输出接收帧数、实际交付帧率，以及消息头时间戳到本机 ROS 回调的平均值、P50、P95 和最大延迟：

- `/observation/static_orbbec/color/image_raw`
- `/observation/left_hand_realsense/color/image_raw`
- `/observation/right_hand_realsense/color/image_raw`

可通过 `--duration 60 --report-period 10` 修改采样时长和中间进度输出周期。任一相机在采样期内没有收到图像时，脚本会显示 `NO DATA` 并以非零状态退出。

这里的“延迟”是从 `Image.header.stamp` 到本机 ROS 订阅回调的时间差，反映驱动发布与 DDS 传输后的消息到达情况；它不是从曝光到主机的完整硬件端到端延迟。默认脚本会同时订阅三路相机；三路相机连接在本机时无需跨主机对时。若把相机驱动放到另一台主机，两个主机必须先完成 chrony 对时，详见 [`docs/CLOCK_SYNC.md`](../../docs/CLOCK_SYNC.md)。

### Orbbec 采集到驱动主机的时间

此命令直接使用 Orbbec SDK 比较同一帧的设备采集时间（global timestamp）与主机收到帧的时间（system timestamp）。它会独占 USB 相机，因此先停止 Orbbec ROS 驱动；测量完成后再重新启动 `orbbec_camera_bringup.launch.py`。

```bash
pixi run bash -lc 'source install/setup.bash && ros2 run piper_manipulation_controller_bringup orbbec_capture_latency --samples 300'
```

输出的 `global -> system` 即为相机采集到驱动主机收到完整帧的时间；如果设备不支持 global timestamp，工具会明确退出而不会给出错误数值。

### D435i 采集到驱动主机的时间

同样先停止 RealSense ROS 驱动。工具以相机曝光中点为起点，以 librealsense 在主机用户态的到达时间为终点，并会临时启用 RealSense 的 global-time 校正。两台 D435i 需分别执行：

```bash
pixi run bash -lc 'source install/setup.bash && ros2 run piper_manipulation_controller_bringup d435i_capture_latency --serial 332522075913 --samples 300'
pixi run bash -lc 'source install/setup.bash && ros2 run piper_manipulation_controller_bringup d435i_capture_latency --serial 332322073584 --samples 300'
```

---

## 5. 相机本地/远程测量记录（2026-08-20）

最终部署决定：三台相机均直连本地工作站。下表保留本次本地与 `delta` 远程主机试验数据，便于后续重新评估。除“SDK 采集到主机”外，所有数值都是 `Image.header.stamp -> ROS 订阅回调`；远程“附加时间”是同一次配置下的 `本地回调均值 - delta 本机回调均值`，包括 DDS 发布、网线、本地 DDS 接收与调度，并非纯网线时延。

| 相机 / 配置 | 本地直连测量 | 相机接 `delta`：delta 本机回调 | 相机接 `delta`：本地回调 | `delta -> 本地` 附加时间 |
| --- | ---: | ---: | ---: | ---: |
| Orbbec Femto Bolt，彩色 1280×720@30 | SDK 采集到主机：65.56 ms；ROS 回调：76.50 ms | 576.50 ms | 588.25 ms | 11.75 ms |
| D435i `332522075913`，彩色 1280×720@30 | — | 39.48 ms | 46.97 ms | 7.49 ms |
| D435i `332522075913`，彩色 640×480@30 | SDK 曝光中点到主机：13.08 ms | 34.93 ms | 37.93 ms | 3.00 ms |
| D435i `332322073584`，彩色 640×480@30 | SDK 曝光中点到主机：25.81 ms | 未测 | 未测 | 未测 |

Orbbec 远程配置同时开启了彩色、深度和 IR，出现了约 576 ms 的驱动侧延迟；这不是网线导致。D435i 的远程 640×480 试验中，跨机附加均值约 3 ms，低于 1280×720 的约 7.5 ms。表中不同分辨率、流配置或测量方法的“总延迟”不能直接横向比较；只有同一行的远程附加时间可用于判断网络代价。
