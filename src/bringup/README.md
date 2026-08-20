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

### 3.1 On RT Host (Physical Realtime Machine)
```bash
# Start dual-arm follower controllers on physical robot via SocketCAN interfaces:
pixi run ros2 launch piper_manipulation_controller_bringup rt_launch/rt_stack.launch.py left_can_interface:=piper0 right_can_interface:=piper1 use_fake_hardware:=false
```

### 3.2 On Workstation (Peripherals & Perception Services)

Workstation peripherals can be launched using either of the two standard workflows:

#### 方式 1: 一键聚合启动（默认全开：3路相机 + 2个示教主臂 + MCAP录制服务）
```bash
# 启动工作站全部外设与录制服务（with_cameras=true, with_leaders=true, with_recorder=true）:
pixi run ros2 launch piper_manipulation_controller_bringup workstation_launch/workstation_stack.launch.py
```

#### 方式 2: 分别独立启动常驻服务（适用于分终端或容器化部署）
```bash
# 1. 静态 Orbbec 顶视/前视相机服务 -> /observation/static_orbbec/...
pixi run ros2 launch piper_manipulation_controller_bringup workstation_launch/orbbec_camera_bringup.launch.py

# 2. 左右手腕双 RealSense D435 相机服务 -> /observation/{left,right}_hand_realsense/...
pixi run ros2 launch piper_manipulation_controller_bringup workstation_launch/realsense_camera_bringup.launch.py

# 3. 左右双臂 Piper 示教主臂服务 (can0, can1) -> /action_sources/piper_leader_{left,right}/...
pixi run ros2 launch piper_manipulation_controller_bringup workstation_launch/piper_teleop_leader_bringup.launch.py

# 4. C++ MCAP 数据集录制 Server (自动加载 rmi_piper_bimanual.yaml 契约)
pixi run ros2 launch piper_manipulation_controller_bringup workstation_launch/recorder_bringup.launch.py
```
