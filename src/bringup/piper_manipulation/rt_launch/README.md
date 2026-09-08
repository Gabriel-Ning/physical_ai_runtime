# Piper RT Host

双臂 Piper + 两侧原生 gripper：`ros2_control`、路由控制器、JTC guard。  
本包**不含**腕部相机 / EM / leader（那些在 workstation）。

Package: `piper_manipulation_rt_launch`  
装配：`urdf/piper_bimanual_manipulation.urdf.xacro`（与 Franka 同：bringup 拥有组合 URDF）

## 结构

`rt_stack` 按 `backend` include 子 launch（与 Franka 同形）：

| `backend` | 子 launch |
|-----------|-----------|
| `real` / `fake` | `controller_bringup.launch.py` |
| `mujoco` | `mujoco_bringup.launch.py` |

## 配置

- `config/controller/controllers.yaml` — 500 Hz JSPC / TSKPC / JTC + `*_gripper_fwd`
- `config/mujoco_plugins.yaml` — MuJoCo CameraPlugin / SHM bridge（与 Franka 同位置）
- `mjcf/robot/` + `mjcf/actuators/` — 本体/执行器 MJCF（与 Franka `rt_launch/mjcf` 同位置）
- CAN：`piper0`（左）/ `piper1`（右），udev 见 [`docs/UDEV_HOST_SETUP.md`](../../../../docs/UDEV_HOST_SETUP.md)

## 启动

### 1. 真机（RT）

先确认 CAN 已起来（udev 插拔后通常自动 up；否则手动）：

```bash
ip link show piper0
ip link show piper1
# 若 DOWN：
sudo bash scripts/reset_rt_piper_can.sh piper0 piper1
```

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  backend:=real \
  left_can_interface:=piper0 \
  right_can_interface:=piper1 \
  use_rviz:=false
```

`rt_stack` 固定 `arms:=both` + 两侧 `piper_gripper`。路由控制器与 `*_gripper_fwd` 均为 **inactive**，等 workstation EM claim 后再切。

### 2. MuJoCo 仿真

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  backend:=mujoco \
  task:=click_bell \
  headless:=false
```

或：`pixi run rt-piper backend:=mujoco task:=click_bell headless:=false`

### 3. Fake / 本机冒烟

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  backend:=fake \
  use_rviz:=false \
  cpu_affinity:=none
```

### 4. 单独子模块

```bash
# 真机 / Fake 控制
ros2 launch piper_manipulation_rt_launch controller_bringup.launch.py \
  arms:=both use_fake_hardware:=true

# MuJoCo（含 image bridge）
ros2 launch piper_manipulation_rt_launch mujoco_bringup.launch.py \
  task:=click_bell headless:=false
```

## 启动后检查

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

期望：`joint_state_broadcaster` active；左右 `*_arm_{jspc,tskpc,jtc}` 与 `*_gripper_fwd` 均为 inactive。

细节：[docs/BRINGUP.md](docs/BRINGUP.md) · 接入门控：[docs/INTEGRATION.md](docs/INTEGRATION.md)。

MuJoCo 相机话题对齐 `apps/profiles/piper_bimanual.yaml`；SHM + `mujoco_image_bridge` 见
`rt_launch/config/mujoco_plugins.yaml`。Workstation 需 `use_sim_time:=true`，并关真机相机驱动。

## Workstation（RT 起来之后）

默认：**Orbbec + 双腕 RealSense 开**，**leader teleop 关**。需要主手时再加 `with_leaders:=true`。

```bash
# Fake / real (wall clock) — cameras on, leaders off
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py

# MuJoCo (must follow /clock; disable real camera drivers)
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py \
  use_sim_time:=true with_orbbec:=false with_realsense:=false
```

Profile：`apps/profiles/piper_bimanual.yaml`。
