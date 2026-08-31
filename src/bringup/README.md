# Physical AI Runtime — Embodiment Bringup Stack

This directory contains embodiment bringup packages, split into **RT Host** (realtime
controllers + on-robot sensors) and **Workstation** (Execution Manager, recorder,
workstation cameras, teleop leaders).

## Directory layout

```text
src/bringup/<robot>_manipulation/
├── rt_launch/              # ROS pkg: <robot>_manipulation_rt_launch
│   ├── config/controller|camera|model
│   ├── launch/rt_stack.launch.py
│   └── package.xml
└── workstation_launch/     # ROS pkg: <robot>_manipulation_workstation_launch
    ├── config/camera|teleop
    ├── launch/*_workstation.launch.py
    └── package.xml
```

Robots:

| Robot | RT package | Workstation package |
|-------|------------|---------------------|
| Marvin | `marvin_manipulation_rt_launch` | `marvin_manipulation_workstation_launch` |
| Franka | `franka_manipulation_rt_launch` | `franka_manipulation_workstation_launch` |
| Piper | `piper_manipulation_rt_launch` | `piper_manipulation_workstation_launch` |

## Piper RT host

双臂 Piper + 两侧原生 gripper。RT **不含**腕部相机 / EM / leader。  
细节：`piper_manipulation/rt_launch/README.md`。CAN 别名：[`docs/UDEV_HOST_SETUP.md`](../../docs/UDEV_HOST_SETUP.md)。

真机前先确认 CAN：

```bash
ip link show piper0
ip link show piper1
# 若 DOWN：
sudo bash scripts/reset_rt_piper_can.sh piper0 piper1
```

RT Host：

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  left_can_interface:=piper0 \
  right_can_interface:=piper1 \
  use_rviz:=false
```

启动后检查：

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

期望：`joint_state_broadcaster` active；左右 `*_arm_{jspc,tskpc,jtc}` 与 `*_gripper_fwd` 均为 inactive。

Workstation（RT 起来之后；profile `apps/profiles/piper_bimanual.yaml`）：

```bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py
```

Cameras only:

```bash
ros2 launch piper_manipulation_workstation_launch piper_orbbec.launch.py
ros2 launch piper_manipulation_workstation_launch piper_realsense.launch.py
```

Leaders only:

```bash
ros2 launch piper_manipulation_workstation_launch piper_leaders.launch.py
```

## Marvin workstation

```bash
ros2 launch marvin_manipulation_workstation_launch workstation_stack.launch.py
```

默认机位：Marvin + 双 Pika（各带鱼眼 + D405，RT）+ 工作站两路 D435。
腕部相机在 RT `rt_stack.launch.py` / `pika_camera_bringup.launch.py`。
Workstation 常驻 Execution Manager 和 recorder，并默认起 head D435 + 第三人称 D435。

## Franka RT host (beta)

RT 用 Pixi `cpu` 环境。夹爪 / 鱼眼走 `/dev/pika_left_gripper`、`/dev/pika_left_fisheye`，不要 `/dev/ttyUSB*`。

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101 \
  use_rviz:=false
```

FakeHardware + gamepad + 无相机（当前验证 Gate）：

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=true load_pika_hardware:=true \
  with_cameras:=false use_rviz:=true cpu_affinity:=none

ros2 launch franka_manipulation_workstation_launch workstation_stack.launch.py

python examples/16_franka_gamepad_teleop.py \
  --profile fr3_pika_single_arm.yaml
```

细节：`franka_manipulation/rt_launch/README.md`。DDS 用本机 Cyclone 文件，不要直接用 Marvin 的 `cyclonedds_default.xml`。

## Camera validation (Piper)

Profile topics (`apps/profiles/piper_bimanual.yaml`):

- `/observation/static_orbbec/color/image_raw`
- `/observation/left_hand_realsense/color/image_raw`
- `/observation/right_hand_realsense/color/image_raw`

```bash
ros2 topic hz /observation/static_orbbec/color/image_raw
ros2 topic hz /observation/left_hand_realsense/color/image_raw
ros2 topic hz /observation/right_hand_realsense/color/image_raw
```

Record with cameras and inspect `episode_health.json` for zero `recorder_drops`.
