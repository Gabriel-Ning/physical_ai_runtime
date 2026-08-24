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
    ├── config/recording|camera|teleop
    ├── launch/*_workstation.launch.py
    └── package.xml
```

Robots:

| Robot | RT package | Workstation package |
|-------|------------|---------------------|
| Marvin | `marvin_manipulation_rt_launch` | `marvin_manipulation_workstation_launch` |
| Franka | `franka_manipulation_rt_launch` | `franka_manipulation_workstation_launch` |
| Piper | `piper_manipulation_rt_launch` | `piper_manipulation_workstation_launch` |

## Piper quick start

RT Host:

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  left_can_interface:=piper0 right_can_interface:=piper1 use_fake_hardware:=false
```

Workstation:

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
