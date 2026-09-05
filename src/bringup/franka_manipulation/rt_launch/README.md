# Franka RT Host

Franka FR3 + 一只 Pika。本包只起 RT 闭环和腕部感知，不启 EM / RMI / 规划器。

细节：[docs/BRINGUP.md](docs/BRINGUP.md) · 仿真相机：[docs/MUJOCO_CAMERA.md](docs/MUJOCO_CAMERA.md)

## 结构

`rt_stack` 按 `backend` include 子 launch：

| `backend` | 子 launch |
|-----------|-----------|
| `real` / `fake` | `controller_bringup.launch.py` |
| `mujoco` | `mujoco_bringup.launch.py` |
| （另）`with_cameras:=true` | `camera_bringup.launch.py` |

真机在 **beta**（`192.168.1.100`）上跑。

## 启动

### 1. 真机（beta）

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  backend:=real \
  robot_ip:=192.168.2.101 \
  with_cameras:=true \
  use_rviz:=false
```

### 2. MuJoCo 仿真

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  backend:=mujoco \
  task:=pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate \
  headless:=false
```

`task` 可省略（用默认 LIBERO 任务），或传绝对 `.xml` 路径。

### 3. Fake / Mock

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  backend:=fake \
  with_cameras:=false \
  use_rviz:=true
```

### 4. 单独子模块

```bash
# 真机 / Fake 控制（不含相机）
ros2 launch franka_manipulation_rt_launch controller_bringup.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101

# MuJoCo（含 image bridge）
ros2 launch franka_manipulation_rt_launch mujoco_bringup.launch.py \
  task:=pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate \
  headless:=false

# 腕部 D405 + 鱼眼
ros2 launch franka_manipulation_rt_launch camera_bringup.launch.py
```

### 5. 状态检查

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

期望：`joint_state_broadcaster` active；`franka_arm_*` / `pika_gripper_*` 为 inactive。

Realtime / FCI：[docs/FRANKA_RT_COMMUNICATION.md](../../../../docs/FRANKA_RT_COMMUNICATION.md)
