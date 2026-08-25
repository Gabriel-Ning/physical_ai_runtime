# Piper RT Host

双臂 Piper + 两侧原生 gripper：`ros2_control`、路由控制器、JTC guard。  
本包**不含**腕部相机 / EM / leader（那些在 workstation）。

Package: `piper_manipulation_rt_launch`  
装配：`piper_description/urdf/piper_bimanual_manipulation.urdf.xacro`

## 配置

- `config/controller/controllers.yaml` — 500 Hz JSPC / TSKPC / JTC + `*_gripper_fwd`
- CAN：`piper0`（左）/ `piper1`（右），udev 见 [`docs/UDEV_HOST_SETUP.md`](../../../../docs/UDEV_HOST_SETUP.md)

## 真机启动（RT）

先确认 CAN 已起来（udev 插拔后通常自动 up；否则手动）：

```bash
ip link show piper0
ip link show piper1
# 若 DOWN：
sudo bash scripts/reset_rt_piper_can.sh piper0 piper1
```

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  left_can_interface:=piper0 \
  right_can_interface:=piper1 \
  use_rviz:=false
```

`rt_stack` 固定 `arms:=both` + 两侧 `piper_gripper`。路由控制器与 `*_gripper_fwd` 均为 **inactive**，等 workstation EM claim 后再切。

## Fake / 本机冒烟

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=true \
  use_rviz:=false \
  cpu_affinity:=none
```

只起控制器：

```bash
ros2 launch piper_manipulation_rt_launch controller_bringup.launch.py \
  arms:=both use_fake_hardware:=true
```

## 启动后检查

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

期望：`joint_state_broadcaster` active；左右 `*_arm_{jspc,tskpc,jtc}` 与 `*_gripper_fwd` 均为 inactive。

细节：[docs/BRINGUP.md](docs/BRINGUP.md)。

## Workstation（RT 起来之后）

```bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py
```

Profile：`apps/profiles/piper_bimanual.yaml`。
