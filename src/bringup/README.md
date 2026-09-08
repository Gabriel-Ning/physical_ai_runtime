# Physical AI Runtime — Piper RT Bringup

This branch keeps the v4 `src/bringup/` layout, but only ships Piper RT:

```text
src/bringup/piper_manipulation/
└── rt_launch/              # ROS pkg: piper_manipulation_rt_launch
    ├── config/controller|camera|model
    ├── launch/rt_stack.launch.py
    └── package.xml
```

Workstation, Marvin, and Franka bringup live on `dev-v4`, not this branch.

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
