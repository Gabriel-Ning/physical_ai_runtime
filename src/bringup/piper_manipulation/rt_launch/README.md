# piper_manipulation_rt_launch

RT Host bringup for **dual Piper** manipulation: `ros2_control`, controllers, and JTC guard.

Package path: `src/bringup/piper_manipulation/rt_launch`  
ROS package name: `piper_manipulation_rt_launch`

Workstation assets (Execution Manager, recorder, Orbbec/RealSense cameras, leader teleop)
live in sibling package `piper_manipulation_workstation_launch`.

## RT Host Stack

```bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  left_can_interface:=piper0 \
  right_can_interface:=piper1 \
  use_fake_hardware:=true \
  use_rviz:=false \
  cpu_affinity:=none
```

Controllers only:

```bash
ros2 launch piper_manipulation_rt_launch controller_bringup.launch.py \
  arms:=both use_fake_hardware:=true
```

## Workstation (sibling package)

```bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py
```

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
