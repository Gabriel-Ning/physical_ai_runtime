# franka_manipulation_controller_bringup

Pika TCP and finger limits for this assembly live in
`config/model/gripper_tcp.yaml` and `config/model/joint_limits.yaml`. Edit
those and relaunch; they feed both URDF and Pika `ros2_control` (`max_width`
follows `joint_limits` upper). Controller params are under `config/controller/`.
After changing either model file, regenerate the workstation
cuRobo model (`curobo_robot_models/config/fr3_manipulation.yml`):

```bash
cd src/motion_planning/motion_planners/curobo_robot_models
bash scripts/generate_curobo_robot_model.sh
```

## Visualize (mesh + JSP GUI)

```bash
ros2 launch franka_manipulation_controller_bringup visualize_fr3_manipulation.launch.py
```

```bash
ros2 launch franka_manipulation_controller_bringup visualize_fr3_manipulation.launch.py \
  adaptor_xyz:="0 0 0" \
  adaptor_rpy:="0 0 0.785398163397" \
  gripper_xyz:="0 0 0.004" \
  gripper_rpy:="0 0 0" \
  use_joint_state_gui:=true \
  use_rviz:=true
```

## Fake hardware

```bash
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  load_pika_hardware:=false \
  use_rviz:=false \
  cpu_affinity:=none
```

```bash
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  load_pika_hardware:=true \
  gripper_serial_port:=/dev/ttyUSB0 \
  use_rviz:=false \
  cpu_affinity:=none
```

## Real hardware (RT host)

```bash
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=false \
  use_rviz:=false \
  robot_ip:=192.168.2.101
```

```bash
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=true \
  gripper_serial_port:=/dev/ttyUSB0 \
  use_rviz:=false \
  robot_ip:=192.168.2.101 \
  cpu_affinity:=none
```

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
