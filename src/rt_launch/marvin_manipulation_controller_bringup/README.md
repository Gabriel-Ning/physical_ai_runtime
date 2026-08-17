# marvin_manipulation_controller_bringup

Pika TCP and finger limits for both arms live in
`config/model/gripper_tcp.yaml` and `config/model/joint_limits.yaml`. Edit those
and relaunch; they feed both URDF and Pika `ros2_control` (`max_width` follows
`joint_limits` upper). Controller params are under `config/controller/`.
After changing either model file, regenerate the workstation cuRobo model
(`curobo_robot_models/config/marvin_manipulation.yml`):

```bash
cd src/motion_planning/motion_planners/curobo_robot_models
bash scripts/generate_curobo_robot_model.sh --model marvin
```

## Visualize (mesh + JSP GUI)

```bash
ros2 launch marvin_manipulation_controller_bringup visualize_marvin_manipulation.launch.py
```

```bash
ros2 launch marvin_manipulation_controller_bringup visualize_marvin_manipulation.launch.py \
  use_joint_state_gui:=true \
  use_rviz:=true
```

## Fake hardware

```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  use_rviz:=false \
  cpu_affinity:=none
```

```bash
ros2 launch marvin_manipulation_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=true \
  left_gripper_serial_port:=/dev/ttyUSB0 \
  right_gripper_serial_port:=/dev/ttyUSB1 \
  use_rviz:=false \
  cpu_affinity:=none
```

## Real hardware (RT host)

```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  use_rviz:=false \
  robot_ip:=10.19.0.191 \
  cpu_affinity:=none
```

```bash
ros2 launch marvin_manipulation_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=false \
  left_gripper_serial_port:=/dev/ttyUSB0 \
  right_gripper_serial_port:=/dev/ttyUSB1 \
  use_rviz:=false \
  robot_ip:=10.19.0.191 \
  cpu_affinity:=none
```

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
