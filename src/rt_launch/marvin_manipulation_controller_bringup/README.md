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

## Pre-flight: Clear Hardware Errors (Real Hardware)

On real hardware, Marvin CCS M6 controller may require clearing servo / bus error state before launching the RT stack (otherwise controller launch may fail):

```bash
# 1. Clear errors with default IP (10.19.0.191):
ros2 run marvin_manipulation_controller_bringup clear_errors

# 2. Or specify custom controller IP:
ros2 run marvin_manipulation_controller_bringup clear_errors --ip 10.19.0.191
```

## Real hardware (RT host)

### A. Arm Only (No Pika Grippers Attached)
```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=false \
  robot_ip:=10.19.0.191 \
  use_rviz:=false \
  cpu_affinity:=none
```

### B. Full Manipulation Cell (Bimanual Arms + Dual Pika Grippers)
```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=true \
  left_gripper_serial_port:=/dev/ttyUSB0 \
  right_gripper_serial_port:=/dev/ttyUSB1 \
  robot_ip:=10.19.0.191 \
  use_rviz:=false \
  cpu_affinity:=none
```

---

## Key Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `use_fake_hardware` | `true` | `false` for physical Marvin; `true` for mock hardware. |
| `robot_ip` | `10.19.0.191` | Marvin CCS controller IP address. |
| `load_pika_hardware` | `true` | Whether to load Pika gripper `ros2_control` hardware drivers. Set `false` when grippers are not attached. |
| `left_gripper_serial_port` | `/dev/ttyUSB0` | Serial device port for Left Pika gripper. |
| `right_gripper_serial_port` | `/dev/ttyUSB1` | Serial device port for Right Pika gripper. |
| `cpu_affinity` | `none` | CPU pinning affinity (e.g. `12,13` or `none`). |
| `use_rviz` | `false` | Launch RViz2 visualization on local host. |

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
