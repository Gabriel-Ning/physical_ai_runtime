# marvin_manipulation_controller_bringup

Unified Bringup and runtime deployment package for **Marvin Bimanual Humanoid Arms + Dual Pika Grippers**.

## Configuration Layout (Single Source of Truth)

All hardware drivers and runtime parameters are sourced strictly from `config/`:
- `config/controller/controllers.yaml`: 500Hz RT-Host controllers (JTC, JSPC, TSKPC per arm) and Pika Gripper forward position controllers.
- `config/camera/marvin_cameras.yaml`: Workstation perception camera drivers (Head camera + Dual wrist cameras).
- `config/recording/rmi_marvin_bimanual.yaml`: Topic stream contract for C++ MCAP episode recording.
- `config/model/gripper_tcp.yaml` & `config/model/joint_limits.yaml`: Pika TCP transformations and physical joint limits.

---

## 1. RT Host Real-Time Stack

### A. Mock / Fake Hardware (Local Dev & Testing)
```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  use_rviz:=false \
  cpu_affinity:=none
```

### B. Pre-flight: Clear Hardware Errors (Physical Marvin CCS M6 Controller)
```bash
ros2 run marvin_manipulation_controller_bringup clear_errors --ip 10.19.0.191
```

### C. Physical Hardware (Dual 7-DOF Arms + Dual Pika Grippers)
```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=true \
  left_gripper_serial_port:=/dev/ttyUSB0 \
  right_gripper_serial_port:=/dev/ttyUSB1 \
  robot_ip:=10.19.0.191 \
  use_rviz:=false
```

---

## 2. Workstation Peripherals Stack

### A. Full Workstation Stack (Cameras + MCAP Recorder)
```bash
ros2 launch marvin_manipulation_controller_bringup workstation_stack.launch.py
```

### B. Individual Workstation Modules
```bash
# 1. Perception Cameras only (Head + Dual Wrist):
ros2 launch marvin_manipulation_controller_bringup camera_bringup.launch.py

# 2. C++ MCAP Episode Recorder Backend only:
ros2 launch marvin_manipulation_controller_bringup recorder_bringup.launch.py
```

---

## 3. Visualization

```bash
ros2 launch marvin_manipulation_controller_bringup visualize_marvin_manipulation.launch.py \
  use_joint_state_gui:=true \
  use_rviz:=true
```

---

## Key Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `use_fake_hardware` | `true` | `false` for physical Marvin; `true` for mock hardware. |
| `robot_ip` | `10.19.0.191` | Marvin CCS controller IP address. |
| `load_pika_hardware` | `true` | Whether to load Pika gripper `ros2_control` hardware drivers. |
| `left_gripper_serial_port` | `/dev/ttyUSB0` | Serial device port for Left Pika gripper. |
| `right_gripper_serial_port` | `/dev/ttyUSB1` | Serial device port for Right Pika gripper. |
| `cpu_affinity` | `""` | Comma-separated CPUs for `ros2_control_node`. Empty uses `RT_CM_CPU_AFFINITY`. |
| `use_rviz` | `false` | Launch RViz2 visualization. |

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
