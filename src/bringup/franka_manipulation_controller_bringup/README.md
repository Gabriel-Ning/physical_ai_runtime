# franka_manipulation_controller_bringup

Workstation bringup package for the Franka FR3 + AgileX Pika Gripper manipulation cell.

Pika TCP and finger limits for this assembly live in `config/model/gripper_tcp.yaml` and `config/model/joint_limits.yaml`. Edit those and relaunch; they feed both URDF and Pika `ros2_control` (`max_width` follows `joint_limits` upper). Controller params are under `config/controller/`. Camera parameters (resolutions, framerates, formats) are under `config/camera/pika_cameras.yaml`.

After changing either model file, regenerate the workstation cuRobo model (`curobo_robot_models/config/fr3_manipulation.yml`):

```bash
cd src/motion_planning/motion_planners/curobo_robot_models
bash scripts/generate_curobo_robot_model.sh
```

---

## 1. Real Hardware Bringup (RT Host)

On the RT host (`192.168.1.100`), launch the real FR3 + Pika RT stack:

### A. Pure Motion Control (Default, Zero Camera Overhead)
```bash
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101 \
  load_pika_hardware:=true \
  gripper_serial_port:=/dev/ttyUSB0
```
> **Note**: Do not set `cpu_affinity:=none` on the real RT host; allow the real-time kernel / RT profile to manage CPU pinning for `ros2_control_node`.

### B. Full Manipulation Cell with Wrist Perception (Arm + Gripper + Cameras)
Launch controllers together with the wrist RealSense D405 and Sunplus Fisheye cameras:
```bash
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101 \
  load_pika_hardware:=true \
  gripper_serial_port:=/dev/ttyUSB0 \
  with_cameras:=true
```

### C. Standalone Camera Perception Stream
If you wish to launch or restart only the camera perception stack without touching the RT controllers:
```bash
ros2 launch franka_manipulation_controller_bringup camera_bringup.launch.py \
  camera_config:=$(ros2 pkg prefix franka_manipulation_controller_bringup)/share/franka_manipulation_controller_bringup/config/camera/pika_cameras.yaml
```

---

## 2. Fake Hardware (Workstation Simulation)

For local simulation / fake hardware without real robots:

```bash
# Fake robot arm + fake gripper
ros2 launch franka_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  load_pika_hardware:=true \
  cpu_affinity:=none
```

---

## 3. Visualization (Mesh & Joint State Publisher GUI)

```bash
ros2 launch franka_manipulation_controller_bringup visualize_fr3_manipulation.launch.py \
  use_joint_state_gui:=true \
  use_rviz:=true
```

---

## 4. Key Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `use_fake_hardware` | `true` | `false` for physical FR3; `true` for mock hardware. |
| `robot_ip` | `192.168.2.101` | Franka Control Interface (FCI) IP address. |
| `load_pika_hardware` | `true` | Whether to load Pika gripper `ros2_control` hardware driver. |
| `gripper_serial_port` | `/dev/ttyUSB0` | Serial device port for Pika gripper (CH340). |
| `with_cameras` | `false` | Launch RealSense D405 + Sunplus Fisheye perception cameras. |
| `camera_config` | `config/camera/pika_cameras.yaml` | YAML parameter file configuring camera streams, resolutions, and framerates. |
| `d405_serial` | `_323622270897` | RealSense D405 serial number override. |
| `fisheye_device` | `/dev/video12` | V4L2 video device node for the Sunplus Fisheye camera. |
| `use_rviz` | `false` | Launch RViz2 visualization. |

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
