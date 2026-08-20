# marvin_manipulation_controller_bringup

Unified Bringup and runtime deployment package for **Marvin Bimanual Humanoid Arms + Dual Pika Grippers**.

## Configuration Layout (Single Source of Truth)

All hardware drivers and launch defaults are sourced strictly from `config/`:
- `config/controller/controllers.yaml`: 500Hz RT-Host controllers (JTC, JSPC, TSKPC per arm) and Pika Gripper forward position controllers.
- `config/camera/marvin_cameras.yaml`: Dual Pika wrist cameras (D405 + DECXIN fisheye ROS parameters).
- `config/recording/rmi_marvin_bimanual.yaml`: Topic stream contract for C++ MCAP episode recording.
- `config/model/gripper_tcp.yaml` & `config/model/joint_limits.yaml`: Pika TCP transformations and physical joint limits.

> **Defaults**: Camera ROS params live in `config/camera/`; gamma site identity for `robot_ip` / gripper tty / fisheye udev ports lives in launch + xacro + `scripts/udev`. CLI args are overrides only (e.g. alternate controller IP after rewiring).

---

## 1. RT Host Real-Time Stack (`launch/rt_launch/`)

### Hardware & Cabling Topology
On the physical Marvin robot (gamma RT host site defaults already baked into launch / xacro):
- **Realtime Controllers**: Marvin CCS M6 dual 7-DOF arm controller (`robot_ip` default `10.19.0.191`) + dual Pika Grippers (left `/dev/ttyUSB1`, right `/dev/ttyUSB0`).
- **Wrist Perception**: RealSense D405 (RGB-D) + Sunplus DECXIN Fisheye (`/dev/fisheye0`, `/dev/fisheye1`) per arm.
- **Single Cable Harness**: Each wrist's Pika Gripper USB serial, RealSense D405, and Fisheye share one combined cable to the **RT Host**, so left/right gripper tty and camera identity stay locked together — do not remap one without the other.

Host prep for dual fisheye symlinks (`/dev/fisheye0` left, `/dev/fisheye1` right):
```bash
sudo bash scripts/udev/install.sh
```
The udev rule is site-specific to the gamma RT host (`3-9.1` left, `3-7.1` right) — see `docs/UDEV_HOST_SETUP.md`.

### A. Pre-flight: Clear Hardware Errors (Physical Marvin CCS M6 Controller)
Before launching controllers on real hardware, clear servo/driver faults:
```bash
ros2 run marvin_manipulation_controller_bringup clear_errors --ip 10.19.0.191
```

### B. Aggregate Launch: `rt_stack.launch.py` (Default Full Bringup)
On the physical RT Host, **`rt_stack.launch.py`** is the aggregate launcher. By default on real hardware, both the **Pika Grippers** (`load_pika_hardware:=true`) and **Pika Wrist Cameras** (`with_cameras:=true`) are started together. Gripper serial ports and `robot_ip` use the gamma defaults above — no need to pass them on the CLI.

```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  use_rviz:=false
```

### C. Sub-launch 1: Realtime Controllers (`controller_bringup.launch.py`)
To launch or restart only the arm controllers and Pika grippers without cameras:
```bash
ros2 launch marvin_manipulation_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=true \
  use_rviz:=false
```

Field workaround when one gripper is down (example: right off, left on `/dev/ttyUSB0`):
```bash
ros2 launch marvin_manipulation_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=false \
  load_pika_hardware:=true \
  load_left_pika_hardware:=true \
  load_right_pika_hardware:=false \
  left_gripper_serial_port:=/dev/ttyUSB0 \
  use_rviz:=false
```

### D. Sub-launch 2: Pika Wrist Cameras (`pika_camera_bringup.launch.py`)
To launch or restart only the wrist perception cameras (defaults from `config/camera/marvin_cameras.yaml`):
```bash
ros2 launch marvin_manipulation_controller_bringup pika_camera_bringup.launch.py
```

Expected image topics on the network:

| Camera | Topic |
| :--- | :--- |
| Left D405 color | `/left_pika_d405/camera/color/image_raw` |
| Left D405 depth | `/left_pika_d405/camera/depth/image_rect_raw` |
| Left fisheye | `/left_pika_fisheye/image_raw` |
| Right D405 color | `/right_pika_d405/camera/color/image_raw` |
| Right D405 depth | `/right_pika_d405/camera/depth/image_rect_raw` |
| Right fisheye | `/right_pika_fisheye/image_raw` |

---

## 2. Workstation Stack (`launch/workstation_launch/`)

The Workstation hosts simulation, visualization, high-level applications (cuRobo planner, ExecutionManager, Teleop, Policy, RMI SDK), and the data recording backend.

### A. Mock / Fake Hardware (Local Dev & Simulation)
> **Note**: Fake hardware is launched on the **developer's Workstation**, NOT on the RT Host. When running fake hardware on the workstation, explicitly pass `load_pika_hardware:=false` and `with_cameras:=false`.

```bash
ros2 launch marvin_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  load_pika_hardware:=false \
  with_cameras:=false \
  use_rviz:=false \
  cpu_affinity:=none
```

### B. Visualization (URDF Mesh & Joint State GUI)
```bash
ros2 launch marvin_manipulation_controller_bringup visualize_marvin_manipulation.launch.py \
  use_joint_state_gui:=true \
  use_rviz:=true
```

### C. Aggregate Workstation Services: `workstation_stack.launch.py`
```bash
ros2 launch marvin_manipulation_controller_bringup workstation_stack.launch.py
```

### D. Sub-launch 1: Episode Recording (`recorder_bringup.launch.py`)
```bash
# C++ MCAP Episode Recorder Backend (loads config/recording/rmi_marvin_bimanual.yaml):
ros2 launch marvin_manipulation_controller_bringup recorder_bringup.launch.py
```

### E. Sub-launch 2: Workstation Cameras (`camera_bringup.launch.py`)
External RealSense on the workstation (current site: D430 `309422070502`, depth-only).
Prefer a **USB 3** port.

```bash
ros2 launch marvin_manipulation_controller_bringup camera_bringup.launch.py
```

Expected topic:
- `/workstation_realsense/camera/depth/image_rect_raw`

Or via stack:
```bash
ros2 launch marvin_manipulation_controller_bringup workstation_stack.launch.py \
  with_cameras:=true
```

---

## 3. Key Launch Arguments

All arguments automatically default to paths in `config/`:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `controllers_yaml` | `config/controller/controllers.yaml` | ros2_control controller parameters. |
| `camera_config` | `config/camera/marvin_cameras.yaml` | Dual D405 + fisheye camera ROS parameters. |
| `recording_stream_config` | `config/recording/rmi_marvin_bimanual.yaml` | Stream contract YAML for MCAP recording. |
| `use_fake_hardware` | `true` | `false` for physical Marvin; `true` for mock hardware. |
| `load_pika_hardware` | `true` | Master switch for Pika gripper `ros2_control` (set `false` for workstation fake hardware). |
| `load_left_pika_hardware` / `load_right_pika_hardware` | `true` | Per-side gripper drivers (require `load_pika_hardware:=true`). |
| `with_cameras` | `true` | Whether to launch Pika wrist perception cameras in `rt_stack` (set `false` for workstation fake hardware). |
| `robot_ip` | `10.19.0.191` | Marvin CCS controller IP. |
| `left_gripper_serial_port` | `/dev/ttyUSB1` | Left Pika serial (gamma default). |
| `right_gripper_serial_port` | `/dev/ttyUSB0` | Right Pika serial (gamma default). |
| `with_left` / `with_right` | `true` | Enable left/right wrist camera groups. |
| `with_d405` / `with_fisheye` | `true` | Enable D405 / Fisheye camera types. |
| `cpu_affinity` | `""` | Comma-separated CPUs for `ros2_control_node`. Empty uses `RT_CM_CPU_AFFINITY`. Pass `none` on workstation. |
| `use_rviz` | `false` | Launch RViz2 visualization. |

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
