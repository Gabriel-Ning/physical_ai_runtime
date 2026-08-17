# RMI Example Applications

This directory provides interactive reference examples demonstrating the complete capabilities of the **RMI Python SDK**, **Execution Management**, **Multi-Source Arbitration**, **Workstation Sensors & Recorders**, and **Motion Planning Adapters (cuRobo & VLA Policy)** with RT Host fake / real hardware.

---

## 1. Quick Start

### Start RT Host Bringup (Fake Hardware)

```bash
# Franka FR3 Single Arm (Fake Hardware + RViz)
ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true use_rviz:=true

# OR Marvin Dual Arm
ros2 launch marvin_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

# OR Piper Dual Arm
ros2 launch piper_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true
```

---

## 2. Examples Catalog

### Core SDK and execution

| Demo | Focus | Description |
| :--- | :--- | :--- |
| **`01_context.py`** | Context & topology | Profile parts, joints, frames, controllers, live state |
| **`02_policy_camera.py`** | Multimodal sensing | Camera facade with synchronized `(joints, image)` |
| **`03_teleop_preempt.py`** | HIL preemption | Teleop preempts Policy, then hands back |
| **`05_policy_recovery.py`** | Planner-in-the-loop | Anomaly → cuRobo recovery → resume Policy |
| **`06_partial_ownership.py`** | Multi-part scope | Planner on `arm`, Policy on `gripper` |
| **`07_record_episode.py`** | Recording | Synchronized episode capture |

### Motion planning, resolvers, and streamers

| Demo | Family | Core Capability | Target Controller |
| :--- | :--- | :--- | :--- |
| **`04_plan_execute.py`** | **Planner** | cuRobo collision-free trajectory planning dispatched to JTC Action Server | `JTC` |
| **`08_ik_resolver.py`** | **Resolver** | Real-time cuRobo Inverse Kinematics resolution (`resolve() -> q*`) | `JSPC` |
| **`09_joint_streamer.py`** | **Streamer** | Receding horizon VLA action chunking or cuRobo MPC joint stream | `JSPC` |
| **`10_marker_teleop.py`** | **Streamer** | Cartesian 6-DoF RViz Interactive Marker teleop & velocity twist streaming | `TSKPC` |
| **`11_joint_jog.py`** | **Teleop** | Interactive joint delta jogger with auto-homing | `JSPC` |

---

## 3. Running the Examples

```bash
# 01. Context & robot state
python examples/01_context.py --profile fr3_pika_single_arm.yaml

# 04. Segment trajectory planner (cuRobo -> JTC)
python examples/04_plan_execute.py --profile fr3_pika_single_arm.yaml

# 08. Online IK resolver
python examples/08_ik_resolver.py --profile fr3_pika_single_arm.yaml --mode marker

# 09. Joint Streamer & Action Chunking
python examples/09_joint_streamer.py --profile fr3_pika_single_arm.yaml --backend vla_dummy

# 10. Cartesian Teleop (Drag RViz Marker to Drive Robot)
python examples/10_marker_teleop.py --profile fr3_pika_single_arm.yaml --mode marker

# 11. Interactive Joint Jogger
python examples/11_joint_jog.py --profile fr3_pika_single_arm.yaml --joint 4 --delta 0.1
```
