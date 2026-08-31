# gamepad_teleop

Gamepad teleoperation input source package for ROS 2 Jazzy.

Treats USB/Bluetooth gamepads (Sony DualSense PS5, Microsoft Xbox, Switch Pro, etc.) as **independent workstation input devices**, producing standardized 6-DoF Cartesian twist commands, continuous gripper setpoints, and human-intervention clutch/preemption signals.

---

## 1. Topic & Service Contracts

| Entity | Type | Name | Description |
| :--- | :--- | :--- | :--- |
| **Publisher** | `geometry_msgs/msg/TwistStamped` | `/action_sources/gamepad/arm/twist` | 6-DoF Cartesian spatial velocity for RT **TSKPC** |
| **Publisher** | `trajectory_msgs/msg/JointTrajectory` | `/action_sources/gamepad/end_effector/joint_reference` | Target opening width ($m$) for gripper controller |
| **Publisher** | `std_msgs/msg/Bool` | `/teleop/gamepad/clutch` | Preempt/Clutch signal (`true` when L1 is held) |
| **Publisher** | `std_msgs/msg/String` | `/teleop/gamepad/status` | Real-time JSON diagnostics and teleop state |
| **Subscriber**| `sensor_msgs/msg/Joy` | `/joy` | Raw gamepad state from native `joy` driver |
| **Service**   | `std_srvs/srv/SetBool` | `~/enable` | Enable/disable the teleop source |

---

## 2. Control Mapping (Sony PS5 DualSense)

### 6-DoF Cartesian Arm Control
* **L1 (Hold)**: **Clutch / Preemption** (enables arm twist output and signals preemption)
* **L2 (Hold)**: **Turbo Speed Multiplier** ($2.0\times$)
* **Left Stick (X / Y)**: **Surge / Sway** translation:
  * Up / Down: $\pm v_x$ ($0.15\text{ m/s}$)
  * Left / Right: $\pm v_y$ ($0.15\text{ m/s}$)
* **Right Stick (Y)**: **Heave** translation:
  * Up / Down: $\pm v_z$ ($0.10\text{ m/s}$)
* **Right Stick (X)**: **Yaw** rotation:
  * Left / Right: $\pm \omega_z$ ($0.35\text{ rad/s}$)
* **D-Pad (十字键)**: **Pitch & Roll** rotation:
  * **Up / Down**: $\pm \omega_y$ (Pitch: $0.35\text{ rad/s}$)
  * **Left / Right**: $\pm \omega_x$ (Roll: $0.35\text{ rad/s}$)

### Continuous Gripper Servoing
* **○ 圆圈键 (Hold)** (Xbox **B**): 连续平滑**张开**（速度 $25\text{ mm/s}$，行程 $0.000 \to 0.040\text{ m}$）
* **△ 三角键 (Hold)** (Xbox **Y**): 连续平滑**闭合**（速度 $25\text{ mm/s}$，行程 $0.040 \to 0.000\text{ m}$）
* **松开按键**: 夹爪开度稳定保持在当前位置（实现任意中间开度精确伺服）
* `require_clutch_for_gripper`: 设为 `false`，无需按住 L1 即可随时自由调控夹爪。

---

## 3. Quick Start

### Build Package
```bash
colcon build --symlink-install --packages-select gamepad_teleop
source install/setup.bash
```

### Launch Standalone
```bash
# Launch with PS5 DualSense
ros2 launch gamepad_teleop gamepad_teleop.launch.py
```

### Check Topic Outputs
```bash
# Monitor clutch state (press and release L1)
ros2 topic echo /teleop/gamepad/clutch

# Monitor 6-DoF Cartesian twist references (hold L1 + use sticks/D-pad)
ros2 topic echo /action_sources/gamepad/arm/twist

# Monitor Gripper setpoints (press ○ / △)
ros2 topic echo /action_sources/gamepad/end_effector/joint_reference

# View real-time JSON diagnostics
ros2 topic echo /teleop/gamepad/status
```
