# franka_manipulation_rt_launch

RT Host bringup for **Franka FR3 + Pika Gripper**.

Package path: `src/bringup/franka_manipulation/rt_launch`  
ROS package name: `franka_manipulation_rt_launch`

Workstation assets (Execution Manager / recorder) live in sibling package
`franka_manipulation_workstation_launch`.

## Configuration Layout

- `config/controller/controllers.yaml`: real FR3 effort controllers + Pika forward controller
- `config/controller/controllers_fake.yaml`: fake hardware position controllers
- `config/camera/pika_cameras.yaml`: wrist RealSense D405 + Sunplus fisheye
- `config/model/gripper_tcp.yaml` & `config/model/joint_limits.yaml`: TCP and finger limits
- `urdf/fr3_manipulation.urdf.xacro`: FR3 + Pika assembly entry

## RT Host Stack

### Aggregate: `rt_stack.launch.py`

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101 \
  load_pika_hardware:=true \
  gripper_serial_port:=/dev/ttyUSB0 \
  use_rviz:=false
```

Fake / local (run on **workstation**, not RT host):

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=true \
  load_pika_hardware:=true \
  with_cameras:=false \
  use_rviz:=true \
  cpu_affinity:=none
```

Start Execution Manager in a second terminal:

```bash
ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py
# EM only:
# ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py \
#   with_recorder:=false
```

### Controllers only / cameras only

```bash
ros2 launch franka_manipulation_rt_launch controller_bringup.launch.py use_fake_hardware:=false
ros2 launch franka_manipulation_rt_launch camera_bringup.launch.py
```

### Visualization

```bash
ros2 launch franka_manipulation_rt_launch visualize_fr3_manipulation.launch.py \
  use_joint_state_gui:=true use_rviz:=true
```

## Workstation (sibling package)

```bash
ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py
```

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
