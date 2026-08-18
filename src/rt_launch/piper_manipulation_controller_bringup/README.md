# piper_manipulation_controller_bringup

The dual-arm cell (table + arms + grippers) lives in
`piper_description/urdf/piper_bimanual_manipulation.urdf.xacro`. Gripper TCP is
`piper_description/config/gripper_tcp.yaml`. Controller params are
`config/controller/controllers.yaml`. After changing description geometry or TCP,
regenerate the workstation cuRobo model
(`curobo_robot_models/config/piper_bimanual_manipulation.yml`):

```bash
cd src/motion_planning/motion_planners/curobo_robot_models
bash scripts/generate_curobo_robot_model.sh --model piper
```

## Visualize (mesh + JSP GUI)

Visualization is owned by `piper_description`:

```bash
ros2 launch piper_description visualize_piper_bimanual.launch.py
```

## Fake hardware

```bash
ros2 launch piper_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=true \
  use_rviz:=false \
  cpu_affinity:=none
```

```bash
ros2 launch piper_manipulation_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=true \
  use_rviz:=false \
  cpu_affinity:=none
```

## Real hardware (RT host)

Bring SocketCAN up externally first. Real dual-arm defaults to the site
aliases `piper0` / `piper1`. On a cpu RT host, omit `cpu_affinity` so
`ros2_control_node` pins to `RT_CM_CPU_AFFINITY`. `cpu_affinity:=none`
disables pinning — that is for fake / workstation only.
`left_end_effector` / `right_end_effector` are gripper **types**
(`none` | `piper_gripper`); both sides can be `piper_gripper`.

```bash
ros2 launch piper_manipulation_controller_bringup rt_stack.launch.py \
  use_fake_hardware:=false \
  use_rviz:=false
```

```bash
ros2 launch piper_manipulation_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=false \
  use_rviz:=false
```

Details: [docs/BRINGUP.md](docs/BRINGUP.md).
