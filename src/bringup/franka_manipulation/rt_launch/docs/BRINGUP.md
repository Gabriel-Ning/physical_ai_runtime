# FR3 + Pika RT bringup

RT-host stack for one Franka FR3:

```text
workstation (planner / policy / teleop)
  --DDS-->  execution_manager
              -> inactive route controllers
                   JSIC / TSJIC / JTC
              -> this package: xacro + ros2_control_node
```

This package owns FR3+Pika composition (no vendor `franka.launch.py` fork)
and the three manipulation controllers:

- `urdf/fr3_manipulation.urdf.xacro` — assembly entry (this package)
- `config/model/gripper_tcp.yaml` / `config/model/joint_limits.yaml` — Pika TCP +
  finger travel for this bringup (overrides `pika_gripper_description` defaults)
- `config/controller/` — controller_manager + route controllers (real + fake)
- Arm URDF / `franka_hardware` — upstream, unmodified, not maintained here
- Pika gripper + `pika_adaptor` — `pika_gripper_description` (ours)

Planning (cuRobo) stays on the workstation. Do not import `motion_planner_core`
on the RT host. cuRobo robot YAMLs live in `curobo_robot_models`
(`src/motion_planning/motion_planners/curobo_robot_models`), not in this RT package.
If you change `config/model/gripper_tcp.yaml` or `config/model/joint_limits.yaml`, regenerate
`curobo_robot_models/config/fr3_manipulation.yml` on the workstation (see that
package README).

All route controllers start **inactive**. The Execution Manager activates one after a provider
acquires control.

Launch commands (visualize / fake / real): see [../README.md](../README.md).
Use a **pixi-activated** shell so `ROS_DOMAIN_ID=1` and CycloneDDS are set.
Shortcut: `pixi run rmi-fr3-fake-rt`.

## Controllers that are spawned

Serialized: `joint_state_broadcaster` → inactive `franka_arm_tsjic` /
`franka_arm_jsic` / `franka_arm_jtc`. If `load_pika_hardware:=true`, also
`pika_gripper_fwd`.

**Not spawned:** `franka_robot_state_broadcaster`. Planning and the Execution Manager only need
`/joint_states`. The vendor broadcaster is FCI diagnostics (`FrankaRobotState`,
wrench, O_T_EE) and is not on the RMI path.

## Joint-state remap

JSB is remapped so hardware joints do not publish directly on `/joint_states`:

```text
joint_state_broadcaster  --remap-->  /franka/joint_states
joint_state_publisher    source_list:
                           franka/joint_states
                           franka_gripper/joint_states   # unused if no gripper
                         --> /joint_states
```

The Execution Manager and `rmi.Context` subscribe to `/joint_states`.

## Pika without hardware

URDF still includes the Pika adaptor + `pika_gripper_tcp` for planning.
`load_pika_hardware:=false` skips Pika `ros2_control` and `pika_gripper_fwd`
(no serial). Use Planner with `parts=[arm]` only.

## Fake vs real controllers

| Provider | Real HW | Fake HW |
|---|---|---|
| Policy | `franka_arm_jsic` effort | same name, JSPC position |
| Teleop | `franka_arm_tsjic` effort | same name, TSKPC position |
| Planner | `franka_arm_jtc` effort | same name, JTC position |

`use_fake_hardware:=true` loads `config/controller/controllers_fake.yaml`. Names and
topics stay the same.

Controller endpoints are under `/execution/<group>/`.
`/execution_manager/authority_status` and `/execution_manager/authority_events`
are authority telemetry only, not command paths.
inputs.

Topics:

- `/execution/arm/joint_reference`
- `/execution/arm/pose_reference`
- `/execution/arm/twist_reference`
- `/execution/arm/follow_joint_trajectory`
- `/execution/end_effector/joint_reference` (if `load_pika_hardware:=true`)

## Workstation (after RT is up)

The RT stack does not launch the Execution Manager or planner. Workstation composition uses the
`fr3_pika_single_arm` RMI profile
(`apps/profiles/fr3_pika_single_arm.yaml`).
That profile still lists the old controller-named command topics; update it
after this RT topic lock. Keep `use_rviz:=false` on the RT host; open RViz on
the workstation with Fixed Frame `fr3_link0`.

## Perception Cameras

When `with_cameras:=true`, `rt_stack.launch.py` includes `camera_bringup.launch.py` to stream:
- **RealSense D405**: `/pika_d405/camera/color/image_raw`, `/pika_d405/camera/aligned_depth_to_color/image_raw`
- **Sunplus Fisheye**: `/pika_fisheye/image/compressed` (`sensor_msgs/CompressedImage`, original MJPEG)

Parameters (resolution, framerate, formats) are configured in `config/camera/pika_cameras.yaml`.

## CPU affinity

On the **real RT host**, do not disable CPU affinity (`cpu_affinity:=none`). Allow `RT_CM_CPU_AFFINITY` or the real-time profile to pin `ros2_control_node` to dedicated RT isolated cores (see [docs/CPU_HOST_SETUP.md](../../../../docs/CPU_HOST_SETUP.md)). Only use `cpu_affinity:=none` during local fake hardware simulation on developer workstations.

## Distributed DDS

Workstation and RT host must share:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `ROS_DOMAIN_ID=1` (from pixi `[activation.env]`)
- `CYCLONEDDS_URI` pointing at `.config/cyclonedds_default.xml`

Bind Cyclone to the **robot LAN** (not WiFi/VPN). Example: workstation
`192.168.1.13`, RT host `192.168.1.100`. If `ros2 topic list` is empty, the
usual cause is domain 0 vs 1 (RT launched without pixi activation) or DDS
exiting via the wrong NIC.

```bash
ros2 daemon stop
ros2 topic list   # expect /joint_states, /franka/joint_states, ...
```

## Check

This RT bringup does not start the Execution Manager. After `rt_stack.launch.py`:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

`franka_arm_*` and `pika_gripper_fwd` (if loaded) must be `inactive`.
`/execution_manager/authority_status` appears only after the workstation Execution Manager launch.

## Scope

- single FR3
- Pika URDF/TCP for planning; hardware optional
- effort on real HW, position on fake HW
- no planner process on the RT host
