# Dual Piper RT bringup

RT-host stack for the experiment-table dual Piper cell:

```text
workstation (planner / policy / teleop)
  --DDS-->  ExecutionManager
              -> inactive route controllers
                   JSPC / TSKPC / JTC (+ gripper forward)
              -> this package: xacro mappings + ros2_control_node
```

This package launches the cell; it does not own the URDF:

- Assembly — `piper_description/urdf/piper_bimanual_manipulation.urdf.xacro`
- Gripper TCP — `piper_description/config/gripper_tcp.yaml`
- `config/controller/controllers.yaml` — controller_manager + route controllers
- Arm / gripper `ros2_control` — `piper_hardware_interface`

Planning (cuRobo) stays on the workstation. Do not import `motion_planner_core`
on the RT host. cuRobo robot YAMLs live in `curobo_robot_models`. If you change
the description geometry or gripper TCP, regenerate
`curobo_robot_models/config/piper_bimanual_manipulation.yml` on the workstation.

The table in that xacro is visual-only. Planner world cuboids own table
collision.

All route controllers start **inactive**. EM activates one after a provider
acquires control. This app does not override EM class priorities
(`teleop: 100`, `trajectory_planner: 60`, `policy: 50`, `streaming_planner: 40`).
Marker and leader sources are both teleop producers and should not publish
concurrently. Same-priority ownership stays sticky while the current owner is
eligible.

`controller_bringup.launch.py` owns ros2_control only (no EM).
`rt_stack.launch.py` is the RT-host entry and includes controller bringup
only. Workstation composition uses the `piper_bimanual` RMI profile after
this stack is up. Leader teach models are parsed by teleop nodes and never
enter this controller manager.

Launch commands (visualize / fake / real): see [../README.md](../README.md).
Use a **pixi-activated** shell so `ROS_DOMAIN_ID=1` and CycloneDDS are set.

## Controllers that are spawned

Serialized: `joint_state_broadcaster` → inactive per-side `*_arm_jspc` /
`*_arm_tskpc` / `*_arm_jtc`, plus `*_gripper_fwd` when that side's end
effector is `piper_gripper`.

## Execution routes

| EM route | Piper controller | Input |
| --- | --- | --- |
| `joint_servo` | `<side>_arm_jspc` | normalized `joint_reference` |
| `cartesian_servo` | `<side>_arm_tskpc` | normalized Cartesian reference |
| `trajectory_execution` | `<side>_arm_jtc` | proxied `joint_trajectory` action |
| `end_effector_servo` | `<side>_gripper_fwd` | normalized end-effector `joint_reference` |

JSPC `trajectory_behavior.max_points: 1` is a controller-side capability. EM may
accept a multi-point `joint_reference`; how a controller consumes those points
is not an EM responsibility.

Command endpoints are under `/execution/<group>/`. `/execution_manager/*` is EM
telemetry only, not a command namespace.

Endpoints (left shown; right mirrors with `right_arm` / `right_gripper`):

- `/execution/left_arm/joint_reference`
- `/execution/left_arm/pose_reference`
- `/execution/left_arm/cartesian_twist`
- `/execution/left_arm/follow_joint_trajectory`
- `/execution/left_gripper/joint_reference`

The `piper_bimanual` RMI profile still lists the old controller-named command
topics; update it after this RT topic lock.

## End-effector boundary

An end effector is an independently selectable attachment on
`<side>_flange_link`, not joints owned by `PiperHardwareInterface`. Per-side
`left_end_effector` / `right_end_effector`: `none` | `piper_gripper` (default
`piper_gripper`). Unsupported values fail launch. This RT profile does not
load Pika.

TSKPC `tip_frame` is `<side>_gripper_tcp` when the native gripper is enabled.

## Launch arguments

| Argument | Values / notes |
| --- | --- |
| `arms` | `left` \| `right` \| `both` (`both`) |
| `use_fake_hardware` | `true` \| `false` (`true`) |
| `left_can_interface` / `right_can_interface` | SocketCAN names (`piper0` / `piper1`). Real dual-arm must differ. |
| `left_end_effector` / `right_end_effector` | `none` \| `piper_gripper` (`piper_gripper`) |
| `left_xyz` / `right_xyz`, `*_rpy` | Empty defers to `piper_description` (`±0.32 0.29 0.72`, yaw `-π/2`) |
| `use_rviz` | `true` \| `false` (`false`) |
| `cpu_affinity` | e.g. `14,15`; empty uses `RT_CM_CPU_AFFINITY`; `none` disables pinning |

`rt_stack.launch.py` keeps `arms:=both`, site CAN aliases `piper0` / `piper1`,
and native Piper grippers. It forwards fake/RViz/affinity. Pass end-effector
overrides through `controller_bringup.launch.py`. Empty `cpu_affinity` pins
`ros2_control_node` to `RT_CM_CPU_AFFINITY` from the cpu RT profile.

The hardware components never reconfigure SocketCAN. Bring the links up
externally before `use_fake_hardware:=false`.

## Workstation (after RT is up)

This package does not launch EM or planner. Workstation composition uses the
`piper_bimanual` RMI profile
(`src/interfaces/rmi/config/embodiment_profiles/piper_bimanual.yaml`).
That profile still lists the old controller-named command topics; update it
after this RT topic lock. Keep `use_rviz:=false` on the RT host.

## CPU affinity

On a host configured per [`docs/CPU_HOST_SETUP.md`](../../../../docs/CPU_HOST_SETUP.md),
bringup pins only `ros2_control_node` to `RT_CM_CPU_AFFINITY`. Override with
`cpu_affinity:=12,13` or disable with `cpu_affinity:=none`.
