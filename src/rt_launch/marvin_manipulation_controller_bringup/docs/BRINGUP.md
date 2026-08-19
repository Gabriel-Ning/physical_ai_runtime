# Marvin + dual Pika RT bringup

RT-host stack for Marvin CCS M6 bimanual:

```text
workstation (planner / policy / teleop)
  --DDS-->  ExecutionManager
              -> inactive route controllers
                   JSPC / TSKPC / JTC (+ Pika forward)
              -> this package: xacro + ros2_control_node
```

This package owns Marvin+Pika composition and the manipulation controllers:

- `urdf/marvin_manipulation.urdf.xacro` — assembly entry (this package)
- `config/model/gripper_tcp.yaml` / `config/model/joint_limits.yaml` — Pika TCP +
  finger travel for both arms (overrides `pika_gripper_description` defaults)
- `config/controller/controllers.yaml` — controller_manager + route controllers
- Arm / stand URDF — `marvin_description`
- Arm `ros2_control` — `marvin_hardware_interface`
- Pika gripper + Marvin flange adaptor — `pika_gripper_description` (ours)

Planning (cuRobo) stays on the workstation. Do not import `motion_planner_core`
on the RT host. cuRobo robot YAMLs live in `curobo_robot_models`
(`src/motion_planning/motion_planners/curobo_robot_models`), not in this RT package.
If you change `config/model/gripper_tcp.yaml` or `config/model/joint_limits.yaml`,
regenerate `curobo_robot_models/config/marvin_manipulation.yml` on the
workstation (see that package README).

All route controllers start **inactive**. EM activates one after a provider
acquires control.

Launch commands (visualize / fake / real): see [../README.md](../README.md).
Use a **pixi-activated** shell so `ROS_DOMAIN_ID=1` and CycloneDDS are set.

## Pre-flight: Clear Servo Errors (Real Hardware)

Before launching `rt_stack.launch.py` or `controller_bringup.launch.py` on real hardware, run:

```bash
ros2 run marvin_manipulation_controller_bringup clear_errors
# Default IP: 10.19.0.191 (or specify --ip <ip>)
```

## Controllers that are spawned

Serialized: `joint_state_broadcaster` → inactive left/right `*_arm_tskpc` /
`*_arm_jspc` / `*_arm_jtc`. If `load_pika_hardware:=true`, also
`*_pika_gripper_fwd`. `load_pika_hardware:=false` skips Pika `ros2_control`
and the gripper spawners; URDF/TCP stay for planning.

## Fake vs real

| EM provider | Controllers |
|---|---|
| Policy | `left/right_arm_jspc` |
| Teleop | `left/right_arm_tskpc` |
| Planner | `left/right_arm_jtc` |

`use_fake_hardware:=true` is the safe default. Real hardware must be powered,
safed, and attended at the e-stop after the fake-hardware gate passes.

Command endpoints are under `/execution/<group>/`. `/execution_manager/*` is EM
telemetry only, not a command namespace.

Endpoints (left shown; right mirrors with `right_arm` / `right_gripper`):

- `/execution/left_arm/joint_reference`
- `/execution/left_arm/pose_reference`
- `/execution/left_arm/cartesian_twist`
- `/execution/left_arm/follow_joint_trajectory`
- `/execution/left_gripper/joint_reference`

The `marvin_bimanual` RMI profile still lists the old controller-named command
topics; update it after this RT topic lock.

## Workstation (after RT is up)

This package does not launch EM or planner. Workstation composition uses the
`marvin_bimanual` RMI profile
(`apps/profiles/marvin_bimanual.yaml`).
That profile still lists the old controller-named command topics; update it
after this RT topic lock. Keep `use_rviz:=false` on the RT host; open RViz on
the workstation with Fixed Frame `world`.

## CPU affinity

On a host configured per [`docs/CPU_HOST_SETUP.md`](../../../../docs/CPU_HOST_SETUP.md),
bringup pins only `ros2_control_node` to `RT_CM_CPU_AFFINITY`. Override with
`cpu_affinity:=12,13` or disable with `cpu_affinity:=none`.

## Check

This bringup does not start EM. After `rt_stack.launch.py`:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

`left/right_arm_*` and `*_pika_gripper_fwd` must be `inactive`.
`/execution_manager/status` appears only after the workstation/RMI EM launch.
