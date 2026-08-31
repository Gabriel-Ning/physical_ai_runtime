# Marvin + dual Pika RT bringup

RT-host stack for Marvin CCS M6 bimanual:

```text
workstation (planner / policy / teleop)
  --DDS-->  execution_manager
              -> route controllers (JTC primed active on real HW)
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

Launch commands: see [../README.md](../README.md).
Use a **pixi-activated** shell so `ROS_DOMAIN_ID=1` and CycloneDDS are set.

## Pre-flight: Clear Servo Errors (Real Hardware)

Before launching `rt_stack.launch.py` or `controller_bringup.launch.py` on real hardware, run:

```bash
ros2 run marvin_manipulation_rt_launch clear_errors
# Default IP: 10.19.0.191 (or specify --ip <ip>)
```

## Controllers that are spawned

Serialized: `joint_state_broadcaster` → inactive left/right `*_arm_tskpc` /
`*_arm_jspc` / `*_arm_jtc`. If `load_pika_hardware:=true`, also
`*_pika_gripper_fwd` / `*_pika_gripper_action` (fake or real per `use_fake_hardware`).
`load_pika_hardware:=false` omits both grippers' URDF, ros2_control, and spawners.

### Real-hardware position prime (`rt_stack` only)

On `use_fake_hardware:=false` (and `prime_arm_position:=true`, default),
`rt_stack.launch.py` includes `prime_arm_position.launch.py` **after**
`controller_bringup`: it waits for `left_arm_jtc` / `right_arm_jtc` to be loaded,
then activates both. That runs the hardware interface cold
`enter_position_modes` during RT bringup.

**Why (vendor CCS / libmarvin):** the first cold CCS position-mode enter can race
the first workstation EM FollowJointTrajectory claim (example 17 first-run
`left_arm: ABORTED`). After any controller has claimed the arms, Execution
Manager never fully unloads route controllers—it only swaps—so a primed JTC
keeps CCS in position and later switches use `restart — keeping active`.

This is a **bringup workaround**. Prefer removing `prime_arm_position` once
official CCS / an updated `libmarvin` makes cold enter reliable under concurrent
claim. Fake hardware skips priming. `controller_bringup.launch.py` alone does
not prime; use `rt_stack` or launch `prime_arm_position.launch.py` manually.

## Fake vs real

| Provider | Controllers |
|---|---|
| Policy | `left/right_arm_jspc` |
| Teleop | `left/right_arm_tskpc` |
| Planner | `left/right_arm_jtc` |

`use_fake_hardware:=true` is the safe default. Real hardware must be powered,
safed, and attended at the e-stop after the fake-hardware gate passes.

Controller endpoints are under `/execution/<group>/`. `/execution_manager/authority_*` is authority telemetry
telemetry only, not a command namespace.

Endpoints (left shown; right mirrors with `right_arm` / `right_gripper`):

- `/execution/left_arm/joint_reference`
- `/execution/left_arm/pose_reference`
- `/execution/left_arm/twist_reference`
- `/execution/left_arm/follow_joint_trajectory`
- `/execution/left_gripper/joint_reference`

The `marvin_bimanual` RMI profile still lists the old controller-named command
topics; update it after this RT topic lock.

## Workstation (after RT is up)

The RT stack does not launch the Execution Manager or planner. Workstation composition uses the
`marvin_bimanual` RMI profile
(`apps/profiles/marvin_bimanual.yaml`).
That profile still lists the old controller-named command topics; update it
after this RT topic lock.

## CPU affinity

On a host configured per [`docs/CPU_HOST_SETUP.md`](../../../../docs/CPU_HOST_SETUP.md),
bringup pins only `ros2_control_node` to `RT_CM_CPU_AFFINITY`. Override with
`cpu_affinity:=12,13` or disable with `cpu_affinity:=none`.

## Check

This RT bringup does not start the Execution Manager. After `rt_stack.launch.py`:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

On **fake** hardware: `left/right_arm_*` and gripper forwards should be `inactive`.
On **real** hardware with priming: `left/right_arm_jtc` should be `active`; other
arm routes and gripper forwards remain `inactive`.
`/execution_manager/authority_status` appears only after the workstation Execution Manager launch.
