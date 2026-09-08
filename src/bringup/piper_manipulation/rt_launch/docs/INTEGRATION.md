# Piper integration gates (vs Franka template)

Physical AI Runtime dual-Piper cell. Same RT ↔ Workstation ↔ RMI boundary as Franka;
content is embodiment-owned.

Cross-refs: [tasks assets](../../../../tasks/README.md) ·
[MuJoCo reuse](../../../../mujoco_ros2_control/doc/PHYSICAL_AI_REUSE.md) ·
[controllers](../../../../controller/README.md) ·
[profile/EM/recording](../../franka_manipulation/workstation_launch/docs/PROFILE_EM_RECORDING.md)

## Reuse vs own

| Layer | Reuse | Own for Piper |
|-------|-------|---------------|
| RoboTwin scene assets / task MJCF | Yes (`src/tasks/robotwin_tasks`) | — |
| Robot MJCF + actuators | — | `rt_launch/mjcf/` (aligned with real CAN arms; Franka pattern) |
| `MujocoSystemInterface` + SHM camera stack | Yes | `rt_launch/config/mujoco_plugins.yaml` topic/frame map |
| MPC plugins | Yes | `rt_launch/config/controller/controllers.yaml` |
| Profile / EM / recording contracts | Pattern | `piper_bimanual.yaml`, EM YAML, `apps/recording/piper_*` |

## Gaps closed by this integration

1. Camera ROS topics in `mujoco_plugins.yaml` aligned to profile
   (`/observation/static_orbbec/...`, wrist RealSense paths).
2. `output: shm` + `mujoco_image_bridge` + bridge-only Cyclone `spdp` in
   `mujoco_bringup` when `backend:=mujoco` (Franka-style split from
   `controller_bringup`).
3. Workstation `use_sim_time` wired through stack; sim recording contract
   `apps/recording/piper_bimanual_mujoco.yaml`.
4. Joint hold / Policy sync gate documented below (neural checkpoint optional).

## Gates

### Gate 1 — Fake HW + EM (no cameras / leaders)

```bash
pixi run rt-piper use_fake_hardware:=true use_rviz:=false cpu_affinity:=none
# Fake has no real cameras; disable drivers. Leaders default off.
pixi run workstation-piper with_orbbec:=false with_realsense:=false
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /execution_manager/authority_status --once
python apps/check_runtime.py --profile piper_bimanual.yaml --duration 5
```

Pass: JSB active; route controllers inactive; authority_status present;
`check_runtime` `passed: true`.

**Recorded 2026-09-06 (this host):** Fake RT+WS — JSB active, all route/gripper
controllers inactive, 14 joints on `/joint_states`, EM wall clock,
`check_runtime` passed (`max_state_age_s≈0.002`).

### Gate 2 — MuJoCo + sim workstation

```bash
pixi run rt-piper backend:=mujoco task:=table_pick_cube headless:=true
# Sim images come from RT bridge; do not start real Orbbec/RealSense.
pixi run workstation-piper use_sim_time:=true \
  with_orbbec:=false with_realsense:=false
ros2 topic hz /clock --window 20
ros2 topic hz /observation/static_orbbec/color/image_raw --window 20
ros2 topic info /observation/static_orbbec/color/image_raw --verbose
```

Pass: `/clock` advances; Image publisher is `mujoco_image_bridge`; SHM under
`/dev/shm/pai_mj_cam_*`. MJCF camera names remain `head_camera` / `left_camera` /
`right_camera`; only ROS topic namespaces match the profile.

Cube spawn is site-shifted: MJCF `y=0.05` (−0.15 from RoboTwin nominal `0.20`)
so the object sits farther from Piper mounts at `y=0.29`.

**Recorded 2026-09-06:** `backend:=mujoco task:=open_laptop headless:=true` —
SHM for all three cameras; `/clock` ~870 Hz; head RGB ~29 Hz from
`mujoco_image_bridge` only; route controllers inactive until claim.

### Gate 3 — Joint Policy closed loop

Hold smoke (no checkpoint):

```bash
python apps/eval.py --profile piper_bimanual.yaml --node-name JointPolicy \
  --resource dual_manipulator --task hold --hold --use-sim-time --preempt
```

Remote cloud pi05 (verified):

```bash
# Cloud: ssh modelarts-4d-assert 'bash -s' < scripts/start_policy_server.sh
# Local tunnel: ssh -N -L 50051:127.0.0.1:50051 modelarts-4d-assert
pixi run rt-piper backend:=mujoco task:=click_bell headless:=false cpu_affinity:=none
pixi run workstation-piper use_sim_time:=true with_orbbec:=false with_realsense:=false
# Then in lerobot env:
python apps/eval.py --profile piper_bimanual.yaml --node-name JointPolicy \
  --resource dual_manipulator --policy-type pi05 --inference remote \
  --remote-address 127.0.0.1:50051 --use-sim-time --preempt --reset-mode none \
  --normalize-gripper --gripper-max-width 0.04 \
  --checkpoint /mnt/dev/lerobot/outputs/robotwin_unified_pi05/checkpoints/last/pretrained_model \
  --task "Click the bell's top center on the table." \
  --rename-map observation.images.top=observation.images.cam_high \
  --rename-map observation.images.left_wrist=observation.images.cam_left_wrist \
  --rename-map observation.images.right_wrist=observation.images.cam_right_wrist
```

Pass: EM claim on `JointPolicy` once; joints track abs refs; optional slider
(`examples/22_piper_joint_slider.py --use-sim-time`) preempts via TeleopJoint.

**Recorded 2026-09-06:** `--hold` + remote `robotwin_unified_pi05` on MuJoCo
`click_bell` closed loop verified.

## Deferred

Real CAN, leader teleop recording, RTC inference, Cartesian policy.
