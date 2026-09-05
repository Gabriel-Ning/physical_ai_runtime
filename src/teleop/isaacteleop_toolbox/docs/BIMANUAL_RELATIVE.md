# Bimanual relative retargeter

Application code: `python/isaacteleop_toolbox/retargeters/bimanual_relative.py`  
Profile: `configs/quest3_bimanual_relative.yaml`  
Node: `quest3_bimanual_target`

## Pipeline

```text
ControllersSource
  -> BimanualRelativeRetargeter
  -> OutputCombiner
  -> Quest3BimanualTargetNode
  -> PoseStamped + status + clutch snapshots
```

IsaacTeleop owns device I/O and session execution. This package owns the
relative retargeter, ROS boundary, and source-only launch.

## Clutch behavior

Relative mode is deadman / clutch based:

1. On inactive → active (both squeezes valid and pressed), capture controller
   poses and align to current robot EE poses via TF (when `*_flange_frame` is configured).
   If TF lookup fails, activation is strictly refused (`reason="no_fk"`) to prevent
   sudden setpoint jumps on real hardware. In source-only mode (when both flange frames
   are explicitly unconfigured), static YAML anchors are used.
2. While held, apply scaled translation/rotation deltas to those EE anchors.
3. Releasing either squeeze makes output inactive (`reason="clutch_released"`);
   the next press relatches so the operator can reposition hands without moving the robot.

## Features in 0.1.0

- Fixed OpenXR → robot basis change (`openxr_to_base_rotation_xyzw`)
- Dynamic FK alignment at clutch activation with strict `no_fk` refusal on TF failure
- Explicit source-only mode using YAML anchors when `*_flange_frame` are unconfigured
- Independent `linear_scale` / `angular_scale`
- First-order low-pass (`lowpass_alpha`)
- Per-cycle clamps (`max_linear_step_m`, `max_angular_step_rad`)
- Latched clutch snapshots for recording / diagnosis
- **Continuous Button-Servo Gripper Control**:
  - Right controller: `A` (Primary Click) = Close, `B` (Secondary Click) = Open
  - Left controller: `X` (Primary Click) = Close, `Y` (Secondary Click) = Open
  - Release-to-Hold: releasing buttons freezes the current position setpoint ($0.0 \sim 0.04\text{ m}$)
  - Publishes standard `trajectory_msgs/msg/JointTrajectory` for downstream controller consumption

## Tuning

Human hand workspace and arm workspace often do not feel one-to-one. For first
real-motion passes prefer reduced scales (for example `0.3`) until tracking
looks correct.

| Parameter | Role |
|---|---|
| `linear_scale` / `angular_scale` | Workspace gain |
| `lowpass_alpha` | Lag vs noise |
| `max_linear_step_m` / `max_angular_step_rad` | Safety step clamps |
| `pose_source` | `aim` or grip pose |
| `deadman_source` / `deadman_threshold` | Clutch input |
| `require_both_deadman` | Require both hands |

Treat scale, filtering, and clamps as separate knobs.

## Next mode (not implemented)

An explicit **absolute** retargeter should live as a sibling under
`retargeters/`, selected by profile — not as branches inside the relative
class. It will need a calibrated operator/robot frame, workspace bounds, an
activation policy that avoids target jumps, and the same validity / deadman /
status contract.

Absolute mapping needs a real translation+rotation extrinsic calibration
(headset tracking origin vs. robot/world frame); relative mode only needs the
fixed rotation convention (`openxr_to_base_rotation_xyzw`) because deltas are
translation-invariant. Do not skip that calibration step by hardcoding a
per-room position offset, the way some sim-only teleop demos do.

## Possible future refinement: per-axis rotation lock

Not implemented. Idea from comparing against
[SimPublisher](https://github.com/intuitive-robots/SimPublisher)'s
`Se3SimPubHandTrackingRel` device, which exposes a separate toggle
(`_lock_rot`) to freeze rotation delta while translation keeps tracking — lets
an operator align position first, then dial in wrist orientation separately.

Our config has no equivalent: `require_both_deadman` gates translation and
rotation together per hand. A future `lock_rotation_source` (a second
deadman-like input that, while active, holds `rotation_delta` at identity
while `position_delta` keeps updating) could add this without disturbing the
existing clutch/anchor contract — the rotation half of `_relative_target`
would just skip updating `previous_target.rotation` while locked.
