# PlaCo backend integration

Reference: `ref/placo/docs/kinematics/`. Implementation:
`manipulation_position_controllers/.../differential_ik/placo_solver.hpp`.

## Task stack (aligned with ref/placo)

| PlaCo task | YAML / `SolverConfig` | Role |
|---|---|---|
| `PositionTask` + `OrientationTask` (soft) | `position_gain`, `orientation_gain` | EE tracking toward the full Cartesian target; gains are PlaCo soft-task weights |
| `JointsTask` (soft) | `posture_weight` | Pink/Mink-style soft posture objective toward `q_neutral` (pose at controller activation) |
| `RegularizationTask` | `damping` → weight `damping²` | Penalise ‖Δq‖; smoother steps |
| per-joint Δq cost | `joint_motion_weights[]` | e.g. `[8,1,1,…]` — resist base joint motion during orientation teleop |
| `KineticEnergyRegularizationTask` | `kinetic_energy_weight` | Mass-weighted smooth motion (needs `solver.dt`) |
| `ManipulabilityTask` | `manipulability_weight` | Stay away from singularities on redundant arms |
| velocity / joint limits | URDF + `max_joint_velocity_rad_s` | native PlaCo QP inequalities: `enable_velocity_limits`, `enable_joint_limits` |

Future candidates from `ref/placo/src/placo/kinematics/` (not wired yet):
`FrameTask`, `AxisAlignTask`, `mask_dof`, self-collision constraints.

## Same marker pose, different joints than DLS

**Expected** on 7-DOF: infinite IK solutions (null space). Compare tip error (mm),
not joint angles. PlaCo picks a QP optimum from its weighted task stack; DLS uses
damped pseudoinverse + null-space projector. PlaCo posture is a small soft
`JointsTask` objective in the same QP, matching the Pink/Mink posture-task
pattern rather than an analytical null-space projection.

To keep elbow/base posture stable during teleop:

1. Raise `posture_weight` (hold activation configuration).
2. Raise `joint_motion_weights[0]` (penalise J1 Δq).
3. Enable `kinetic_energy_weight` (prefer distal motion).
4. Do **not** expect joint-for-joint match with DLS.

## Tracking-error resync (all TSKPC backends)

When `|command − feedback| > max_command_tracking_error_rad`, TSKPC **rate-limits**
resync at `max_joint_velocity_rad_s` per cycle instead of jumping in one step.

## Overshoot (MPC-208) — hardware gate only

Prior Marvin **real-hardware** teleop (2026-06-23, post-solve acceleration limit)
reported **overshoot** around the marker goal. That symptom is **not re-tested**
on the current branch yet — it is explicitly **left for hardware validation**.

Fake hardware + operator teleop on `claude/quirky-rubin-1oic3t` (2026-06-25)
was previously reported **good** on a branch that experimented with native
in-QP acceleration + braking (Pink #103). The current PlaCo wrapper sends the
full Cartesian target to PlaCo and relies on native joint velocity limits for
speed limiting. Until the hardware overshoot gate is run, use validated
**OSQP** or `pinocchio_dls` for production teleop.

## Smoothness and limits

The PlaCo wrapper currently enables native joint-position and joint-velocity
limits only:

- `robot.set_velocity_limit(joint, max_joint_velocity_rad_s)`
- `solver.enable_joint_limits(true)`
- `solver.enable_velocity_limits(true)`
- `solver.dt = period`

`solver.max_joint_acceleration_rad_s2` is still parsed by the controller config
but is not wired into this PlaCo backend. Smoothness should be tuned with
`damping`, `joint_motion_weights`, and `kinetic_energy_weight`. A final
joint-velocity clamp remains as a defensive safety net.

## Suggested Marvin PlaCo YAML

```yaml
solver:
  backend: placo
  position_gain: 4.0
  orientation_gain: 1.0
  damping: 0.10
  posture_weight: 5.0e-3
  manipulability_weight: 1.0e-4
  kinetic_energy_weight: 5.0e-5
  joint_motion_weights: [8.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  max_iterations: 1
```

**Tuning:** start with moderate `damping`, then use `joint_motion_weights` to
discourage specific joints and `kinetic_energy_weight` to prefer smoother
mass-weighted motion.

**MPC-208 overshoot is still OPEN** (see the Overshoot section above) — the
full-target PlaCo formulation still needs hardware validation.

Rebuild `manipulation_position_controllers` after wrapper changes; restart launch
for YAML-only edits.

Tracked: MPC-208, MPC-103.
