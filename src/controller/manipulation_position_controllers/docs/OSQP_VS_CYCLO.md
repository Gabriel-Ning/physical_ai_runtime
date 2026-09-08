# OSQP in TSKPC vs cyclo_control

> **UPDATE (2026-06-23): the dominant cause of OSQP shake was a solver bug, now
> fixed.** The QP cost targeted the Cartesian *velocity* `gain·error` while the
> decision `dq` is a position *increment*, so it solved `dq = J⁺·v_desired` with
> no `·dt` — a step ≈500× larger than DLS at 500 Hz, clamped each cycle to
> `v_max·dt`, i.e. a period-2 limit cycle of amplitude ≈ `v_max·dt` that appears
> with a **stationary target**. Fixed by scaling the gradient by `dt`
> (`dq = dt·qdot_DLS`); see ISSUES MPC-207 and `test_osqp_solver`. OSQP gains now
> match DLS semantics (use ≈ DLS values). The architecture comparison below
> still applies to cyclo-style *feel* (rate, velocity reference), but is no
> longer needed to remove the limit cycle.

Our `OsqpIkSolver` (`osqp_solver.hpp`) is aligned with cyclo's `QPBase` /
`OpenManipulatorMoveLController` formulation (task Jacobian cost, CBF joint
limits, slack variables). Marvin fake-hardware testing still showed visible
shaking with OSQP under marker teleop while cyclo teleop feels stable.

## What cyclo does differently

Reference: `ref/cyclo/cyclo_control/cyclo_motion_controller_core` and ROS nodes
under `cyclo_motion_controller_ros/src/nodes/`.

| Aspect | cyclo (VR / MoveL) | Our TSKPC + Marvin marker teleop |
|---|---|---|
| Control rate | **100 Hz** default (`control_frequency`) | **500 Hz** `controller_manager` |
| QP decision | `qdot` (joint velocity) | `dq` (position increment per cycle) |
| Task input to QP | `task_xdot_desired` — Cartesian **velocity** set upstream | `gain × pose_error` computed **inside** each IK step |
| Integration seed | `q_feedback + qdot·dt` (VR/MoveL nodes) | `command_ + dq` (previous command, not feedback) |
| Redundancy | Often 6-DOF arms; damping on `qdot` in cost | 7-DOF Marvin; `posture_weight: 0` removed null-space pin |
| Pose reference | VR goals / cubic MoveL trajectory at 100 Hz | `PoseStamped` at 50 Hz, no pose OTG in TSKPC |
| Typical gains | `kp_position=50` at **100 Hz** outside QP; `weight_damping=0.1` in QP | Prior test used `position_gain=10` at **500 Hz** inside solver |
| CBF / slack | `cbf_alpha=5`, `slack_penalty=1000` | `kCbfGain=2`, `kSlackPenalty=1e4` (constants in header) |

## Why cyclo can run OSQP smoothly

1. **Lower loop rate** — ADMM runs 5× less often; each step has a larger `dt`,
   so the same Cartesian error produces a smaller per-step correction.

2. **Velocity-level closure** — Pose error is turned into `task_xdot_desired`
   once per 10 ms, then QP tracks velocity. Our solver re-applies
   `position_gain × error` every 2 ms on a fixed pose target, which amplifies
   small numerical differences in redundant 7-DOF IK.

3. **Feedback-based integration** — `q_command = q_feedback + qdot·dt` damps
   limit cycles. Integrating from `command_` can sustain A↔B oscillation when
   the QP has multiple near-optimal solutions (`posture_weight = 0`).

4. **Stronger velocity regularization** — cyclo's `weight_damping` on every
   joint in the QP cost acts like a null-space pin; our `damping` maps to λ² in
   the task Hessian only (default 0.02–0.05).

5. **Arm morphology** — many cyclo demos use 6-DOF or heavily damped bimanual
   tasks; Marvin left/right is 7-DOF with a large redundant null space.

## Observed Marvin behavior (2026-06-23)

**Aggressive OSQP** (`position_gain: 10`, `orientation_gain: 3`, `posture_weight: 0`):

- Stationary marker: period-2 limit cycle (~0.002 rad joints, ~1 mm task error).
- TSKPC status: `solve_fail = 0`, `ACTIVE`, alternating task error and ~0.09 ms
  solve latency — controller thinks it is healthy.

**DLS + posture** (`pinocchio_dls`, `posture_weight: 1e-3`): stable stationary
hold and good subjective teleop (default Marvin config).

**Conservative OSQP** (`solver.backend: osqp` in `marvin_*_task_space.yaml`):
`position_gain` 4 / `orientation_gain` 1, `damping` 0.10, `posture_weight` 1e-3.

## Recommendations

| Use case | Backend |
|---|---|
| RViz marker / streaming pose teleop | `pinocchio_dls` + `posture_weight: 1e-3` |
| Discrete pose reach near limits | OSQP with conservative gains + posture |
| cyclo-like VR at 500 Hz | Not drop-in — needs outer rate reduction, velocity
  reference path, and/or feedback integration (see MPC-202) |

To make OSQP behave more like cyclo without forking the solver:

1. Lower effective rate — e.g. solve OSQP every Nth controller cycle, or add
   pose reference smoothing upstream of TSKPC.
2. Use `TwistStamped` / velocity outer loop instead of raw `PoseStamped` at 500 Hz.
3. Keep `posture_weight > 0` on 7-DOF arms.
4. Consider integrating from measured `q` when error is below a threshold
   (behavior change — track under MPC-202 / review).

Tracked in `ISSUES.md`: MPC-103 (benchmark), MPC-108 (controller harness).
