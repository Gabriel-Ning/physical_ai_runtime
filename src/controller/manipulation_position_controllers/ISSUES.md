# manipulation_position_controllers Issues

This backlog is scoped to the generic ros2_control position controller
repository. Execution-manager source arbitration, teleop device SDKs, robot
bringup, calibration, recording, and policy inference belong in other
repositories.

## Release Gates

### MPC-001: Optional solver packaging

Current v0.1 policy: `pinocchio`, `osqp`, `osqp-eigen`, `placo`, and `ruckig`
are required. CMake, `package.xml`, installed headers, and README now agree, so
source builds have deterministic capabilities.

Future target:
- Revisit backend plugins or feature packages when dependency footprint becomes
  a demonstrated adoption problem.
- Discuss private/binary distribution only after that source-package boundary is
  stable.

Reason: a generic public repo should have predictable install/build behavior.

### MPC-002: Sync docs with current implementation — resolved

Current state: some round3 docs still describe Ruckig and TwistStamped as future
even though both are implemented and real-hardware validated.

Resolution: README documents all current inputs, reference behavior modes,
staleness/frame contracts, and backend tradeoffs.

### MPC-003: Clarify public examples versus test utilities — resolved

Resolution: generic publishers, bridges, fault injection, and IK-to-JTC A/B
programs live in `examples/` and are not installed. Robot-specific demos and
launch files remain downstream. A self-contained controller-manager
fake-hardware fixture remains part of MPC-108.

### MPC-108: TSKPC controller-lifecycle regression harness

**Release gate** (identified during the 2026-06-23 controller review).

Current state: solver/chunk unit tests are joined by a lightweight controller
lifecycle fixture. It now verifies effort-command zeroing on deactivate,
NaN-gain rejection, and unknown-solver rejection. The following still have
**no controller-manager-level automated test** and are validated only by
manual launch gates:

- TSKPC chunk playback → grace window → `STALE_HOLD` → recovery on new chunk
- Zero-stamp rejection for joint, pose-trajectory, and twist inputs
- JSPC Ruckig `control_cycle` sourced from controller-manager update rate
- OSQP / PlaCo non-root `base_frame` parity with the DLS solver tests
- TSKPC status `kTargetAgeS` during chunk-only execution (code fixed; not
  exercised in a controller harness)

Target:
- Add a `launch_testing` or lightweight controller update-loop fixture that
  drives TSKPC/JSPC through the state transitions above without real hardware.
- Register tests in `CMakeLists.txt` and gate CI on them before claiming
  motion-control validation for merge/release.

Reason: unit tests on extracted helpers do not prove the realtime state machine
contracts that changed on `claude/busy-hamilton-2673fy`.

### MPC-109: Establish formatting and CI gates

Current state: the Release build and 95 behavior tests pass, but enabling the
full `ament_lint_common` set produces a large pre-existing formatting diff
across C++ and Python example sources. Mixing that mechanical rewrite into the
release cleanup would obscure functional history.

Target:
- Land one formatting-only change using the selected ROS 2 style tools.
- Enable ament copyright, cpplint, flake8, pep257, uncrustify, and XML checks.
- Add CI once the required PlaCo dependency has a reproducible public install
  path.

### MPC-111: Target-runtime allocation and jitter instrumentation

Current state: controller-owned task-space work vectors, DLS scratch matrices,
and all three solver result buffers are allocated during configuration and
reused. Regression tests assert that repeated solves retain the same result
storage. The package no longer creates a `std::vector` or resizes a result in
the normal task-controller update path.

Remaining deployment gate:
- Measure allocation count and p99.9 update latency on the target RT image.
- In particular, treat OSQP and PlaCo internals as third-party code that still
  requires allocator tracing; stable wrapper storage alone does not certify
  that those libraries never allocate internally.
- Run the measurement separately for each enabled backend and robot profile.

## Near-Term Iteration

### MPC-110: Latency-aware sampling of receding-horizon joint chunks

Current JSPC behavior already accepts an N-point `JointTrajectory` chunk and
samples it at controller time using `header.stamp + time_from_start`. It uses
linear interpolation when waypoint velocities are absent and cubic Hermite
interpolation when they are present. A newly received chunk replaces the old
one. Therefore this is **not** a request for another `limiter`/`ruckig`
`reference_behavior.mode`; those modes describe output shaping after sampling.

Gap observed with online MPC: the end-to-end delay between state readout,
optimization, ROS transport, and the controller update means sampling exactly
at controller time can select a reference that was intended for an earlier
robot state. Selecting a hard-coded `first`, `middle`, or `last` point is not a
good primary interface because its physical lookahead changes with planner
frequency, horizon `dt`, and dropped/replaced chunks.

Proposed extension under `reference_behavior`:

- Add a separate chunk-sampling policy, preserving current controller-time
  sampling as the default and backward-compatible behavior.
- Support a fixed lookahead in seconds and, later, measured/estimated pipeline
  latency compensation. Evaluate the chunk at
  `controller_time + effective_lookahead` using the existing interpolation.
- Clamp sampling to the available horizon and define stale/expired-chunk
  behavior explicitly. Do not extrapolate beyond the last waypoint by default.
- Keep configuration and validation outside the realtime update path; sampling
  must remain allocation-free and deterministic.
- Expose diagnostics for reference age, requested/effective sample time,
  selected segment, horizon clamp, and replacement/preemption jump.

Suggested acceptance tests:

- zero lookahead is identical to current `sample_reference()` behavior;
- fixed lookahead selects the expected interpolated position and velocity for
  both linear and Hermite chunks;
- sampling before/after the chunk is bounded deterministically;
- chunk replacement, stale timeout, and single-point ZOH remain compatible;
- realtime update tests cover delayed and irregular planner publication.

Ownership: this belongs in JSPC because it defines how a received joint-space
reference is consumed. The execution manager should continue routing and
normalizing chunks without choosing an MPC waypoint.

### MPC-101: Stabilize status message schemas

Current state: controller status uses `std_msgs/Float64MultiArray` with
documented indices in code/docs.

Target:
- Publish a stable schema document for JSPC and TSKPC status fields.
- Consider a typed status message in a future `manipulation_msgs` package if
  downstream consumers grow.

### MPC-102: Add automated tests for TSKPC command-state publishing

Current state: TSKPC publishes `~/commanded_joint_state`, enabling external
safety monitoring. The behavior is validated in fake/real launch gates.

Target:
- Add a focused test or launch test that confirms commanded state is published
  during ACTIVE, STALE_HOLD, and IK_FAIL_HOLD paths.

### MPC-103: Add deterministic solver evaluation harness

Current state: Marvin fake-hardware marker teleop (2026-06-23) showed:

- `pinocchio_dls` + `posture_weight: 1e-3`: stable hold and teleop.
- `osqp` with `position_gain: 10`, `posture_weight: 0`: period-2 limit cycle at
  500 Hz (~0.002 rad, ~1 mm task error, `solve_fail = 0`).
- cyclo_control uses the same QP family at **100 Hz** with velocity-level task
  input and `q_feedback` integration — see `docs/OSQP_VS_CYCLO.md`.

Target:
- Build a repeatable benchmark for DLS/OSQP/PlaCo under pose and twist inputs.
- Measure solve latency, command variance, tracking error, limit-boundary
  behavior, and failure recovery.
- Include a stationary-hold limit-cycle detector (joint/command variance over N
  cycles) so OSQP regressions are caught without RViz.

### MPC-207: OSQP unstable for Marvin 500 Hz marker teleop

**Observed (2026-06-23, Marvin fake hardware, TSKPC, 7-DOF left/right):**

- Aggressive OSQP (`position_gain: 10`, `orientation_gain: 3`,
  `posture_weight: 0`): period-2 limit cycle (~0.002 rad joints, ~1 mm task
  error). TSKPC status: `solve_fail = 0`, `ACTIVE` — controller reports healthy.
- Conservative OSQP (`position_gain: 4`, `orientation_gain: 1`, `damping: 0.10`,
  `posture_weight: 1e-3`): static regression PASS on fake hardware; manual RViz
  marker teleop still visibly shakes.
- `pinocchio_dls` + `posture_weight: 1e-3`: stable stationary hold and good
  subjective teleop (current production workaround).

**Scope:** OSQP is used only on the task-space path
(`PoseStamped` → EM → TSKPC). The joint-space path (EM → JSPC) does not use
OSQP.

**ROOT CAUSE FOUND + FIXED (missing timestep integration):** the OSQP cost
targeted the Cartesian *velocity* `v_desired = gain·error` directly while the
decision variable `dq` is a position *increment*. The QP therefore solved
`dq = J⁺·v_desired` with **no `·dt`**, i.e. a step `1/dt` (≈500× at 500 Hz)
larger than DLS, which computes `qdot = J⁺·v_desired` and integrates
`dq = qdot·dt`. The over-correction is clamped every cycle to `v_max·dt`,
producing a sustained period-2 limit cycle of amplitude **≈ v_max·dt = 0.002 rad**
— exactly the observed magnitude, and it appears with a **stationary target**
immediately at startup (independent of the 50 Hz reference stepping; the earlier
discrete-step hypothesis was wrong).

Fix (`osqp_solver.hpp`): scale the QP gradient by `dt` so
`(JᵀJ + λ²I)·dq = Jᵀ·(v_desired·dt)` ⇒ `dq = dt·qdot_DLS`, aligning OSQP exactly
with DLS. Regression: `test_osqp_solver`
(`SingleStepIncrementMatchesDls`, `StationaryTargetConverges`).
**Note:** OSQP gains now share DLS semantics — use comparable
`position_gain`/`orientation_gain` (≈ DLS values, e.g. 4 / 1), not the previous
inflated values.

**On the `command_` (vs `q_feedback`) seed — intentional and correct, NOT this
bug.** TSKPC seeds IK from the previous `command_` (open-loop / feed-forward
integration) so the commanded trajectory advances at control rate regardless of
hardware following lag; seeding from `q_feedback` makes the command chase the
lagging feedback and can stall. The tracking guard + rate-limited resync is the
required safety net for this open-loop choice. This is unrelated to the limit
cycle (which was the missing `dt`). Keep `command_` seeding.

**Workaround (pre-fix):** use `pinocchio_dls` + `posture_weight: 1e-3`.

**Remaining (separate, optional):** MPC-202 velocity-servo / decimated QP rate
for cyclo-style feel; not required to remove the limit cycle.

**Status (2026-06-23): RESOLVED / validated.** After the `dt` fix and removal of
TSKPC `command_blend_alpha`, conservative OSQP (`position_gain: 4`,
`orientation_gain: 1`, `damping: 0.05`, `posture_weight: 1e-3`) passes fake-hardware
hold regression and **works well for marker teleop on Marvin real hardware**.
Active YAML: `src/apps/marvin_bringup/config/marvin_*_task_space.yaml` (`backend:
osqp`). PlaCo follow-up is tracked separately under MPC-208.

### MPC-208: PlaCo aggressive on Marvin teleop; real-hardware resync trip

**Observed (2026-06-23, Marvin real hardware, TSKPC left arm, `placo` backend,
`position_gain: 4`, `posture_weight: 1e-3`):**

- Subjective teleop: less smooth and more aggressive than `pinocchio_dls`.
- Fake-hardware static regression PASS (same params as DLS).
- Real-hardware failure chain:
  1. `command_` diverges from measured feedback during teleop.
  2. TSKPC tracking guard fires at `max_command_tracking_error_rad: 0.30`.
  3. Resync writes `command = feedback` in **one** 500 Hz cycle (no rate limit).
  4. Marvin `write()` rejects left J1 step `0.0293 > 0.0126` rad
     (`~6.28 rad/s` per-cycle cap) → `on_error` → controller deactivated.

**Why PlaCo felt hotter than DLS (pre-2026-06-23 wrapper):**

- Old wrapper used `add_regularization_task(posture_weight)` — minimises ‖Δq‖,
  not pull toward `q_neutral` (see `docs/PLACO_INTEGRATION.md`).
- No post-solve joint velocity clamp (DLS has one).
- `damping` unused; no manipulability task.

**Wrapper update (2026-06-23):** `JointsTask` posture → `q_neutral`,
`RegularizationTask` ← `damping²`, optional `manipulability_weight`, post-solve
joint velocity clamp, `test_placo_solver`. Re-validate on Marvin real hardware.

**Remaining gaps vs DLS:**

- Cartesian clamp is approximate (task-weight scaling, not identical to DLS `v_desired`).
- TSKPC still integrates from `command_` (MPC-202).

**Follow-up fixes and current PlaCo formulation:**

- TSKPC **rate-limited** tracking resync (no single-cycle 0.30 rad jump).
- PlaCo: `JointsTask` posture toward `q_neutral`, `RegularizationTask` from
  `damping²`, optional `KineticEnergyRegularizationTask`, optional
  `ManipulabilityTask`, and per-joint `joint_motion_weights`.
- PlaCo: removed `scaled_task_weight`; `position_gain` / `orientation_gain` are
  fixed soft-task weights. Cartesian velocity limits are not approximated through
  task-weight scaling.
- PlaCo: current backend relies on native joint-position / joint-velocity limits
  plus the final defensive joint-velocity clamp. `max_joint_acceleration_rad_s2`
  is parsed by shared config but is not wired into this backend.

**Workaround until re-validated:** `pinocchio_dls` or validated **OSQP** for production
teleop (MPC-207).

**2026-06-23 PlaCo re-test** (`command_blend_alpha` removed, earlier experimental
smoothness tuning):

| Aspect | Result |
|--------|--------|
| Fake-hardware hold (`task_left`) | **PASS** — solve 500 Hz, task error ~10⁻¹⁶ m |
| Real-hardware smoothness | Improved in experiment; current branch uses damping/kinetic regularization instead |
| Real-hardware overshoot | **Open** — arm passes the marker goal and oscillates/settles late |

**Overshoot hypothesis:** the PlaCo wrapper sets soft task targets to the **full
marker pose** each 500 Hz cycle. The QP closes the whole Cartesian error while
the native joint velocity limit can drive large steps until the QP optimum
changes sign. OSQP does not show this because its single-step `dq` is explicitly
`dt`-scaled (MPC-207); DLS integrates proportional `v·dt` per cycle.

**Overshoot status (2026-06-30): hardware gate only.**

Prior real-hardware session (2026-06-23) reported overshoot with an earlier PlaCo
wrapper. Current branch uses the final PlaCo-native weighted task stack described
above and Marvin tuning should be validated in one attended real-hardware marker
teleop session. **Workaround until hardware gate:** OSQP / `pinocchio_dls` for
production teleop.

**Fix direction (historical):**

- Rate-limit tracking-error resync (Marvin `max_step_rad`).
- MPC-202 velocity-servo / feedback integration for QP backends.

### MPC-104: Near-zero twist behavior

Current issue: for gamepad-style jogging, physical stick drift can produce
small residual twist.

Status: the DLS posture task is now applied through the *damped* null-space
projector (`qdot += (I − J⁺J)·z`, `J⁺ = (JᵀJ + λ²I)⁻¹Jᵀ`). This is materially
better than the previous bare additive posture term, but with `λ > 0` it is an
approximate projector: `J·N` is small but not identically zero, so a little
posture motion leaks into the task. Leakage scales with `λ²·posture_weight` and
→ 0 as `λ → 0`. For a non-redundant arm (square Jacobian) the null space is
empty and posture has ~no effect by construction.

Status: `test_dls_solver.PostureDoesNotMateriallyPerturbTask` bounds tip motion
(< 1 mm) at zero task error with `posture_weight: 1e-3` on a 3R fixture.

Target:
- Extend leakage bounds to Marvin-scale 7-DOF / non-root `base_frame` if needed.
- Treat device deadzone/zero-latch as source-layer responsibility.
- In TSKPC, optionally freeze the posture pull when twist velocity is below a
  configured threshold (opt-in, well documented).

### MPC-105: EMA-to-control_toolbox low-pass migration

Current state: `limiter` mode in JointSpacePositionController smooths the
sampled reference with a hand-written EMA (`joint_space::apply_ema`) whose
`ema_alpha` is a per-sample coefficient. Its effective cutoff therefore drifts
with the controller update rate — the same `ema_alpha` behaves differently at
500 Hz vs 1 kHz.

Target:
- Replace `apply_ema` with `control_toolbox::LowPassFilter<double>` (one per
  joint), parameterized by sampling frequency (= controller update rate) and a
  cutoff/damping frequency, so the smoothing time constant is rate-independent.
- This is a public-config change (`reference_behavior.ema_alpha` →
  cutoff-frequency-based parameters) and a behavior change, so it should land as
  its own reviewed + build-validated commit rather than bundled with unrelated
  fixes. control_toolbox is already a dependency.

### MPC-107: Hybrid reference-behavior mode

Current state: `reference_behavior.mode` (`limiter` / `ruckig`) is a per-instance
config choice and cannot change per message. The two streaming sources we target
favor different stages:

- Sparse single-point setpoints (slower than the control rate) are smoothed well
  by `ruckig` but produce a zero-order-hold staircase under `limiter`.
- MPC-style multi-point chunks are tracked faithfully by `limiter` +
  `sample_reference()` cubic-Hermite sub-interpolation, while `ruckig` re-shapes
  them through a second OTG (added lag, may clip aggressive moves).

A deployment that receives both kinds on one topic currently has to standardize
the upstream onto short multi-point trajectories (limiter) or accept ZOH jitter
for setpoints (limiter) / extra lag for chunks (ruckig).

Target:
- Add an optional `hybrid` mode that branches on point count at runtime: drive
  single-point references through Ruckig and multi-point references through the
  `sample_reference()` interpolator (limiter clamp as safety). One controller,
  one topic, both source shapes.
- Keep `limiter` and `ruckig` as explicit modes; `hybrid` is opt-in.

## Future Features

### MPC-201: Body/tool-frame twist integration

Current contract: TSKPC twist mode assumes base-frame spatial twist:
`TwistStamped.header.frame_id == base_frame`.

Future direction:
- Add explicit `twist_reference: base|tool` semantics if needed.
- Keep frame conversion/calibration outside the controller; only integrate
  twist according to a declared convention.

### MPC-202: Constrained differential IK controller mode

Current contract: TwistStamped follows
`twist -> integrated target pose -> existing pose IK path`.

Future direction:
- Add a separate constrained velocity-servo mode such as
  `twist -> QP over dq/q_dot -> joint target`.
- Candidate constraints: joint position, velocity, acceleration, singularity or
  manipulability costs, posture/null-space cost.
- Do not replace the current integrated-pose path without measurement.

### MPC-203: Ruckig segment executor and richer trajectory semantics

Current contract: Ruckig is used as an online setpoint filter inside
`JointSpacePositionController`.

Future direction:
- Add segment-by-segment Ruckig execution where waypoints carry endpoint
  velocity/acceleration constraints.
- Support offline full-trajectory smoothing for planned trajectories if that
  belongs below the execution manager.

### MPC-204: Multi-EEF and whole-body task allocation

Current contract: TSKPC is single-arm/single-EEF for the Piper use case.

Future direction:
- Multiple named task targets, task weights, conflict resolution, and
  priority-aware IK.
- Likely requires a typed task message or named-topic contract rather than
  relying only on PoseArray ordering.

### MPC-205: Dynamics controller family

Reserved future package area:
`TaskSpaceDynamicsPositionController` for TSID/MPC/impedance/admittance-style
position-interface controllers. Torque/effort may be an internal adjoint
variable, but the public command interface remains a repo-level design choice.

Design intent (contact handling): outward force is realized through motion. The
planned mechanism is a virtual reference produced by impedance dynamics
(`position + impedance`), kept on the position interface. The current
kinematic controllers intentionally command stiff position only and assume
free-space / quasi-static contact; compliant-contact behavior is deferred to
this family.

### MPC-206: High-order command output

Current state: both controllers compute joint velocity (DLS `qdot`, Ruckig
velocity/acceleration) internally but write only the position command
interface. The target robots expose a high-rate, position-only command
interface, so velocity/acceleration setpoints would be ignored by the hardware
plugin today.

Future direction:
- When a robot's `ros2_control` hardware interface gains real velocity /
  acceleration handling, expose optional `velocity` (and later `acceleration`)
  command interfaces and feed the already-computed high-order terms as
  feed-forward to cut the downstream differentiation lag.
- Keep position as the always-present interface; high-order outputs are opt-in.

## Known Non-Issues / Boundaries

- Controllers consume clean references; they do not decode raw policy vectors,
  joystick packets, VR device frames, or planner internals.
- Controllers do not perform arbitrary TF lookup in the realtime update loop.
  Task-space targets must already be expressed in `base_frame`. The solver
  expresses FK/Jacobian relative to `base_frame` (so any URDF frame may be the
  base), but it does not convert references from other frames.
- `manipulation_execution_manager` is not a compile/runtime dependency of this
  repo; integration is through ROS topics and message contracts.
- Task-space acceleration/jerk limiting: the DLS path is a proportional-error +
  per-cycle velocity clamp, i.e. a first-order law. For streaming references
  (twist jog, smooth pose chunks) acceleration is implicitly bounded by how
  fast the task error can change, which is the primary use case. Large discrete
  pose steps (a jumped `PoseStamped`, a chunk-boundary discontinuity) are NOT
  jerk-bounded — the onset is limited only by the joint-velocity clamp. If a
  use case needs bounded jerk on discrete steps, use the OSQP backend or smooth
  the input upstream rather than adding a task-space OTG stage here.
