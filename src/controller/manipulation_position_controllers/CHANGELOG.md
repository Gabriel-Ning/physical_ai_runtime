# Changelog

All notable changes to `manipulation_position_controllers` are recorded here.

## [Unreleased]

## [0.3.1] - 2026-08-16

### Changed

- Controllers advance OTG / Diff-IK from the controller-manager period only.
  The optional `robot_time_interface` GPIO claim is removed so fake hardware
  does not need a second controller YAML just to omit that interface.

## [0.3.0] - 2026-08-07

### Breaking

- **Target A pose inputs:** task-space controllers accept exactly two command
  topics — `input_topic` (`moveit_msgs/CartesianTrajectory`, 1 or N points)
  and optional `twist_topic` (`TwistStamped`). Removed `PoseStamped`
  subscription, `pose_topic` / `pose_sequence_topic` / `pose_chunk_topic`, and
  `PoseArray` support.
- Trajectory params live under `trajectory_behavior.max_points` and
  `trajectory_behavior.untimed_frame_dt_s` (JSPC `reference_behavior`
  analogue namespace).
- **Present consume strategy:** always command **first trajectory point**
  only (1- or N-point messages). Horizon look-ahead / timed play is reserved
  for future `trajectory_behavior` modes. Freshness uses
  `receive_time + stale_timeout`. Priority: trajectory > twist.

## [0.2.3] - 2026-07-29

### Added

- Give `TaskSpaceJointImpedanceController` the same pose, pose-chunk, and
  base-frame twist input contract as `TaskSpaceKinematicPositionController`,
  including freshness handling and chunk/pose/twist precedence.

### Fixed

- Pin the binary build to the compatible `libosqp 1.0.0` and
  `osqp-eigen 0.10.3` pair.
- Rewrite the versioned OsqpEigen SONAME to the package-private runtime name
  and assert the final ELF dependency graph during package tests.

## [0.2.2] - 2026-07-29

### Fixed

- Isolate the binary package's OSQP 1.x and OsqpEigen runtime libraries behind
  private names and RPATH. This prevents MoveIt's older
  `ros-jazzy-osqp-vendor` library from replacing the ABI required by the
  controller when both packages are installed in one Prefix environment.

## [0.2.1] - 2026-07-29

### Fixed

- Re-prime the optional Franka robot-time clock when it moves backwards, so
  joint-space and task-space impedance controllers resume cleanly after a
  hardware/controller restart instead of waiting on a stale timestamp.

## [0.2.0] - 2026-07-26

### Added

- **`JointSpaceImpedanceController`** (Joint × Dynamic): `JointTrajectory` →
  Ruckig/EMA shaping → joint impedance `τ = Kp e + Kd ė` → `<joint>/effort`.
- **`TaskSpaceJointImpedanceController`** (Task input × joint impedance):
  `PoseStamped` → Diff-IK (OSQP/PlaCo/DLS) seeded from measured `q` → joint
  impedance → `<joint>/effort`. Prefer this over position/velocity MG on
  Franka FCI for pose teleop.
- Optional **`robot_time_interface`** on JSPC / JSIC / TSJI: advance OTG /
  Diff-IK from hardware clock deltas (e.g. `fr3/robot_time`) instead of CM
  `period` jitter.
- Package layout by taxonomy: `joint_space/{kinematics,dynamics}`,
  `task_space/{kinematics,dynamics}`.
- Reference YAML templates for JSIC and TSJI; Franka-specific values belong
  in consuming bringup packages.
- Binary Prefix recipe updated for four controllers / four YAML files;
  `build.sh` strips headers after install.

### Changed

- Documented `velocity_filter_alpha` as **measurement weight**
  `(1-α)·prev + α·meas` (1.0 = no filter). Default `0.99` unchanged.
- TSJI status publish decimation uses `get_update_rate()`; `update()` uses a
  preallocated `q_meas_` buffer (no per-cycle `std::vector` alloc).
- JSIC clamps `dtau` like TSJI when advancing torque rate limits.

### Removed

- Diff-IK → `<joint>/velocity` prototype (`TaskSpaceKinematicVelocityController`)
  is not shipped. Prefer effort paths on Franka FCI.
- Temporary JSPC RT ring-buffer `/tmp` dump used during discontinuity debug.

## [Unreleased]

### Fixed (low-severity: sentinel checks, cross-backend dt floor, QoS depth)

- **OSQP joint-limit sentinel check used an unexplained `1e9` threshold.**
  `q_lb <= -1e9` / `q_ub >= 1e9` decided whether to fall back to the URDF
  model's position limits. Replaced with `q_lb <= -OsqpEigen::INFTY` /
  `q_ub >= OsqpEigen::INFTY`: the "no limit" sentinel reaching this code can
  be either true IEEE infinity (from the controller's
  `lower_limits_`/`upper_limits_` defaults) or OSQP's own finite
  pseudo-infinity (`OsqpEigen::INFTY` = `OSQP_INFTY`, ~1e30 -- OSQP's C API
  bound arrays have no representation for real IEEE infinity), which is also
  why plain `std::isinf()` would have been an incorrect fix here: it would
  silently stop catching the OSQP-sentinel case.
- **Solver `period`/`dt` floor was inconsistent across backends: DLS applied
  none, OSQP floored at `1e-9`, PlaCo floored at `1e-6`.** DLS multiplying an
  unclamped `period` by `qdot` is harmless today (controller_manager always
  passes a non-negative duration) but was one un-defended edge away from the
  other two backends' behavior. Added a shared `kMinSolverDtS = 1e-6`
  constant to `solver_interface.hpp` and applied it uniformly in DLS
  (new), OSQP (was `1e-9`), and PlaCo (already `1e-6`, unchanged).
- **JSPC input subscription QoS depth was a hardcoded `KeepLast(10)`.**
  Added `input_qos_depth` parameter (default `10`, validated `> 0`,
  no behavior change at default). The `best_effort()` reliability policy is
  intentionally left fixed -- it is the deliberate choice for a streaming
  realtime reference input, not a per-deployment tuning knob.
- Reviewed but left unchanged: `~/commanded_joint_state` and `~/status`
  output topic names on both controllers. These use ROS 2's private (`~/`)
  topic convention -- already independently remappable via standard ROS 2
  remapping/namespacing without a controller parameter, matching the pattern
  used by `joint_state_broadcaster` and other ros2_control controllers.
  Adding a YAML parameter for them would duplicate an existing ROS mechanism.

### Fixed (JSPC observability decimation hardcoded, no parameter path)

- **JSPC `publish_decimation_` fixed at 10 with no YAML parameter.**
  `~/commanded_joint_state` and `~/status` were decimated by a
  default-member-initializer `publish_decimation_{10}` that `on_configure()`
  never touched -- there was no `status_rate_hz`-equivalent parameter at all
  (unlike TSKPC), so the effective publish rate was an unconfigurable,
  update-rate-dependent side effect (e.g. 50 Hz at a 500 Hz loop, silently
  different at any other loop rate). Added a declared `status_rate_hz`
  parameter (default `10.0`, matching TSKPC's default and validated `> 0`)
  and compute `publish_decimation_` from it and `get_update_rate()` in
  `on_configure()`, mirroring `TaskSpaceKinematicPositionController`.
  Behavior change: the default observability publish rate at a 500 Hz loop
  moves from an implicit 50 Hz to an explicit, TSKPC-consistent 10 Hz.

### Fixed (OSQP backend solver-internals had no YAML parameter path)

- **OSQP ADMM iteration budget, CBF gain, slack penalty, and
  tolerances/rho were compile-time constants.** `kAdmmMaxIter` (200),
  `kCbfGain` (2.0), and `kSlackPenalty` (1.0e4) were `static constexpr` in
  `osqp_solver.hpp`, and `setAbsoluteTolerance`/`setRelativeTolerance`/`setRho`
  were hardcoded literals in `configure()` -- none had a YAML parameter path,
  and `solver.max_iterations` (the one existing knob, used by PlaCo for outer
  time-substeps) was silently ignored by this backend entirely. `cbf_gain` in
  particular is robot- and joint-limit-margin-dependent, so one hardcoded
  default across every robot was never going to be right for all of them.
  Added `SolverConfig.osqp_*` fields (`osqp_max_admm_iterations`,
  `osqp_cbf_gain`, `osqp_slack_penalty`, `osqp_abs_tolerance`,
  `osqp_rel_tolerance`, `osqp_rho`) with defaults matching the previous
  hardcoded values (no behavior change at defaults), wired through new
  `solver.osqp.*` TSKPC parameters, validated `> 0` in
  `validate_configuration()` when `solver.backend == "osqp"`.

### Fixed (magic numbers reviewed; one promoted, one left as an epsilon)

- **TSKPC tracking-resync completion tolerance.** `1.0e-4` (rad) in
  `update()`'s rate-limited tracking-resync path is a numerical convergence
  epsilon (radians are already robot-scale-invariant), not a per-robot tuning
  knob -- promoted to a named `kTrackingResyncDoneToleranceRad` constant with
  a comment explaining why it stays a compile-time constant instead of a YAML
  parameter, rather than leaving it as a bare literal.

### Fixed

- **Pose-chunk inter-frame timestep hardcoded 50 Hz.** `pose_chunk_callback()`
  built each `PoseArray` frame's `time_s` as `receive_time + i * 0.02`, i.e. it
  assumed every chunk publisher sends frames spaced exactly 0.02 s (50 Hz)
  apart. `geometry_msgs/PoseArray` carries no per-pose timestamp, so this
  constant was the *only* source of intra-chunk timing; a publisher at any
  other rate (10 Hz, 25 Hz, ...) had every frame in the chunk timed wrong,
  corrupting `sample_pose_chunk()`'s interpolation/IK timing silently (no
  error, no warning). Added a declared parameter `pose_chunk_frame_dt_s`
  (default `0.02`, validated `> 0` in `validate_configuration()`) so the
  spacing is explicit and configurable instead of a silent literal.

- **TSKPC status/command-state decimation hardcoded 500 Hz.**
  `status_decimation_` (drives both `~/status` and `~/commanded_joint_state`
  publish rate limiting) computed its divisor from a hardcoded `500.0`
  controller-loop assumption instead of the controller's actual update rate.
  On a `controller_manager` running at any rate other than 500 Hz,
  `status_rate_hz` silently published at the wrong frequency. Now uses
  `get_update_rate()`, falling back to 500 Hz only if the rate is unavailable
  -- the same pattern already used for JSPC's Ruckig `control_cycle_s_` fix.
- **Dead `ema_alpha` parameter in TSKPC config templates.**
  `ema_alpha` belongs only to `JointSpacePositionController`;
  `TaskSpaceKinematicPositionController` never declares or reads it. Removed
  the stale `ema_alpha: 0.2` line from `config/task_space_kinematic_position_controller.yaml`
  (the file this package's binary distribution ships and the README points to
  as "the full annotated configuration") and from four `examples/config/`
  templates (`task_space_controller_test.yaml`, `_chunk.yaml`, `_dls.yaml`,
  `_osqp.yaml`); `_twist_test.yaml` was already clean.

### Release packaging

- Flattened the repository so it is directly cloneable into a ROS 2 workspace
  as one package.
- Added Apache-2.0 license text, public repository metadata, and compiler
  warnings.
- Moved fake publishers, bridges, fault injection, and the IK-to-JTC A/B node
  together with test-only YAML into a non-installed `examples/` tree. The
  installed runtime surface now contains only the controller plugin library,
  headers, plugin description, and two reference configurations.
- Revalidated the flattened package in Release mode: 72 tests passed.
- Added a rattler-build runtime-only recipe for private Prefix distribution.
  The binary package excludes headers, sources, examples, tests, and CMake
  development exports while retaining ROS plugin discovery and configuration.

### Changed (PlaCo task-stack formulation)

- **PlaCo now uses a PlaCo-native Pink/Mink-style weighted task stack.**
  `PositionTask` and `OrientationTask` track the full Cartesian target with
  fixed soft weights from `position_gain` / `orientation_gain`; `JointsTask`
  provides a small posture objective toward `q_neutral`; `RegularizationTask`,
  optional `KineticEnergyRegularizationTask`, and optional `ManipulabilityTask`
  shape smoothness and redundancy.
- **Removed PlaCo `scaled_task_weight`.** Cartesian velocity limits are no longer
  approximated by shrinking task weights as a function of pose error. PlaCo speed
  limiting is handled by native joint velocity limits plus the final defensive
  joint-velocity clamp.
- **Removed the experimental PlaCo acceleration constraint path.**
  `solver.max_joint_acceleration_rad_s2` is still parsed in shared config, but
  the current PlaCo backend does not wire it into the QP. Tune PlaCo smoothness
  with `damping`, `joint_motion_weights`, and `kinetic_energy_weight`.

### Removed

- **`command_blend_alpha` (TSKPC).** Output-side first-order lag toward the solver
  command is removed. Smoothness belongs in solver dynamics and
  `max_joint_velocity_rad_s` step clamping; PlaCo additionally exposes damping,
  per-joint regularization, and kinetic-energy regularization.

### Fixed (OSQP timestep integration — stationary-target limit cycle)

- **OSQP missing `·dt`.** `osqp_solver.hpp` built the QP cost from the Cartesian
  velocity `v_desired = gain·error` while the decision variable `dq` is a
  position increment, so it solved `dq = J⁺·v_desired` — a per-cycle step ≈`1/dt`
  (≈500× at 500 Hz) larger than DLS, which integrates `dq = qdot·dt`. The
  over-correction was clamped to `v_max·dt` each cycle, producing a sustained
  period-2 limit cycle of amplitude ≈ `v_max·dt` (~0.002 rad) that appeared with
  a **stationary target** at startup. Fixed by scaling the QP gradient by `dt`
  (`dq = dt·qdot_DLS`), aligning OSQP with DLS. **OSQP gains now share DLS
  semantics** — use comparable `position_gain`/`orientation_gain`. Regression:
  `test_osqp_solver` (`SingleStepIncrementMatchesDls`, `StationaryTargetConverges`).
  See ISSUES MPC-207, `docs/OSQP_VS_CYCLO.md`.
- Confirmed (no change): the `command_` IK seed (vs `q_feedback`) is intentional
  open-loop/feed-forward integration and is **not** related to the limit cycle.

### Validated (Marvin TSKPC, 2026-06-23)

- **OSQP marker teleop** on Marvin real hardware with `position_gain: 4`,
  `orientation_gain: 1`, `damping: 0.05`, `posture_weight: 1e-3` — **works well**
  (subjective teleop; no stationary limit cycle). Active in
  `marvin_bringup/config/marvin_*_task_space.yaml`. TSKPC `command_blend_alpha`
  removed; smoothness from solver dynamics + `max_joint_velocity_rad_s` clamp only.

### Known (PlaCo Marvin teleop)

- **PlaCo** remains a hardware-validation item for MPC-208. The current branch
  uses full Cartesian targets, native joint velocity limits, fixed task weights,
  and soft `JointsTask` posture. Tune the position/orientation task-weight scale
  and kinetic-energy regularization on Marvin before treating PlaCo as the
  production teleop backend.

### Fixed

- **DLS null-space posture projection.** The DLS backend's posture/regularization
  task is now applied through an explicit null-space projection
  `qdot += (I − J⁺J)·z` (computed matrix-free as `z − J⁺(J·z)`, reusing the
  already-factored LDLT). Previously the posture velocity was added directly to
  the primary-task solution and only labeled "null-space", which pulled the
  end-effector off target.
- **Base-frame task poses (DLS / OSQP / PlaCo).** All task-space poses are now
  expressed relative to `base_frame`. Targets are transformed into the model
  root via `oMb` before the IK error/Jacobian step, and `compute_tip_pose()`
  returns the tip pose in `base_frame`. `base_frame` is validated against the
  URDF at `configure()` time. When `base_frame` is the URDF root this reduces to
  identity (no behavior change for the current Piper setup).
- **Ruckig control cycle (JointSpacePositionController).** Ruckig is now driven
  by the controller's real update rate (`get_update_rate()`) instead of the
  hardcoded `reference_behavior.ruckig_control_cycle_s` parameter, so the OTG
  time-parameterization matches the loop it actually runs in. The parameter
  remains only as a fallback when the rate is unavailable. A throttled warning
  fires if the live update period diverges from the configured cycle.
- **Pose-chunk staleness (TaskSpaceKinematicPositionController).** Pose chunks
  now follow the same hold-and-recover contract as pose/twist inputs: a chunk
  stays active through its own time horizon plus a `stale_timeout_s` grace
  window, then enters `STALE_HOLD` instead of holding its last frame forever.
- **Zero-stamp references (TaskSpaceKinematicPositionController).** Unstamped
  references (`header.stamp == 0`) on the pose, pose-chunk, and twist topics now
  fall back to receive time, so they are no longer treated as permanently stale.
  Mirrors the existing joint-space behavior.
- **Realtime data race (TaskSpaceKinematicPositionController).** Scalar counters
  and flags written from the subscriber thread and read in the realtime
  `update()`/publish path (frame/quaternion/zero-stamp reject counts, twist
  sequence, `target_initialized`) are now `std::atomic`.

### Documentation

- Recorded the `base_frame` contract and uniform stale handling in `README.md`.
- Added a "Connecting streaming sources to a 500–1000 Hz JSPC" guide to
  `README.md` (single-setpoint→ruckig, MPC-chunk→limiter, timing/velocity/
  `max_reference_points` gotchas) and tracked a future `hybrid` mode (MPC-107).
- `ISSUES.md`: added the deferred EMA→`control_toolbox::LowPassFilter` migration
  (MPC-105), high-order command-output as a future opt-in (MPC-206), the
  impedance/contact design intent (MPC-205), and the task-space jerk-limiting
  boundary.

### Review follow-ups

- Corrected the DLS posture comments and `ISSUES.md` (MPC-104) to describe the
  projector honestly: with `λ > 0` it is a *damped* null-space projector with
  bounded task leakage (∝ `λ²·posture_weight`, → 0 as `λ → 0`), not an exact
  projector. OSQP posture documented as a soft competing cost that can trade
  away primary accuracy if the weight is large.
- Removed the per-cycle heap allocations the posture block introduced in the DLS
  realtime path: `z`, `Jᵀ(J·z)`, and the projection are now preallocated in
  `configure()` and reused.
- `last_target_age_s_` (status `kTargetAgeS`) now folds in chunk age (from the
  chunk receive time), so chunk-only execution no longer reports an infinite
  target age.

### Tests (P1 from the review)

- Extracted the chunk freshness logic into a pure `task_space::chunk_is_fresh()`
  helper (in `pose_chunk_buffer.hpp`) so the staleness contract is unit-testable
  without a controller harness; the controller now calls it.
- Added `test_pose_chunk_buffer.cpp`: `chunk_is_fresh` (empty / within-horizon /
  grace-window / expired) and `sample_pose_chunk` (hold-first / hold-last /
  interpolation, unit-quaternion).
- Added `test_dls_solver.cpp` against an embedded 3R URDF with a **non-root**
  `base_frame`: configure validation (good / unknown base / unknown tip),
  solve↔compute_tip_pose frame consistency (zero error round-trip under the
  non-identity `oMb`), one-step IK direction (tip moves toward target), and a
  bounded damped-null-space posture-leakage check.

These tests pass in the ROS 2 Jazzy release build. Still not covered by
automated tests: controller-level chunk STALE_HOLD/recovery transitions,
zero-stamp fallback, and Ruckig update-rate selection (these need a
controller_manager harness).

### Deferred / not changed

- **EMA→control_toolbox migration (MPC-105).** The `limiter` mode still uses the
  hand-written rate-dependent EMA. Replacing it with
  `control_toolbox::LowPassFilter` is a public-config change and is tracked
  separately so it can land with build validation.

### Reference-smoothing behavior (clarification, no code change)

The `limiter` mode does **not** contain a time-based sub-interpolation /
online-trajectory-generation stage. The only interpolation in the joint-space
path is `sample_reference()`, which interpolates **within a single multi-point
`JointTrajectory`** (linear, or cubic Hermite when velocities are present).

Consequence for sparse streaming setpoints (e.g. 50/100 Hz single-point
targets):

- `sample_reference()` sees a 1-point reference and returns it as a zero-order
  hold (a staircase) between arrivals.
- `limiter` mode then applies only an EMA first-order lag + per-joint
  velocity/accel/jerk **clamp** on that staircase — no velocity-continuous
  resampling — which shows up as ripple/jitter at the input rate.
- `ruckig` mode re-plans a jerk-bounded, velocity/acceleration-continuous
  profile toward each new target every control cycle, so it produces smooth
  motion from the same sparse setpoints.

Guidance: for sparse streaming setpoints, use `ruckig` mode. Use `limiter` mode
when the upstream already sends dense/smooth references, or multi-point
trajectories that `sample_reference()` can interpolate.
