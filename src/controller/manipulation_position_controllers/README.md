# Manipulation Position Controllers

Generic `ros2_control` `ControllerInterface` plugins for manipulator position
control. ROS 2 Jazzy.

## Repository layout

This repository is a standalone ROS 2 package: `package.xml` and
`CMakeLists.txt` live at the repository root. It can therefore be cloned
directly into the `src/` directory of any ROS 2 workspace, or tracked there as
a Git submodule.

## Build and test

From the ROS 2 workspace root:

```bash
colcon build --symlink-install --packages-select manipulation_position_controllers
colcon test --packages-select manipulation_position_controllers
colcon test-result --verbose
```

The controllers are pluginlib plugins loaded by `controller_manager`; this
package intentionally does not prescribe a robot-specific bringup launch file.
Use the YAML files under `config/` as templates in the consuming robot's
`ros2_control` configuration.

Development-only publishers, bridges, and fault-injection programs live under
`examples/`. They are intentionally not installed as runtime executables and
may require additional ROS packages. Robot-specific launch and hardware
configuration belong in the consuming robot's bringup package.

## Controllers

### JointSpacePositionController

Consumes `trajectory_msgs/JointTrajectory` on the configured `input_topic`.
Supports single-target, receding chunk, or short trajectory windows.

**Reference behaviors** (selectable per controller instance):

| Mode | Description |
|------|-------------|
| `limiter` | EMA filter + per-joint velocity/acceleration limits |
| `ruckig` | Online jerk-bounded OTG setpoint filter (Ruckig) |

**Key parameters**:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `joints` | `[]` | List of joint names |
| `input_topic` | `/execution/joint_reference` | Subscribed JointTrajectory topic |
| `input_qos_depth` | `10` | Subscription queue depth for `input_topic` (QoS is best-effort, fixed) |
| `reject_zero_stamped_references` | `true` | Reject references without a source timestamp |
| `trajectory_behavior.mode` | `limiter` | `limiter` or `ruckig` |
| `trajectory_behavior.stale_timeout_s` | `0.5` | Switch to measured-state hold when no new reference |
| `trajectory_behavior.max_velocity_rad_s` | `1.5` | Per-joint velocity limit |
| `trajectory_behavior.max_acceleration_rad_s2` | `3.0` | Per-joint acceleration limit |
| `trajectory_behavior.ema_alpha` | `0.2` | EMA smoothing (limiter mode) |
| `trajectory_behavior.max_jerk_rad_s3` | `10.0` | Jerk limit (ruckig mode) |
| `allow_partial_joint_references` | `true` | Accept trajectories with subset of joints |
| `lower_limits` / `upper_limits` | `[]` | Position limits in rad. Empty = unbounded |
| `status_rate_hz` | `10.0` | Publish rate for `~/commanded_joint_state` and `~/status` (observability only) |

See `config/joint_space_position_controller.yaml` for the full annotated
configuration.

#### Connecting streaming sources to a 500–1000 Hz JSPC

JSPC only consumes `trajectory_msgs/JointTrajectory`. Both common streaming
sources map onto the *same* topic and type — only the **point count** differs,
and `sample_reference()` adapts:

- **1 point** → zero-order hold (a staircase) between message arrivals.
- **N points** → sub-interpolated at the control rate between waypoints:
  cubic Hermite when velocities are present, otherwise positions plus
  finite-differenced velocities. Point times are absolute:
  `point_time = header.stamp + time_from_start`, compared in the controller
  clock domain. By default, `header.stamp == 0` is rejected.

The `mode` is a per-instance config choice — it cannot be switched per message.

| Source | Publish as | Mode | Notes |
|--------|-----------|------|-------|
| Single setpoint slower than control rate (RL policy, teleop) | 1-point trajectory, positions only | `ruckig` | Ruckig synthesizes the jerk-bounded continuous motion toward each setpoint — this is the only stage that smooths a sparse staircase. |
| MPC-style chunk (receding horizon, N waypoints, replanned) | N-point trajectory with `time_from_start`, ideally `velocities` | `limiter` with `ema_alpha ≈ 1.0` | Track the planned trajectory faithfully via Hermite sub-interpolation; the rate limiter acts only as a safety clamp (set at the true joint limits). Avoids stacking a second OTG (Ruckig) on top of the MPC. |

Gotchas for the chunk path:

- Set `max_reference_points` ≥ your chunk length, or tail waypoints are dropped
  (default 32).
- Time sync matters: stamp `header.stamp` with the plan's reference start in the
  controller clock domain. Zero stamps are rejected by default so transport
  latency cannot be hidden by anchoring a plan at receive time.
- Replan from the current measured state — the controller holds latest-wins and
  does not blend across chunks (it only reports the preemption-jump norm).
- Output is position-only; provided velocities shape the interpolation but are
  not commanded (see MPC-206 in `ISSUES.md`).

If a single deployment must accept *both* sparse setpoints and chunks, either
standardize the upstream onto short multi-point trajectories (even a 2-point
micro-segment for setpoints) and use `limiter`, or track the hybrid-mode item
(MPC-107) in `ISSUES.md`.

### JointSpaceImpedanceController

Joint × Dynamic path: same external `JointTrajectory` contract as
`JointSpacePositionController`, but commands `<joint>/effort` with a joint
impedance law:

```text
tau = Kp (q_ref - q) + Kd (qdot_ref - dq_filt)
```

`q_ref` / `qdot_ref` come from Ruckig (or EMA limiter). Measured velocity is
low-pass filtered via `velocity_filter_alpha` (measurement weight in
`(1-α)·prev + α·meas`; `1.0` = no filter). Prefer this
controller on Franka FCI teleop / sparse setpoints — the position
motion-generator interface is far more brittle.

**Absorbed from community stacks**: required per-joint `kp_stiffness` /
`kd_damping` / `max_torques`, measured-velocity LPF
(`velocity_filter_alpha`), torque rate limiting, soft stale hold (pin
`q_ref` to measured `q`), and activate seed `q_ref = q_meas` with no hard
snap on the first reference. Leader–follower **SYNCING** from
`franka_follower_controllers` is intentionally **not** used — marker/planner
already initialize from the current end-effector pose. Gravity compensation
is assumed from hardware (e.g. libfranka); Coriolis feed-forward is deferred.

See `config/joint_space_impedance_controller.yaml`. Robot-specific bringup
(e.g. Franka FR3 joint names, torque caps) belongs in the
consuming app package, not this template.

### TaskSpaceKinematicPositionController

Exactly two command inputs (JSPC parity for pose):

| Topic param | Type | Role |
|-------------|------|------|
| `input_topic` | `moveit_msgs/CartesianTrajectory` | Pose targets; **1 point ≡ former PoseStamped**, N = horizon |
| `twist_topic` | `geometry_msgs/TwistStamped` | Spatial velocity (optional; empty = disabled) |

Solves IK via Pinocchio at the controller rate.

**Solver backends**:

| Backend | Use Case |
|---------|----------|
| `pinocchio_dls` | Streaming jog/teleop (smooth, deterministic) |
| `osqp` | Pose reaching with joint-limit CBF constraints |
| `placo` | Alternative QP solver |

> **Solver guidance**: DLS is the recommended default for streaming
> (jog/teleop) applications — its analytic solution is smooth and
> deterministic. QP-based solvers (OSQP, Placo) are better suited for
> discrete-pose reaching tasks where joint-limit constraint satisfaction
> matters more than per-cycle smoothness.

**Input priority**: pose trajectory > TwistStamped

**Present trajectory consume strategy**: **first point only** (whether the
message has 1 or N points)—JSPC-aligned simple streaming consume. Freshness is
`receive_time + stale_timeout_s`.

All four controllers use `trajectory_behavior.*` for trajectory ingestion and
output-shaping parameters. Joint-space controllers additionally expose
`mode: limiter|ruckig`.
Documented future sketches (see Physical AI Runtime
`docs/RUNTIME_ORCHESTRATION.md`): stamp the received trajectory, then
time-sample with `sample_pose_chunk` / `chunk_is_fresh`, or more complex
look-ahead sampling. Today the namespace mainly holds ingest knobs
(`max_points`, `untimed_frame_dt_s`).

**Stale handling**: under **first_point**, a trajectory is fresh while
`now - receive_time ≤ stale_timeout_s`, then STALE_HOLD (same family as a
single pose tick). TwistStamped uses `twist_stale_timeout_s`. Unstamped
headers (`stamp == 0`) are rejected by default. A future
`sample` mode would instead keep the chunk active through its last frame
time plus grace (`chunk_is_fresh`).

**Frame contract**: all task-space inputs must be expressed in `base_frame`
(`header.frame_id == base_frame`); mismatched frames are rejected, not
transformed. `base_frame` may be any frame present in the URDF — the solver
expresses FK and the Jacobian relative to it, so it need not be the URDF root.

**TwistStamped** inputs are integrated into an internal base-frame pose
target, then solved through the same IK path. The integrated pose is
re-seeded from FK each time the controller transitions into twist mode,
so each jog session starts from the current end-effector pose.

**Key parameters**:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `joints` | `[]` | List of joint names |
| `base_frame` | `base_link` | Controller base frame |
| `tip_frame` | `flange_link` | End-effector frame |
| `input_topic` | `/execution/pose_reference` | CartesianTrajectory pose input |
| `reject_zero_stamped_references` | `true` | Reject unstamped trajectory and twist inputs |
| `trajectory_behavior.max_points` | `64` | Max accepted points (capped by internal horizon) |
| `trajectory_behavior.untimed_frame_dt_s` | `0.02` | Frame spacing for untimed (all-zero `time_from_start`) trajectories |
| `twist_topic` | (empty) | TwistStamped input. Empty = twist disabled |
| `stale_timeout_s` | `0.5` | Pose stale hold timeout |
| `twist_stale_timeout_s` | `0.2` | Twist stale hold timeout |
| `solver.backend` | `pinocchio_dls` | IK solver |
| `solver.position_gain` | `4.0` | Position error gain (PlaCo: soft-task weight) |
| `solver.orientation_gain` | `1.0` | Orientation error gain (PlaCo: soft-task weight) |
| `solver.damping` | `0.05` | DLS damping factor |
| `solver.posture_weight` | `1e-3` | Posture soft-task weight. Set to 0 to disable |
| `solver.osqp.max_admm_iterations` | `200` | OSQP-only: ADMM convergence iteration budget per solve |
| `solver.osqp.cbf_gain` | `2.0` | OSQP-only: joint-limit CBF approach gain. Robot- and limit-margin-dependent, tune per robot |
| `solver.osqp.slack_penalty` | `1e4` | OSQP-only: penalty on CBF slack variables |
| `solver.osqp.abs_tolerance` / `solver.osqp.rel_tolerance` | `1e-6` / `1e-6` | OSQP-only: solver convergence tolerances |
| `solver.osqp.rho` | `0.1` | OSQP-only: fixed ADMM step-size parameter |
| `max_linear_velocity_m_s` | `0.5` | Cartesian linear velocity clamp for DLS/OSQP backends |
| `max_angular_velocity_rad_s` | `1.0` | Cartesian angular velocity clamp for DLS/OSQP backends |
| `max_joint_velocity_rad_s` | `1.0` | Per-joint velocity limit |
| `lower_limits` / `upper_limits` | `[]` | Joint limits in rad |

See `config/task_space_kinematic_position_controller.yaml` for the full
annotated configuration.

### TaskSpaceJointImpedanceController

Task pose → Diff-IK → joint-space impedance → `<joint>/effort`. Prefer this on
Franka FCI for marker / teleop pose tracking. Distinct from a future
`TaskSpaceCartesianImpedanceController` (operational-space / Cartesian
stiffness; not implemented).

See `config/task_space_joint_impedance_controller.yaml`.

A Diff-IK → `<joint>/velocity` prototype was evaluated on Franka velocity MG
and is **not shipped** (FCI continuity vs software Δv bounds). Use effort.

## Observability

Task-space kinematic position controller publishes rate-limited status arrays.
TSKPC publishes a 19-field `Float64MultiArray` including
`active_input_mode` (0=none, 1=chunk, 2=pose, 3=twist), twist age, velocity
norms, IK success/fail counters, and frame-reject counters.

All four controllers also publish standard `diagnostic_msgs/DiagnosticArray`
entries through `/diagnostics`. The diagnostic entry is named
`<controller_fqn>: safety` and contains stable machine-readable values:

- `safety_state`: `IDLE`, `TRACKING`, `SAFE_HOLD`, or `ERROR`;
- `fault_code`: for example `REFERENCE_TIMEOUT`, `ZERO_STAMP_REFERENCE`,
  `FRAME_MISMATCH`, `NONFINITE_REFERENCE`, or `INVALID_MEASURED_STATE`;
- `action`: `MEASURED_STATE_HOLD` after a recoverable rejection, or
  `RETURN_ERROR` when the controller cannot construct a safe measured-state
  hold;
- `fault_sequence`, `last_fault_code`, `observed_ms`, and `threshold_ms` for
  correlation and timeout debugging. `last_fault_code` survives recovery to
  `TRACKING`, so a short fault is not hidden by the next healthy snapshot.

Diagnostics are emitted from a non-realtime callback. The controller update
loop only changes fixed-size atomic state; it performs no diagnostic DDS
publish or string construction. A successful measured-state hold therefore
keeps `update()` operational while `/diagnostics` retains the cause instead of
reporting an ambiguous OK.

Production bringup profiles intentionally do not configure a JTC as a
manipulation controller fallback. Recoverable reference faults stay inside the
active controller and hold measured state without a controller switch. An
unrecoverable `update()` error means measured state is unavailable or invalid;
Controller Manager may deactivate the failed controller, but switching to a
JTC that depends on the same measured state would not restore safety. JTC
goals use the separate RT trajectory guard and JTC cancel-deceleration path.

## Dependencies

- **Required**: `pinocchio`, `eigen`, `ruckig`, `control_toolbox`,
  `realtime_tools`, `controller_interface`, `hardware_interface`,
  `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `osqp`, `osqp-eigen`,
  `placo`

The source package keeps all three IK backends required so a build has
deterministic capabilities. A binary-only ROS 2 Jazzy package is also supported;
its recipe and exact runtime dependency policy are documented below.

## Release status

The Release build, solver/reference behavior, parameter fail-fast rules, and
effort deactivation safety are covered by tests. Binary publication is ready
for a clean-environment package build. Full controller-manager state-machine
coverage and target-hardware allocation/jitter measurements remain deployment
gates tracked in `ISSUES.md`; attended real-motion validation is still required.

For private binary distribution without installed sources or headers, see
[`docs/BINARY_DISTRIBUTION.md`](docs/BINARY_DISTRIBUTION.md).

For FR3/FER FCI limits, Marvin bringup defaults, and how they map onto JSPC
Ruckig parameters, see [`docs/ROBOT_SPECIFIC.md`](docs/ROBOT_SPECIFIC.md).

## License

Apache-2.0
