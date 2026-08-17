# Robot-specific limits and controller mapping

This package is robot-agnostic. Downstream bringups must set position,
velocity, acceleration, and (for Ruckig) jerk limits from the robot vendor's
control interface specification — not from the YAML template defaults.

This note collects the robot profiles used in Physical AI Runtime and maps
them onto `JointSpacePositionController` / `TaskSpaceKinematicPositionController`
parameters.

## Why robot identity matters

| Concern | Generic template | Robot-specific requirement |
|---------|------------------|----------------------------|
| Position bounds | Empty = unbounded | Must match URDF / FCI `q_min`/`q_max` |
| Velocity / acceleration | Conservative scalars | Must stay under hardware necessary limits |
| Jerk | Only enforced in `ruckig` mode | Some robots (Franka FCI) **abort** if commanded jerk exceeds the necessary bound |
| Control rate | Template assumes ~500 Hz | FR3 position interface is typically 1 kHz |
| Rate shaping ownership | JSPC `limiter` or `ruckig` | Prefer one shaper; do not stack vendor rate limiters on top of Ruckig without measuring latency |

Rule of thumb for sparse setpoints (teleop / global IK setpoint):

```text
planner / marker  →  1-point JointTrajectory  →  JSPC mode:=ruckig  →  HW position
```

Ruckig is the stage that turns a staircase into a continuous, jerk-bounded
command. For pre-timed multi-point chunks, prefer `limiter` with
`ema_alpha ≈ 1.0` so a second OTG is not stacked on an already-timed plan
(see README).

### FR3 position-interface discontinuity (measured)

Attended A/B on `franka_controller_bringup` + RViz marker (TSKPC → position,
no PyRoki/JSPC): temporarily enabling stock `franka::limitRate` in
`franka_hardware` (upstream default **off**) did not change the outcome —
OFF and ON both aborted with velocity + acceleration discontinuity. That
vendor fork was **reverted**; keep `franka_ros2` stock.

Do not treat HW `limitRate` as the FR3 discontinuity fix on this non-RT
stack. Prefer RT / CAP_SYS_NICE and a continuous 1 kHz generator.
Related: [franka_ros2#55](https://github.com/frankarobotics/franka_ros2/issues/55).

---

## Franka Research 3 (FR3)

Primary source (authoritative; re-check when upgrading `libfranka`):

- [Control Interface Specification and Robot Limits](https://frankarobotics.github.io/docs/robot_specifications.html#overview)

FCI distinguishes **necessary** conditions (violation → motion abort) from
**recommended** conditions (trajectory may be adjusted). Joint-space necessary
bounds include position, velocity, acceleration, **and jerk**. That is why a
jerk-unaware EMA/`limiter` path is insufficient for real FR3 joint-position
control, and why the validation bringup uses `trajectory_behavior.mode: ruckig`.

### FR3 joint-space necessary limits (FCI)

| Limit | J1 | J2 | J3 | J4 | J5 | J6 | J7 | Unit |
|-------|----|----|----|----|----|----|----|------|
| \(q_{max}\) | 2.9007 | 1.8361 | 2.9007 | -0.1169 | 2.8763 | 4.6216 | 3.0508 | rad |
| \(q_{min}\) | -2.9007 | -1.8361 | -2.9007 | -3.0770 | -2.8763 | 0.4398 | -3.0508 | rad |
| \(\dot{q}_{max}\) | 2.62 | 2.62 | 2.62 | 2.62 | 5.26 | 4.18 | 5.26 | rad/s |
| \(\ddot{q}_{max}\) | 10 | 10 | 10 | 10 | 10 | 10 | 10 | rad/s² |
| \(\dddot{q}_{max}\) | 5000 | 5000 | 5000 | 5000 | 5000 | 5000 | 5000 | rad/s³ |
| \({\tau_j}_{max}\) | 87 | 87 | 87 | 87 | 12 | 12 | 12 | Nm |

JSPC currently applies **scalar** `max_velocity_rad_s` / `max_acceleration_rad_s2`
/ `max_jerk_rad_s3` to every joint. Choose the scalar from the **most
restrictive** joint you intend to exercise (often J2 for suggested rectangular
profiles), not from J5/J7 peak velocity.

### FR3 Cartesian necessary limits (FCI)

| Limit | Translation | Rotation | Elbow |
|-------|-------------|----------|-------|
| \(\dot{p}_{max}\) | 3.0 m/s | 2.5 rad/s | 2.620 rad/s |
| \(\ddot{p}_{max}\) | 9.0 m/s² | 17.0 rad/s² | 10.0 rad/s² |
| \(\dddot{p}_{max}\) | 4500.0 m/s³ | 8500.0 rad/s³ | 5000.0 rad/s³ |

Use these when sizing TSKPC `max_linear_velocity_m_s` /
`max_angular_velocity_rad_s` (still keep a safety margin below FCI).

### Position-dependent velocity (FR3)

Near joint stops, FCI further reduces allowed velocity using
\(\dot{q}_{offset}\) and \(\ddot{q}_{dec}\). Nominal \(\dot{q}_{max}\) is not
always available. Prefer the vendor's suggested rectangular envelope for
long-stroke motions:

| Limit | J1 | J2 | J3 | J4 | J5 | J6 | J7 | Unit |
|-------|----|----|----|----|----|----|----|------|
| \(q_{max}\) | 2.3476 | 1.5454 | 2.4937 | -0.4226 | 2.5100 | 4.2841 | 2.7045 | rad |
| \(q_{min}\) | -2.3476 | -1.5454 | -2.4937 | -2.7714 | -2.5100 | 0.7773 | -2.7045 | rad |
| \(\dot{q}_{max}\) | 2 | 1 | 1.5 | 1.25 | 3 | 1.5 | 3 | rad/s |

### Recommended JSPC mapping (FR3)

| JSPC parameter | Role vs FCI | Notes |
|----------------|-------------|-------|
| `lower_limits` / `upper_limits` | \(q_{min}\)/\(q_{max}\) | Copy FCI table; enable `reject_out_of_bounds_targets` for hard rejects |
| `trajectory_behavior.max_velocity_rad_s` | ≤ \(\dot{q}_{max}\) (per most restrictive joint) | Uniform scalar today |
| `trajectory_behavior.max_acceleration_rad_s2` | ≤ \(\ddot{q}_{max}\) (= 10) | Leave headroom for impedance tracking error |
| `trajectory_behavior.max_jerk_rad_s3` | ≤ \(\dddot{q}_{max}\) (= 5000) | Primary smoothness/speed knob for sparse setpoints |
| `trajectory_behavior.mode` | — | `ruckig` for 1-point streaming; required for FCI jerk necessary condition |
| `ruckig_control_cycle_s` | — | `1 / controller_manager.update_rate` (FR3: `0.001` at 1000 Hz) |

Attended validation profile used by `franka_motion_planning_bringup` on a
**non-RT** host (smooth first; do not jump straight to FCI ceilings):

| Parameter | Validation default | FCI necessary (uniform floor) |
|-----------|--------------------|-------------------------------|
| `max_velocity_rad_s` | 0.6 | 2.62 (J1–J4) |
| `max_acceleration_rad_s2` | 2.0 | 10 |
| `max_jerk_rad_s3` | 15.0 | 5000 |

Observed failure (2026-07-25): on a non-RT kernel, FCI aborted with
velocity/acceleration discontinuity under CM jitter (~0.8–1.3 ms vs 1.00 ms).
After enabling Franka joint-position rate limiting, velocity discontinuity
was reduced but `acceleration_discontinuity` could still appear.

Mitigations outside JSPC (do not change the generic controller for this):

1. Enable Franka `joint_position_command_rate_limit` in `franka_hardware`.
2. Keep the conservative Ruckig profile above on non-RT hosts.
3. Prefer PREEMPT_RT + CAP_SYS_NICE for higher limits.

Keep the PyRoki planner `max_joint_velocity` and `max_step_rad` consistent with
the controller: at `plan_rate_hz = 30`,
`max_step_rad ≈ max_joint_velocity / plan_rate_hz`. Otherwise the planner
starves Ruckig of feasible setpoints and the arm still feels slow.

### Franka Emika Robot (FER / Panda)

Same FCI document, different numbers — do not reuse FR3 YAML on FER.

| Limit | J1 | J2 | J3 | J4 | J5 | J6 | J7 | Unit |
|-------|----|----|----|----|----|----|----|------|
| \(\dot{q}_{max}\) | 2.175 | 2.175 | 2.175 | 2.175 | 2.610 | 2.610 | 2.610 | rad/s |
| \(\ddot{q}_{max}\) | 15 | 7.5 | 10 | 12.5 | 15 | 20 | 20 | rad/s² |
| \(\dddot{q}_{max}\) | 7500 | 3750 | 5000 | 6250 | 7500 | 10000 | 10000 | rad/s³ |

Cartesian translation \(\dot{p}_{max}\) is **1.7 m/s** on FER vs **3.0 m/s** on
FR3. Acceleration/jerk Cartesian caps also differ — see the FCI page.

---

## Marvin (bimanual 7-DOF)

Marvin has no Franka-style FCI jerk abort, but the same JSPC Ruckig path is used
for sparse marker/planner setpoints so teleop feel stays consistent across
embodiments.

Bringup reference (`marvin_motion_planning_bringup`):

| Parameter | Typical value |
|-----------|---------------|
| `controller_manager.update_rate` | 500 Hz |
| `ruckig_control_cycle_s` | 0.002 |
| `max_velocity_rad_s` | 2.0 |
| `max_acceleration_rad_s2` | 8.0 |
| `max_jerk_rad_s3` | 20.0 |
| Position limits | From Marvin URDF / bringup YAML (not FCI) |

Marvin's low jerk (20) is a **feel** choice, not a hardware abort threshold.
FR3 can legally run much higher jerk; the validation profile above uses 250 so
FR3 teleop is not artificially slower than Marvin solely due to an outdated
`0.6 / 15` clamp.

---

## Tuning checklist

1. Copy vendor position limits into JSPC/TSKPC `lower_limits` / `upper_limits`.
2. Set Ruckig velocity/acceleration/jerk **below** necessary vendor limits with
   margin for impedance lag and tracking error.
3. Align planner step size with controller velocity:
   `max_step_rad ≈ max_joint_velocity / plan_rate_hz`.
4. On FR3 real hardware, do **not** fork `franka_ros2` to force-enable
   joint-position `franka::limitRate`: A/B on `franka_controller_bringup`
   showed OFF and ON both abort; vendor code stays stock (rate limit off).
   Prefer RT / CAP_SYS_NICE and a continuous 1 kHz generator.
5. Raise limits in small attended steps; a single large marker jump at high
   jerk still stresses the impedance loop even when FCI accepts the command.

## Related packages

| Package | Role |
|---------|------|
| `franka_motion_planning_bringup` | FR3 marker → PyRoki → EM → JSPC(Ruckig) validation |
| `franka_controller_bringup` | FR3 TSKPC path (Cartesian IK) |
| `marvin_motion_planning_bringup` | Marvin dual-arm JSPC(Ruckig) planning bringup |
