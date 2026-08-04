# Manipulation Execution Architecture

Status: **Implemented (v1)**  
Last updated: 2026-08-03

This is the **single workspace architecture document** for the Manipulation
Execution Manager (EM). Wire contracts and parameters live in the package:

- [`src/execution/manipulation_execution_manager/docs/`](../src/execution/manipulation_execution_manager/docs/)
- especially [architecture](../src/execution/manipulation_execution_manager/docs/architecture.md),
  [contracts](../src/execution/manipulation_execution_manager/docs/contracts.md), and
  [configuration](../src/execution/manipulation_execution_manager/docs/configuration.md)

## Design objective

EM is the execution boundary between manipulation command producers and
robot-specific position controllers. Producers must not publish directly to
controller inputs or switch controllers themselves.

```text
teleop / streaming planner / trajectory planner / policy
                         |
                         v
+---------------------------------------------------------+
| Manipulation Execution Manager                          |
| validate -> normalize -> arbitrate (side-wide owner)    |
|   -> arm route + optional end_effector_servo            |
|   -> switch controllers -> dispatch                     |
+---------------------------------------------------------+
                         |
          +--------------+----------------+
          v                               v
   arm routes                      end_effector_servo
   joint_servo /                   forward_position
   cartesian_servo /               controller
   trajectory_execution
```

## V1 decisions (locked)

1. One EM instance owns one manipulation **side** (one arm + its selected end
   effector).
2. Arm and end effector are independent `ros2_control` hardware components and
   may run concurrently under **one** producer owner.
3. **Ownership is side-wide.** Independent per-resource owners are deferred.
4. An owned but uncommanded resource **holds last safe command** (no new EM
   publish for that resource). Before any EE command has ever been dispatched,
   EM does not invent a width (ForwardCommand may remain NaN until the first
   owning source commands EE).
5. Parallel grippers use streaming `end_effector/joint_reference` →
   `Float64MultiArray` → preloaded `forward_position_controller`. **No gripper
   action proxy in v1.**
6. Dexterous hands and mobile bases are out of scope for this EM.
7. Contracts describe capability, never vendor/SDK names.

## Resources, contracts, routes

| Resource | Contracts | Routes |
|---|---|---|
| `arm` | `joint_reference`, `cartesian_pose`, `cartesian_pose_sequence`, `cartesian_twist`, `joint_trajectory` | `joint_servo`, `cartesian_servo`, `trajectory_execution` (mutually exclusive) |
| `end_effector` | `joint_reference` (one named position-only point) | `end_effector_servo` (concurrent with arm under same owner) |

Endpoints:

```text
/action_sources/<source>/<resource>/<contract>
```

Examples:

```text
/action_sources/marker_left/arm/cartesian_pose
/action_sources/motion_planner_left/arm/joint_reference
/action_sources/piper_demo_left/arm/joint_trajectory
/action_sources/marker_left/end_effector/joint_reference
```

## Ownership and hold

- Highest-priority eligible source wins; equal priority is sticky.
- Fresh arm stream, fresh EE stream, or an active arm trajectory retains
  side-wide ownership.
- Preemption cancels the whole side (including the other resource).
- Bringup: arm route controllers and the gripper forward controller start
  **inactive** (JSB active). Ownership activates the configured gripper
  controller; arm route switches **must not** deactivate it.
- Release: `end_effector_deactivate_on_owner_release` (default `false`) keeps
  the gripper controller active so hardware can hold.

“Hold” means EM does **not publish a new** EE command. The last value written
to the forward controller / hardware remains in effect until a new owner
command arrives.

## Producer classes

| Class | Default priority |
|---|---:|
| `teleop` | 100 |
| `trajectory_planner` | 60 |
| `policy` | 50 |
| `streaming_planner` | 40 |

## Metadata

Streaming dispatch publishes JSON metadata (`schema_version` 2) including
`resource` and `ownership_epoch`. Status JSON uses `schema_version` 3 with
`selected_routes` keyed by resource.

## Application composition

Per-side end-effector selection (`none` / `piper_gripper` / `pika_gripper`) is
an application launch concern. The app wires description, independent
hardware, controllers, and EM `end_effector_*` / `route_controllers` overlays.

## Explicitly deferred / future

V1 does **not** implement the following. They are expected drivers for later
architecture revisions; they must not be inferred from current behavior.

### Near-term: split resource ownership

Today one source owns the whole side. A near-term requirement is that
**source A may own `arm` while source B owns `end_effector`** (independent
per-resource owners, priorities, and leases). That needs a new arbitration
table, preemption rules across resources, and metadata that can attribute each
dispatch to a different owner. Side-wide ownership remains the locked v1
default until that revision lands.

### Relative command contracts

External sources may publish **relative** joint positions and **relative**
poses (deltas / increments), not only absolute `joint_reference` /
`cartesian_pose`. A future revision must admit, validate, and **normalize**
those contracts into the absolute controller-facing setpoints EM already
dispatches (including stamp, frame, joint-order, and limit policy). Relative
inputs are rejected or unsupported in v1.

### Whole-body and mobility

Later systems will add other body parts, including a **mobile base**, under a
broader whole-body execution story. Mobility timing, short-lease velocity, and
navigation actions differ from manipulation; the architecture will likely be
updated so manipulation EM stays a domain boundary while a sibling mobility /
whole-body layer (or a revised multi-domain supervisor) coordinates leases.
Mobile is not an EM resource in v1.

### Other deferred items

- Dexterous-hand contracts and routes
- Gripper finite open/close **action** APIs
- Coordinated arm/EE fault mapping and E-stop
- Dynamic producer registration sessions

## Initialization (intentional)

EM does **not** seed the end-effector forward controller from `joint_states`
on first activation. Until a producer publishes an EE command, ForwardCommand
may remain at NaN and does not invent a width. That is intentional: the
**first owning source** is responsible for startup pose / open-width (for
example a JTC probe that homes the arm and streams EE open before planner or
teleop takes over). EM remains a gate, not an initializer.

Demo UIs that stream a default open on launch (for example a slider
`initial=0.04`) are producer behavior and are documented at the demo layer.

## Related docs

- Workspace runtime layout: [ARCHITECTURE.md](ARCHITECTURE.md)
- End-effector packaging conventions: [END_EFFECTOR_CONVENTIONS.md](END_EFFECTOR_CONVENTIONS.md)
- Package API truth: `manipulation_execution_manager/docs/`
