# Motion Planner Source Interface

> **RMI integration boundary (2026-08-09):** planners are peer
> ActionProviders. Their native ROS 2 outputs enter RMI deployment-declared
> topic/action gateways; planners do not switch controllers or publish directly
> to controller inputs.

Backend-neutral planner families for Physical AI Runtime. Package home:
`src/motion_planning/motion_planners` (`motion_planner_core` +
solver adapters).

> [!IMPORTANT]
> **Layering & Atomic Verbs Boundary**:
> Normative runtime motion verbs remain in [`RUNTIME_ORCHESTRATION.md`](RUNTIME_ORCHESTRATION.md)
> (`sdk.move_joints`, `sdk.move_pose`, `sdk.apply_cartesian_twist`, `sdk.move_trajectory`).
>
> Planners **compose** those SDK verbs. They are **not** atomic SDK APIs and must
> not invent a parallel EM topic vocabulary. Wire contracts are the locked
> SDK/EM ones only.
>
> **No legacy compatibility.** Historical names (`Global*`, `joint_target`,
> `joint_chunk`, `joint_trajectory_goal`, `cartesian_pose` as contract id) are
> deleted, not aliased.

---

## 1. Planner Families Overview

| Family | Core Role | Backend Protocol API | Typical Output | Dispatch → Controller |
| --- | --- | --- | --- | --- |
| **Resolver** | **IK Resolution** | `resolve(current, target) → q*` | 1-point joint $q^*$ | `sdk.move_joints(q*)` → JSPC |
| **Planner** | **Segment Trajectory Generation** | `plan()` once per segment | Complete timed joint path | `sdk.move_trajectory(traj)` → JTC |
| **Streamer** | **Receding Horizon Guidance** | `step(current, dt)` (periodic) | Short joint **or** pose horizon | `sdk.move_joints(N×DoF)` → JSPC **or** `sdk.move_pose` → TSKPC |

### Dispatch mapping (SDK-first)

| Family | Backend protocol | Result | SDK verb | EM wire | Controller |
| --- | --- | --- | --- | --- | --- |
| **Resolver** | `ResolverBackend.resolve()` | $q^*$ (`ResolveResult`) | `sdk.move_joints(q*)` | `joint_reference` **1-pt** | JSPC |
| **Planner** | `PlannerBackend.plan()` | timed joint path (`PlanResult`) | `sdk.move_trajectory(traj)` | `joint_trajectory` **action** | JTC |
| **Streamer (Joint)** | `JointStreamerBackend.step()` | joint horizon (`JointHorizonResult`) | `sdk.move_joints(N×DoF)` | `joint_reference` **N-pt** | JSPC |
| **Streamer (Cartesian)** | `CartesianStreamerBackend.step()` | pose horizon (`PoseHorizonResult`) | `sdk.move_pose` (1 or N) | **`pose_reference`** := `moveit_msgs/CartesianTrajectory` (1 or N) | TSKPC |

**Servo stream naming (symmetric):**

| Stream | EM contract | Message type | Controller |
| --- | --- | --- | --- |
| Joint | `joint_reference` | `trajectory_msgs/JointTrajectory` (1 or N) | JSPC |
| Pose | `pose_reference` | `moveit_msgs/CartesianTrajectory` (1 or N) | TSKPC |

Streamer uses **two parallel protocols** (`JointStreamerBackend` /
`CartesianStreamerBackend`), not one overloaded backend.

---

## 2. Resolver Family (IK Resolution)

Resolver is **not** a motion planner. Verb is **`resolve()`**, never `plan()`.

```text
while running:
    q_star = resolver.resolve(current_state, target)
    sdk.move_joints(q_star)    # → joint_reference 1-pt → JSPC
    sleep(dt)                  # dt = IK budget (app-owned loop)
```

**Target rules (locked):**

| `target` | Behavior |
| --- | --- |
| **Pose** | Run IK → $q^*$ (or explicit failure) |
| **Joint** | **Validate / passthrough** only (limits, DoF, finiteness)—**not** a second IK. Prefer skipping Resolver and calling `sdk.move_joints(q*)` directly when the app already holds $q^*$. |

```text
App (pose) ──► resolve() ──► q* ──► sdk.move_joints(q*) ──► JSPC
App (q*)   ──► sdk.move_joints(q*)   # bypass Resolver
```

`resolve()` is one request → one $q^*$ (or failure). Any loop is owned by App/Source.

---

## 3. Planner Family (Segment Trajectory Generation)

**Joint-space segments only** (JTC). Cartesian segment generators (Pilz LIN/CIRC,
etc.) are **future**—out of scope for this family until explicitly added.

```text
App / Source ──► PlannerBackend.plan() ──► complete timed joint traj
              ──► sdk.move_trajectory(traj) ──► joint_trajectory action ──► JTC
```

1. `plan()` runs **once** per segment; output is a complete time-parameterized
   joint path—never a streaming chunk.
2. **`sdk.move_trajectory` is atomic execution**: input is a full timed
   `JointTrajectory` (or equivalent structured points + `time_from_start`).
   The SDK/EM **must not** synthesize paths (no quintic / interpolation inside
   the execution API). Path generation belongs in Planner (or the app).
3. Candidate solvers (joint path + time param): OMPL, TrajOpt, STOMP/CHOMP,
   TOPPRA, TOTG, offline Ruckig. LIN/CIRC reserved for a future Cartesian
   segment family.

---

## 4. Streamer Family (Receding Horizon Guidance)

Two parallel protocols; same online shape:

```text
backend.update_target(goal)
backend.reset(current)
while running:
    result = backend.step(current, dt)
    # JointStreamer  → sdk.move_joints(horizon)
    # CartesianStreamer → sdk.move_pose(...)  # → pose_reference / CartesianTrajectory
```

```text
Periodic callback (app / source timer owns dt):
  update state → update_target (if needed) → step(current, dt)
       │                              │
       ▼                              ▼
 JointStreamerBackend          CartesianStreamerBackend
  (cuRobo / PyRoki MPC, …)      (ndcurves SE3, …)
       │                              │
 sdk.move_joints(N×DoF)         sdk.move_pose (1 or N)
       │                              │
 EM joint_reference             EM pose_reference
       │                              │
     JSPC                           TSKPC
```

### Cartesian Streamer (`ndcurves` notes)

- Rebuilds `current → [vias] → final` each tick; TOPP-RA discretization is
  controlled by path-sample and reachability-grid counts.
- **Horizon point 0 = 1-step look-ahead** (not current)—required while TSKPC
  consumes `first_point` only.
- Output chunk sampling is controlled only by caller-owned `dt`; TOPP-RA grid
  counts affect numerical fidelity, not controller timestamps.

---

## 5. Naming & Execution Rules

```text
ResolverBackend          → resolve()           → q*
PlannerBackend           → plan()              → complete timed joint path
JointStreamerBackend     → update_target/reset/update_world/step(dt) → joint horizon
CartesianStreamerBackend → update_target/reset/update_world/step(dt) → pose horizon
```

Only **Planner** uses `plan()`. App/Source owns all timers.

## Related Docs

- [`RUNTIME_ORCHESTRATION.md`](RUNTIME_ORCHESTRATION.md) — atomic SDK verbs + EM wires
- `src/motion_planning/motion_planners/README.md` — package map (must match this doc)
