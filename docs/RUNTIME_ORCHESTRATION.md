# Orchestrator SDK and API

Normative contract for the Runtime Orchestration SDK. Roadmap and honest
implementation status live in
[`RUNTIME_ORCHESTRATION_SDK_MASTER_PLAN.md`](RUNTIME_ORCHESTRATION_SDK_MASTER_PLAN.md).

## Purpose

The Runtime Orchestration SDK is the control plane for Physical AI Runtime. It
bridges policies, teleoperation, browser UIs (including LeLab-class frontends),
and agents to heterogeneous embodiments through one **unified profile**.

Ownership evolves in two stages:

| Stage | Process composition | Configuration truth |
| --- | --- | --- |
| **Present (main)** | ROS 2 launch owns drivers, `ros2_control`, Execution Manager (EM), recorder, and sensors | Profiles exist; many apps still use package-local launch/YAML |
| **Target** | Once the SDK API and underlying packages are stable, the SDK **replaces ad-hoc launch graphs** as the orchestrator: one unified profile drives RT-host and policy-host bringup | Single profile + `profile_hash`; host roles extract only what each machine needs |

Today the SDK may still *invoke* `ros2 launch` as an implementation detail. The
end state is profile-driven composition owned by the SDK, not a zoo of divergent
launch files and duplicate configs.

```text
Python policy / Gym online-RL     Browser UI / LeLab / agent
          |                              |
          +-------- Runtime SDK ----------+
                           |
                   HTTP / WebSocket adapter
                           |
              Unified Profile + Transport Adapter
          +----------------+-----------------+
          |                |                 |
         EM         episode_recorder       sensors / ros2_control
```

High-frequency policy and online-RL control remain in-process (or on the policy
host) through the SDK ROS adapter. HTTP and WebSocket are for orchestration,
session/operation progress, capability/schema exchange, and bounded-rate
previews—not the realtime control path.

## Stable SDK model

No SDK module may hard-code a robot vendor, arm count, controller name, or
camera model. Embodiment detail lives in the unified profile.

- `EmbodimentConfig`: joint groups, frames, sensors, recorder endpoints, stream
  contracts, ROS bindings, and **host roles** (`rt_host`, `policy_host`, …).
- `profile_hash()`: stable digest shared by every host that loaded the same
  profile; required for session arming and cross-host consistency.
- `Capability`: profile plus live readiness/preflight evidence (usable now,
  not assumed).
- `FeatureSchema`: typed observation/action features; runtime values are
  `dict[str, numpy.ndarray]`. Exportable to LeRobot-compatible feature dicts.
- `RuntimeSession`: exclusive-owner lifecycle
  `prepare -> ready -> active -> finalizing -> completed | failed`.
- `Operation`: long-running **ticket** (queryable, cancellable, waitable) with
  id, phase, progress, result, and terminal error reason—not fire-and-forget.

## Control plane and data plane

### Authoritative recording (offline / high fidelity)

The orchestrator does not write bags, transcode images, or own training-row
alignment. `episode_recorder` remains the authoritative data-plane owner for
raw MCAP payloads, health accounting, validation, checksums, and atomic
finalization. Finalized episodes record `profile_hash` and the resolved stream
snapshot in the manifest. Offline MCAP → LeRobotDataset export is a separate
gate.

### Online RL path (core capability)

Distributed online RL is a **first-class** Runtime capability. The preferred
split is:

```text
Local Physical Runtime                    Cloud / remote trainer
  experience collector (obs+action)  --RPC-->  policy trainer / learner
  Gym reset/step, session owner              updated weights / policy server
  publishes actions via EM policy source     (may also host a LAN policy source)
```

Design must cover:

- synchronized observation–action pairs at the control rate on the collector;
- RPC (or equivalent) between local collector and remote trainer—not only
  co-located processes;
- a replay-buffer contract (schema, clocks, drop/lag) on collector and/or
  trainer side;
- policy actions still enter the robot only through EM + session owner.

**Tension with `episode_recorder` (must be designed, not ignored):**

| Path | Purpose | Fidelity |
| --- | --- | --- |
| `episode_recorder` MCAP | Authoritative raw capture for offline datasets | Highest; offline alignment/export |
| Online RL experience stream | Low-latency synced pairs for replay/training over RPC | Control-rate sync; may subsample or drop under load |

Both paths share the same `FeatureSchema` names and `profile_hash`. Neither may
masquerade as the other: an RL experience RPC is not an episode finalize, and a
recording failure is not an RL step failure. Prefer one profile, two writers,
orthogonal status machines.

Before a session can arm **recording**, `resolve_streams(profile)` compares the
declared stream contract with the live graph (topic/action, type, QoS,
publisher, disk). Required failures reject arming.

Recorder lifecycle commands use a recorder-specific transport boundary, not
the motion action dispatcher. Finalization succeeds only after the recorder
returns the requested MCAP path and checksum; the runtime does not synthesize
recording evidence.

Status producers stay orthogonal:

- recorder: recording phase and health;
- EM: active source, route, validation, execution state;
- orchestrator: session owner and operation state;
- online-RL adapter: episode/step/buffer health (when active).

## API and concurrency rules

External API is grouped by domain (HTTP/WebSocket and in-process):

- `capabilities`: embodiment/profile/readiness; frontend-safe schema export;
- `sessions`: create, inspect, close; assign/query session owner;
- `episodes`: arm, start, stop, discard, finalize via session (recorder path);
- `actions`: bounded commands through EM;
- `operations`: query, stream progress, cancel;
- `policies` (target): discover/select which LAN policy action source is active
  under the session owner (still no browser-to-ROS path).

Session ownership enforces mutual exclusion among teleoperation, policy,
trajectory, recording finalization, calibration, and online-RL clients.
The active session owner and EM active source must be **queryable**.
Repeated commands are idempotent.

The HTTP action handler requires an active session, a leased group, one of the
canonical motion verbs, and a verb-specific payload. It calls the corresponding
SDK handle; raw transport dictionaries are not an external API.

The API server owns neither ROS publishers nor session state; it calls the SDK.
WebSocket streams carry session/operation changes, EM status, recorder health,
online-RL step health (when enabled), and bounded-rate previews.

## Motion API (in-process)

High-level motion is exposed on `ArmHandle` / `CompoundGroupHandle`. Routes are
selected by the EM from the action contract; the SDK does not invent a second
arbiter.

Atomic streaming verbs (parallel naming):

| Verb | Route | Space |
| --- | --- | --- |
| `move_joints` | joint_servo (JSPC) | joint |
| `move_pose` | cartesian_servo (TSKPC) | Cartesian pose (1-pt or sequence) |
| `apply_cartesian_twist` | cartesian_servo (TSKPC) | Cartesian twist |
| `move_trajectory` | trajectory_execution (JTC) | joint trajectory action |

Canonical name is **`move_pose`** (no `move_to_pose` alias in the target API).

### `move_joints` → joint_servo (JSPC)

Sole app-facing **move-to-joint** stream API. Apps must follow this API and
must **not** publish `joint_reference` (or bypass the handle via transport)
themselves. The caller owns a fixed-rate loop (e.g. 50 Hz). Each call
publishes one tick on `joint_reference`. **Never** promotes to JTC.

Any upstream (VLA / planner / policy / teleop) may send absolute or relative
values. Explicit ``mode`` decides whether the SDK converts:

| `mode` | Payload | SDK behavior |
| --- | --- | --- |
| `absolute` (default) | 1-point `q*` or N-point absolute chunk | Publish as-is (safety checks only) |
| `delta` | 1-point Δq only | Convert vs latest **measured** q → absolute, then publish |

Typical call patterns (all valid):

| Upstream | Example |
| --- | --- |
| Policy that already denorms to abs (GR00T-style) | `move_joints(q_abs)` |
| Policy / VLA that still outputs Δq | `move_joints(dq, mode="delta")` |
| Planner / MPC absolute horizon | `move_joints(chunk_NxD, times_from_start_sec=t)` |
| Teleop gamepad / jog | `move_joints(dq, mode="delta")` |
| Fixed IK / hand-set target | repeat `move_joints(q*)` each tick → Ruckig |

Notes:

- Keep streaming while the target/chunk is active; JSPC `stale_timeout` will
  drop the route if ticks stop.
- Returned `Operation` is a **stream-tick ack**, not “settled at target.”
- Wire contract stays **absolute** `joint_reference` (EM v1). `mode` is SDK
  interpretation only—not a second ROS contract.
- `mode="delta"` fails hard if measured joints are unavailable. Delta chunks
  are out of scope—expand upstream or stream 1-point deltas.
- Multi-point chunks always carry `time_from_start`. Pass the planner-owned
  `times_from_start_sec` to preserve a nonuniform parameterization; when it is
  omitted, `duration_sec` creates a uniform time axis.
- Do not wrap Franka-style `MotionGenerator` on top of Ruckig for fixed targets.

### `move_pose` → cartesian_servo (TSKPC)

Sole app-facing **Cartesian pose** stream API (name aligned with
`move_joints`). Apps must not publish TSKPC pose/twist topics themselves.
**No** Cartesian Ruckig in the SDK; smoothing and limits are TSKPC DiffIK /
trajectory-behavior config.

**Controller inputs (locked — JSPC parity, Target A, no legacy):**

| TSKPC topic | Type | Role | JSPC analogue |
| --- | --- | --- | --- |
| **pose input** (one topic) | `moveit_msgs/CartesianTrajectory` | Pose targets: **1 point ≡ former `PoseStamped`**, N points = horizon | `input_topic` ← `JointTrajectory` |
| **twist input** | `geometry_msgs/TwistStamped` | Spatial **velocity** command (integrate → DiffIK) | *(none — joints have no twist line)* |

No separate `PoseStamped` subscription. A 1-point `CartesianTrajectory` is the
wire form of a single EE pose. **Present controller strategy (simple):** always
command **point 0** only, whether the message carries 1 or N points. Freshness
is `receive_time + stale_timeout` (not full-horizon play-out).

`trajectory_behavior.*` is the JSPC-analogue hook for **future** richer
consumption (look-ahead sample / timed play / etc.), parallel to
`reference_behavior.*` on JSPC. Today it mainly holds ingest knobs
(`max_points`, `untimed_frame_dt_s`).

SDK / EM alignment with `move_joints`:

| Call shape | Wire (target) | Low-level (present) |
| --- | --- | --- |
| 1-point pose | `CartesianTrajectory` with **1** point | DiffIK tracks that pose |
| N-point pose horizon | same type / same EM contract, N points | **Still uses point 0 only**; horizon kept for future `trajectory_behavior` |
| twist | `apply_cartesian_twist` → `TwistStamped` | separate verb; not a pose target |

**`mode` is first-class** (same rule as `move_joints`): any upstream (VLA /
planner / policy / teleop) may pass absolute or relative values; explicit
`mode` decides whether the SDK converts before publish.

| `mode` | Payload | SDK behavior |
| --- | --- | --- |
| `absolute` (default) | Pose or sequence in the controller frame | Publish as-is (safety checks only) |
| `delta` | 1-point Δpose only (vs **latest measured** EE pose) | Convert to absolute, then publish. Requires measured pose; **fails hard** if unavailable |

Wire contract stays **absolute** (EM v1). `mode` is SDK interpretation only—not
a second ROS contract. Relative/delta **chunks** are out of scope: expand to
absolute upstream (policy/planner host) or stream 1-point deltas.

**Timing** (wire parity with `JointTrajectory`; present consume = pt0):

- A 1-point stream tick may have zero `time_from_start`. Every N-point chunk
  must have a finite, non-negative, strictly increasing time axis. EM rejects
  an untimed chunk instead of guessing a sample period.
- `move_pose(..., times_from_start_sec=t)` preserves the planner-owned time
  parameterization (including ndcurves + TOPPRA output). If `t` is omitted,
  `duration_sec` creates a uniform time axis at the SDK boundary.
- **Present TSKPC:** ignores timing for motion — commands first point only
  (aligned with the simple JSPC “use the latest reference setpoint” streaming
  habit when callers send 1-pt ticks; N-pt is accepted but only pt0 is used).
- **Future `trajectory_behavior`:** richer consume strategies (below).

#### Future `trajectory_behavior` sketches (not implemented)

Namespace parallel to JSPC `reference_behavior.*`. Wire stays
`CartesianTrajectory`; only **how the RT loop picks a pose** changes.

Possible modes (names illustrative):

1. **`first_point` (present default)** — command `points[0]`; freshness =
   `receive_time + stale_timeout`. Fits teleop / 1-pt policy ticks; N-pt
   messages are still buffered for a later mode switch without wire change.
2. **`sample` (timed play / look-ahead)** — assign absolute frame times
   (`header.stamp + time_from_start`), then each control cycle call
   existing `task_space::sample_pose_chunk(chunk, now)` (hold-first /
   lerp+normalize quat / hold-last) and use
   `chunk_is_fresh` for horizon + grace. Reuses code already unit-tested in
   `pose_chunk_buffer.hpp`.
3. **More complex sampling (later)** — e.g. look-ahead index (command a pose
   `Δt` ahead of `now`), denser re-sample, better SO3 slerp, or coupling to
   DiffIK rate limits the way JSPC couples chunk timing to
   limiter/ruckig. Still one message type; strategy stays a controller
   parameter, not a new ROS contract.

Until a mode other than `first_point` ships, apps must not assume the arm
tracks an N-pt Cartesian horizon inside TSKPC—emit 1-pt ticks or put
guidance in `motion_planners` and still expect pt0 consume at the controller.

**Far single goals are not an SDK concern.** Guiding curves belong to
`motion_planners` (streaming Cartesian / pose-sequence family); execution
still uses this atomic API. See
[`MOTION_PLANNER_SOURCE_INTERFACE.md`](MOTION_PLANNER_SOURCE_INTERFACE.md).
Geometric SE3 sampling lives in `ndcurves_planner_adapter` (conda
[`ndcurves`](https://prefix.dev/channels/gabriel-robotics/packages/ndcurves)).

Returned `Operation` is a **stream-tick ack** (1-point) or sequence-publish
ack (chunk)—not “settled at goal.”

### `apply_cartesian_twist` → cartesian_servo (TSKPC)

Sole app-facing **Cartesian twist** stream API. Semantics differ from
`move_pose` (rate command, not a pose target), so it is a separate verb.

```text
apply_cartesian_twist(linear, angular, frame_id=...)
  → EM cartesian_twist (TwistStamped)
  → TSKPC
```

| Horizon | Behavior |
| --- | --- |
| **Present** | SDK publishes EM `cartesian_twist`; TSKPC integrates twist in the base frame into an internal pose, then DiffIK (same solver path as pose). Re-seed on mode entry; stale-hold when ticks stop. |
| **Future** | Prefer a real task/joint **velocity hardware interface** and feed twist (with optional LPF / limits) without pose integration. Tracked as a controller/HW TODO—not an SDK contract change. |

Notes:

- Caller streams at a fixed rate; do not bypass the handle with raw publishers.
- Returned `Operation` is a **stream-tick ack**.
- One EM owner should not mix pose and twist ticks in the same control intent;
  TSKPC priority is **pose trajectory > twist** when both are fresh.

### Other motion verbs (walked next)

- `move_trajectory` → trajectory_execution (JTC / FollowJointTrajectory)

### Cartesian motion refactor (decided)

Lock (EM + TSKPC + SDK; **no legacy aliases**):

1. **TSKPC Target A — exactly two input topics (B2; controller-side):**
   - **Pose:** `CartesianTrajectory` only. **1 point = former `PoseStamped`
     semantic**; N points accepted on the wire. **Present:** command point 0
     only. **Future:** richer `trajectory_behavior` (JSPC `reference_behavior`
     parity). No `PoseStamped` / `PoseArray` / `pose_chunk_topic` path.
   - **Twist:** `TwistStamped` (spatial velocity; separate from pose).
2. **JSPC parity on the pose line:** one message type for 1-pt and N-pt;
   timed/untimed on the wire; `trajectory_behavior.*` namespace for how to
   consume (present = first-point; future = look-ahead / etc.).
3. **Atomic SDK (C2 done):** `move_pose` only (no `move_to_pose` alias). Always
   publish `CartesianTrajectory` (1 or N) on **`pose_reference`**. Explicit
   `mode=absolute|delta` (delta = 1-point only, hard-fail without measured pose).
4. **EM:** one pose streaming contract **`pose_reference`** =
   `CartesianTrajectory`; `cartesian_twist` remains independent.
5. **`move_trajectory`:** atomic execution — caller supplies a complete timed
   joint trajectory. SDK must **not** synthesize paths (no internal quintic).
6. **Geometry planner** in `motion_planners`; can emit timed sequences.
7. **Endpoints:** profile `ros_topics` only (no transport `SOURCE_MAP`).

Detail: [`MOTION_PLANNER_SOURCE_INTERFACE.md`](MOTION_PLANNER_SOURCE_INTERFACE.md).

## Action sources and multi-policy (arbitration)

**Intent (canonical):** on one robot LAN, several **action sources** may publish
at once (teleop, trajectory, planner, and **one or more policy sources**).
Something must arbitrate priorities, inactivity timeouts, controller route
switching, and non-preemption.

**Present:** that arbitrator is the authoritative RMI **ExecutionManager** on
the RT host. `orchestrator` consumes RMI and must not duplicate its
ownership, lifecycle, controller-switching, or generation state machines.

Typical priorities (profile-configurable, enforced by the arbitrator):

```text
teleop       100   cartesian / TSKPC
trajectory    50   JTC
planner       20   streaming trajectory
policy*       10   joint_reference / JSPC   (*may be several named policy sources)
```

Multiple policy implementations on the LAN are **named action sources** (for
example `policy_a`, `policy_b`) under the unified profile. Who may command the
arm = arbitrator rules + SDK session owner.

### RMI is the execution implementation

RMI now owns the arbitration product API and its authoritative RT-host service.
The remaining refactor is consumer migration: `orchestrator` should
call RMI rather than synthesize or embed another EM implementation.

Constraints if/when replacing EM:

- Arbitration and controller switching stay **near `ros2_control`** (RT host /
  same latency domain). The SDK Python process and LeLab HTTP/WS path must not
  become the realtime arbiter.
- Source names, priorities, and FeatureSchema remain profile-driven so LAN
  multi-policy behavior does not change for clients.
- Refactor gate: fake-HW and real-robot tests for priority, non-preemption,
  and route switch must keep passing.

The function remains mandatory and clients see only the RMI API plus deployment
profile. The upper task engine is deliberately unspecified.

SDK responsibilities (stable across stages):

- expose which policy sources exist / are healthy;
- hold session ownership for the intended client;
- publish observations/actions through the transport adapter;
- never require the UI to open ROS topics directly.

## Deployment note (Docker on the RT host)

Packaging the RT stack (ros2_control, EM, drivers) in **Docker** is a later
**deployment** choice for the RT host. It is orthogonal to “multiple policy
action sources on the LAN.” Docker does not replace EM arbitration and is not
required for multi-policy support.

## LeLab and LeRobot

LeLab-class UIs are a **primary** integration surface. They must talk only to
the SDK HTTP/WebSocket API and frontend-safe capability/schema payloads (no
robot vendor names or ROS topic strings as UI primary data).

Reuse LeRobot ecosystem pieces where they fit: feature dictionaries, offline
dataset layout, and optional Robot/Teleoperator adapters. Do not pull SO-101,
serial-port calibration, or online row-alignment assumptions into the SDK or
`episode_recorder`.

```text
Runtime SDK (HTTP/WS + in-process)
  -> LeLab / reference web client
  -> LeRobot feature dict / offline MCAP exporter
  -> optional Robot/Teleoperator adapter
```

## Deliberate non-goals

- No controller-specific FSM, OCS2, CRISP, MoveIt, or vendor API in SDK core.
- No MP4 sidecar replacement for raw image payloads.
- No Runtime-owned training job orchestration or Hugging Face hub lifecycle as
  SDK core (cloud trainers stay external; SDK only defines collector↔trainer RPC).
- No ZMQ/pickle control protocol or monolithic `SendCommand` service.
- No direct browser-to-ROS control path.
- No treating online-RL experience RPC / `log_*`-style online rows as a
  substitute for authoritative MCAP.
- No second realtime arbitration FSM in Python that duplicates EM.

## Reference projects: adopted patterns

| Reference | Adopted pattern | Reason |
| --- | --- | --- |
| `crisp_py` | YAML device configuration, readiness gates | Multi-embodiment without hard-coding |
| `ros2_robot_interface` | Capability-granular facade | Readable scripts; controller details outside SDK |
| `lerobot-ros` | Observation/action feature dictionaries | Route to LeRobot/LeLab without hard dependency |
| [`neuracore`](https://github.com/NeuracoreAI/neuracore) | Local collector/daemon boundary; stamped experience stream; remote trainer RPC; explicit embodiment description | Online-RL data path + cross-embodiment IO—without replacing MCAP |
| [`r2_labs`](https://github.com/Reimagine-Robotics/r2_labs) | Operation tickets; exclusive owner/mode query; domain-split clients; optional agent MCP over SDK | Browser/agent UX—without ZMQ/pickle or appliance lock-in |
| `cyclo_intelligence` | Control/data-plane split, preflight | EM + transactional MCAP recorder |
| `physical_ai_tools` | Phase-aware UI, profile selection | Reference web / LeLab-facing patterns |

### What we take from [r2_labs](https://github.com/Reimagine-Robotics/r2_labs)

Learn the **control-plane UX**, not the transport or appliance shape:

1. **Operation = ticket** — wait, cancel, and terminal failure/cancel reasons
   (`BehaviourClient`-class semantics). Operations must stay `RUNNING` until
   transport completes.
2. **Exclusive owner / mode query** — who may command the robot is explicit and
   inspectable. Map to **session owner + EM active source**, not a parallel
   global FSM beside EM.
3. **Domain clients** — split surfaces (`capabilities`, `sessions`, `episodes`,
   `actions`, `operations`, `policies`) instead of one giant command bag.
4. **Optional MCP** — agents may sit behind a thin MCP→SDK adapter; MCP must
   not talk ROS or bypass session ownership.

Do **not** adopt: ZMQ REQ/REP + pickle control protocol; R2 appliance-only
assumptions; replacing LAN multi-policy + EM arbitration with a single robot
server model.

### What we take from [neuracore](https://github.com/NeuracoreAI/neuracore)

Learn the **data / learning-plane boundary**, not the cloud-platform product:

1. **Local collector (or daemon) process** — stays near the robot; scripts and
   Gym loops attach to it; can auto-start with the session when enabled.
2. **Stamped experience stream** — timestamped obs/action (and optional
   language) rows for online RL / preview, sharing FeatureSchema names with the
   profile.
3. **Remote trainer RPC** — collector uploads experiences; trainer returns
   weights or a serving endpoint that re-enters as a LAN **policy action
   source** under EM.
4. **Explicit embodiment description** — every connect/log path hangs off a
   declared robot contract (our unified YAML profile + FeatureSchema, not
   URDF/MJCF-only).

Do **not** adopt: cloud login/org/GPU training as SDK core dependencies;
`log_*` online rows as the authoritative dataset; conflating ffmpeg/daemon
video storage with raw MCAP semantics.

## Verification

- Fake-hardware path verifies EM action routing and session ownership.
- Recorder preflight rejects missing required streams, type/QoS mismatch, and
  insufficient disk before arming.
- Repeated start/stop/finalize (and policy attach) are idempotent.
- HTTP/WebSocket contract tests cover domain APIs used by LeLab-class clients.
- Frontend schema contains no robot names or ROS topic strings as primary UI
  data.
- Online-RL obs–action sync and replay-buffer contracts are tested separately
  from MCAP finalize.
- Multi-camera MCAP throughput remains the recording performance authority.
- Cross-host bringup: RT and policy hosts agree on `profile_hash` before active
  control.
