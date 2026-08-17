# Execution Manager and Robot SDK Refactor Plan

> **Status (2026-08-09): Phase 5 complete for fake-hardware scope.** RMI is
> the canonical implementation of Robot/controller access and ExecutionManager
> behavior. The former execution-manager package has been removed from the
> active workspace. Phase 6 is only consumer migration and does not select or
> require a particular orchestration architecture.

## 1. Purpose

This document defines the target boundary and migration plan for the Robot SDK,
Execution Manager (EM), controller clients, embodiment profiles, and episode
recording integration.

The design intentionally reuses established concepts:

- `strands-robots`: `Robot`, policy/action-provider, action-chunk, and sim/real
  SDK ergonomics;
- `crisp_py`: a Python robot client that talks to an already running remote
  ROS 2 controller stack through controller and controller-manager clients;
- `robosuite`: Robot -> Part -> part controller / composite controller
  composition;
- `episode_recorder` 0.3.0: transactional, stream-first ROS 2 episode
  recording and validation.

The Runtime must add one capability not supplied by those projects: an EM that
keeps multiple ActionProviders ready and can hand control between policy,
teleoperation, and motion-planning providers while the robot is running.

## 2. Ownership boundaries

The target composition is:

```text
Application / Agent / Lab frontend
  |
  +-- Robot SDK -------------------------- embodiment and controller clients
  |     +-- Robot
  |          +-- Parts
  |          |    +-- controller clients
  |          |    +-- controller-switcher client
  |          +-- composite controllers
  |          +-- sensors / observations
  |
  +-- ExecutionManager ------------------- provider lifecycle and handover
  |
  +-- Recorder --------------------------- recording-session facade
```

`ExecutionManager` and `Recorder` are not children owned by `Robot`. They are
independent runtime services that operate on a Robot:

```python
robot = Robot.from_profile(profile)
em = ExecutionManager(robot)
recorder = Recorder(recording_config)
```

Applications may aggregate them in an application/runtime object for
convenience, but that aggregation is not part of the Robot embodiment model.

### 2.1 Robot owns

- the Part hierarchy;
- joint and frame metadata;
- sensors and observation clients;
- controller clients attached to each Part;
- a controller-switcher client for each distinct controller manager;
- composite-controller views over multiple Parts.

### 2.2 Robot does not own

- Action Provider priority or lifecycle;
- human/policy/planner arbitration;
- recording-session lifecycle;
- dataset conversion or training;
- remote controller-server process lifecycle.

The ROS 2 controller bringup is expected to be running on the RT/controller PC.
The SDK is a client on the same ROS domain and does not need to launch or own
that remote server.

### 2.3 ExecutionManager owns

- Action Provider registration and readiness;
- provider `STOPPED -> STARTING -> READY <-> ACTIVE -> FAILED` lifecycle;
- selection of the active provider for each controlled Part set;
- same-controller and cross-controller handover;
- invalidation of stale chunks after a handover;
- provider/controller-switch events used for intervention labels;
- coordination of shared-epoch actions for composite controllers.

### 2.4 Recorder owns

- recorder-node lifecycle and episode commands;
- stream profile or arbitrary stream-contract selection;
- start, stop, discard, status, finalization, and recovery;
- access to validation, time-index, clock-mapping, and episode-reader tools.

The authoritative data plane remains `episode_recorder` 0.3.0. The SDK does
not implement another bag writer or another stream-contract format.

## 3. Robot, Part, and controller clients

### 3.1 Part hierarchy

Parts form the embodiment tree:

```text
Robot
  +-- base
  +-- torso
  +-- left_arm
  |    +-- left_end_effector
  +-- right_arm
  |    +-- right_end_effector
  +-- left_leg
  +-- right_leg
```

`type` may be retained as descriptive metadata, but the core must not use a
closed enum that prevents adding a new Part kind.

### 3.2 Controller map

A controller-map value is the client itself, not a descriptor containing a
second `.client` object:

```python
arm.controllers["joint_trajectory"]
arm.controllers["joint_space_reference"]
arm.controllers["task_space_reference"]
```

Concrete Python classes may use explicit client names:

```text
JointTrajectoryControllerClient
JointSpaceReferenceControllerClient
TaskSpaceReferenceControllerClient
ControllerSwitcherClient
```

The public map key describes the input contract, not the hardware command
interface. This is necessary because the existing controller plugins share
input contracts across different hardware interfaces:

| Input contract | Position-output implementation | Effort-output implementation |
| --- | --- | --- |
| timed joint reference | `JointSpacePositionController` | `JointSpaceImpedanceController` |
| timed Cartesian reference / twist | `TaskSpaceKinematicPositionController` | `TaskSpaceJointImpedanceController` |

Marvin and Piper use the position implementations. Franka may select the
effort implementations. Both variants should use the same SDK client when
their ROS input message and endpoint contract are the same.

The selected controller configuration records the implementation and hardware
interface separately:

```yaml
joint_space_reference:
  name: arm_jspc
  implementation: manipulation_position_controllers/JointSpacePositionController
  command_interface: position

task_space_reference:
  name: arm_tskpc
  implementation: manipulation_position_controllers/TaskSpaceKinematicPositionController
  command_interface: position
```

Franka can select:

```yaml
joint_space_reference:
  name: arm_jsic
  implementation: manipulation_position_controllers/JointSpaceImpedanceController
  command_interface: effort

task_space_reference:
  name: arm_tsjic
  implementation: manipulation_position_controllers/TaskSpaceJointImpedanceController
  command_interface: effort
```

The SDK still exposes `joint_space_reference` and `task_space_reference` in
both cases.

### 3.3 ControllerSwitcherClient

Each Part refers to a controller-manager client:

```python
await arm.switch_controller("task_space_reference")
```

Internally this uses:

```python
await arm.controller_switcher_client.switch_controller(
    activate=[arm.controllers["task_space_reference"].name],
    deactivate=[arm.active_controller.name],
)
```

The implementation borrows the direct-client shape from `crisp_py`, with the
following production changes:

- asynchronous service completion with bounded timeout, not busy waiting;
- `STRICT` switching;
- only controllers in the Part's declared conflict set are deactivated;
- broadcasters are explicitly configured, not detected by a name suffix;
- normal handover requires controllers to be loaded and configured by bringup;
- the result is verified with `list_controllers` before commands are released.

The controller-manager namespace is configurable because different Parts may
be served by different controller-manager instances.

## 4. Async code and realtime behavior

`async` / `await` does not enter the ros2_control realtime update loop.

The system has two timing domains:

```text
Non-realtime control plane
  Python SDK, EM decisions, DDS callbacks, controller switching, actions
                       |
                       v
Realtime data plane
  ros2_control C++ update loop, reference sampling, IK/OTG, hardware write
```

Python async code is appropriate for:

- waiting for services and actions;
- controller switching;
- provider start/stop/reset;
- atomic provider release with explicit or displaced-provider resume;
- remote inference completion;
- recording commands;
- supervision and status.

It must not be used to schedule each 500-1000 Hz hardware-control cycle.
Python/network arrival jitter is isolated by sending stamped setpoints or
bounded action chunks to the C++ controller. JSPC/TSKPC and their effort
variants sample or hold those references from their ros2_control `update()`
loop.

The relevant latency metric for handover is not Python loop frequency. It is:

```text
human/provider event
  -> EM decision
  -> optional controller-manager switch
  -> first accepted stamped chunk
  -> controller update consumes it
```

Requirements:

- no blocking call or busy loop in a ROS callback;
- no Python publisher used as the hardware servo clock;
- stamped chunks use a clock domain understood by the controller;
- controller input buffers are bounded and latest/preemption behavior is
  explicit;
- handover latency and controller-loop jitter are measured separately;
- controller-manager switching is outside the periodic realtime update code,
  although the switch is committed by controller manager at a safe update
  boundary.

Therefore `async` affects command-plane response latency, but it does not
degrade industrial controller-loop frequency or jitter when this boundary is
preserved. It cannot, however, make LAN communication hard realtime; local C++
controller buffering, stale guards, watchdogs, and timestamps remain required.

## 5. Action Provider and ExecutionManager

All producers share one provider lifecycle protocol (`ProviderLifecycle`):

```text
PolicyProvider
TeleopProvider
MotionPlannerProvider
ReplayProvider
```

`ActionReplayProvider` is a peer of policy, teleop, and planner providers. It
adapts an explicit recorded-action timeline to the current ROS clock while EM
continues to own priority arbitration, allocation generation, and admission.
Its timestamp utility supports native `JointTrajectory`, `CartesianTrajectory`,
and `TwistStamped` payloads without adding a replay wire message.

Providers target one or more Part controller contracts:

```python
TeleopProvider(target="arm", controller="task_space_reference")
MotionPlannerProvider(target="arm", controller="joint_trajectory")
```

Streaming ROS command gateways use the locked SDK/EM wire contracts directly:

- `joint_reference`: `trajectory_msgs/JointTrajectory`;
- `pose_reference`: `moveit_msgs/CartesianTrajectory`;
- `cartesian_twist`: `geometry_msgs/TwistStamped`.

Provider and Part identity come from the configured gateway source endpoint.
Generation and admission sequence are EM-owned runtime context and are not
duplicated in ROS payload messages. Message headers retain their normal frame
and ROS-time semantics.

### 5.1 Same-controller handover

For `policy/task_space_reference -> teleop/task_space_reference`, EM switches
provider generation but does not switch the ros2_control controller.

### 5.2 Cross-controller handover

For `planner/joint_trajectory -> teleop/task_space_reference`, EM:

1. stops admitting old-provider chunks;
2. cancels the downstream JTC goal;
3. invalidates the old generation;
4. samples measured and last-commanded state;
5. asks the Part to switch controller;
6. verifies the target controller is active;
7. resets the new provider from current observation;
8. admits the new generation from the handover epoch.

EM coordinates this sequence but does not implement the controller-manager
client itself.

### 5.3 Composite control

A composite action has one epoch and independent member tracks:

```text
epoch
  +-- arm track: 250 Hz or timed horizon
  +-- end-effector track: independent rate / event timing
```

Members do not need equal sample counts or frequencies. When members use
different controller managers, EM provides coordinated admission and rollback,
not a false claim of a cross-manager atomic switch.

Each member track also carries its own monotonically increasing sequence:

```text
epoch 12
  arm:            sequence 40, 41, 42, ...
  end_effector:   sequence  7,  8,  9, ...
```

EM validates the shared epoch and the sequence of each Part independently. A
duplicate arm chunk therefore does not prevent a newer end-effector chunk from
being admitted. A chunk spanning multiple Parts is admitted atomically only if
its sequence is newer for every included track.

**Implemented and covered by SDK tests.** Partial overlap is intentionally
supported: if Policy owns `arm + end_effector` and Teleop takes only `arm`,
Policy remains active for `end_effector`. Releasing Teleop restores only the
arm allocation to Policy with a new arm generation; the unchanged
end-effector generation remains valid. An already-active provider is therefore
a valid partial-release resume target and is not mislabeled as failed or
stopped.

Native command dispatch applies admission before resolving the allocated
Part's controller client. `JointTrajectory`, `CartesianTrajectory`, and
`TwistStamped` payloads use their existing ROS/MoveIt types; no composite wire
message is introduced. Multi-Part actions remain independent Part tracks under
one epoch instead of being flattened into one controller payload.

## 6. Embodiment profile decision

The existing MoveIt-style `groups` concept has value and should not be
discarded merely for naming. It already provides:

- named joint sets;
- frames;
- parent relationships;
- controller attachment;
- compound groups used by planning and execution.

The refactor should keep `groups` as the named kinematic/control grouping
concept and add explicit controller-client and controller-manager information.
`Part` is the SDK runtime object built from a group. A separate `parts:` YAML
section is not required unless a real semantic mismatch is found during the
Marvin/dual-arm migration.

The current `ros_actions` and `ros_topics` form is preferable to a generic
`client:` block because it:

- exposes transport kind directly;
- supports controllers with multiple inputs, such as CartesianTrajectory and
  TwistStamped;
- remains inspectable by launch/preflight tooling;
- avoids inventing another endpoint schema.

The problem in the current profile is not the field names. It is that endpoint
paths encode specific providers (`policy`, `teleop`, `trajectory`) while the
same group must support any number of ActionProviders.

The profile should distinguish controller-server endpoints from EM gateway source
templates:

```yaml
groups:
  arm:
    type: arm
    joint_names: [...]
    base_frame: base
    flange_frame: fr3_link8

    controller_manager: /controller_manager
    default_controller: joint_trajectory

    controllers:
      joint_trajectory:
        name: arm_jtc
        implementation: joint_trajectory_controller/JointTrajectoryController
        command_interface: position
        ros_actions:
          follow_joint_trajectory: /arm_jtc/follow_joint_trajectory

      joint_space_reference:
        name: arm_jspc
        implementation: manipulation_position_controllers/JointSpacePositionController
        command_interface: position
        ros_topics:
          joint_reference: /position_controller/joint_reference

      task_space_reference:
        name: arm_tskpc
        implementation: manipulation_position_controllers/TaskSpaceKinematicPositionController
        command_interface: position
        ros_topics:
          pose_reference: /position_controller/pose_reference
          cartesian_twist: /position_controller/twist_reference

execution_manager:
  source_namespace: /action_sources
  source_endpoint_templates:
    joint_reference: "{source}/arm/joint_reference"
    joint_trajectory: "{source}/arm/joint_trajectory"
    pose_reference: "{source}/arm/pose_reference"
    cartesian_twist: "{source}/arm/cartesian_twist"
```

This preserves the useful group model and `ros_actions` / `ros_topics`, while
preventing the Robot profile from hard-coding one policy or teleop provider.

The exact endpoint names must be generated from the actual robot bringup and EM
configuration; the example above is structural, not a final FR3 configuration.

## 7. Recorder SDK

Recorder is an independent SDK facade, not `robot.recorder` ownership.

```python
recorder = Recorder(config)
await recorder.activate()
await recorder.start_recording()
status = await recorder.get_status()
episode = await recorder.stop_recording()
```

Its API must cover every capability currently composed by
`src/apps/episode_recorder_app` and the installed `episode_recorder` 0.3.0:

- bundled profile and arbitrary `stream_config_uri`;
- lifecycle activation with fail-stop behavior;
- start, asynchronous stop/finalize, discard, and status;
- task, experiment, session, operator, and manifest context;
- duration, disk, bitrate, and bounded-queue preflight;
- immutable MCAP episode, partial recovery, health, snapshots, and checksums;
- validation profiles and device health adapters;
- time-observation index and explicit clock mapping;
- read-only episode data provider.

The dependency direction is:

```text
episode_recorder core <- Recorder SDK facade <- episode_recorder_app
```

EM provider/controller events, raw provider actions, selected execution
commands, robot state, and sensor observations remain independent recorded
streams. Intervention labels are compiled offline from this evidence.

## 8. Migration plan

### Phase 1: contracts and profile

1. Freeze `Robot`, `Part`, controller-client, switcher, `ProviderLifecycle`,
   ActionChunk, EM, and Recorder boundaries.
2. Update the embodiment parser without compatibility aliases.
3. Migrate `fr3_pika_single_arm.yaml` first and validate it against actual
   bringup controller names, plugin types, topics, and actions.
4. Migrate Piper and Marvin, verifying that position controllers reuse the
   same SDK input-contract clients as the Franka effort variants.

### Phase 2: Robot controller clients

1. Implement controller-manager client with async timeout and verified STRICT
   switching.
2. Implement JTC, joint-space-reference, and task-space-reference clients.
3. Build Parts and group/composite views from profiles.
4. Verify cross-PC discovery, readiness, send, cancel, and switch behavior.

### Phase 3: ExecutionManager

1. Move controller switching out of the EM node into the shared Part client.
2. Implement provider READY/ACTIVE lifecycle.
3. Implement handover generation and stale-chunk rejection.
4. Implement same-controller and cross-controller handover.
5. Implement composite shared-epoch admission with independent member tracks.

### Phase 4: Recorder

1. Implement the Recorder facade over `episode_recorder` 0.3.0. **Complete:**
   the workspace source checkout now provides the async facade, profile/URI
   resolution, lifecycle/service backend, and asynchronous finalization polling
   in the canonical `episode_recorder` Python package.
2. Move profile resolution and lifecycle activation from the app into the SDK.
   **Complete:** lifecycle activation and contract resolution ship with
   `episode_recorder`; application-specific YAML remains in the app.
3. Keep the app as a thin launch/profile/operator package. **Complete.**
4. Add EM provider/controller event streams to recording profiles. **Complete:**
   profiles retain `/execution_manager/events` as optional raw evidence while
   older EM deployments remain usable.

### Phase 5: removal and validation

This phase is scoped to `rmi`, ExecutionManager, ActionProviders, and their ROS
adapters. Existing higher-level orchestration packages are consumers of these
interfaces and must not block completion of the RMI refactor.

1. Confirm that no duplicate controller-switch state machines or old route
   aliases remain inside the RMI/EM implementation. **Complete:** controller
   switching is owned by each shared Part controller-manager client; profile
   parsing rejects legacy controller shapes and the old `parts` alias.
2. Run RMI/EM unit and fake-controller-manager integration tests. **Complete:**
   the Marvin RMI demos cover embodiment construction,
   lifecycle/arbitration, ROS telemetry, atomic teleop release, recorder event
   capture, and replay arbitration. Demo 7 additionally validates the real RMI
   client against Marvin's launched `ros2_control` GenericSystem: inactive to
   dual JTC plus dual Piper forward controllers, native arm/gripper dispatch,
   arm-only dual TSKPC takeover while both grippers remain owned by Policy,
   atomic arm resume, and final deactivation.
   Demo 8 covers the provider/controller switch matrix and Demo 9 validates
   the distributed RT-host/workstation topology with the authoritative RMI
   server and episode recorder.
3. Complete composite-control SDK behavior. **Complete:** partial arm takeover
   preserves an independently owned EEF track, release restores only the arm,
   and per-Part native command dispatch is fenced by provider, generation, and
   sequence before reaching a controller client. Marvin's fake bringup now
   includes two Piper grippers and forward controllers; the integration gate
   verifies both gripper joints reach the commanded `0.045 m` state while an
   arm-only Teleop takeover leaves their controllers active.
4. Validate single-arm simulation, then arm+EEF and dual-arm simulation.
   **Deferred:** wait for an environment with the required simulation and EEF
   controller coverage.
5. Validate attended real-hardware handover, beginning with same-controller
   policy-to-teleop switching before cross-controller switching. **Deferred:**
   do not run without the robot, operator, and hardware safety gates.

### Phase 6: orchestration consumer migration

This is a downstream simplification, not an RMI acceptance gate.

**Status: in progress.** The ROS/Python package and physical source directory
are both `orchestrator` (`src/execution/orchestrator`). Public class and CLI
are `Orchestrator` / `orchestrator`, with no legacy import alias. Completion of
Phase 5 does not prescribe a behavior tree, state machine, task graph, or other
consumer architecture. Phase 6 must preserve the existing orchestration
contract until its replacement contract is explicit.

1. Refactor the renamed `orchestrator` package around the completed RMI/EM
   interfaces. **Rename and canonical profile migration complete; remaining
   RMI consumer migration is in progress.** `rmi.EmbodimentConfig` is now the
   only profile parser/model, owns the installed production profiles, and
   provides Robot, EM, Recorder, and Orchestrator consumer views. The duplicate
   Orchestrator model has been removed. Marvin's EM `providers` and `sources`
   are now embedded in that profile; the server/launch no longer accepts a
   second `deployment_config` file.
2. Remove controller routing, ownership, and lifecycle behavior that duplicates
   RMI or ExecutionManager.
   - **Session/provider ownership: done.** `RuntimeSession.activate()`/
     `release()` now go directly through the synchronous
     `rmi.ExecutionManagerClient`, calling `prepare`/`acquire`/`release` on the RT-host EM, in
     place of the old `ROS2TransportAdapter.lock_execution_manager_source`
     stub (which always returned `True` and enforced nothing) and its
     `active_em_owner` field (removed). `rmi.ExecutionManagerClient` gained
     `get_status()`/`get_allocations()`/`is_ready()`, backed by a subscription
     to the RT-host's latched `/execution_manager/status` topic, so ownership
     is observable without re-deriving it locally.
     Following the `synchros2` future-waiting pattern, the client blocks on a
     future completion Event while the shared ROS executor continues spinning;
     no Orchestrator-owned asyncio loop thread remains. `Orchestrator`
     auto-wires this client from the transport's rclpy node
     when RMI is importable and an EM service is already reachable (bounded
     probe, no blind timeout), and gracefully falls back to local-only phase
     transitions otherwise (mocked transports, RMI not sourced, EM not up
     yet). `SessionManager._group_owners` remains, but only as a same-process
     fast-admission cache — it is documented as non-authoritative; RMI's
     `acquire` is the arbiter of record, and `get_status()`/`get_allocations()`
     are the authoritative, cross-process read path.
   - **Streaming motion migration: done.** `ArmHandle`/`GripperHandle` route
     joint, pose, twist, and gripper streams through RMI
     `ActionProviderClient`. It converts provider values into stamped native
   ROS messages and targets EM gateway endpoints, never ros2_control
   endpoints. Ambiguous `part.command` routes are rejected instead of
   silently selecting one of multiple providers. The production
   `ROS2TransportAdapter` now owns graph discovery and observations only; its
   duplicate motion publishing, endpoint resolution, trajectory action, and
   cancellation implementation has been removed. Unit tests now inject fake
   `ActionProviderClient` objects and verify provider calls directly; the last
   test-only motion transport fallback has also been removed.
   - **Trajectory migration: done for profiles with EM providers.** Single-arm
     and compound handles submit through `ActionProviderClient`, wait for the
     terminal ROS action result, map aborted/canceled/timeout outcomes to
     failure, and propagate cancellation. Compound trajectories submit member
     goals concurrently before waiting, preserving synchronized dispatch.
3. Replace recorder command/finalization stubs in generic transport adapters
   with the independent `episode_recorder.Recorder` facade. **Complete.**
   `EpisodeManager` now calls the Recorder SDK directly; stop waits for the
   asynchronous finalizer, and finalize verifies the recorder-owned episode
   directory plus `checksums.sha256`. Generic transport recorder stubs and the
   caller-supplied fake MCAP path have been removed. When launch has already
   activated the recorder (the demo9 deployment), the ROS backend attaches
   without rewriting parameters; only an unconfigured recorder is configured
   by the SDK.
4. Migrate its HTTP/UI/application consumers and remove obsolete aliases only
   after the new consumer contract is explicit. **Inventory started (below).**

   **`src/new_apps` migration complete.** Active Orchestrator applications use
   synchronous `Orchestrator.control_session()` ownership and provider-bound
   arm/gripper handles; they no longer reproduce prepare/acquire/release or
   select ambiguous action routes. The Gym adapter follows the same provider
   contract. All three production RT profiles now resolve to a uniform
   `rt_stack.launch.py` that composes controller bringup with the RMI
   ExecutionManager. Controller-only launch files remain diagnostic tools;
   archived demos are explicitly outside the active consumer surface.

   **Application migration boundary:** application code uses Orchestrator
   sessions/handles/episodes; ROS action-provider processes use RMI
   `ActionProviderClient`; neither application category publishes through
   Orchestrator's generic transport adapter. Profiles are loaded exclusively
   through `rmi.EmbodimentConfig` and session preparation receives that parsed
   object, never a second YAML path.

#### Phase 6.4 consumer inventory (draft)

| Consumer | Entry | Uses today | Gaps / obsolescence |
|----------|-------|------------|---------------------|
| In-process SDK | `from orchestrator import Orchestrator` | sessions, handles, episodes, EM bridge, recorder | Canonical contract target |
| CLI | `orchestrator` → `cli:main` | `--inspect`, `--serve`, default profile print | `--serve` does **not** bind a real HTTP server (prints URL and exits) |
| HTTP adapter | `OrchestratorServer` | handler methods only (capabilities, sessions, EM status, episodes, actions, operations) | No ASGI/aiohttp/FastAPI loop; contract tests assert method names, not wire protocol |
| Gym | `PhysicalRobotGymEnv` / `Orchestrator.make_gym_env` | session + `move_joints` streaming | Prototype: simulated auto-reset; not a designed RL collector |
| App demo | `franka_orchestrated_demo.py` | `Orchestrator(profile)` + session + arm handle | Migrated: parsed RMI profile is passed to session; no direct transport publish |
| App demos | `src/new_apps/orchestrator/{profile_driven_demo,single_manipulator,all_parts}.py` | profile-selected `Orchestrator` + sync control session | Same apps cover all three production profiles; local fake RT startup is explicit |
| Motion-provider demos | Franka / Marvin / Piper JTC, marker and planner demos | Native ROS publishers/actions to EM ingress | Migrate provider-by-provider against canonical RMI profiles (`piper_bimanual` added; demo source paths still use legacy `/action_sources/marker_*` / `piper_demo_*` until providers migrate) |
| LeLab / frontend | contract tests in `test_contract_frontend_lab.py` | expects domain handlers + frontend-safe schema | `get_frontend_capabilities` / `to_frontend_schema` still optional/incomplete |

**Draft consumer contract (to freeze before deleting aliases):**

1. **Primary API** is in-process `Orchestrator` (sync). HTTP/CLI/Gym are thin adapters over it — they must not re-implement ownership, routing, or recording.
2. **Domains** exposed to remote clients: `capabilities`, `sessions`, `execution_manager`, `episodes`, `operations`, `actions` (verbs already on `OrchestratorServer`).
3. **Motion** only through session-leased groups → `get_arm` / `get_gripper` / `get_group` (already RMI-backed when EM deployment present).
4. **Recording** only through `EpisodeManager` → `episode_recorder.Recorder` (done in 6.3).
5. **Ownership** only through session → synchronous RMI `ExecutionManagerClient` (done in 6.2); HTTP must not invent a second lock.
6. Before deleting adapters: implement or explicitly defer real HTTP serve; decide Gym collector scope; add frontend schema export or drop the contract test requirement.

## 9. Acceptance criteria

- `Robot` can exist without EM or Recorder.
- EM and Recorder can be constructed independently around the same deployment.
- a Part exposes controller clients directly from `controllers[...]`;
- controller clients work against a bringup server on another PC;
- Marvin/Piper position and Franka effort variants reuse input-contract client
  APIs;
- Python async code never becomes the ros2_control servo clock;
- same-controller provider handover does not switch controllers;
- cross-controller handover rejects old-generation chunks;
- groups and composite groups support dual arm and arm+EEF without a flat
  shared sample vector;
- Recorder SDK reproduces the existing app's lifecycle, profile, transaction,
  validation, and recovery behavior;
- no backward-compatibility aliases remain after migration.
