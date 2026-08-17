# RMI Application API Refactor Discussion

> **Status:** native RMI refactor baseline implemented. The examples describe
> the current application-facing contracts covered by offline tests. ROS graph
> and fake-hardware gates called out below still require a normal workstation.
>
> **Current scope:** make the native RMI API coherent. LeRobot and Gymnasium
> adapters are intentionally deferred.

## 1. Design direction

RMI should present small synchronous Python objects while ROS executors,
services, actions, and event subscriptions run behind those objects.

```python
context = rmi.Context.from_profile("marvin_bimanual.yaml")
robot = context.robot
camera = context.make_camera("head_camera")
sensor = context.make_sensor("wrench", topic=..., message_type=...)
policy_agent = context.make_agent(
    "Policy", robot=robot, sensors=[camera, sensor]
)
teleop_agent = context.make_agent("Teleop", robot=robot)
planner = context.make_planner("curobo")
resolver = context.make_resolver("curobo_ik")
streamer = context.make_joint_streamer("curobo_mpc")
recorder = context.make_recorder()
```

Agent scheduling defaults are application configuration, separate from EM
provider arbitration:

```yaml
agents:
  Policy:
    provider: Policy
    frequency: 30.0
  CuroboStreamer:
    provider: Planner
    frequency: 30.0
```

`Agent.run()` inherits this frequency. The lightweight `Session` exposes
`frequency`, `period`, and monotonic `wait()`; a streaming planner may pass
`session.period` explicitly to its `step(..., dt=...)` call. One-shot planners
do not need to call `wait()`.

The object roles are:

| Object | Role |
| --- | --- |
| `Robot` / `Part` | Embodiment state and command facade |
| `Camera` / `Sensor` | Timestamped observation producers |
| `Policy` | Learned action producer |
| `Teleop` | Human-input action producer and retargeter |
| `Resolver` | One-request pose IK or joint-target validation |
| `Planner` | One-shot complete timed joint-trajectory generation |
| `JointStreamer` | App-clocked receding-horizon joint guidance |
| `CartesianStreamer` | App-clocked receding-horizon pose guidance |
| `Recorder` | Independent episode lifecycle and recording facade |
| `Execution` | Advanced EM status, event, and diagnostic access |

`Policy`, `Teleop`, and future `Replay` objects are concrete `Agent` roles.
Resolver/planner/streamer facades are
computation capabilities: merely producing a solution, plan, or horizon does
not grant control authority. Gate 3 will compose their results with a Planner
Agent for execution through EM.

The Execution Manager (EM) remains a thin RT-host action-plane guard. It owns:

- source ownership and priority arbitration;
- part-level and partial allocation;
- provider handover and controller routing;
- generation and sequence fencing;
- bounded action/chunk admission, stale checks, and watchdog behavior;
- source, handover, rejection, and execution events.

Hard realtime joint, velocity, torque, workspace, and hardware-fault checks
remain in the C++ controller/hardware path. Sensors, policy inference, motion
planning, observation assembly, and recording do not move into EM.

## 2. Basic application form

Policy control:

```python
camera = context.make_camera("head_camera")
policy_agent = context.make_agent("Policy", robot=robot, sensors=[camera])

with policy_agent.run(robot, resume=True) as session:
    while True:
        observation = session.observe()
        action = policy_infer(observation)  # app-owned inference
        session.act(Action("arm", "joint_reference", action), observation=observation)
```

Teleoperation:

```python
teleop_agent = context.make_agent("Teleop", robot=robot)

with teleop_agent.run(robot) as session:
    while True:
        observation = session.observe()
        action = teleop_get_action(observation)
        session.act(Action("arm", "joint_reference", action), observation=observation)
```

One-shot planning and execution:

```python
planner = context.make_planner("curobo")
planner_agent = context.make_agent("Planner", robot=robot)
plan = planner.plan(robot=robot, target=target)

if plan.valid:
    with planner_agent.run(robot) as session:
        session.execute("arm", plan).wait(timeout=10.0)
```

Recording is orthogonal to the active action source:

```python
recorder = context.make_recorder()

with recorder.episode(
    task="pick",
    metadata={"policy": "diffusion"},
) as episode:
    with policy_agent.run(robot, resume=True) as session:
        run_policy_loop(policy_agent, session)

print(episode.final_status.episode_path)
```

An `Episode` owns task and recording lifecycle, while an Agent `Session` owns
execution authority and optional loop timing. Neither lifecycle controls the
other, so one Episode may contain Policy, Teleop, and Planner Sessions.

The local `Recorder` object may be a client for a writer on another deployment
host. Writer placement is profile-owned and must not change this API.
The facade is synchronous; internally it delegates to the existing
`episode_recorder` SDK and waits for transactional finalization on scope exit.
The task and manifest metadata are updated together immediately before each
start, so an already-active lifecycle node does not retain stale episode labels.

## 3. Policy, Teleop, and Planner participation

Several sources may be READY concurrently. For any controlled Part, EM admits
commands from only the currently allocated source.

### 3.1 Teleop intervention during a policy loop

Policy and Teleop normally run as independent processes. The Policy process
does not need a direct connection to the Teleop process.

Policy application:

```python
policy = context.make_agent("Policy")
with policy.run(robot, resume=True) as control:
    while True:
        observation = robot.get_observation()
        action = policy_infer(observation)
        control.send(Action("arm", "joint_reference", action), observation=observation)
```

Teleop application:

```python
teleop = context.make_agent("Teleop")
while True:
    if teleop_takeover_requested():
        with teleop.run(robot):
            while teleop_takeover_active():
                robot.send_action(
                    Action("arm", "joint_reference", teleop_get_action(robot.state))
                )
```

The intended ownership sequence is:

```text
Policy ACTIVE
  -> Teleop acquire
Policy READY/displaced; Teleop ACTIVE
  -> Teleop release
Policy reset from current observation; Policy ACTIVE with a new generation
```

EM events provide direct intervention labels. A recorder should retain the
acquire, displacement, release, reset, resumed, accepted-action, and
rejected-action evidence instead of reconstructing ownership from commands.

### 3.2 Planner recovery after policy uncertainty

Planning and trajectory execution are separate operations. Planning alone
does not require control ownership.

```python
policy = context.make_agent("Policy")
planner = context.make_planner("curobo")
planner_source = context.make_agent("Planner")

with policy.run(robot, resume=True) as policy_control:
    while True:
        observation = robot.get_observation()
        prediction = policy_predict(observation)

        if prediction.confidence >= 0.7:
            policy_control.send(
                Action("arm", "joint_reference", prediction.action),
                observation=observation,
            )
            continue

        plan = planner.plan(
            robot=robot,
            start=observation,
            target=prediction.recovery_target,
        )
        if not plan.valid:
            break

        with planner_source.run(robot):
            robot.execute("arm", plan).wait(timeout=10.0)
```

Only execution changes the active source:

```text
Policy ACTIVE
  -> planner computes a candidate while Policy retains ownership
  -> valid plan and Planner acquire
Planner ACTIVE; Policy READY
  -> trajectory completes and Planner releases
Policy reset from current observation; Policy ACTIVE
```

### 3.3 Planner proposal consumed by Policy

When Planner supplies a waypoint, cost, subgoal, or candidate trajectory but
Policy remains the final action producer, Planner does not acquire control:

```python
observation = robot.get_observation()
proposal = planner.propose(robot=robot, start=observation, target=task.target)
action = policy(observation=observation, planner_proposal=proposal)
policy_control.send(action, observation=observation)
```

Attribution rule:

- a proposal consumed and modified/selected by Policy is a Policy action;
- a planner trajectory sent directly for controller execution is a Planner
  action and requires Planner ownership through EM.

### 3.4 Partial ownership

EM already permits different sources to own different Parts. For example,
Planner may execute arm trajectories while Policy retains both grippers:

```python
with policy.run(robot, parts=all_parts, resume=True) as policy_control:
    with planner_source.run(robot, parts=["left_arm", "right_arm"]):
        robot.execute("left_arm", arm_plan).wait()
        # Nested robot.send_action routes by Part to the owning scope.
        robot.send_action(
            Action("left_gripper", "joint_reference", gripper_cmd),
            observation=robot.get_observation(),
        )
```

## 4. Inference crossing a control handover

The correct behavior at `T3` is to compute again from a fresh `T3`
observation. The race exists because a synchronous model call that started at
`T0` cannot retroactively replace its input while it is running:

```text
T0  read observation O0; start Policy inference under generation 10
T1  Teleop acquires control
T2  Teleop releases; Policy resumes under generation 12
T3  the already-running inference(O0) returns action A0
```

`A0` must not be retained or sent. It is discarded, then the loop reads a new
observation and invokes Policy again:

```text
T3  discard A0 because observation/control generation 10 != current 12
T3  read O3
T3  start inference(O3)
```

`generation` is an EM allocation incarnation, not an inference identifier.
Every acquire/resume produces a new ownership generation so work associated
with an earlier ownership interval can be fenced.

The normal public API requires neither an explicit `InferenceToken` nor an
action receipt:

```python
observation = robot.get_observation()
action = policy_infer(observation)
control.send(Action("arm", "joint_reference", action), observation=observation)
```

The timestamped observation carries the control snapshot internally. Before
publishing, the policy-host control facade compares that snapshot with the
latest authoritative EM allocation. A changed generation causes a local
discard; RT-host admission remains the final guard. Rejection details remain
available through `robot.execution` diagnostics and EM events, not as required
control-loop return handling.

This policy-host check is required because native ROS command messages do not
currently carry the generation at which inference began. Header staleness is
useful but cannot prove that no short takeover occurred during inference.

### 4.1 Shadow prediction while another source owns control

Action generation and action output are separate. Policy may continue to run
while Teleop or Planner is ACTIVE, for monitoring, uncertainty estimation, or
warm inference state:

```python
policy = context.make_agent("Policy")
with policy.run(robot, resume=True) as control:
    while True:
        observation = robot.get_observation()
        prediction = policy_infer(observation)  # always allowed
        # shadow/gated publish: Session.send drops when inactive/stale
        control.send(
            Action("arm", "joint_reference", prediction),
            observation=observation,
        )
```

While Policy is not the active owner, `control.send(...)` drops locally
(counted in diagnostics) and does not enqueue or publish a command. The
prediction may still be recorded as a shadow prediction; it is not an
attempted robot action.

On Policy displacement, its output side must:

1. close the current output generation;
2. clear queued action chunks and pending output;
3. prevent an inference started in the old generation from publishing later.

On resume, it must:

1. sample the current robot observation;
2. reset/re-anchor temporal policy state when the policy requires it;
3. open a new output generation;
4. admit only predictions based on observations from that new generation.

The last shadow prediction produced during Teleop ownership is not
automatically executed after release. A fresh post-resume observation produces
the first executable Policy action. Ownership events and shadow predictions
remain observable through diagnostics/recording without complicating the
normal application loop.

## 5. Current motion-planner capability audit

The planner implementation has useful backend-neutral foundations. Gate 2 now
provides the pure one-shot planning portion of the application API; execution
and the other planner families remain intentionally separate work.

### 5.1 Already available

- Backend protocols are ROS-independent:
  `ResolverBackend.resolve()`, `PlannerBackend.plan()`,
  `JointStreamerBackend.step()`, and `CartesianStreamerBackend.step()`.
- Backend-neutral `JointState`, `CartesianState`, `World`, `ResolveResult`,
  `PlanResult`, `JointHorizonResult`, and `PoseHorizonResult` contracts exist.
- PyRoki and cuRobo implement resolver, trajectory-planner, and joint-streamer
  backends; ndcurves implements the Cartesian streamer backend.
- `RobotContext` separates state, model metadata, and execution feedback.
- Planner source wrappers validate state/target freshness and publish
  diagnostics.
- RMI profiles already declare Planner as a peer provider with trajectory
  action endpoints, and `ActionProviderClient` can submit a complete joint
  trajectory through the EM gateway.

### 5.2 Missing application-facing composition

1. Explicit `Context.make_planner()`, `make_resolver()`,
   `make_joint_streamer()`, and `make_cartesian_streamer()` factories now
   preserve the four backend capabilities; configuration-driven adapter
   registration remains open.
2. Applications must construct backend configuration, robot loader,
   `RobotContext`, source wrapper, timers, and output sink themselves.
3. The legacy `TrajectoryPlannerSource.execute_plan()` still combines solving
   and dispatch, while the new facade path separates pure `planner.plan(...)`
   from `robot.execute(part, plan)`.
4. Planner contracts are not yet adapted to RMI `Robot`, `Part`, observation,
   world, or action types.
5. `PlanExecution` now exposes synchronous wait, cancel, terminal result,
   latest JTC feedback, bounded trajectory progress, and EM event correlation.
6. Resolver and streamer families need facade behavior distinct from one-shot
   segment planning; one overloaded `plan()` method must not erase those
   semantics.
7. Cartesian streamer output is computed but its current source wrapper does
   not dispatch `PoseHorizonResult` through a command sink.
8. `PlannerSourceConfig.command_sink_mode` defaults to `"em"`, while
   `make_command_sink()` currently supports only `direct` and `recording`.
   Production RMI/EM composition therefore remains an injected, unfinished
   boundary rather than a working default.
9. Planner readiness, warmup, world revision, plan diagnostics, and execution
   events are not unified under one public health/status contract.

### 5.3 Target facade without flattening planner families

```python
context.register_planner("curobo", make_curobo_backend)
planner = context.make_planner("curobo")  # lazy construction and warmup
planner_source = context.make_agent("Planner")
planner.update_world(world)

plan = planner.plan(robot=robot, target=target)
if plan.valid:
    with planner_source.run(robot):
        execution = robot.execute("arm", plan)
        execution.wait(timeout=10.0)
```

Resolver:

```python
context.register_resolver("curobo_ik", make_curobo_ik_backend)
resolver = context.make_resolver("curobo_ik")

with resolver_source.run(robot) as control:
    while control.active:
        observation = robot.get_observation()
        solution = resolver.resolve(
            robot=robot,
            start=observation,
            target=latest_pose_target(),
        )
        if solution.valid:
            control.send(
                Action("arm", "joint_reference", solution),
                observation=observation,
            )
```

`resolve()` is one backend request yielding one `q*`; it is not necessarily a
one-shot application behavior. The app/source loop may call it periodically
with the latest observation and target, producing a stream of 1-point
`joint_reference` commands directly for JSPC. Resolver does **not** feed a
`JointStreamerBackend` in this standard path.

Online joint streamer:

```python
context.register_joint_streamer("curobo_mpc", make_curobo_mpc_backend)
streamer = context.make_joint_streamer("curobo_mpc")

with streamer.run(robot) as control:
    streamer.reset(robot=robot)
    streamer.set_target(target)
    while control.active:
        horizon = streamer.step(robot=robot, dt=0.02)
        control.send(horizon)
```

Cartesian streaming deliberately takes an explicit measured end-effector pose,
because the current RMI robot observation contains joint state but does not yet
provide an FK/TF-derived Cartesian state:

```python
streamer = context.make_cartesian_streamer("ndcurves")
streamer.reset(measured_ee_pose)
streamer.set_target(target_pose)
horizon = streamer.step(measured_ee_pose, dt=0.02)
```

The distinct facade types prevent accidental use of `plan()` on a resolver or
`step()` on a segment planner. The examples show future execution composition;
Gate 2 facades themselves return results but do not implement `control.send()`.

## 6. Refactor gates

The refactor should advance through narrow, observable gates.

### Gate 1: synchronous native RMI object facade

Implement shared context plus synchronous `Robot`, `Camera`, `Sensor`, and
action-source factories over the existing clients.

**Implementation status:** the current slices provide shared `Context`,
`Context.robot` (`Robot`), timestamped joint-state `Observation`, lazy
profile-declared `Agent` and `CameraFacade`, generic `SensorFacade`,
synchronous control scopes, bounded sensor histories, and a local
ownership/observation-generation output gate. Generic non-camera sensors are
still created from explicit topic/message/converter arguments because the
profile has no generic sensor schema yet. Final low-level `Robot` migration
also remains open, so Gate 1 is not yet complete.

Pass evidence:

- no application-created ROS node is required;
- readiness, state reads, and source acquire/release work with fake hardware;
- current RMI/EM tests continue to pass.

### Gate 2: pure Planner facade

Adapt one existing trajectory backend to:

```python
plan = planner.plan(robot=robot, target=target)
```

No command is published in this gate.

**Implementation status:** implemented for all four pure backend capabilities
through `PlannerCatalog`: `PlannerFacade`, `ResolverFacade`,
`JointStreamerFacade`, and `CartesianStreamerFacade`. Adapter factories are
registered on the application `Context`, constructed lazily, and cached in
separate capability namespaces. Joint-based facades convert timestamped RMI
observations into backend-neutral `JointState`; solve/step calls optionally
reject stale observations and return family-specific invalid result objects.
The facades have no EM client, command sink, private streaming timer, or command
dispatch method. Profile-based cuRobo/PyRoki/ndcurves registration is deferred
to a later configuration slice.

Pass evidence:

- deterministic `PlanResult` for fake/current state;
- invalid/stale state and invalid target return explicit reasons;
- backend diagnostics are preserved.

### Gate 3: Planner execution through EM

Implement:

```python
planner_source = context.make_agent("Planner")
with planner_source.run(robot):
    robot.execute("arm", plan).wait()
```

The pure `PlannerFacade` computes the plan; the profile-declared Planner
`Agent` owns execution authority. They are deliberately separate
objects even when an application gives them the same human-facing name.

**Implementation status:** the first synchronous execution slice is
implemented. `robot.execute(part, plan)` requires an active control generation,
converts `PlanResult` into the canonical complete trajectory, routes it through
the provider's EM `joint_trajectory` action, and returns `PlanExecution` with
synchronous `wait(timeout)` and `cancel()`. Resolver, joint-streamer, and
Cartesian-streamer results are accepted as explicit `Action.value` objects and
converted to their profile-declared topic wires. `PlanExecution.state` exposes
accepted, completed, cancel-requested, failed, and timed-out local states. The
gateway carries the ROS action goal UUID as an optional EM `correlation_id` and
records authoritative `trajectory_accepted`, `trajectory_succeeded`,
`trajectory_canceled`, or `trajectory_aborted` events. Advanced callers may use
`execution.events` / `execution.wait_event(...)`; normal control loops do not
handle receipts. Downstream JTC feedback is proxied through the gateway;
`execution.feedback` exposes the latest sample and `execution.progress`
normalizes desired trajectory time against the plan duration. Fake-hardware
controller-level integration remains open.

#### Gate 3 fake-hardware validation

This is a single-arm FR3, no-gripper gate. The smoke trajectory contains two
identical copies of the measured joint state, so it validates the action path
without requesting displacement. The client refuses to run unless the explicit
`--confirm-fake-hardware` flag is present.

Terminal 1:

```bash
pixi run rmi-fr3-fake-rt
```

Expected before running the client:

- `franka_arm_jtc` is loaded and available for EM switching;
- `Authoritative RMI ExecutionManager is ready`;
- `/joint_states` is publishing from `mock_components/GenericSystem`;
- no trajectory has been sent.

Terminal 2:

```bash
pixi run rmi-fr3-fake-plan-smoke
```

Pass evidence is one JSON line with `state: "COMPLETED"`, a non-empty
`correlation_id`, and a `trajectory_succeeded` terminal event carrying the same
ID. Failure to confirm the profile hash, receive joint state, acquire Planner,
reach JTC, or receive the terminal action result makes the command non-zero.

**Current validation status (2026-08-11):** static contract tests and selective
build of `motion_planner_core`, `rmi`, and the canonical
`src/new_apps/rt/franka_manipulation_controller_bringup` package pass. The ROS
graph run is not validated inside the Codex sandbox: CycloneDDS fails interface
enumeration, while Fast DDS can create processes but cannot create sockets or
discover peers (`getifaddrs: Operation not permitted`). Consequently the
controller manager cannot receive `robot_description`. Run the two commands
above on the normal workstation before advancing to a non-zero fake trajectory.

Pass evidence:

- fake-hardware trajectory reaches a terminal result;
- cancellation and timeout are observable;
- direct controller bypass is absent;
- EM events correlate acquire, accepted goal, completion/cancel, and release.

### Gate 4: Policy -> Planner recovery -> Policy resume

Offline facade validation is now implemented. A nested Planner control scope
executes a `PlanResult`, release restores Policy under a newer authoritative
generation, the pre-takeover observation is fenced, and a fresh observation can
produce the next Policy action. `Session` tracks generations per Part so
a partial takeover does not incorrectly invalidate an untouched Part; the
single `generation` property remains a compatibility view of the latest value.

The remaining Gate 4 item is the same flow against the launched fake-hardware
stack on a host where DDS networking is permitted.

Pass evidence:

- policy source starts active;
- uncertainty triggers planning without ownership change;
- valid plan acquires Planner and executes through EM;
- release resets/resumes Policy with a new generation;
- pre-handover in-flight policy output is discarded;
- all transitions and action attribution are recorded.

### Gate 5: Teleop intervention and partial ownership

Validate independent policy and teleop processes, then Part-scoped takeover.

Offline facade validation is complete. `Session.active_for(part)` gates
each output independently: an arm-only Teleop takeover blocks Policy arm output
without blocking its untouched gripper output. On release, the arm adopts the
new resumed generation while the gripper retains its original generation. An
arm inference based on the pre-takeover observation remains fenced.

The FR3 recorder profile now captures raw Policy/Teleop proposals alongside
the authoritative EM status/event streams. The remaining integration evidence
is recording this flow through the launched EM and validating the finalized
episode on a host where DDS networking is permitted.

Pass evidence:

- stale policy output cannot pass after intervention;
- untouched Parts retain their original source allocation;
- recorder output contains authoritative intervention and generation events.

## 7. Deferred work

- LeRobot Robot/Camera adapters;
- Gymnasium `reset()` / `step()` environment;
- direct LeRobot dataset recording;
- distributed recording topology changes;
- a universal planner API that hides resolver/planner/streamer differences.

These can build on the native RMI contracts after the gates above are stable.
