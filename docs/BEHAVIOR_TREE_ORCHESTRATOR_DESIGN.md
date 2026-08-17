# Behavior Tree Orchestrator & Human-in-the-Loop DAgger Architecture Plan

> **Status**: Implemented baseline  
> **Target Subsystem**: `src/execution/orchestrator`  
> **Key References**: ROBOTIS
> [`cyclo_intelligence/orchestrator`](https://github.com/ROBOTIS-GIT/cyclo_intelligence/tree/main/orchestrator)
> and its Behavior Tree runtime,
> `docs/EXECUTION_MANAGER_ARCHITECTURE_DECISIONS.md`,
> `docs/EXECUTION_MANAGER_SDK_REFACTOR_PLAN.md`, `src/interfaces/rmi`

> **Scope note (2026-08-09):** Behavior trees are an optional example of an
> upper-level RMI consumer, not the selected Phase 6 architecture. Canonical
> execution and orchestration boundaries are defined by the RMI and Runtime
> Orchestration documents; this proposal must not drive RMI ownership or APIs.

> **Revision note (2026-08-11):** Superseded the original `py_trees` +
> `await em.*` leaf-node sketch below (sections 2–5 as originally written).
> The Orchestrator is now scoped as a **thin BT runtime only** — tree loader,
> tick engine, blackboard, task lifecycle, node registry, status reporting.
> Every leaf node calls the RMI SDK (`RobotFacade.control()` / `.execute()`,
> `RecorderFacade.episode()`, `Context`) synchronously; it never talks to EM,
> ros2_control, or ROS transports directly. Teleop arbitration lives entirely
> in EM/RMI — the tree only observes ownership facts, it never calls
> `handover()` itself. Cyclo's Orchestrator (`bt_node.py`, `bt_core.py`,
> `blackboard.py`, `node_registry.py`, `bt_nodes_loader.py`, `actions/`,
> `controls/`, `templates/`, `trees/`) is the structural reference for the
> runtime shape, not for its leaf-node semantics.

---

## 1. Executive Summary & Design Vision

In real-world Physical AI and long-horizon manipulation (e.g., 5–10 minute multi-step assembly or bin-picking), pure end-to-end monolithic models suffer from **severe error accumulation**. When a single sub-step encounters out-of-distribution (OOD) states, the entire task collapses, and diagnosing the root cause is exceptionally difficult.

This document defines the architectural blueprint for refactoring the Orchestrator into a **pure Behavior Tree (BT) Task Graph Engine** layered cleanly above our **RMI Core SDK** (`Robot`, `ExecutionManager`, `Recorder`, `ProviderLifecycle` (protocol)).

### Key Capabilities Enabled:

1. **Modular Skill Composability**: Breaking long-horizon tasks into discrete, modular Skill Policies (`PickSkill`, `AlignSkill`, `AssembleSkill`). Each Skill is independently runnable, testable, and upgradable.
2. **Human-in-the-Loop DAgger Pipeline**: When a specific Skill encounters low confidence or OOD errors, the human operator intervenes via teleoperation (`TeleopProvider`). RMI `ExecutionManager` handles instant preemption, and `Recorder` automatically marks and tags the intervention segment in MCAP for targeted DAgger fine-tuning.
3. **Hot-Pluggable Skill Updating**: Operators can isolate a low-performing Skill, collect intervention data, fine-tune that specific policy, and hot-swap it back into the BT Task Graph without re-training the entire long-horizon pipeline.
4. **Online RL & Shared Autonomy Safety Guarding**: 1kHz C++ hardware limits (ADR 3) enforce joint/torque safety bounds, while EM fault events pass negative reward (penalty) signals back to Online RL exploration algorithms.

---

## 2. Layered Architecture & Separation of Concerns

The Orchestrator is a **thin BT runtime**, not a mixed bag of Robot SDK,
ExecutionManager client, Recorder SDK, Gym adapter, dataset exporter, and ROS
transport code. Everything below the tree belongs to RMI:

```text
Web UI / CLI
      │
      ▼
Orchestrator
  BT loader / tick engine
  blackboard
  task lifecycle (start / abort / pause / resume)
  node registry
  status reporting (current node, tree state, failure reason)
      │
      ▼
RMI
  Robot / Camera / Sensor facades
  Policy / Teleop / Planner action sources
  ControlSession (robot.control())
  Recorder (recorder.episode())
  EM events / status
      │
      ▼
EM / ros2_control / hardware
```

* **Orchestrator (What & When)**: tree structure, node lifecycle, blackboard
  data flow, task-level start/abort/pause/resume, and observability for the
  Web UI (node catalog + live tree status). It returns `SUCCESS` / `FAILURE`
  / `RUNNING` and nothing else — no ROS publishing, no EM RPCs, no dataset
  I/O of its own.
* **RMI (How & Safety)**: `RobotFacade.control()` / `.execute()` for arm
  commands and planner-executed plans, `RecorderFacade.episode()` for MCAP
  lifecycle, EM priority arbitration, generation fencing, and `ros2_control`
  STRICT controller switching. Leaf nodes call these APIs directly and
  synchronously — there is no `await em.*` anywhere in the tree.

### Cyclo-inspired runtime shape

Cyclo Intelligence's Orchestrator (fixed-frequency tick, dynamic node
catalog, XML tree, blackboard, run-state visualization) is the reference for
*how the runtime is organized*, not for what the leaves do:

```text
bt/
├── bt_node.py
├── bt_core.py
├── blackboard.py
├── node_registry.py
├── bt_nodes_loader.py
├── actions/
├── controls/
├── templates/
└── trees/
```

Our leaf nodes call RMI instead of publishing ROS commands directly.

---

## 3. RMI-Backed Leaf Nodes

Leaf nodes are synchronous wrappers around RMI SDK calls — no `await em.*`.

### 3.1 `RunPolicy` (Action)

```python
class RunPolicy(Node):
    def initialise(self):
        self.control = self.robot.control(self.source, resume=True)
        self.control.__enter__()

    def update(self):
        obs = self.observe()
        result = self.policy(obs)

        if result.done:
            return SUCCESS

        if result.uncertain:
            self.blackboard.recovery_target = result.recovery_target
            return FAILURE

        self.control.send(result.action, observation=obs)
        return RUNNING

    def terminate(self, status):
        self.control.__exit__(None, None, None)
```

`policy` is the inference capability and `source` is the distinct RMI
`ActionSource`. They may share a logical name in a profile, but the
Orchestrator never assumes the inference object can command the robot.

### 3.2 `ExecuteRecoveryPlan` (Action)

```python
class ExecuteRecoveryPlan(Node):
    def update(self):
        plan = self.planner.plan(
            robot=self.robot,
            target=self.blackboard.recovery_target,
        )
        if not plan.valid:
            return FAILURE

        with self.robot.control(self.planner_source):
            result = self.robot.execute("arm", plan).wait()

        return SUCCESS if result else FAILURE
```

### 3.3 `RecordEpisode` (Decorator, not a Sequence action)

The recorder wraps its child subtree instead of appearing as a sibling
start/stop pair of actions:

```text
RecordEpisode(task="pick")
└── Sequence
    ├── PickPolicy
    ├── RecoverWithPlanner
    └── PlacePolicy
```

which corresponds to:

```python
with recorder.episode(task="pick"):
    tick_child_until_terminal()
```

### 3.4 Teleop — observed, not driven

Teleop is an independent `ActionSource` / process; EM alone handles
high-priority preemption. The tree never calls `handover()` — it only
observes the ownership fact RMI already exposes on the `Observation`:

```text
PolicySkill
├── Policy currently owns arm       → RUNNING
├── Teleop currently owns arm       → RUNNING / INTERVENED
└── Policy resumed, obs refreshed   → continue RUNNING
```

This keeps arbitration and generation handling exclusively inside EM/RMI;
the BT does not duplicate or shadow it.

---

## 4. Behavior Tree Task Graph Scenarios

### Scenario A: Imitation Learning Teleop Data Collection Tree

```text
RecordEpisode(task="teleop_collect")
└── Sequence
    ├── PolicySkill              (RUNNING while Teleop owns arm → INTERVENED)
    └── UntilOperatorDone
```

### Scenario B: DAgger & Multi-Skill Long-Horizon Task Graph

```text
RecordEpisode(task="assembly")
└── Sequence
    ├── RunPolicy(policy=Pick_v1)          ──> SUCCESS
    ├── Fallback
    │     ├── RunPolicy(policy=Align_v2)   ──> FAILURE (uncertain / OOD)
    │     └── ExecuteRecoveryPlan          ──> SUCCESS (resumes policy after)
    └── RunPolicy(policy=Fasten)           ──> SUCCESS
```

---

## 5. Target Layout & Module Migration

### 5.1 Target directory

```text
src/execution/orchestrator/
├── orchestrator/
│   ├── runtime.py
│   ├── tree.py
│   ├── blackboard.py
│   ├── registry.py
│   ├── status.py
│   ├── nodes/
│   │   ├── policy.py
│   │   ├── planner.py
│   │   ├── teleop.py
│   │   ├── recorder.py
│   │   ├── conditions.py
│   │   └── task.py
│   ├── controls/
│   │   ├── sequence.py
│   │   ├── fallback.py
│   │   ├── parallel.py
│   │   └── retry.py
│   └── loaders/
│       └── xml.py
├── trees/
├── launch/
└── test/
```

### 5.2 Completed module disposition

The former mixed runtime has been removed from
`src/execution/orchestrator/orchestrator/`. Its completed disposition is:

| Current module | Disposition |
| --- | --- |
| `transport_adapter.py` | Deleted — superseded by RMI `Context` |
| `session.py` | Deleted — superseded by `RobotFacade.control()` |
| `group_handle.py` | Deleted — superseded by `RobotFacade` / RMI action API |
| `observation_reader.py` | Removed — RMI owns observation/alignment |
| `safety_guard.py` | Hard safety belongs to the controller; event conditions read RMI |
| `episodes.py` | Replaced by the `RecordEpisode` decorator (§3.3) |
| `preflight.py` | Capability checks sink into RMI; BT only consumes the result |
| `calibration_manager.py` | Split out into an independent calibration capability |
| `dataset_exporter.py` | Split out into an independent offline tool |
| `gym_adapter.py` | Deferred adapter — not part of the Orchestrator |
| `action_normalizer.py` | Policy / streamer capability |
| `launch_helper.py` | Deployment / app launch layer |
| `feature_schema.py` | Policy / data schema layer |
| `orchestrator.py` | Replaced by `runtime.py` + `tree.py` |
| `server.py` | Reduced to task control + tree status API |

### 5.3 Implemented phases

**Phase 1 — Runtime core**: `runtime.py`, `tree.py`, `blackboard.py`,
`registry.py`, `status.py`, and the `controls/` node set
(sequence/fallback/parallel/retry), plus the XML tree loader.

**Phase 2 — RMI leaf nodes**: `nodes/policy.py`, `nodes/planner.py`,
`nodes/teleop.py`, `nodes/recorder.py`, `nodes/conditions.py`, `nodes/task.py`
as described in §3, backed by `src/interfaces/rmi`.

**Phase 3 — App surface**: trim `server.py` to task start/abort/pause/resume
and tree-status endpoints for the Web UI, expose the node catalog from
`registry.py`.

**Phase 4 — Extraction**: non-orchestration capabilities are no longer exported
or owned by this package. Dataset conversion and Gym/LeRobot adapters remain
deferred outside the native Orchestrator contract.

Application deployments construct RMI policy, planner, source, camera, and
recorder objects, then inject a `NodeContext`. The generic CLI accepts a
`module:callable` context factory; it does not guess how to construct model
backends from names in an XML file.
