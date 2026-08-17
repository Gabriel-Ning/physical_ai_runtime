# ExecutionManager & RMI Architecture Decisions

> **Canonical status (2026-08-09):** RMI is the active workspace
> implementation of the execution capability described here. The former
> `manipulation_execution_manager` is no longer an active package. RMI exposes
> the Robot SDK, ExecutionManager, remote server/client gateways and integration
> points for the independent Recorder service. Upper orchestration is a
> consumer and may use any task architecture.

## 1. Purpose

This document records key architectural decisions made during the refactoring of `rmi` (Robot Management Interface) and `ExecutionManager` (EM), derived from technical evaluations against industry and research implementations (including `personalrobotics/robot-code`, `geodude`, and `ros2_control`).

---

## 2. Component Responsibility Definitions

### 2.1 ExecutionManager (EM)

**ExecutionManager** is the central coordination component of the Runtime control plane. Its responsibilities are defined as:

> **ExecutionManager (执行管理器)**：作为 Runtime 控制平面的协调组件，负责 **动作提供者（Action Provider）的生命周期管理与优先级仲裁（Lifecycle Management & Priority Arbitration）**、**控制器控制权切换（Controller Switching）与同控制器指令源平滑接管（Action Source Handover）**、**过时动作指令帧校验与代次防污（Stale Action Chunk Rejection & Generation Fencing）**，以及向 **Web 仪表盘（Dashboard）** 与 **数据集录制器（Episode Recorder）** 提供**运行时状态遥测（Runtime Telemetry）与人工干预事件（Intervention Event）**的发布接口。

Action Provider lifecycle semantics are:

```text
STOPPED -> STARTING -> READY <-> ACTIVE
                  failure \      \ runtime fault
                           +------> FAILED -> STOPPED
```

- normal control handover transitions the displaced provider from `ACTIVE`
  back to `READY`; it remains healthy and warm;
- an active provider may explicitly release ownership and stop while EM
  atomically resumes the provider it displaced, or a named `READY` provider;
- explicit shutdown transitions a provider to `STOPPED` after cleanup;
- `FAILED` is reserved for initialization failure, provider health failure,
  reset failure, or controller/runtime fault. Losing control ownership is not
  a failure.

### 2.2 Robot Management Interface (RMI) SDK

The **RMI public capability** contains the embodiment facade, ExecutionManager,
remote server/client gateways and controller clients. Within that capability,
`Robot` remains only the embodiment facade: it does not own execution
lifecycle, policy arbitration, or recording sessions. `ExecutionManager` and
Recorder remain independently constructible services.

### 2.3 Low-Level Controllers (ros2_control)

**Controllers** running in the C++ `ros2_control` realtime update loop (500–1000 Hz) own closed-loop setpoint interpolation, hardware command output, and hard real-time safety enforcement.

---

## 3. Architecture Decision Records (ADR)

### ADR 1: Simulation and Hardware Abstraction via `ros2_control`

#### Decision
Simulation abstraction shall be realized via standard `ros2_control` `hardware_interface` implementations (such as `mujoco_ros2_control` or topic-based hardware interfaces), rather than conditional `if sim:` branching inside the Python SDK (`rmi`).

#### Rationale
- **100% Behavioral Parity**: Controller switching, action handling, DDS message serialization, and rate dynamics remain identical between simulation and physical hardware.
- **SDK Cleanliness**: `rmi` and `ExecutionManager` connect to identical ROS 2 endpoints without environment-dependent code paths.

#### Unit Testing Exception
For fast CI/CD execution without running ROS 2 nodes or DDS daemons, `rmi` leverages Python `Protocol` interfaces (`ControllerClient`, `ControllerSwitcherClient`) to inject lightweight `MockControllerClient` stubs executing in < 10 ms.

---

### ADR 2: Runtime Telemetry & Human Intervention Event Publishing

#### Decision
`ExecutionManager` shall expose explicit runtime state telemetry and event streams for external visualization (Dashboard/UI) and data recording (`episode_recorder`).
The workstation publisher is :class:`rmi.ExecutionManagerTelemetry` (formerly
`ExecutionManagerTelemetry`): it mirrors the ros-free core onto
`/execution_manager/status` and `/execution_manager/events` and owns no
arbitration state.

#### Stream Design
1. **Dashboard Telemetry Channel (UI/Viser)**:
   - Publishes latched status updates (`/execution_manager/status`) at 10–20 Hz or upon state transitions.
   - Exposes active provider allocations per `Part`, provider lifecycle states (`STOPPED` / `READY` / `ACTIVE` / `FAILED`), and current handover generations.
2. **Episode Recorder Channel (Dataset Labeling)**:
   - Streams provider lifecycle, provider allocation, controller switch,
     generation, handover, takeover, and fault events directly to
     `episode_recorder` (MCAP).
   - These raw events are the authoritative evidence. Operator intervention
     periods and `is_intervention` are derived offline so datasets can be
     recompiled, algorithms compared, and previously unknown failure patterns
     investigated without losing the original runtime facts.
   - The initial ROS adapter publishes these facts on
     `/execution_manager/events` using a versioned JSON envelope. Every event
     retains its core timestamp and explicit clock domain, plus the ROS
     publication timestamp as `sec` / `nanosec`. This prevents steady-time
     control-plane measurements from being mistaken for the ROS/MCAP timeline.

---

### ADR 3: Hierarchical Safety Protection & Dynamic Parameter Reconfiguration

#### Decision
Hard real-time safety protections must be enforced within the 1 kHz C++ controller update loop. Low-frequency control plane components (`rmi` / `EM`) shall configure safety thresholds dynamically, but must not serve as the primary safety interlock.

#### Multi-Tier Safety Responsibilities

```text
[ Python RMI / Execution Manager ] (Low-frequency Control Plane: 10–50 Hz)
            |
            |-- Dynamic parameter update (e.g., set force limit = 15 N via Parameter Client)
            v
[ C++ ros2_control Controller ]     (High-frequency Realtime Loop: 1000 Hz)
            |
            |-- 1 kHz evaluation of joint limits, velocity bounds, F/T thresholds, workspace boundaries
            |-- On bound violation: Immediate 1 ms hold/stop + publish controller fault state
            v
[ ExecutionManager Reaction ]       (Fault Handling)
            |
            |-- Catches controller fault -> transitions the active provider to FAILED state
            |-- Initiates safe fallback or handover
```

#### Detailed Contract
- **C++ Controller**: Enforces joint limits, Cartesian velocity saturation, force/torque thresholds, and virtual workspace boundaries every 1 ms. On violation, it immediately halts hardware movement and outputs a fault status.
- **RMI SDK**: Exposes parameter reconfiguration methods (e.g., `async def set_safety_limits(...)`) allowing upper-level tasks to modify force bounds or speed limits during task phase transitions (e.g., transit vs. insertion).
- **Workstation supervisor**: May consume controller/guard diagnostics and ask
  `ExecutionManager` to fence the affected provider. This diagnostic-to-EM
  policy bridge is intentionally separate from the RT safety reaction.

---

### ADR 4: Distributed Topology & Thread Affinity

#### Decision
The authoritative `ExecutionManager` and `episode_recorder` run on the
workstation, composed by `rmi.server` (`execution_manager_server` entry point).
The RT host contains only `ros2_control`, manipulation-controller safety logic,
and one local JTC guard per trajectory controller. Application processes
interact with the workstation EM through:

1. `rmi.ExecutionManagerClient` → `rmi_msgs/ProviderCommand` service
   (`PREPARE` / `ACQUIRE` / `RELEASE` / `STOP`);
2. the deployment-declared **source endpoints** (topics/actions) for
   commands;
3. `/execution_manager/status` and `/execution_manager/events` for feedback.

Module layout follows deployment role: the core and clients are host-agnostic;
`rmi.server` composes the workstation control plane. Direct controller clients
reach `ros2_control` endpoints over ROS 2/DDS, but safety does not depend on the
workstation remaining connected.

#### Thread Affinity Contract
The EM core is **not thread-safe**. All state mutation runs on one asyncio
event loop. ROS executor callbacks (subscriptions, services, action servers)
never touch the manager directly; they post work onto the EM loop
(`run_coroutine_threadsafe`). The `ProviderCommand` service additionally
serializes commands: a second in-flight request is rejected with
`success=False` instead of blocking executor threads.

#### Workstation Provider Lifecycle Hooks
Server-side registrations use a no-op `NullProviderLifecycle`; application
providers observe displacement through the events topic, while generation
fencing guarantees stale commands are rejected regardless.

---

### ADR 5: Command Gateways (Streaming Topics + Trajectory Actions)

#### Decision
External command entry points are called **gateways** (`rmi.server.gateway`),
replacing the earlier "ingress" naming. A deployment config declares
`sources:` bindings (`provider` / `part` / `command` / endpoint):

- **StreamingCommandGateway** — topics for `joint_reference`,
  `pose_reference`, `cartesian_twist`; optional `max_age_s` staleness guard
  rejects unstamped or stale references before admission;
- **TrajectoryCommandGateway** — a `FollowJointTrajectory` action server per
  binding. Goals are fenced through `ExecutionManager.admit`, forwarded to
  the Part's allocated trajectory controller client, and the downstream
  result/cancellation is proxied back to the caller. Transactional
  trajectories therefore keep ROS action semantics end-to-end across hosts.

---

### ADR 6: Motion Stop on Fencing & Provider Liveness

#### Decision
Whenever a provider loses its allocation — takeover (including
same-controller), `release`, `stop`, or `fail` — the EM cancels in-flight
transactional goals on the affected Parts before the fence event is
published. Streaming controllers hold their last reference and remain
guarded by the C++ stale guards (ADR 3).

`ProviderLivenessWatchdog` monitors providers with a configured
`inactive_timeout_s` while they hold allocations; gateways report command
activity, and an expired window triggers `ExecutionManager.fail`
(`inactive_timeout`) and fences the provider.

That workstation watchdog is not the RT safety guarantee. For each accepted
downstream JTC goal, the direct RMI controller client publishes a reliable
heartbeat to the corresponding RT-host `joint_trajectory_controller_guard`.
Normal completion/cancel sends an explicit disarm; workstation loss or a
failed cancel path stops heartbeats without disarming. The local guard then
cancels the JTC goal, and JTC's configured cancellation deceleration transitions
the robot to hold. Streaming-reference timeout/invalid-value hold behavior and
fault diagnostics remain inside the manipulation controllers.

---

### ADR 7: Arbitration Semantics of Explicit Handover

#### Decision
`handover` is an explicit control-plane command (issued by an orchestrator,
BT node, or operator), not a passive bid. The ROS/`ExecutionManagerClient`
wire name is `acquire` (`ProviderCommand.ACQUIRE`); `ExecutionManager.acquire`
is an alias of `handover`:

- only a **strictly higher-priority** active owner blocks a takeover
  (`ArbitrationRejected`); equal-priority providers may deliberately
  displace each other;
- passive stickiness (old EM's implicit publish-based election) is the
  caller's policy, not the manager's;
- a failed handover leaves the targeted Parts **fenced and unowned**:
  controllers are rolled back (or deactivated) and hold, previous providers
  stay `READY`, and a supervisor must issue a new handover to resume motion.
