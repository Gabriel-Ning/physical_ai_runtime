# Manipulation Execution Architecture

Status: **RMI implementation baseline complete**  
Last updated: 2026-08-09

## Canonical implementation

RMI is the workspace implementation of manipulation execution capability. The
former `manipulation_execution_manager` package has been removed from the
active workspace and is not a runtime dependency.

The detailed contracts and decisions are defined by:

- [ExecutionManager and RMI decisions](EXECUTION_MANAGER_ARCHITECTURE_DECISIONS.md)
- [RMI/ExecutionManager refactor plan](EXECUTION_MANAGER_SDK_REFACTOR_PLAN.md)
- [motion-planner source interface](MOTION_PLANNER_SOURCE_INTERFACE.md)
- [recording architecture](RECORDING_ARCHITECTURE.md)

## Runtime boundary

```text
task application / orchestrator / optional task-graph engine
                              |
                              v
RMI public capability
  Robot + Part/component controller clients
  ExecutionManager + provider arbitration
  ExecutionManagerClient + typed command gateways
  Recorder facade and execution telemetry
                              |
                              v
ros2_control controller_manager + C++ controllers + hardware
```

RMI owns execution semantics: provider lifecycle, explicit handover, priority
checks, partial Part allocation, generation fencing, controller switching and
execution telemetry. It accepts Policy, Teleop, Planner and Replay as peer
ActionProviders.

The upper orchestration layer owns task semantics: when a skill starts, which
provider should request control, when an episode begins or ends, and which
recovery branch to choose. It may use a behavior tree, state machine, task
graph or direct application code; RMI does not depend on that choice.

## Deployment

Controller bringup starts hardware, `controller_manager`, broadcasters and
preloaded inactive routes. It does not start an execution manager.

On the RT host, an RMI deployment composes:

- controller bringup;
- one authoritative `rmi` ExecutionManager server;
- `episode_recorder` where required.

Workstation providers use `ExecutionManagerClient` for lifecycle/control-plane
requests and publish native ROS 2 command types to deployment-declared topic or
action gateways. Producers never publish directly to controller input topics
and never switch controllers themselves.

## Timing and safety

Python RMI and orchestration code are non-realtime control-plane software.
Stamped references and bounded trajectories cross into C++ controllers; the
`ros2_control` update loop owns interpolation, hardware writes and hard
realtime limits. Provider liveness, stale-reference guards and controller
fault feedback complete the distributed safety boundary.

## Validated capability

Demo 1 through Demo 9 cover embodiment construction, provider lifecycle,
telemetry, preemption/release, recording integration, replay arbitration,
native controller dispatch, partial allocation, controller/provider switch
combinations and RT-host/workstation distribution using fake hardware.

Simulation-specific and attended real-hardware acceptance remain deferred as
recorded in the refactor plan.
