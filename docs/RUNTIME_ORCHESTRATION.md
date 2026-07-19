# Runtime Orchestration SDK and API

## Purpose

The Runtime Orchestration SDK is the control plane above an already launched
ROS 2 runtime. It is deliberately not a replacement for ROS launch:

- ROS launch owns drivers, ros2_control, Execution Manager (EM), recorder,
  sensors, and their process composition.
- The SDK owns capability discovery, session resource leases, orchestration
  operations, and a hardware-independent observation/action schema.
- An HTTP/WebSocket server is a thin transport adapter over the SDK for a
  browser UI, external CLI, or agent integration.
- A web application is a reference orchestration client. It never publishes
  ROS commands directly.

```text
Python policy / script        Browser UI / external client
          |                              |
          +-------- Runtime SDK ----------+
                           |
                   HTTP / WebSocket adapter
                           |
                    ROS 2 client adapter
          +----------------+-----------------+
          |                |                 |
         EM         episode_recorder       sensors
          |
    ros2_control / hardware
```

High-frequency policy control remains in-process: it reads observations and
publishes actions through the SDK's ROS adapter. HTTP and WebSocket are for
orchestration, progress, and bounded-rate previews rather than a realtime
control path.

## Stable SDK model

The SDK uses a configuration profile to describe an embodiment and its
endpoints. No SDK module may encode a robot vendor, arm count, controller
name, or camera model.

- `EmbodimentConfig`: joint groups, end-effector frames, sensors, recorder
  endpoint, stream profiles, and ROS endpoint bindings.
- `Capability`: the configuration plus launch-time readiness/preflight
  evidence. It states what is currently usable, rather than assuming every
  robot has arms, grippers, cameras, calibration, or a teleoperator.
- `FeatureSchema`: typed observation/action features with name, shape, unit,
  frame, rate, and safety constraints. Runtime values are
  `dict[str, numpy.ndarray]`.
- `RuntimeSession`: a leased, explicit lifecycle:
  `prepare -> ready -> active -> finalizing -> completed | failed`.
- `Operation`: a long-running, queryable and cancellable request with an ID,
  phase, progress, result, and error.

`FeatureSchema` must be exportable to a LeRobot-compatible feature dictionary.
That makes a later LeRobot adapter possible without making LeRobot a Runtime
dependency.

## Control plane and data plane

The orchestrator does not write bags, transcode images, align observations, or
write training rows. `episode_recorder` remains the authoritative data-plane
owner for raw serialized MCAP payloads, health accounting, validation,
checksums, and atomic finalization.

Before a session can arm recording, `resolve_streams(profile)` compares the
declared stream contract with the live ROS graph and produces PASS/WARN/FAIL
evidence for topic, type, QoS, publisher, and disk preflight. Required failures
reject arming. A finalized episode records the selected profile hash and the
resolved stream snapshot in its manifest.

Each domain has one authoritative status producer:

- recorder: recording phase and recording health;
- EM: active source, route, validation, and execution state;
- orchestrator: session lease and operation state.

These state machines are intentionally orthogonal. A recorder finalizing state
must not overwrite EM state, and an execution hold must not masquerade as a
recording failure.

## API and concurrency rules

The external API is grouped by domain, not by a large command enum:

- `capabilities`: list and inspect embodiment/profile/readiness information;
- `sessions`: create, inspect, and close a resource lease;
- `episodes`: arm, start, stop, discard, and finalize through a session;
- `actions`: submit a bounded command through EM;
- `operations`: query, stream progress for, and cancel long work.

Session leases enforce explicit mutual exclusion between control owners such as
teleoperation, policy, trajectory execution, recording finalization, and
calibration. Repeated commands are idempotent: they return the existing
operation or a well-defined terminal response instead of creating crossing ROS
lifecycle transitions.

The API server owns neither ROS publishers nor session state; it calls the
SDK. Its HTTP endpoints handle commands and queries. WebSocket streams provide
session/operation changes, EM status, recorder health, and bounded-rate
previews.

## Reference projects: adopted patterns

| Reference | Adopted pattern | Reason |
| --- | --- | --- |
| `crisp_py` | YAML device configuration, readiness gates, composable devices | Supports multiple embodiments without hard-coding hardware. |
| `ros2_robot_interface` | capability-granular facade and arrival/wait semantics | Makes scripts readable while leaving controller-specific behavior outside the SDK. |
| `lerobot-ros` | observation/action feature dictionaries and Robot adapter boundary | Preserves a route to LeRobot-compatible policy and dataset integration. |
| `neuracore` | explicit embodiment/data descriptions and policy observation APIs | Makes cross-embodiment IO explicit. |
| `r2_labs` | queryable, cancellable long operations and exclusive robot modes | Required for browser and agent-driven workflows. |
| `cyclo_intelligence` | control/data-plane split, preflight, orthogonal status phases, readiness clients | Matches EM plus a transactional MCAP recorder. |
| `physical_ai_tools` | command/status separation, phase-aware UI, profile selection, health presentation | Useful for a small reference web client. |

## Deliberate non-goals

- No controller-specific FSM, OCS2, CRISP, MoveIt, or robot-vendor API in the
  SDK core.
- No online alignment or `log_*` training rows; raw MCAP remains authoritative
  and conversion is offline.
- No MP4 sidecar replacement for raw image payloads.
- No Runtime-owned training, Hugging Face dataset lifecycle, Docker
  supervision, or policy-container management.
- No ZMQ/pickle control protocol or monolithic `SendCommand` service.
- No direct browser-to-ROS control path.

## Future LeRobot and LeLab integration

LeLab is currently a SO-101/Feetech and online `LeRobotDataset` workflow, not a
hardware-neutral UI plug-in. It cannot connect to this Runtime unchanged.

The compatible future path is:

```text
Runtime SDK
  -> LeRobot Robot/Teleoperator adapter
  -> offline MCAP-to-LeRobotDataset exporter
  -> optional LeLab frontend change or narrow compatibility shim
```

The adapter and exporter are separate gates. They must not introduce SO-101,
serial-port calibration, or online row-alignment assumptions into the SDK or
recorder.

## Verification

- A fake-hardware launch verifies action routing through EM and session leases.
- Recorder preflight rejects missing required streams, mismatched type/QoS, and
  insufficient disk before an episode is armed.
- Repeated start/stop/finalize requests are idempotent and cannot cross
  lifecycle transitions.
- API contract tests verify HTTP commands and WebSocket operation/status events.
- The reference UI renders only capability/schema data and contains no robot
  names or ROS topic names.
- The existing multi-camera MCAP throughput gate remains the recording
  performance authority.
