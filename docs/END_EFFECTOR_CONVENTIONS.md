# End-Effector Integration Conventions

Development paradigm for integrating end effectors (parallel grippers,
dexterous hands, flange-mounted tools) into this runtime. Every new
end-effector integration MUST follow these conventions; reviewers should
reject deviations that do not document a reason.

Reference implementations:

- `marvin_hardware_interface` — safety guards, README contract tables, offline tests
- `piper_hardware_interface` (PiperGripperInterface) — blocking-SDK thread pattern
- `pika_gripper` packages — first integration built under this document

## 1. Core principles (non-negotiable)

1. **One end effector = one independent ros2_control hardware component.**
   Never merge an end effector into the arm's hardware interface. Each device
   gets its own `<ros2_control>` tag and its own pluginlib plugin, loaded by
   the **same** controller_manager as the arm.
2. **`read()`/`write()` never block.** Budget is microseconds. Any blocking
   SDK/transport goes behind background threads (see §5).
3. **Perception and actuation are separate.** Cameras or other sensors that
   are physically integrated into the end effector do NOT enter the hardware
   interface. They are separate driver packages under `embodiments/sensors/`
   (or upstream drivers), composed at launch time and related only through TF.
4. **description / hardware_interface / bringup are separate packages.**
   Hardware-interface packages export only the plugin and its SDK bridge — no
   launch files, no controller YAML, no GUI tools.
5. **Offline tests and a contract README are mandatory** before any real
   hardware bring-up (see §7, §8).

## 2. Package layout

```text
src/embodiments/end_effectors/<vendor>_<type>/     # e.g. pika_gripper, xx_hand
├── <name>_description/           # xacro fragments, meshes, joint limits, camera TF
│   └── urdf/
│       ├── <name>.xacro                # links/joints macro
│       └── <name>.ros2_control.xacro   # <ros2_control> block macro
├── <name>_hardware_interface/    # plugin lib + transport bridge + offline tests
└── <name>_bringup/               # OPTIONAL: bench-only debug launch/config
```

- Host-robot composition (mounting the effector on Marvin, combined
  controllers, deployment launch) lives in the host robot's bringup package
  (`marvin_bringup`), not here. `<name>_bringup` exists only for standalone
  bench validation of the device on a desk.
- The host description assembles effectors through a xacro argument, e.g.
  `end_effector:=pika_gripper|piper_gripper|none`. Swapping effectors must not
  require C++ changes.
- Each package group is an independent git repository imported through
  `repos/embodiment.repos`, same as the Marvin packages.

## 3. Vendor SDK packaging

- Non-ROS vendor SDKs are packaged as conda packages on the
  `gabriel-robotics` prefix.dev channel (`libpiper`, `libmarvin` pattern) and
  consumed via `find_package(<sdk> CONFIG REQUIRED)` +
  `target_link_libraries(<target> <sdk>::<sdk>)`.
- Never vendor SDK sources into a ROS package.
- Plain-transport devices (raw serial/CAN protocols) need no SDK package:
  implement the protocol in the package's bridge (see §5) and treat the
  vendor's reference implementation as protocol documentation, pinned by
  commit in the README.

## 4. ros2_control contract

| Device class | Command interfaces | Units | Standard controller |
|---|---|---|---|
| Parallel gripper | 1 joint, `position` | meters (full opening width) | `parallel_gripper_action_controller` |
| Dexterous hand | N joints, `position` | radians | `joint_trajectory_controller` / forward controller |

- State interfaces: at minimum `position`; add `velocity`/`effort` when the
  hardware reports them. Diagnostic bits (fault/enabled/homed/temperature)
  are exported as extra state interfaces via `on_export_state_interfaces()`,
  not as ad-hoc topics.
- Target the **Jazzy 4.x API**: no manual `export_*_interfaces()` overrides
  for URDF-declared interfaces, use `set_state()`/`get_command()`, framework
  owns interface memory. Do not mix distro API generations.
- The exception (Franka pattern): a standalone action-server node instead of
  ros2_control is allowed ONLY when the device offers nothing but blocking,
  seconds-long high-level commands and cannot stream setpoints. Document the
  justification in the package README.

## 5. Realtime discipline

The controller_manager calls every hardware component's `read()`/`write()` in
one loop; a slow effector stalls the arm's 500 Hz cycle.

Required pattern for blocking or slow transports (reference:
`PiperGripperInterface`):

- **Reader thread**: owns the transport read path, parses frames,
  publishes the latest state into `std::atomic` caches (or a mutex-free
  double buffer). `read()` only copies the caches.
- **Writer thread**: waits on a condition variable. `write()` validates the
  command, stores the target under a short-held mutex, and notifies.
  The writer thread performs the actual transport write / blocking SDK call.
- `read()`/`write()` perform no heap allocation, no logging with
  `make_shared` clocks, no syscalls other than the atomic/mutex handoff.

Fast non-blocking transports (e.g. SocketCAN send of one frame) may write
directly in `write()` if measured latency stays in the microsecond range.

## 6. Safety guards

All thresholds parameterized as URDF `hardware_parameters`, **time-based** so
their physical meaning survives `update_rate` changes (reference:
`marvin_hardware_interface`):

| Guard | Requirement |
|---|---|
| Command validity | reject NaN/Inf; clamp to joint limits |
| First-write delta | command must start at the measured position (sync in `on_activate`) |
| Per-cycle step | `max_speed × period` clamp |
| Command deadband | skip re-sending unchanged targets (protects chatty transports) |
| Stale feedback | warn / error thresholds in ms; error path returns `return_type::ERROR` |
| Deactivate / destructor | send disable/safe command; destructor is the safety net |

## 7. Quality gates (review checklist)

A new end-effector package is mergeable when:

1. **Offline unit tests** (no hardware) cover: unit conversions and
   kinematic mappings, frame/protocol encode+parse (including malformed and
   fragmented input), and every write-path guard in §6.
2. **README contract tables** exist: command/state interfaces (count, units),
   hardware parameters (name, required, default, description), safety guards
   (threshold, parameter).
3. Known issues live in a README "Design Notes / Known Issues" section, not
   in chat logs or commit messages.
4. `colcon build` + `colcon test` pass inside the pixi environment.

## 8. Bring-up gates

Follow `.codex/skills/physical-ai-incremental-validation`. Minimum ladder:

1. **Transport sniff** (read-only): observe live frames with a terminal tool;
   confirm framing, rate, and quiescent content. No commands sent.
2. **mock_components rehearsal**: controller + broadcaster against
   `mock_components/GenericSystem`; verify `/joint_states` and controller
   lifecycle.
3. **Real read-only**: plugin activated with NO controller claiming command
   interfaces; verify live positions in `/joint_states` and stale-guard
   behavior on unplug.
4. **Low-speed motion**: enable device, small command steps, hand on the
   power switch. Verify guards clip as configured.
5. **Host integration**: mount on robot, combined launch, recorder check
   (effector joints present in episodes without recorder changes).

Sensors integrated in the effector (cameras) validate independently through
their own driver packages and join only at gate 5 via TF.
