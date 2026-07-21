# Runtime Architecture

## Workspace layout

```text
src/                       ownership-oriented ROS 2 source tree
  interfaces/              shared custom ROS msg/srv/action packages, when justified
  execution/               source arbitration and execution contracts
  controller/              reusable ros2_control controllers
  embodiments/
    robots/                robot descriptions and hardware interfaces
    sensors/               sensor model defaults; no application composition
  teleop/                  device/session adapters and retargeters
  motion_planning/         planner contracts and backend adapters
  policy_inference/        deployed policy inference and runtime adapters
  recording/               raw episode recording and replay tools
  visualization/           runtime inspection and debug sources
  toolbox/                 small reusable tools owned by this workspace
  apps/                    application checkout root (optional examples)
docs/                      architecture, contracts, validation, migration
scripts/                   idempotent workspace setup and diagnostics
.config/                   checked-in templates (e.g. CycloneDDS)
```

This repository is a **shared workspace template**: Pixi + colcon + docs and
workspace-owned tools. Teams import reusable functional modules through
`repos/necessary.repos`; optional runnable examples use `repos/example.repos`;
robot-specific descriptions and hardware integrations use
`repos/embodiment.repos`.

[`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources) is a
flat ROS-package repository imported once at `src/apps` through
`repos/example.repos`. Marvin embodiment packages stay in their own repos
under `src/embodiments/robots/marvin/`. Reusable marker teleop remains a
workspace-owned package at `src/toolbox/rviz_interactive_marker_teleop`.

Reusable functional modules (`repos/necessary.repos`):

- [`manipulation_execution_manager`](https://github.com/Gabriel-Ning/manipulation_execution_manager)
  → `src/execution/manipulation_execution_manager` (`vcs import src < repos/necessary.repos`)
- [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox)
  → `src/teleop/isaacteleop_toolbox` (`repos/necessary.repos`)
- [`motion_planners`](https://github.com/Gabriel-Ning/motion_planners)
  → `src/motion_planning/motion_planners` (`repos/necessary.repos`; contains
  `manipulation_motion_planning`, `pyroki_planner_adapter`,
  `curobo_planner_adapter`)

Optional example applications (`repos/example.repos`):

- [`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources)
  → `src/apps` (`repos/example.repos`; flat Marvin app packages)

Robot embodiment (`repos/embodiment.repos`):

- [`marvin_description`](https://github.com/Gabriel-Ning/marvin_description)
  → `src/embodiments/robots/marvin/marvin_description` (`repos/embodiment.repos`,
  branch `dev` = site-calibrated geometry; `main` = official CAD)
- [`marvin_hardware_interface`](https://github.com/Gabriel-Ning/marvin_hardware_interface)
  → `src/embodiments/robots/marvin/marvin_hardware_interface` (`repos/embodiment.repos`)

Example 1 (Marvin apps from `runtime_resources`) — fetch/build steps and
package map: [EXAMPLE1.md](EXAMPLE1.md).

- `runtime_resources` → `src/apps` (`repos/example.repos`)
- `rviz_interactive_marker_teleop` → `src/toolbox/` (workspace-owned)

Directories express ownership. External repositories are imported at the roots
declared by the manifests; workspace-owned reusable utilities remain directly
under `src/toolbox`. Empty category directories use `.gitkeep` until their
first package arrives.

`interfaces/` is not a mandate to invent a universal Physical AI message
layer. Prefer standard ROS messages (`PoseStamped`, `TwistStamped`,
`JointState`, `JointTrajectory`, …). A custom interface moves under
`src/interfaces/` only when multiple producers and consumers need a stable
shared schema.

## Product boundary

Physical AI Runtime owns online robot execution:

- ROS 2 communication and launch composition;
- descriptions, calibration inputs, and hardware interfaces;
- execution management and position/trajectory controllers;
- teleoperation and motion-planning runtime adapters;
- raw episode recording, replay, diagnostics, and visualization;
- the minimum inference interface required to execute a deployed policy.

The runtime does not own model training, dataset curation, annotation,
benchmark notebooks, or general offline AI environments. Those systems consume
versioned runtime recordings and contracts without controlling runtime
dependencies.

## Dependency ownership

| Layer | Responsibility |
|---|---|
| `pixi.toml` | Direct integrated runtime dependencies and three public tasks |
| `pixi.lock` | Exact Conda, PyPI, CUDA, ROS, Git, and transitive versions |
| `.envrc` | Frozen Pixi activation, colcon overlay, lightweight ROS defaults |
| `package.xml` | ROS package dependency and release contract |
| CMake/setup.py | Build and install one ROS package |
| colcon | Workspace discovery, topological build, test, and install |
| ROS launch | Runtime node and hardware composition |

Workspace-owned runtime paths are exported by Pixi activation rather than
hard-coded by packages:

```text
WORKSPACE_ROOT = $PIXI_PROJECT_ROOT
CLOUDXR_DIR    = $PIXI_PROJECT_ROOT/.cloudxr
RMW_IMPLEMENTATION = rmw_cyclonedds_cpp
```

Packages consume these as defaults and retain explicit arguments for
standalone overrides. They must not silently place runtime data under `$HOME`.
CycloneDDS is the fixed workspace RMW. Its ROS package is already provided by
the resolved Jazzy runtime dependency graph, so it is not repeated as a direct
Pixi dependency. Cross-machine DDS (camera LAN, dedicated NIC, peers) is
optional and configured via `CYCLONEDDS_URI` from
[`.config/cyclonedds_template.xml`](../.config/cyclonedds_template.xml); see
the README.

There is one supported Pixi environment. CUDA-enabled PyTorch, IsaacTeleop,
planning backends, ROS, camera dependencies, and robot SDK dependencies share
the same solved runtime. A new environment is introduced only after a concrete
incompatibility or deployment boundary is demonstrated.

## Package dependency direction

```text
interfaces
    ^
    |
execution     controller    runtime domains
    ^              ^            ^
    +--------------+------------+
                   |
              embodiments
                   ^
                   |
                  apps
```

More precisely:

- interface packages contain only justified shared custom `msg/srv/action`
  definitions and depend on no implementation package;
- execution and controllers never depend on a robot application;
- hardware interfaces depend on reusable contracts, not teleop/planners;
- teleop and planning packages emit source intent and never command hardware
  directly;
- apps own launch, parameter, frame, and embodiment composition;
- robot-specific integration launch files belong to apps, not generic domain
  packages.

Standard ROS messages are preferred over workspace-owned interfaces. A custom
interface moves under `src/interfaces/` only when multiple producers and
consumers require a stable shared schema. Interfaces that are still local to
one application remain with that application.

## Performance and safety defaults

- Colcon builds use `Release` by default.
- Python, GPU inference, retargeting, perception, and planning remain outside
  the ros2_control realtime update loop.
- Real-hardware launch defaults are never inferred from a generic build task.
- Each motion-capable integration advances through offline, source-only, fake
  hardware, state-only real hardware, and conservative motion gates.
- Runtime streams retain stamped source intent, normalized execution output,
  controller command/status, and robot feedback for audit and data generation.

## Data boundary

The runtime records independent raw streams, preferably in MCAP through
rosbag2. It preserves capture/source timestamps when producers expose them and
also retains record/receive time. Observation/action alignment, dataset
conversion, training schemas, and policy-specific sampling happen outside the
runtime recording loop.
