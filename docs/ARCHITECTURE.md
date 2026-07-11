# Runtime Architecture

## Workspace layout

```text
src/
  interfaces/              shared custom ROS msg/srv/action packages, when justified
  execution/               source arbitration and execution contracts
  controller/              reusable ros2_control controllers
  embodiments/
    robots/                robot descriptions and hardware interfaces
    sensors/               sensor model defaults; no application composition
  teleop/                  device/session adapters and retargeters (submodules OK)
  motion_planning/         planner contracts and backend adapters
  recording/               raw episode recording and replay tools
  visualization/           runtime inspection and debug sources
  toolbox/                 uncategorized reusable runtime tools
  apps/                    robot/application launch and configuration
docs/                      architecture, contracts, validation, migration
scripts/                   idempotent workspace setup and diagnostics
```

Directories express ownership; only real ROS packages should be added beneath
them. Empty category directories may hold `.gitkeep` until the first package
lands.

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
Pixi dependency.

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
