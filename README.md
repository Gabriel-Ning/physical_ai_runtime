# Physical AI Runtime

A release-first, ROS 2-oriented runtime for manipulation systems.

This repository provides one coherent and reproducible runtime environment for
robot descriptions, hardware access, reference execution, position control,
teleoperation, motion planning, recording, replay, and runtime visualization.
Dataset processing, training, and experimentation remain outside this runtime
unless they are required to execute a deployed policy.

## Runtime contract

```text
teleop | planner | replay | deployed policy | debug source
                         |
                         v
                execution manager
          validation, arbitration, observability
                         |
                         v
        position controller or trajectory controller
                         |
                         v
                   ros2_control
                         |
                         v
                  robot hardware
```

The repository follows four rules:

1. One locked Pixi environment is the supported development and runtime
   environment.
2. Every component remains a standard ROS 2 package built and installed by
   colcon; the repository has no top-level `add_subdirectory()` build graph.
3. `package.xml` defines ROS dependencies and package ownership. `pixi.lock`
   defines the exact integrated runtime.
4. Application packages compose reusable packages. Generic packages never
   depend on a particular robot application.

## Workspace layout

```text
src/
  interfaces/              shared custom ROS msg/srv/action packages, when justified
  execution/               source arbitration and execution contracts
  controller/              reusable ros2_control controllers
  embodiments/
    robots/                robot descriptions and hardware interfaces
    sensors/               sensor model defaults; no application composition
  teleop/                  device/session adapters and retargeters
  motion_planning/         planner contracts and backend adapters
  recording/               raw episode recording and replay tools
  visualization/           runtime inspection and debug sources
  toolbox/                 uncategorized reusable runtime tools
  apps/                    robot/application launch and configuration
docs/                      architecture, contracts, validation, migration
scripts/                   idempotent workspace setup and diagnostics
```

Directories express ownership; only real ROS packages should be added beneath
them. Empty category directories are intentionally not populated with dummy
packages.

`interfaces/` is not a mandate to invent a universal Physical AI message
layer. It contains only custom ROS interface packages whose contracts are
shared by multiple implementations and cannot be represented cleanly with
standard ROS messages. `PoseStamped`, `TwistStamped`, `JointState`,
`JointTrajectory`, and standard actions remain the first choice. An
application-local message stays with its owning package until reuse proves a
stable shared contract.

## Environment and commands

Clone with submodules (teleop packages live in separate public repos):

```bash
git clone --recurse-submodules git@github.com:Gabriel-Ning/physical_ai_runtime.git
cd physical_ai_runtime
# or after a plain clone:
git submodule update --init --recursive
```

The final public workflow is intentionally small:

```bash
pixi run setup
pixi run build
pixi run test
```

The default build type is `Release`. With direnv enabled, entering this
directory activates the locked Pixi environment (including `CLOUDXR_DIR` from
`pixi.toml`) and sources `install/setup.bash` when it exists. Developers can
then use native commands directly:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch <package> <launch-file>
ros2 topic list
ros2 control list_controllers
```

The three Pixi tasks are stable workspace lifecycle entrypoints. ROS launch
files and ordinary ROS CLI commands own application workflows; the top-level
Pixi file must not become a catalog of every launch combination.

`isaacteleop_toolbox` is vendored as a git submodule under
`src/teleop/isaacteleop_toolbox`
([Gabriel-Ning/isaacteleop_toolbox](https://github.com/Gabriel-Ning/isaacteleop_toolbox)).


## Current stage

The single Pixi environment is solved with the full runtime dependency set.
The first package, `isaacteleop_toolbox`, has passed its isolated Release build
and unit-test gate. Live Quest, fake-robot integration, and real hardware have
not yet been revalidated in this repository, so it does not currently claim a
hardware-capable runtime. See [Migration plan](docs/MIGRATION.md).
