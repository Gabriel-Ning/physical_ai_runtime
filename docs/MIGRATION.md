# Incremental Migration Plan

Migration starts from the validated `ros2_workspace` and moves one closed loop
at a time. The old workspace remains the behavioral reference until the new
repository passes the equivalent gate.

## Environment policy

There is one Pixi environment. The former `base` / `teleop` / `planner` /
`robot` feature split is not carried forward. Conda and PyPI dependencies live
in flat `[dependencies]` and `[pypi-dependencies]` tables and are solved once
into `pixi.lock`.

Package migration remains gated. Dependency availability in Pixi does not mean
a subsystem is migrated or supported for real hardware.

Dedup notes applied when merging the old workspace:

- `ros-jazzy-moveit` is the only direct MoveIt dependency; configs-utils,
  move-group, visualization, simple-controller-manager, kinematics, planners,
  and setup-assistant arrive transitively.
- `torch` is declared once under `[pypi-dependencies]` (shared by IsaacTeleop
  and planner backends).
- `pin`, `pyyaml`, `scipy`, and `msgpack` are not listed as PyPI deps; conda
  already supplies `pinocchio` (via `placo`), `pyyaml`, `scipy`, and
  `msgpack-python`. Import path is `import pinocchio` / `import pink`
  (`pin-pink`), not bare `import pin`.
- `[system-requirements] libc = "2.35"` matches the IsaacTeleop
  `manylinux_2_35` wheel so `pixi install --locked` stays consistent.
- RealSense uses robostack `ros-jazzy-realsense2-camera` (+ msgs/description).
  The old `librealsense` + colcon-overlay workaround is obsolete: current
  channels solve `librealsense 2.57.7` with `realsense2-camera 4.57.7`.

## Gate 0 — filesystem and contracts

Current scope:

- establish ownership directories;
- document the single-environment, Release-first policy;
- define dependency direction and the runtime/data boundary;
- do not copy packages or claim buildability.

Pass evidence:

- repository contains only intentional documentation and category directories;
- no generated build/install/log artifacts;
- migration order and validation gates are explicit.

## Gate 1 — Pixi and ROS shell (completed)

The single `pixi.toml` and `.envrc` provide the ROS 2 Jazzy baseline and
workspace tools. Full runtime dependency consolidation (control, sensors,
teleop, planning, CUDA) is tracked under Gate 1b below.

Pass evidence:

```text
pixi install --locked
direnv reload
which python, ros2, colcon resolve inside .pixi
ros2 doctor reports the expected Jazzy runtime
```

## Gate 1b — single full runtime solve (completed)

Merge the validated `ros2_workspace` dependency set into one environment with
no feature tables and no redundant direct pins.

Pass evidence:

```text
unset PIXI_FROZEN
pixi lock
pixi install --locked
pixi run setup
python -c "import rclpy, torch, isaacteleop, pyroki, curobo, pink, pinocchio; print('ok')"
```

Validated:

- lock contains only `default`;
- `pixi install --locked` succeeds with `libc = "2.35"`;
- core imports resolve inside `.pixi/envs/default`.

No ROS source package is migrated in this gate.

## Gate 2 — execution and controller foundation (in progress)

Migrate any required shared interfaces, the execution manager, and
manipulation position controllers without robot applications.

Pass evidence:

```text
pixi run build
pixi run test
```

Tests must cover package installation and fake/reference behavior. No real
hardware is enabled. Required Pixi packages (`ros2_control`, Eigen,
launch-testing) are already in the single environment.

Current evidence:

- `manipulation_execution_manager` migrated as a submodule under
  `src/execution/manipulation_execution_manager`, pinned to `main` after the
  `gracious-heisenberg` arbitration refactor merged (JSPC/TSKPC route
  exclusion, `joint_trajectory_goal` as an independent path, contract-drift
  and code-review fixes R-01..R-04/R-12); no Piper-specific launch or package
  coupling carried over;
- deps are standard ROS messages + numpy, already resolved by the single
  environment; no `pixi.toml` additions required for this package;
- Release build and 78 deterministic unit tests pass.

Environment fix landed in this gate: Robostack's `ros-jazzy-launch-testing`
(3.4.10) registers a pytest11 plugin against the pre-9.0
`pytest_pycollect_makemodule(path, parent)` hookspec. `pytest` had solved to
9.1.1, which dropped that legacy parameter, so every `colcon test` /
`ament_cmake_pytest` run failed at plugin load — including previously-passing
`isaacteleop_toolbox` tests. Pinned `pytest = "<9"` in `pixi.toml` (now
`8.4.2`) until Robostack ships a `launch_testing` build compatible with
pytest 9. Re-verified both packages' tests pass after the pin.

Remaining for this gate:

- migrate `manipulation_position_controllers` into `src/controller/`;
- confirm its native deps (`osqp`, `control_toolbox`, `osqp-eigen`, `placo`,
  `pinocchio`, `ruckig`) resolve or land in `pixi.toml`;
- Release build and test with no robot application wired in.

## Gate 3 — one embodiment foundation

Migrate one robot description and its fake-hardware bringup first. Add the real
hardware interface only after description, joint ordering, lifecycle, and fake
execution match the reference workspace.

Pass evidence progresses through:

1. URDF and TF inspection;
2. fake ros2_control activation;
3. controller hold and bounded fake motion;
4. real state-only connection;
5. conservative real motion and clean shutdown.

Description tooling, MuJoCo vendor, and robot SDKs (`libpiper`, `libmarvin`)
are already in Pixi. Confirm whether `libmarvin` stays with the embodiment or
moves with the app package when that integration starts.

## Gate 4 — IsaacTeleop vertical slice (package gate completed)

Migrate `isaacteleop_toolbox` with a clean boundary:

- source/session runtime, retargeters, CloudXR compatibility, ROS publishers,
  source-only launch, and deterministic tests remain in the teleop package;
- Marvin/robot-specific EM, controller, description, and RViz composition
  belongs to the relevant app package;
- CloudXR setup is an installed executable, not a source-tree-only script;
- the supported IsaacTeleop wheel/revision and web assets are pinned.

Pass evidence:

1. unit and parameter-validation tests;
2. installed source-only MCAP replay;
3. live Quest source with no robot controller;
4. fake bimanual EM/TSKPC;
5. conservative real-hardware validation.

Current evidence:

- generic package migrated under `src/teleop/isaacteleop_toolbox`;
- Marvin/EM/controller composition excluded from the package;
- CloudXR setup installed as a ROS executable;
- source-only launch no longer depends on a workspace-root path;
- Release build succeeds from the new workspace;
- nine deterministic unit tests pass, including invalid retarget-parameter,
  release-contract, and archive path-safety coverage;
- installed executables and launch arguments are discoverable from the colcon
  install space.

Remaining gates are source-only MCAP replay, live Quest, fake robot
integration, and conservative real hardware after their dependent packages are
migrated.

IsaacTeleop and retargeting PyPI deps are already in the single environment.

## Gate 5 — planners, sensors, and recording

Migrate each subsystem independently. Preserve the current ROS contracts and
avoid combining planner, camera, recorder, and real motion changes in one gate.
Recording acceptance includes stream timestamps, episode metadata, message
counts/gaps, and raw replay before any dataset conversion.

Sensors, MoveIt, PyRoki, cuRobo, Foxglove, and related packages are already in
Pixi. Migration still lands one closed loop at a time.

## Gate 6 — release acceptance

- clean clone and `pixi install --locked`;
- full Release build and test result with zero failures;
- source-only and fake-hardware smoke gates;
- documented real-hardware compatibility evidence;
- license and third-party notices;
- dependency and hardware support matrix;
- no dependency on the old workspace through `PYTHONPATH`, prefix paths, or
  source-tree-relative executables.
