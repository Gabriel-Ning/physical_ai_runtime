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

## Gate 2 — execution and controller foundation (completed)

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

Second environment fix landed in this gate: colcon's default Python testing
step only selects its `pytest` step extension when a package's *exact*
extracted test dependency name is `pytest`; ROS `package.xml`
`<test_depend>` entries declare the rosdep key `python3-pytest`, which never
matches that check. Affected `ament_python` packages silently fell back to
`python -m unittest` discovery, which drops every pytest-style test (plain
functions/classes not inheriting `unittest.TestCase`) with zero error
output — `manipulation_execution_manager` reported "Ran 0 tests" while a
direct `pytest` invocation passed all 84. The `pixi run test` task now passes
`--python-testing pytest` to `colcon test` explicitly, forcing pytest for
every `ament_python` package regardless of that mismatch (pytest also
collects `unittest.TestCase`-style tests fine, so `isaacteleop_toolbox` is
unaffected). Verified from a clean `pixi run clean && pixi run build && pixi
run test`: 98 tests across 4 test-bearing packages, 0 failures.

Completed for this gate:

- publish `manipulation_position_controllers` as the runtime-only Prefix
  package `ros-jazzy-manipulation-position-controllers ==0.1.0`;
- consume the controller from the `gabriel-robotics` channel instead of
  carrying its source repository under `src/controller/`;
- verify its ament plugin index and shared library load from the locked Pixi
  environment with no robot application wired in.

## Gate 3 — one embodiment foundation (in progress)

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

Current evidence:

- `marvin_description` migrated as a submodule under
  `src/embodiments/robots/marvin/marvin_description`, pinned to `dev` HEAD
  (`baa8a27`, includes the bimanual arm-mount calibration commits not yet on
  `main`); the package is hardware-free (no SDK) and defaults to
  `mock_components/GenericSystem`;
- `marvin_hardware_interface` migrated as a submodule under
  `src/embodiments/robots/marvin/marvin_hardware_interface`, pinned to
  `main` (`c078243`); depends only on the already-present `libmarvin` conda
  package; Release build succeeds and its 3 deterministic gtest suites (SDK
  bridge, write guard, transaction) pass under `pixi run test` with no
  network or real hardware involved; `hardware_interface::SystemInterface`
  plugin `marvin_hardware_interface/MarvinBimanualArmHardware` is
  pluginlib-discoverable from the install space;
- `xacro ... ros2_control:=true use_fake_hardware:=true` expands and
  `check_urdf` passes: one connected tree from `world`, 14 arm joints,
  `flange_L`/`flange_R` present; `use_fake_hardware:=false` correctly
  swaps to the real plugin with `robot_ip` param wired through, verified by
  xacro expansion only (no real hardware connection attempted from this
  session);
- first closed loop composed in `src/apps/marvin_rviz_debug_bringup`, made
  bimanual: two independent chains, one per arm --
  `rviz_interactive_marker_teleop` (`target_marker_left`/`_right`) ->
  `manipulation_execution_manager` (`em_left`/`em_right`) ->
  `TaskSpaceKinematicPositionController`
  (`left_task_space_kinematic_position_controller`/`right_...`) ->
  ros2_control hardware, sharing one `controller_manager` process and one
  URDF; each arm's `base_frame`/`tip_frame` is its own native frame
  (`Base_L`/`flange_L`, `Base_R`/`flange_R`), not the shared `base_link`;
- both TSKPC instances declare explicit `lower_limits`/`upper_limits` in
  `config/controllers.yaml`, copied from
  `marvin_description/config/joint_limits.yaml` (the same native M6 numbers
  the URDF xacro loads); this duplication is intentional --
  `TaskSpaceKinematicPositionController` does not parse the URDF's `<limit>`
  tags itself (it only clamps if given explicit `lower_limits`/`upper_limits`
  parameters, defaulting to unbounded otherwise), so leaving them unset would
  silently run both arms with no position limit at all;
- the bringup exposes `use_fake_hardware` (default `true`), `hardware_plugin`,
  and `robot_ip` launch arguments so the same launch file runs the fake and
  real hardware gates; the real-hardware path is opt-in only and undocumented
  as a default per the project's real-hardware safety rule;
- both TSKPCs' spawners are sequenced after `joint_state_broadcaster`'s
  spawner exits (`RegisterEventHandler(OnProcessExit(...))`, matching the
  reference workspace's `marvin_bimanual_task_space_teleop.launch.py`
  pattern) rather than raced in parallel: three concurrent spawner calls
  against one `controller_manager` had two of them lose a "configure from
  active state" race and exit non-zero, even though the net controller state
  was already correct;
- verified end-to-end with the real controller_manager on fake hardware (not
  a unit test): `joint_state_broadcaster` and both TSKPC instances reach
  `active` with clean spawner exits; a `PoseStamped` streamed on
  `/action_sources/marker_left/pose_target` and
  `/action_sources/marker_right/pose_target` is validated/forwarded by each
  EM instance (`active_streaming_route: tskpc`) and both arms' 7 joints move
  symmetrically in `/joint_states` toward their mirrored IK solutions;
- stages 4-5 (real state-only connection, conservative real motion) are not
  attempted; running `use_fake_hardware:=false` requires the physical robot
  present, powered, and safed, and is left to the user's own hands-on gate.

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

Current evidence (partial):

- `rviz_interactive_marker_teleop` migrated under
  `src/toolbox/rviz_interactive_marker_teleop` as a pure PoseStamped source;
- Piper-specific RViz bringup removed from the package launch; apps compose
  RViz and embodiment frames;
- default output topic aligned to the EM pose contract
  (`/action_sources/marker/pose_target`);
- Release build succeeds; no unit tests in the upstream package yet.

## Gate 6 — release acceptance

- clean clone and `pixi install --locked`;
- full Release build and test result with zero failures;
- source-only and fake-hardware smoke gates;
- documented real-hardware compatibility evidence;
- license and third-party notices;
- dependency and hardware support matrix;
- no dependency on the old workspace through `PYTHONPATH`, prefix paths, or
  source-tree-relative executables.
