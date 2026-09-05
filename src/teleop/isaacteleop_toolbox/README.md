# isaacteleop_toolbox

ROS 2 (Jazzy) teleoperation **source** package built on
[IsaacTeleop](https://github.com/NVIDIA/IsaacTeleop).

It turns Quest-class controller input into stamped pose targets and status that
a robot application can consume. This repository does **not** own robot
controllers, hardware interfaces, or safety limits.

```text
Quest / OpenXR
  -> IsaacTeleop ControllersSource
  -> application retargeter
  -> ROS PoseStamped + status (+ clutch snapshots)
  -> your execution manager / controllers
```

Version `0.1.0` is an experimental Apache-2.0 source release. The first shipped
application is clutch-based **bimanual relative** teleop for Quest 3 — see
[Bimanual relative](docs/BIMANUAL_RELATIVE.md). To add another retargeter or
app slice, see [Extending](docs/EXTENDING.md).

Detailed guides live under [`docs/`](docs/):

- [Dependencies](docs/DEPENDENCIES.md) — IsaacTeleop / CloudXR ownership;
  Physical AI Runtime vs standalone workspaces
- [Usage](docs/USAGE.md) — build, test, topics, live / replay launch
- [CloudXR](docs/CLOUDXR.md) — asset setup, Quest browser URL, network overrides
- [Bimanual relative](docs/BIMANUAL_RELATIVE.md) — clutch behavior and tuning
- [Extending](docs/EXTENDING.md) — template extension points

## Compatibility

| Component | Baseline |
|---|---|
| ROS 2 | Jazzy |
| Python | 3.12 |
| IsaacTeleop | 1.5.124rc1 (pinned in pixi.toml curobo feature) |
| XR input | Quest 3 / Touch Plus (Continuous Button-Servo Grippers: A/B, X/Y) |
| Status JSON schema | 1 |

IsaacTeleop is **not** installed by this ROS package. The validated path is
[Physical AI Runtime](https://github.com/Gabriel-Ning/physical_ai_runtime);
standalone requirements are in [Dependencies](docs/DEPENDENCIES.md).

## Quick start (recommended)

Use with [Physical AI Runtime](https://github.com/Gabriel-Ning/physical_ai_runtime),
which locks IsaacTeleop, CUDA/PyTorch, and ROS together:

```bash
cd /path/to/physical_ai_runtime
git clone https://github.com/Gabriel-Ning/isaacteleop_toolbox.git \
  src/teleop/isaacteleop_toolbox
pixi install --locked
pixi run build
source install/setup.bash
ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup
ros2 launch isaacteleop_toolbox bimanual_target_live.launch.py
```

More commands (topics, replay, tests): [Usage](docs/USAGE.md).  
CloudXR setup and Quest connection: [CloudXR](docs/CLOUDXR.md).

Then open on the Quest browser:

```text
https://<pc-lan-ip>:48322/client/
```

On a normal LAN the PC IP is auto-detected; you do not put it in ROS YAML.

## License

Package source is Apache-2.0. Downloaded CloudXR / WebXR assets keep their
upstream terms and are not redistributed in this repository.
