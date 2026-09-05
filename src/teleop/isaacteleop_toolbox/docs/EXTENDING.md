# Extending the template

This package is a concrete Quest bimanual template, not a generic teleop SDK.
Add the next application as the smallest vertical slice.

## Extension points

| Location | Own |
|---|---|
| `retargeters/` | New `BaseRetargeter` (follow `bimanual_relative.py`) |
| `session_builders.py` | IsaacTeleop graph wiring |
| `nodes/` | ROS conversion only when the output contract differs |
| `configs/` + `launch/` | One profile + source-only launch |
| Robot **app** package | EM, controllers, frames, safety, fake/real hardware |

`runtime.py` owns the shared live/replay `TeleopSession` loop. Do not duplicate
CloudXR lifecycle there.

`cloudxr_host_client.py` is the only local host-client compatibility layer
(nested WebXR profile serving). Prefer deleting code when upstream covers it.

## Checklist for a new app

1. Retargeter + unit tests under `retargeters/`
2. Session graph in `session_builders.py` (reuse when possible)
3. Thin ROS node only if needed
4. Parameter YAML + source-only launch
5. Compose with EM/controllers in the robot app
6. Pass: unit → CloudXR → source-only / replay → fake HW → low-speed real HW

## New physical inputs

If IsaacTeleop cannot represent the device, add a `DeviceIO` source **upstream**
in IsaacTeleop (transport, timestamps, validity, record/replay schema). Keep
robot frame mapping and application retargeting here. Pin the consuming
workspace to the IsaacTeleop revision that contains the device.

## IsaacTeleop upgrades

On every wheel / revision bump in the consuming lockfile:

1. Review `DeviceIO`, `TeleopSession`, `ControllersSource`, retargeter specs,
   MCAP schemas, host-client asset behavior.
2. Recheck `cloudxr_host_client.py`.
3. Re-run unit tests, CloudXR setup, source-only live/replay, then robot-app
   gates.
4. Do not treat `import isaacteleop` success as compatibility.
5. Record the tested revision in the runtime lock / changelog.
