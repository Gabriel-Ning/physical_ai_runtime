# Example 1 — Marvin bringup

The current example is Marvin. Its application packages come from one flat
[`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources)
checkout at `src/apps`, plus Marvin embodiment packages and the
workspace-owned marker toolbox.
It builds on the necessary execution manager (RViz marker → EM →
task-space controller). Prefer `use_fake_hardware:=true` until you
intentionally gate real hardware.

Prerequisites: complete [Getting Started](../README.md#getting-started) through
cloning necessary packages (`repos/necessary.repos`), example applications
(`repos/example.repos`), and a successful build.

## Fetch and build

```bash
# Necessary functional packages
vcs import src < repos/necessary.repos

# Optional example applications (flat runtime_resources checkout)
vcs import src < repos/example.repos

# Marvin description and hardware integration
vcs import src < repos/embodiment.repos

pixi run build
```

Then follow `src/apps/marvin_motion_planning_bringup` or
`src/apps/marvin_controller_bringup`.

Application launches live under `src/apps/`; Pixi
tasks stay limited to workspace lifecycle (`setup` / `build` / `test` /
`clean` / `stop`).

## Package map

| Role | Package | Place under |
|------|---------|-------------|
| Necessary | [`manipulation_execution_manager`](https://github.com/Gabriel-Ning/manipulation_execution_manager) | `src/execution/…` via `repos/necessary.repos` |
| Necessary | [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox) | `src/teleop/…` via `repos/necessary.repos` |
| Necessary | [`motion_planners`](https://github.com/Gabriel-Ning/motion_planners) | `src/motion_planning/motion_planners` via `repos/necessary.repos` |
| Example 1 | Flat `runtime_resources` app packages | `src/apps/` via `repos/example.repos` |
| Toolbox | `rviz_interactive_marker_teleop` | Workspace-owned at `src/toolbox/` |
| Embodiment | [`marvin_description`](https://github.com/Gabriel-Ning/marvin_description) (`dev`) | `src/embodiments/robots/marvin/…` via `repos/embodiment.repos` |
| Embodiment | [`marvin_hardware_interface`](https://github.com/Gabriel-Ning/marvin_hardware_interface) | `src/embodiments/robots/marvin/…` via `repos/embodiment.repos` |

`colcon` discovers packages recursively under `src/`.

See also [Architecture](ARCHITECTURE.md) for ownership layout and product
boundary.
