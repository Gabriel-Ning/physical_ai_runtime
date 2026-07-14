# Example 1 — Marvin bringup

The current example is Marvin:
[`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources)
apps (→ `src/apps/`) plus Marvin embodiment packages and the marker toolbox.
It builds on the necessary execution manager (RViz marker → EM →
task-space controller). Prefer `use_fake_hardware:=true` until you
intentionally gate real hardware.

Prerequisites: complete [Getting Started](../README.md#getting-started) through
cloning necessary packages (`repos/necessary.repos`) and a successful build.

## Fetch and build

```bash
# Embodiment (separate Git repos) via vcs — re-runnable
vcs import src < repos/embodiment.repos

# Example 1 apps from runtime_resources → src/apps/ (see repos/example1.repos)
bash scripts/fetch_example1_apps.sh

# Marker toolbox from the same monorepo → src/toolbox/
mkdir -p src/toolbox
curl -fsSL https://github.com/Gabriel-Ning/runtime_resources/archive/refs/heads/main.tar.gz \
  | tar -xz --strip-components=2 -C src/toolbox runtime_resources-main/toolbox

pixi run build
```

Do **not** nest the whole `runtime_resources` tree under `src/runtime_resources/`.
Then follow `src/apps/marvin_rviz_debug_bringup`.

Application launches live under `src/apps/` (from `runtime_resources`); Pixi
tasks stay limited to workspace lifecycle (`setup` / `build` / `test` /
`clean` / `stop`).

## Package map

| Role | Package | Place under |
|------|---------|-------------|
| Necessary | [`manipulation_execution_manager`](https://github.com/Gabriel-Ning/manipulation_execution_manager) | `src/execution/…` via `repos/necessary.repos` |
| Necessary | [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox) | `src/teleop/…` via `repos/necessary.repos` |
| Example 1 | `runtime_resources` `apps/*` | `src/apps/` via `repos/example1.repos` + `scripts/fetch_example1_apps.sh` |
| Example 1 | `runtime_resources` `toolbox/*` | `src/toolbox/` via `curl\|tar` |
| Embodiment | [`marvin_description`](https://github.com/Gabriel-Ning/marvin_description) | `src/embodiments/robots/marvin/…` via `repos/embodiment.repos` |
| Embodiment | [`marvin_hardware_interface`](https://github.com/Gabriel-Ning/marvin_hardware_interface) | `src/embodiments/robots/marvin/…` via `repos/embodiment.repos` |

`colcon` discovers packages recursively under `src/`.

See also [Architecture](ARCHITECTURE.md) for ownership layout and product
boundary.
