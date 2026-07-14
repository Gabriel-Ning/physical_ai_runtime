# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace template** for Physical AI systems.
Everyone starts from this shared baseline: install the locked environment, clone
the necessary packages into `src/`, then optionally pull the Marvin example to
learn the layout and develop on top. Component code stays in its own
repositories — this tree only provides Pixi/colcon scaffolding, ownership
directories, and docs.

Architecture and migration notes live under [`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Migration](docs/MIGRATION.md)

## Features

- **Pixi**: single solved environment for ROS 2 Jazzy, system deps, and PyPI
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`, `stop`
- **Empty `src/` layout**: category folders with `.gitkeep`; packages are
  cloned in (not submodules, not vendored here)

## Prerequisites

- [Pixi](https://pixi.sh/latest/#installation)
- [Direnv](https://direnv.net/) (recommended; install steps under Activate)
- Git

## Getting Started

### 1. Clone the workspace template

```bash
git clone git@github.com:Gabriel-Ning/physical_ai_runtime.git
cd physical_ai_runtime
```

### 2. Initialize the environment

```bash
pixi install --locked
pixi run setup
```

### 3. Activate (recommended: Direnv)

Install Direnv if needed (Ubuntu / Debian):

```bash
sudo apt update
sudo apt install -y direnv
```

Hook it into your shell (bash shown; see [Direnv docs](https://direnv.net/docs/hook.html)
for zsh/fish), then restart the shell:

```bash
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
source ~/.bashrc
```

Allow this repository:

```bash
direnv allow
```

After this, entering the repository directory activates the locked Pixi
environment and sources `install/setup.bash` when it exists. Leaving the
directory deactivates it.

Without Direnv:

```bash
eval "$(pixi shell-hook --frozen)"
# or: source .envrc
```

`CLOUDXR_DIR`, `WORKSPACE_ROOT`, and `RMW_IMPLEMENTATION` come from
`pixi.toml` `[activation.env]` via the shell hook.

### 4. Clone necessary packages

These two packages are part of the baseline stack. Clone them into the matching
ownership paths — **do not** add them as git submodules, and **do not** commit
those checkouts back into this template.

```bash
# Execution manager (validation / arbitration / streaming routes)
git clone https://github.com/Gabriel-Ning/manipulation_execution_manager.git \
  src/execution/manipulation_execution_manager

# Isaac Teleop source package (needs the main / GPU Pixi env, not cpu)
git clone https://github.com/Gabriel-Ning/isaacteleop_toolbox.git \
  src/teleop/isaacteleop_toolbox

pixi run build
```

See each package README for launches, CloudXR setup, and tests.

### 5. Build / test / clean

```bash
pixi run build
pixi run test
pixi run clean   # removes colcon build/ install/ log/
```

Default build type is `Release`. After the first successful build,
`install/setup.bash` is sourced automatically when Direnv / `.envrc` is active.

## Example 1 — Marvin bringup

The current example is Marvin:
[`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources)
(`apps/` → `src/apps/`, `toolbox/` → `src/toolbox/`) plus the Marvin
embodiment packages. It builds on the necessary execution manager above
(RViz marker → EM → task-space controller). Prefer
`use_fake_hardware:=true` until you intentionally gate real hardware.

```bash
# Place example packages into matching src/ ownership dirs
git clone https://github.com/Gabriel-Ning/runtime_resources.git /tmp/runtime_resources
cp -a /tmp/runtime_resources/apps/marvin_rviz_debug_bringup \
      /tmp/runtime_resources/apps/marvin_trajectory_jtc_test \
      src/apps/
cp -a /tmp/runtime_resources/toolbox/rviz_interactive_marker_teleop \
      src/toolbox/

# Marvin embodiment (external repos)
mkdir -p src/embodiments/robots/marvin
git clone https://github.com/Gabriel-Ning/marvin_description.git \
  src/embodiments/robots/marvin/marvin_description
git clone https://github.com/Gabriel-Ning/marvin_hardware_interface.git \
  src/embodiments/robots/marvin/marvin_hardware_interface

pixi run build
```

Then follow `src/apps/marvin_rviz_debug_bringup`. Do **not** nest the whole
`runtime_resources` repo under `src/runtime_resources/`.

### Package map

| Role | Package | Clone / place into |
|------|---------|-------------------|
| Necessary | [`manipulation_execution_manager`](https://github.com/Gabriel-Ning/manipulation_execution_manager) | `src/execution/manipulation_execution_manager` |
| Necessary | [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox) | `src/teleop/isaacteleop_toolbox` |
| Example 1 | `runtime_resources` → `apps/marvin_rviz_debug_bringup` | `src/apps/marvin_rviz_debug_bringup` |
| Example 1 | `runtime_resources` → `apps/marvin_trajectory_jtc_test` | `src/apps/marvin_trajectory_jtc_test` |
| Example 1 | `runtime_resources` → `toolbox/rviz_interactive_marker_teleop` | `src/toolbox/rviz_interactive_marker_teleop` |
| Example 1 | [`marvin_description`](https://github.com/Gabriel-Ning/marvin_description) | `src/embodiments/robots/marvin/marvin_description` |
| Example 1 | [`marvin_hardware_interface`](https://github.com/Gabriel-Ning/marvin_hardware_interface) | `src/embodiments/robots/marvin/marvin_hardware_interface` |

`colcon` discovers packages recursively under `src/`.

## Cross-machine DDS (optional)

Same-machine ROS needs no extra CycloneDDS file. When nodes run on **different
machines** over a dedicated LAN (for example camera hosts publishing large
`sensor_msgs/Image` topics), bind each host to that LAN NIC and point peers at
each other:

1. Copy [`.config/cyclonedds_template.xml`](.config/cyclonedds_template.xml) to a
   local path under `.config/` (for example
   `.config/cyclonedds_hik_host_192.168.10.100.xml`).
2. Replace `{{LOCAL_ADDRESS}}` / `{{PEER_ADDRESS}}` with each machine's LAN IP.
3. On every participating shell (or uncomment the matching lines in
   `pixi.toml` `[activation.env]`):

```bash
export ROS_DOMAIN_ID=1   # same domain on all hosts
export CYCLONEDDS_URI=file://$PWD/.config/cyclonedds_hik_host_192.168.10.100.xml
```

Use jumbo frames (MTU 9000) on that LAN when shipping uncompressed camera
frames. Commit only `.config/cyclonedds_template.xml`; filled-in host XML stays
local (gitignored).

## Adding Dependencies

- **Conda / ROS / native packages**: edit `pixi.toml` `[dependencies]`, then
  `pixi lock` / `pixi install` (with `PIXI_FROZEN` unset)
- **PyPI packages**: edit `pixi.toml` `[pypi-dependencies]` the same way
- **ROS package deps**: declare in each package's `package.xml`

Do not pip-install `numpy` or `opencv-python` into this prefix; Conda/RoboStack
owns those ABIs.

## Notes

- ROS distro and the integrated stack are defined in [`pixi.toml`](pixi.toml)
  and locked by [`pixi.lock`](pixi.lock).
- Default ROS distro is **Jazzy**. Use branch `cpu` when you need a
  conda-only env without NVIDIA / Isaac Teleop / cuRobo.
- Application launches for Example 1 live under `src/apps/` (from
  `runtime_resources`); Pixi tasks stay limited to workspace lifecycle
  (`setup` / `build` / `test` / `clean` / `stop`).
- Contribute Pixi/docs/template changes here; contribute package changes in
  each package's own repository.
