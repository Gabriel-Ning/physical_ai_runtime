# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace template** for Physical AI systems. One
locked environment covers ROS, native libraries, CUDA/PyTorch, and
teleop/planning Python dependencies. This repository does **not** vendor ROS
packages under `src/`; it only provides the Pixi/colcon shell, empty ownership
directories, and docs.

Architecture and migration notes live under [`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Migration](docs/MIGRATION.md)

Reusable bringup/toolbox packages live in a separate repository:
[`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources).

## Features

- **Pixi**: single solved environment for ROS 2 Jazzy, system deps, and PyPI
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`, `stop`
- **Empty `src/` layout**: category folders with `.gitkeep`; add packages via
  clone/submodule when needed

## Prerequisites

- [Pixi](https://pixi.sh/latest/#installation)
- [Direnv](https://direnv.net/) (recommended; install steps under Activate)
- Git (submodules optional, only if you pull external packages that way)

## Getting Started

### 1. Clone

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

### Add ROS packages (optional)

This template ships with empty `src/` categories only. Pull packages you need,
for example:

```bash
# Bringups + RViz marker teleop
git submodule add git@github.com:Gabriel-Ning/runtime_resources.git \
  src/runtime_resources

# Other component repos (examples)
# git submodule add git@github.com:Gabriel-Ning/manipulation_execution_manager.git \
#   src/execution/manipulation_execution_manager
# git submodule add git@github.com:Gabriel-Ning/isaacteleop_toolbox.git \
#   src/teleop/isaacteleop_toolbox
```

`colcon` discovers packages recursively under `src/`.

### Cross-machine DDS (optional)

Same-machine ROS needs no extra CycloneDDS file. When nodes run on **different
machines** over a dedicated LAN (for example camera hosts publishing large
`sensor_msgs/Image` topics), bind each host to that LAN NIC and point peers at
each other:

1. Copy [`.config/cyclonedds.xml.template`](.config/cyclonedds.xml.template) to a
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
frames. Commit only `.config/*.template`; filled-in host XML stays local
(gitignored).

### 4. Build

```bash
pixi run build
```

Default build type is `Release`. After the first successful build,
`install/setup.bash` is sourced automatically when Direnv / `.envrc` is active.

### 5. Test

```bash
pixi run test
```

### 6. Clean

Remove colcon `build/`, `install/`, and `log/`:

```bash
pixi run clean
```

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
- Default ROS distro is **Jazzy**.
- Application launches stay in ROS packages pulled into `src/`; Pixi tasks stay
  limited to workspace lifecycle (`setup` / `build` / `test` / `clean` / `stop`).
