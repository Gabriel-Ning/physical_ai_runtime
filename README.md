# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace template** for Physical AI systems.
Teams clone this repository as a shared baseline, then pull the ROS packages
they need into `src/` and develop on top. One locked Pixi environment covers
ROS, native libraries, CUDA/PyTorch, and teleop/planning Python dependencies.

This repository does **not** vendor ROS packages and does **not** use git
submodules for application/component repos. Empty ownership directories under
`src/` are scaffolding only.

Architecture and migration notes live under [`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Migration](docs/MIGRATION.md)

Related package repositories (clone into `src/` as needed):

- [`manipulation_execution_manager`](https://github.com/Gabriel-Ning/manipulation_execution_manager)
- [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox)
- [`marvin_description`](https://github.com/Gabriel-Ning/marvin_description)
- [`marvin_hardware_interface`](https://github.com/Gabriel-Ning/marvin_hardware_interface)
- [`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources) (bringups / toolbox)

## Features

- **Pixi**: single solved environment for ROS 2 Jazzy, system deps, and PyPI
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`, `stop`
- **Empty `src/` layout**: category folders with `.gitkeep`; add packages with
  `git clone` (not submodules)

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

### Add ROS packages (clone into `src/`)

This template ships with empty `src/` categories only. Clone component repos
into the matching ownership path — **do not** add them as git submodules of
this workspace, and **do not** commit those checkouts back into the template.

```bash
# Execution manager
git clone https://github.com/Gabriel-Ning/manipulation_execution_manager.git \
  src/execution/manipulation_execution_manager

# Isaac Teleop source package (needs the main / GPU Pixi env)
git clone https://github.com/Gabriel-Ning/isaacteleop_toolbox.git \
  src/teleop/isaacteleop_toolbox

# Marvin embodiment (description + hardware interface)
mkdir -p src/embodiments/robots/marvin
git clone https://github.com/Gabriel-Ning/marvin_description.git \
  src/embodiments/robots/marvin/marvin_description
git clone https://github.com/Gabriel-Ning/marvin_hardware_interface.git \
  src/embodiments/robots/marvin/marvin_hardware_interface

# Optional: bringups + RViz marker teleop
# git clone https://github.com/Gabriel-Ning/runtime_resources.git \
#   src/runtime_resources
```

`colcon` discovers packages recursively under `src/`. After cloning:

```bash
pixi run build
```

### Cross-machine DDS (optional)

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
- Application launches stay in ROS packages cloned into `src/`; Pixi tasks stay
  limited to workspace lifecycle (`setup` / `build` / `test` / `clean` / `stop`).
- Prefer contributing Pixi/docs/template changes here; contribute package
  changes in each package's own repository.
