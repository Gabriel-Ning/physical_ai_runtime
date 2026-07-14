# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace template** for Physical AI systems.
Everyone starts from this shared baseline: install the locked environment, then
clone external example packages into `src/` to learn the layout and develop on
top. Component code stays in its own repositories — this tree only provides
Pixi/colcon scaffolding, ownership directories, and docs.

Architecture and migration notes live under [`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Migration](docs/MIGRATION.md)

## Features

- **Pixi**: single solved environment for ROS 2 Jazzy, system deps, and PyPI
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`, `stop`
- **Empty `src/` layout**: category folders with `.gitkeep`; tutorials clone
  packages in (not submodules, not vendored here)

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

### 4. Build / test / clean

With an empty `src/` the build is a no-op for packages; after you clone
examples below, run:

```bash
pixi run build
pixi run test
pixi run clean   # removes colcon build/ install/ log/
```

Default build type is `Release`. After the first successful build,
`install/setup.bash` is sourced automatically when Direnv / `.envrc` is active.

## Examples and tutorials

External repos are **examples**: clone only what you need into the matching
ownership path, build, and follow each package's own README. Do **not** add
them as git submodules of this workspace, and do **not** commit those checkouts
back into the template.

### Example A — Execution manager

Learn validation / arbitration / streaming routes:

```bash
git clone https://github.com/Gabriel-Ning/manipulation_execution_manager.git \
  src/execution/manipulation_execution_manager
pixi run build
# then see that package's README for launch and tests
```

### Example B — Marvin fake-hardware bringup

End-to-end familiarization: description + hardware plugin + debug apps
(RViz marker → execution manager → task-space controller).

```bash
mkdir -p src/embodiments/robots/marvin

git clone https://github.com/Gabriel-Ning/marvin_description.git \
  src/embodiments/robots/marvin/marvin_description
git clone https://github.com/Gabriel-Ning/marvin_hardware_interface.git \
  src/embodiments/robots/marvin/marvin_hardware_interface

git clone https://github.com/Gabriel-Ning/manipulation_execution_manager.git \
  src/execution/manipulation_execution_manager

git clone https://github.com/Gabriel-Ning/runtime_resources.git \
  src/runtime_resources

pixi run build
```

Then open [`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources)
and the Marvin app READMEs (for example `marvin_rviz_debug_bringup`) for launch
steps. Prefer `use_fake_hardware:=true` until you intentionally gate real
hardware.

### Example C — Isaac Teleop (GPU / `main` Pixi env)

Quest-class teleop source package (needs CUDA/PyTorch from the `main` branch
environment, not the `cpu` branch):

```bash
git clone https://github.com/Gabriel-Ning/isaacteleop_toolbox.git \
  src/teleop/isaacteleop_toolbox
pixi run build
# see isaacteleop_toolbox README for CloudXR setup and live launch
```

### Package map

| Example package | Clone into |
|-----------------|------------|
| [`manipulation_execution_manager`](https://github.com/Gabriel-Ning/manipulation_execution_manager) | `src/execution/manipulation_execution_manager` |
| [`marvin_description`](https://github.com/Gabriel-Ning/marvin_description) | `src/embodiments/robots/marvin/marvin_description` |
| [`marvin_hardware_interface`](https://github.com/Gabriel-Ning/marvin_hardware_interface) | `src/embodiments/robots/marvin/marvin_hardware_interface` |
| [`runtime_resources`](https://github.com/Gabriel-Ning/runtime_resources) | `src/runtime_resources` |
| [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox) | `src/teleop/isaacteleop_toolbox` |

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
- Application launches live in the external example packages; Pixi tasks stay
  limited to workspace lifecycle (`setup` / `build` / `test` / `clean` / `stop`).
- Contribute Pixi/docs/template changes here; contribute package changes in
  each package's own repository.
