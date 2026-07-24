# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace template** for Physical AI systems.
Everyone starts from this shared baseline: install the locked environment, clone
the necessary packages into `src/`, then optionally pull the Marvin example to
learn the layout and develop on top. Domain and application packages stay in
their own repositories; this workspace owns the Pixi/colcon scaffolding, docs,
and small reusable toolbox packages.

Architecture, examples, and migration notes live under [`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Runtime orchestration SDK and API](docs/RUNTIME_ORCHESTRATION.md)
- [Example 1 — Marvin bringup](docs/EXAMPLE1.md)
- [Migration](docs/MIGRATION.md)

## Features

- **Pixi**: locked multi-env workspace — **default = GPU** (PyPI + CloudXR),
  optional **`cpu`** env (conda-only)
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`, `stop`
- **Ownership-based `src/` layout**: external domain/application repositories
  are imported with `vcstool`; small workspace-level utilities live under
  `src/toolbox`

## Requirements

Python is managed by Pixi (conda-style solver). If you don't have Pixi yet:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

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

Default install is the **GPU** environment:

```bash
pixi install --locked
pixi run setup
```

For the **CPU** / realtime-kernel environment (no NVIDIA / Isaac Teleop /
cuRobo / CloudXR):

```bash
pixi install --locked -e cpu
pixi run -e cpu setup
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

`.envrc` follows the env you used for setup: `pixi run setup` writes
`.pixi/environment` (`default` or `cpu`), and Direnv activates that same
env. Override with `PIXI_ENV=cpu` / `PIXI_ENV=default` if needed.

Pixi selects the dependency environment before `setup.sh` starts. Use
`pixi run setup` for GPU or `pixi run -e cpu setup` for CPU; the setup script
then applies environment-specific resources. Currently that means creating
`CLOUDXR_DIR` only for GPU and explicitly clearing it when Direnv selects CPU.
Future environment-specific setup should follow the same environment-name and
activation-variable pattern.

Without Direnv:

```bash
eval "$(pixi shell-hook --frozen)"           # GPU (default)
# eval "$(pixi shell-hook --frozen -e cpu)"  # CPU
# or: source .envrc
```

`WORKSPACE_ROOT` and `RMW_IMPLEMENTATION` come from `pixi.toml`
`[activation.env]` via the shell hook. `CLOUDXR_DIR` is set only in the
default (GPU) environment.

### 4. Clone functional packages and examples

Import baseline functional packages from `necessary.repos`. Import the optional
flat `runtime_resources` application checkout separately from `example.repos`.
Use [`vcs`](https://github.com/dirk-thomas/vcstool) (already in the Pixi env)
for both — **do not** add the checkouts as git submodules, and **do not** commit
them back into this template.

| Manifest | Purpose | Checkout roots |
| --- | --- | --- |
| `repos/necessary.repos` | Reusable execution, teleop, and motion-planning modules | `src/execution`, `src/teleop`, `src/motion_planning` |
| `repos/example.repos` | Optional runnable example applications | `src/apps` |
| `repos/embodiment.repos` | Robot and sensor embodiment integrations (Marvin and Hikvision) | `src/embodiments/robots/marvin`, `src/embodiments/sensors/hikvision_ros2` |

```bash
vcs import src < repos/necessary.repos
# Optional example applications
vcs import src < repos/example.repos
# Required for the Marvin and Hikvision embodiments
vcs import src < repos/embodiment.repos
pixi run build
```

Safe to run again: missing entries are added; to refresh existing checkouts:

```bash
vcs pull src
```

See each package README for launches, CloudXR setup, and tests.
`isaacteleop_toolbox` and the motion-planner adapters need the default GPU
Pixi env (`pixi install`, not `-e cpu`).

### 5. Build / test / clean

```bash
pixi run build
pixi run test
pixi run clean   # removes colcon build/ install/ log/
```

Default build type is `Release`. After the first successful build,
`install/setup.bash` is sourced automatically when Direnv / `.envrc` is active.

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

- **Shared conda / ROS / native packages**: edit `pixi.toml` `[dependencies]`,
  then `pixi lock` / `pixi install` (with `PIXI_FROZEN` unset)
- **GPU-only PyPI packages**: edit `pixi.toml` `[feature.gpu.pypi-dependencies]`
  the same way
- **ROS package deps**: declare in each package's `package.xml`

Do not pip-install `numpy` or `opencv-python` into this prefix; Conda/RoboStack
owns those ABIs.

## Notes

- ROS distro and the integrated stack are defined in [`pixi.toml`](pixi.toml)
  and locked by [`pixi.lock`](pixi.lock).
- Default ROS distro is **Jazzy**. Default Pixi env is **GPU**; use
  `pixi install -e cpu` for the conda-only stack without NVIDIA / Isaac
  Teleop / cuRobo.
- Pixi tasks stay limited to workspace lifecycle (`setup` / `build` /
  `test` / `clean` / `stop`). For Marvin Example 1, see
  [docs/EXAMPLE1.md](docs/EXAMPLE1.md).
- Contribute Pixi/docs/template changes here; contribute package changes in
  each package's own repository.
