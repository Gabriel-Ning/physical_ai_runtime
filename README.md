# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace** for Physical AI systems. The repository
owns the shared development environment, RMI Python SDK, reusable controllers,
robot RT bringup composition, host setup and architecture contracts. Teleop,
planning and recording repositories are imported into the ownership-oriented
`src/` layout when needed.

Host setup notes live under [`docs/`](docs/):

- [Current repository architecture](docs/ARCHITECTURE.md)
- [Marvin validation status and current gates](docs/MARVIN_VALIDATION_STATUS.md)
- [Dynamic authority design rationale](docs/EM_RMI_DYNAMIC_AUTHORITY_PROPOSAL.md)
- [Episode Recorder lifecycle](docs/EPISODE_RECORDER.md)
- [Cross-host clock sync (workstation ↔ RT PC)](docs/CLOCK_SYNC.md)
- [CPU / isolcpus / RT host setup](docs/CPU_HOST_SETUP.md)
- [udev CAN aliases](docs/UDEV_HOST_SETUP.md)
- [Embodiment bringup (Marvin / Franka / Piper)](src/bringup/README.md)

## Features

- **Pixi**: locked multi-env workspace — **default = robot CPU stack** (previous
  `cpu` env), optional **`cpu`** (same packages + RT host profile), **`curobo`**
  (CUDA 13 / CloudXR), **`lerobot`** (policy training stack)
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
git clone https://github.com/Gabriel-Ning/physical_ai_runtime.git
cd physical_ai_runtime
```

Use HTTPS (not SSH) for cloning.

### 2. Initialize the environment

Default install is the **robot CPU stack** (previous `cpu` environment):

```bash
pixi install --locked
pixi run setup
```

For a **CPU** / realtime-kernel control host (performance governor, `isolcpus`,
`ros2_control` affinity), follow
[docs/CPU_HOST_SETUP.md](docs/CPU_HOST_SETUP.md):

```bash
pixi install --locked -e cpu
pixi run -e cpu setup
# if setup exits 3: sudo reboot && pixi run -e cpu setup
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
`.pixi/environment` (`default`, `runtime`, or `cpu`), and Direnv activates
that same env. Override with `PIXI_ENV=cpu` / `PIXI_ENV=curobo` if needed.

Pixi selects the dependency environment before `setup.sh` starts. Use
`pixi run setup` for the robot stack or `pixi run -e cpu setup` on an RT
host; the setup script then applies environment-specific resources
(`curobo`: `CLOUDXR_DIR`; `cpu`: RT host governor / isolcpus —
[docs/CPU_HOST_SETUP.md](docs/CPU_HOST_SETUP.md)).

Without Direnv:

```bash
eval "$(pixi shell-hook --frozen)"             # robot CPU stack (default)
# eval "$(pixi shell-hook --frozen -e cpu)"    # same packages + RT host profile
# eval "$(pixi shell-hook --frozen -e curobo)" # CUDA 13 / CloudXR
# or: source .envrc
```

`WORKSPACE_ROOT` and `RMW_IMPLEMENTATION` come from `pixi.toml`
`[activation.env]` via the shell hook. `CLOUDXR_DIR` is set only in the
`curobo` environment.

### 4. Clone functional packages

Import baseline functional packages from `necessary.repos`.
Use [`vcs`](https://github.com/dirk-thomas/vcstool) (already in the Pixi env)
— **do not** add the checkouts as git submodules, and **do not** commit
them back into this template.

| Manifest | Purpose | Checkout roots |
| --- | --- | --- |
| `repos/necessary.repos` | Workstation teleop / motion-planning / recorder | `src/teleop`, `src/motion_planning`, `src/recording` |
| `src/embodiments` submodule | Owned Marvin / Piper / Pika, plus pinned Franka vendor trees | [`phy_ai_runtime_embodiments`](https://github.com/Gabriel-Ning/phy_ai_runtime_embodiments) (HTTPS nested submodules) |
| `repos/embodiment.repos` | Vendor Hikvision (not required for RT bringup) | `src/embodiments/sensors/hikvision_ros2` |

RT control host: follow [CPU host setup](docs/CPU_HOST_SETUP.md) — build
`src/rt_launch` + `src/embodiments`, use Pixi controller binaries, do not
import `necessary.repos` or Hikvision.

```bash
vcs import src < repos/necessary.repos
# Owned embodiments + pinned Franka (HTTPS nested submodules)
git submodule update --init --recursive -- src/embodiments
# Vendor Hikvision
vcs import src < repos/embodiment.repos
bash scripts/franka_colcon_ignore.sh   # Franka: core-arm filter
pixi run build
```

Safe to run again: missing entries are added; to refresh existing checkouts:

```bash
vcs pull src
```

See each package README for launches, CloudXR setup, and tests.
`isaacteleop_toolbox` and the motion-planner adapters need the `curobo`
Pixi env (`pixi install -e curobo`).

### 5. Build / test / clean

`build` / `test` / `clean` / `stop` are **runtime** tasks (colcon + the robot
CPU stack). They are not available in `lerobot`. `default` and `cpu` include
the same `runtime` feature, so `pixi run build` and `pixi run -e cpu build`
are equivalent.

```bash
pixi run -e runtime build
pixi run -e runtime test
pixi run -e runtime clean   # removes colcon build/ install/ log/
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
  `pixi install -e cpu` for the conda-only RT control host (see
  [docs/CPU_HOST_SETUP.md](docs/CPU_HOST_SETUP.md)).
- Pixi tasks stay limited to workspace lifecycle (`setup` / `build` /
  `test` / `clean` / `stop`).
- Contribute Pixi/docs/template changes here; contribute package changes in
  each package's own repository.
