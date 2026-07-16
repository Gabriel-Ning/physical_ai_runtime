# Physical AI Runtime

A Pixi-managed **ROS 2 Jazzy workspace template** for Physical AI systems.
Everyone starts from this shared baseline: install the locked environment, clone
the necessary packages into `src/`, then optionally pull the Marvin example to
learn the layout and develop on top. Domain and application packages stay in
their own repositories; this workspace owns the Pixi/colcon scaffolding, docs,
and small reusable toolbox packages.

Architecture, examples, and migration notes live under [`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Example 1 — Marvin bringup](docs/EXAMPLE1.md)
- [Migration](docs/MIGRATION.md)

## Features

- **Pixi**: single solved environment for ROS 2 Jazzy, system deps, and PyPI
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`, `stop`
- **Ownership-based `src/` layout**: external domain/application repositories
  are imported with `vcstool`; small workspace-level utilities live under
  `src/toolbox`

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

### 4. Clone functional packages and examples

Import baseline functional packages from `necessary.repos`. Import the optional
flat `runtime_resources` application checkout separately from `example.repos`.
Use [`vcs`](https://github.com/dirk-thomas/vcstool) (already in the Pixi env)
for both — **do not** add the checkouts as git submodules, and **do not** commit
them back into this template.

| Manifest | Purpose | Checkout roots |
|---|---|---|
| `repos/necessary.repos` | Reusable execution, teleop, and motion-planning modules | `src/execution`, `src/teleop`, `src/motion_planning` |
| `repos/example.repos` | Optional runnable example applications | `src/apps` |
| `repos/embodiment.repos` | Marvin description and hardware integration | `src/embodiments/robots/marvin` |

```bash
vcs import src < repos/necessary.repos
# Optional example applications
vcs import src < repos/example.repos
# Required when running the Marvin example
vcs import src < repos/embodiment.repos
pixi run build
```

Safe to run again: missing entries are added; to refresh existing checkouts:

```bash
vcs pull src
```

See each package README for launches, CloudXR setup, and tests.
`isaacteleop_toolbox` and the motion-planner adapters need the `main` / GPU
Pixi env (not the `cpu` branch).

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
- Pixi tasks stay limited to workspace lifecycle (`setup` / `build` /
  `test` / `clean` / `stop`). For Marvin Example 1, see
  [docs/EXAMPLE1.md](docs/EXAMPLE1.md).
- Contribute Pixi/docs/template changes here; contribute package changes in
  each package's own repository.
