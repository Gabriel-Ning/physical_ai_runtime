# Physical AI Runtime

A modern ROS 2 Jazzy workspace for Physical AI systems, managed with
[Pixi](https://pixi.sh/). One locked environment covers ROS, native libraries,
CUDA/PyTorch, and teleop/planning Python dependencies.

Architecture, ownership rules, and migration gates live under
[`docs/`](docs/):

- [Architecture](docs/ARCHITECTURE.md)
- [Migration](docs/MIGRATION.md)

## Features

- **Pixi**: single solved environment for ROS 2 Jazzy, system deps, and PyPI
- **Direnv** (recommended): enter the directory → frozen Pixi shell + colcon overlay
- **Pre-configured tasks**: `setup`, `build`, `test`, `clean`
- **Submodules**: reusable packages such as
  [`isaacteleop_toolbox`](https://github.com/Gabriel-Ning/isaacteleop_toolbox)

## Prerequisites

- [Pixi](https://pixi.sh/latest/#installation)
- [Direnv](https://direnv.net/) (recommended; install steps under Activate)
- Git with submodule support

## Getting Started

### 1. Clone

```bash
git clone --recurse-submodules git@github.com:Gabriel-Ning/physical_ai_runtime.git
cd physical_ai_runtime
# or after a plain clone:
git submodule update --init --recursive
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
- Application launches stay in ROS packages; Pixi tasks stay limited to
  workspace lifecycle (`setup` / `build` / `test` / `clean`).
