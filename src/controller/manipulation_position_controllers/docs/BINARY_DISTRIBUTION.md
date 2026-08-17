# Binary distribution on prefix.dev

This repository can be built privately and published as the conda package
`ros-jazzy-manipulation-position-controllers` to the
`gabriel-robotics` Prefix channel.

## Distribution boundary

The binary package contains only:

- `libmanipulation_position_controllers.so`;
- package-private, renamed OSQP and OsqpEigen runtime libraries;
- the OSQP Apache-2.0 license and NOTICE, and the OsqpEigen BSD-3-Clause
  license;
- the pluginlib and ament-index resource entries;
- `package.xml` and the plugin description XML;
- the four reference controller YAML files.

It deliberately excludes C++ headers, implementation sources, examples, tests,
and CMake export targets. Consumers can load and configure the controllers
through `controller_manager`, but cannot compile against this package as a C++
development library.

This protects source distribution, not machine code. Native shared libraries
can still be inspected or reverse engineered. Legal protection and channel
access control remain necessary.

## Build locally

Install `rattler-build`, then build from the repository root:

```bash
rattler-build build \
  --recipe packaging/conda/recipe.yaml \
  --channel https://prefix.dev/gabriel-robotics \
  --channel robostack-jazzy \
  --channel conda-forge
```

The recipe uses the checked-out local source. This allows a private GitHub
repository or private CI checkout to build without embedding Git credentials in
the recipe. Do not use `--no-include-recipe` as a substitute for secret
handling: recipes must never contain credentials.

## Upload

Create the `gabriel-robotics` channel before uploading. For an interactive
release, authenticate through the system keychain:

```bash
rattler-build auth login prefix.dev --token <PREFIX_TOKEN>
rattler-build upload prefix \
  --channel gabriel-robotics \
  output/linux-64/ros-jazzy-manipulation-position-controllers-*.conda
```

For GitHub Actions, configure this private repository as a Prefix trusted
publisher and grant the workflow `id-token: write`. Trusted publishing avoids a
long-lived API key in GitHub secrets.

## Consumer usage

Add the private channel after RoboStack and conda-forge in the workspace channel
list, authenticate once, and depend on the binary package:

```toml
channels = [
  "robostack-jazzy",
  "conda-forge",
  "https://prefix.dev/gabriel-robotics",
]

[dependencies]
ros-jazzy-manipulation-position-controllers = "==0.3.1"
```

The consuming workspace must remove the source checkout/submodule, otherwise
colcon will build the source package as an overlay and mask the binary package.

The recipe builds against conda-forge `libosqp` and `osqp-eigen`, then bundles
both shared libraries under package-private names. This is required because
MoveIt brings in `ros-jazzy-osqp-vendor`; that package and conda-forge
`libosqp` both install `lib/libosqp.so` but provide incompatible ABIs. The
controller's private RPATH makes its runtime independent of package-link order.
Because the binary redistributes copies of both libraries, their upstream
license and notice files are included in the conda artifact under
`info/licenses`; renaming their SONAMEs does not change their licensing terms.

## Release policy

- Increase `package.xml`, recipe, and repository tag versions together.
- Never overwrite an already consumed build. Increase `build.number` for a
  packaging-only rebuild of the same source version.
- Build separately for each supported platform and ROS/Python ABI combination.
- Validate installation in a fresh Pixi environment, including plugin discovery
  and controller loading, before upload.
