# Prefix.dev binary package

The recipe builds the ROS 2 Jazzy runtime package for the
`gabriel-robotics` Prefix.dev channel. It deliberately ships only runtime
artifacts. C++ sources, public headers, tests, and CMake development exports
are excluded.

Build from the package root with `rattler-build` available:

```bash
rattler-build build --recipe packaging/conda/recipe.yaml \
  -c https://prefix.dev/gabriel-robotics \
  -c robostack-jazzy \
  -c conda-forge
```

Inspect the resulting `.conda` archive before upload and confirm that the
recipe's tests pass. Uploading is intentionally a separate,
credentialed release step:

```bash
rattler-build upload prefix -c gabriel-robotics output/linux-64/ros-jazzy-execution-manager-*.conda
```
