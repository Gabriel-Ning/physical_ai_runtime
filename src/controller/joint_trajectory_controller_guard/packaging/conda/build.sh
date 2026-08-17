#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build-conda -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_TESTING=OFF

cmake --build build-conda --parallel "${CPU_COUNT:-1}"
cmake --install build-conda

# Runtime-only distribution: the executable does not expose a C++ SDK.
rm -rf "${PREFIX}/include/joint_trajectory_controller_guard"
