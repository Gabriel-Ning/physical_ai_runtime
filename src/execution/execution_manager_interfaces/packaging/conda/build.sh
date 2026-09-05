#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build-conda -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="${PYTHON}"

cmake --build build-conda --parallel "${CPU_COUNT:-1}"
cmake --install build-conda
