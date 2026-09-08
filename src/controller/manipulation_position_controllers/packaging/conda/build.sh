#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build-conda -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_TESTING=OFF

cmake --build build-conda --parallel "${CPU_COUNT:-1}"
cmake --install build-conda

# The CMake build links the controller against package-private OSQP names.
private_lib_dir="${PREFIX}/lib/manipulation_position_controllers"
test -n "$(nm -D "${private_lib_dir}/libmpc_osqp.so" |
  grep ' OSQPCscMatrix_set_data$')"

controller_library="${PREFIX}/lib/libmanipulation_position_controllers.so"
test "$(patchelf --print-needed "${controller_library}" |
  grep -c '^libmpc_osqp_eigen\.so$')" -eq 1
test "$(patchelf --print-needed "${controller_library}" |
  grep -c '^libmpc_osqp\.so$')" -eq 1
! patchelf --print-needed "${controller_library}" |
  grep -Eq '^(libOsqpEigen\.so|libosqp\.so$)'
test "$(patchelf --print-needed "${private_lib_dir}/libmpc_osqp.so" |
  grep -c '^libmpc_qdldl\.so$')" -eq 1
! patchelf --print-needed "${private_lib_dir}/libmpc_osqp.so" |
  grep -Eq '^libqdldl\.so'

# Binary-only distribution: drop headers and CMake export targets.
rm -rf "${PREFIX}/include/manipulation_position_controllers"
rm -rf "${PREFIX}/share/manipulation_position_controllers/cmake"
