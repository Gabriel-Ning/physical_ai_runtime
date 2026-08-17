#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build-conda -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_TESTING=OFF

cmake --build build-conda --parallel "${CPU_COUNT:-1}"
cmake --install build-conda

# Move the exact OSQP ABI used at build time behind package-private names.
# MoveIt installs ros-jazzy-osqp-vendor, whose older libosqp.so otherwise
# collides with the conda-forge libosqp required by OsqpEigen.
private_lib_dir="${PREFIX}/lib/manipulation_position_controllers"
mkdir -p "${private_lib_dir}"
cp -L "${PREFIX}/lib/libosqp.so" "${private_lib_dir}/libmpc_osqp.so"
cp -L "${PREFIX}/lib/libOsqpEigen.so" "${private_lib_dir}/libmpc_osqp_eigen.so"

# Fail during packaging if the solver pair is already ABI-incompatible.
test -n "$(nm -D "${private_lib_dir}/libmpc_osqp.so" |
  grep ' OSQPCscMatrix_set_data$')"

patchelf --set-soname libmpc_osqp.so \
  "${private_lib_dir}/libmpc_osqp.so"
patchelf --replace-needed libosqp.so libmpc_osqp.so \
  "${private_lib_dir}/libmpc_osqp_eigen.so"
patchelf --set-soname libmpc_osqp_eigen.so \
  "${private_lib_dir}/libmpc_osqp_eigen.so"
patchelf --set-rpath '$ORIGIN' \
  "${private_lib_dir}/libmpc_osqp_eigen.so"

controller_library="${PREFIX}/lib/libmanipulation_position_controllers.so"
osqp_eigen_needed="$(
  patchelf --print-needed "${controller_library}" |
    sed -n '/^libOsqpEigen\.so/p'
)"
test -n "${osqp_eigen_needed}"
test "$(printf '%s\n' "${osqp_eigen_needed}" | wc -l)" -eq 1
patchelf --replace-needed "${osqp_eigen_needed}" libmpc_osqp_eigen.so \
  "${controller_library}"
patchelf --replace-needed libosqp.so libmpc_osqp.so \
  "${controller_library}"
patchelf --set-rpath '$ORIGIN/manipulation_position_controllers:$ORIGIN' \
  "${controller_library}"

test "$(patchelf --print-needed "${controller_library}" |
  grep -c '^libmpc_osqp_eigen\.so$')" -eq 1
! patchelf --print-needed "${controller_library}" |
  grep -q '^libOsqpEigen\.so'

# Binary-only distribution: drop headers and CMake export targets.
rm -rf "${PREFIX}/include/manipulation_position_controllers"
rm -rf "${PREFIX}/share/manipulation_position_controllers/cmake"
