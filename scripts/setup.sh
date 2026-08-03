#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_root}"

mkdir -p build install log/ros data .pixi

# Record which Pixi env ran setup so .envrc activates the same one
# (e.g. `pixi run -e cpu setup` → cpu; plain `pixi run setup` → default).
pixi_env="${PIXI_ENVIRONMENT_NAME:-default}"
printf '%s\n' "${pixi_env}" > .pixi/environment

if [[ "${pixi_env}" == "default" ]]; then
  if [[ -z "${CLOUDXR_DIR:-}" ]]; then
    echo "Missing CLOUDXR_DIR in the default GPU environment" >&2
    exit 1
  fi
  mkdir -p "${CLOUDXR_DIR}"
fi

# CPU / RT hosts: performance governor, isolcpus GRUB, affinity profile.
# See docs/CPU_HOST_SETUP.md. Exit 3 from the helper means reboot required.
cpu_rt_rc=0
if [[ "${pixi_env}" == "cpu" ]]; then
  bash "${workspace_root}/scripts/setup_cpu_rt_host.sh" || cpu_rt_rc=$?
  if [[ "${cpu_rt_rc}" -ne 0 && "${cpu_rt_rc}" -ne 3 ]]; then
    echo "WARNING: CPU RT host setup reported errors (rc=${cpu_rt_rc})." >&2
    echo "  See docs/CPU_HOST_SETUP.md" >&2
    cpu_rt_rc=0
  fi
fi

for command in python ros2 colcon vcs rosdep cmake ninja; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "Missing required command in Pixi environment: ${command}" >&2
    exit 1
  fi
done

echo "Physical AI Runtime development environment is ready."
echo "  workspace: ${WORKSPACE_ROOT}"
echo "  pixi env:  ${pixi_env}"
echo "  python:    $(command -v python)"
echo "  ros2:      $(command -v ros2)"
echo "  colcon:    $(command -v colcon)"
if [[ "${pixi_env}" == "default" ]]; then
  echo "  cloudxr:   ${CLOUDXR_DIR}"
fi
if [[ "${pixi_env}" == "cpu" ]]; then
  # shellcheck disable=SC1091
  source "${workspace_root}/scripts/rt_cpu_profile.env"
  echo "  cpu gov:   $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
  echo "  isolated:  $(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || echo none)"
  echo "  cm_aff:    ${RT_CM_CPU_AFFINITY}"
  echo "  ulimit -r: $(ulimit -r 2>/dev/null || echo unknown)"
fi

if [[ "${cpu_rt_rc}" -eq 3 ]]; then
  echo
  echo "Setup finished, but a reboot (or re-login) is required for RT host changes."
  echo "  sudo reboot"
  echo "After reboot: pixi run -e cpu setup && ulimit -r   # expect 99"
  exit 3
fi
