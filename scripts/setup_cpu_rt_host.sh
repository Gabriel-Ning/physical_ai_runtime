#!/usr/bin/env bash
# One RT-host setup for all robots. Isolation numbers live in the env file.
#
#   pixi run setup-rt piper
#   pixi run setup-rt marvin
#   pixi run setup-rt franka
#
# Applies governor, PAM rtprio/memlock, GRUB isolcpus from
# scripts/rt_cpu_profile.<robot>.env. Franka also installs FCI NIC IRQ tuning
# when RT_FRANKA_NIC is set.
#
# Exit: 0 ready, 1 error, 3 reboot/re-login required.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROBOT="${1:-}"

usage() {
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
  echo "Robots: piper | marvin | franka"
}

case "$ROBOT" in
  piper|marvin|franka) ;;
  -h|--help|"")
    usage
    [[ -n "$ROBOT" ]] || exit 2
    exit 0
    ;;
  *)
    echo "Unknown robot '$ROBOT'. Use: piper | marvin | franka" >&2
    exit 2
    ;;
esac

PROFILE="$ROOT/scripts/rt_cpu_profile.${ROBOT}.env"
if [[ ! -f "$PROFILE" ]]; then
  echo "Missing $PROFILE" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$PROFILE"
export RT_CPU_PROFILE_FILE="$PROFILE"

_record_selected_profile() {
  local dest_user dest_group
  mkdir -p "$ROOT/.pixi"
  printf '%s\n' enabled >"$ROOT/.pixi/rt-profile-enabled"
  printf '%s\n' "$ROBOT" >"$ROOT/.pixi/rt-profile"
  dest_user="${SUDO_USER:-$USER}"
  dest_group="$(id -gn "$dest_user" 2>/dev/null || true)"
  if [[ -n "$dest_user" && -n "$dest_group" ]]; then
    chown "$dest_user:$dest_group" \
      "$ROOT/.pixi/rt-profile-enabled" "$ROOT/.pixi/rt-profile" 2>/dev/null || true
  fi
}

expand_cpu_list() {
  local spec="${1// /}" part a b i
  local -a out=()
  IFS=',' read -ra parts <<< "$spec"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    if [[ "$part" == *-* ]]; then
      a="${part%-*}"
      b="${part#*-}"
      for ((i = a; i <= b; i++)); do
        out+=("$i")
      done
    else
      out+=("$part")
    fi
  done
  if ((${#out[@]} == 0)); then
    echo ""
    return 0
  fi
  printf '%s\n' "${out[@]}" | sort -n | uniq | paste -sd,
}

raise_isol_min_freq() {
  local cpus maxf c govpath minpath
  cpus="$(expand_cpu_list "${1:-}")"
  [[ -z "$cpus" ]] && return 0
  IFS=',' read -ra ids <<< "$cpus"
  for c in "${ids[@]}"; do
    govpath="/sys/devices/system/cpu/cpu${c}/cpufreq/scaling_governor"
    minpath="/sys/devices/system/cpu/cpu${c}/cpufreq/scaling_min_freq"
    maxf="/sys/devices/system/cpu/cpu${c}/cpufreq/scaling_max_freq"
    [[ -f "$govpath" && -f "$minpath" && -f "$maxf" ]] || continue
    if [[ -w "$govpath" ]]; then
      printf performance >"$govpath" || true
      cat "$maxf" >"$minpath" || true
    else
      sudo bash -c "printf performance >'$govpath'; cat '$maxf' >'$minpath'" 2>/dev/null || true
    fi
  done
}

print_status() {
  local isolated expected affinity gov rtprio
  isolated="$(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || true)"
  expected="$(expand_cpu_list "$RT_ISOL_CPUS")"
  affinity="${RT_CM_CPU_AFFINITY}"
  gov="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
  rtprio="$(ulimit -r 2>/dev/null || echo unknown)"
  echo "CPU RT host status:"
  echo "  robot:       ${ROBOT}"
  echo "  profile:     ${RT_CPU_PROFILE_NAME}"
  echo "  governor:    ${gov}"
  echo "  isolated:    ${isolated:-"(none)"}"
  echo "  expected:    ${expected}"
  echo "  cm_affinity: ${affinity}"
  echo "  ulimit -r:   ${rtprio}"
  if [[ -f /sys/kernel/realtime ]]; then
    echo "  realtime:    $(cat /sys/kernel/realtime)"
  else
    echo "  realtime:    (no /sys/kernel/realtime — is this a PREEMPT_RT kernel?)"
  fi
}

_record_selected_profile
echo "Configuring CPU RT host (robot=${ROBOT} profile=${RT_CPU_PROFILE_NAME})…"

if ! bash "$ROOT/scripts/enable_cpu_performance_governor.sh" --ensure-boot; then
  echo "WARNING: could not enable CPU performance governor (sudo required)." >&2
  echo "  Run once: sudo bash scripts/enable_cpu_performance_governor.sh --install" >&2
fi

reboot_needed=0
limits_rc=0
bash "$ROOT/scripts/ensure_realtime_limits.sh" --ensure || limits_rc=$?
case "$limits_rc" in
  0) ;;
  3) reboot_needed=1 ;;
  *)
    echo "WARNING: could not ensure realtime limits (sudo required?)." >&2
    echo "  Run once: sudo bash scripts/ensure_realtime_limits.sh --apply" >&2
    echo "  Then re-login and verify: ulimit -r" >&2
    ;;
esac

expected="$(expand_cpu_list "$RT_ISOL_CPUS")"
active="$(expand_cpu_list "$(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || true)")"

if [[ -n "$active" && "$active" == "$expected" ]]; then
  echo "CPU isolation already active: ${active}"
  raise_isol_min_freq "$active"
else
  echo "Applying GRUB isolcpus=${RT_ISOL_CPUS} (replace any previous set)."
  ensure_rc=0
  bash "$ROOT/scripts/apply_rt_isolcpus.sh" --apply --replace || ensure_rc=$?
  case "$ensure_rc" in
    0)
      if [[ -z "$active" || "$active" != "$expected" ]]; then
        echo "GRUB isolation configured for ${expected}, but this boot has no matching isolcpus."
        reboot_needed=1
      fi
      ;;
    3) reboot_needed=1 ;;
    *)
      echo "WARNING: could not write isolcpus in GRUB (sudo required?)." >&2
      echo "  Run once: sudo bash scripts/setup_cpu_rt_host.sh ${ROBOT}" >&2
      ;;
  esac
fi

if [[ -n "${RT_FRANKA_NIC:-}" ]]; then
  echo "Applying Franka FCI NIC IRQ / coalesce (${RT_FRANKA_NIC})…"
  if [[ "$(id -u)" -eq 0 ]]; then
    bash "$ROOT/scripts/apply_franka_rt_networking.sh" --install
  elif sudo -n true 2>/dev/null; then
    sudo bash "$ROOT/scripts/apply_franka_rt_networking.sh" --install
  else
    echo "WARNING: Franka NIC IRQ setup needs root." >&2
    echo "  sudo bash scripts/setup_cpu_rt_host.sh franka" >&2
  fi
fi

print_status

if ((reboot_needed)); then
  echo
  echo "REBOOT (or re-login) REQUIRED for RT host changes to take effect."
  echo "  sudo reboot"
  echo "After reboot: pixi run setup-rt ${ROBOT} && ulimit -r   # expect 99"
  exit 3
fi

echo "CPU RT host ready. Controller bringups pin ros2_control to: ${RT_CM_CPU_AFFINITY}"
exit 0
