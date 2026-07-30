#!/usr/bin/env bash
# CPU / realtime-kernel host setup for Physical AI Runtime.
#
# Called from `pixi run -e cpu setup`. Applies:
#   1. CPU frequency governor → performance (now + boot service)
#   2. Kernel CPU isolation from scripts/rt_cpu_profile.env (GRUB; reboot if needed)
#   3. Raise min frequency on isolated CPUs when isolation is already active
#
# Process affinity for ros2_control is NOT applied here — bringups read
# RT_CM_CPU_AFFINITY and prefix ros2_control_node with taskset.
#
# Exit codes:
#   0  host ready (or soft warnings only)
#   1  hard failure (e.g. missing profile)
#   3  GRUB updated / isolation pending — reboot required before real-robot RT
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/rt_cpu_profile.env"

expand_cpu_list() {
  # Normalize "14,15" / "14-15" / "12,13,14-15" → sorted unique comma list.
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
  local isolated expected affinity gov
  isolated="$(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || true)"
  expected="$(expand_cpu_list "$RT_ISOL_CPUS")"
  affinity="${RT_CM_CPU_AFFINITY}"
  gov="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
  echo "CPU RT host status:"
  echo "  profile:     ${RT_CPU_PROFILE_NAME}"
  echo "  governor:    ${gov}"
  echo "  isolated:    ${isolated:-"(none)"}"
  echo "  expected:    ${expected}"
  echo "  cm_affinity: ${affinity}"
  if [[ -f /sys/kernel/realtime ]]; then
    echo "  realtime:    $(cat /sys/kernel/realtime)"
  else
    echo "  realtime:    (no /sys/kernel/realtime — is this a PREEMPT_RT kernel?)"
  fi
}

echo "Configuring CPU RT host (profile=${RT_CPU_PROFILE_NAME})…"

# ── 1. Performance governor ────────────────────────────────────────────────
if ! bash "$ROOT/scripts/enable_cpu_performance_governor.sh" --ensure-boot; then
  echo "WARNING: could not enable CPU performance governor (sudo required)." >&2
  echo "  Run once: sudo bash scripts/enable_cpu_performance_governor.sh --install" >&2
fi

# ── 2. Kernel isolation (isolcpus / nohz_full / rcu_nocbs) ─────────────────
reboot_needed=0
expected="$(expand_cpu_list "$RT_ISOL_CPUS")"
active="$(expand_cpu_list "$(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || true)")"

if [[ -n "$active" && "$active" == "$expected" ]]; then
  echo "CPU isolation already active: ${active}"
  raise_isol_min_freq "$active"
elif [[ -n "$active" && "$active" != "$expected" ]]; then
  echo "WARNING: active isolcpus=${active} differs from profile expected=${expected}." >&2
  echo "  Edit scripts/rt_cpu_profile.env or GRUB manually; reboot if you change GRUB." >&2
  raise_isol_min_freq "$active"
else
  # Not active in this boot — ensure GRUB has the fragment, then require reboot.
  ensure_rc=0
  bash "$ROOT/scripts/apply_rt_isolcpus.sh" --ensure || ensure_rc=$?
  case "$ensure_rc" in
    0)
      # GRUB already correct but cmdline not active → reboot pending
      echo "GRUB isolation configured for ${expected}, but this boot has no matching isolcpus."
      reboot_needed=1
      ;;
    3)
      reboot_needed=1
      ;;
    *)
      echo "WARNING: could not ensure isolcpus in GRUB (sudo required?)." >&2
      echo "  Run once: sudo bash scripts/apply_rt_isolcpus.sh --apply && sudo reboot" >&2
      ;;
  esac
fi

print_status

if ((reboot_needed)); then
  echo
  echo "REBOOT REQUIRED for CPU isolation to take effect:"
  echo "  sudo reboot"
  echo "After reboot, re-run: pixi run -e cpu setup"
  echo "Then launch controllers without manual taskset (bringup reads RT_CM_CPU_AFFINITY)."
  exit 3
fi

echo "CPU RT host ready. Controller bringups pin ros2_control to: ${RT_CM_CPU_AFFINITY}"
exit 0
