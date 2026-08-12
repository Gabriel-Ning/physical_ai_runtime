#!/usr/bin/env bash
# Print / apply / ensure GRUB isolcpus from scripts/rt_cpu_profile.env.
# Does NOT reboot. After GRUB changes, reboot, then verify:
#   grep -E 'isolcpus|nohz_full' /proc/cmdline
#   cat /sys/devices/system/cpu/isolated
#
# Usage:
#   scripts/apply_rt_isolcpus.sh              # dry-run (print fragment)
#   sudo scripts/apply_rt_isolcpus.sh --apply # write/update GRUB isolcpus=
#   scripts/apply_rt_isolcpus.sh --ensure     # idempotent for setup_cpu_rt_host.sh
#
# --ensure / --apply exit codes:
#   0  GRUB already contains the profile fragment (caller checks /proc)
#   3  GRUB was updated this run — reboot required
#   1  failure
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/rt_cpu_profile.env"

CPUS="$RT_ISOL_CPUS"
FRAGMENT="isolcpus=${CPUS} nohz_full=${CPUS} rcu_nocbs=${CPUS}"
GRUB_FILE="/etc/default/grub"

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

grub_has_profile_fragment() {
  local grub="$GRUB_FILE"
  [[ -f "$grub" ]] || return 1
  grep -Eq "isolcpus=${CPUS}([[:space:]\"]|\$)" "$grub" \
    && grep -Eq "nohz_full=${CPUS}([[:space:]\"]|\$)" "$grub" \
    && grep -Eq "rcu_nocbs=${CPUS}([[:space:]\"]|\$)" "$grub"
}

print_dry_run() {
  echo "Profile: $RT_CPU_PROFILE_NAME"
  echo "Suggested GRUB fragment:"
  echo "  $FRAGMENT"
  echo
  echo "CM affinity (runtime): RT_CM_CPU_AFFINITY=$RT_CM_CPU_AFFINITY"
  echo "  Controller bringups prefix ros2_control_node with taskset -c \$RT_CM_CPU_AFFINITY"
  echo
  local active
  active="$(expand_cpu_list "$(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || true)")"
  echo "Active isolcpus (this boot): ${active:-"(none)"}"
  echo "Expected isolcpus:           $(expand_cpu_list "$CPUS")"
}

# Rewrite GRUB_CMDLINE_LINUX_DEFAULT: drop prior isolcpus/nohz_full/rcu_nocbs, append FRAGMENT.
write_grub_fragment() {
  local grub="$GRUB_FILE"
  local content
  if ! grep -q '^GRUB_CMDLINE_LINUX_DEFAULT="' "$grub"; then
    echo "Expected GRUB_CMDLINE_LINUX_DEFAULT=\"...\" in $grub" >&2
    return 1
  fi
  content="$(sed -n 's/^GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/\1/p' "$grub" | head -n1)"
  content="$(printf '%s\n' "$content" | sed -E 's/[[:space:]]*(isolcpus|nohz_full|rcu_nocbs)=[^[:space:]]*//g')"
  content="$(printf '%s\n' "$content" | sed -E 's/[[:space:]]+/ /g; s/^ //; s/ $//')"
  if [[ -n "$content" ]]; then
    content="${content} ${FRAGMENT}"
  else
    content="${FRAGMENT}"
  fi
  sed -i -E "s|^GRUB_CMDLINE_LINUX_DEFAULT=\".*\"|GRUB_CMDLINE_LINUX_DEFAULT=\"${content}\"|" "$grub"
  if ! grub_has_profile_fragment; then
    echo "Failed to write isolation fragment into $grub" >&2
    return 1
  fi
  update-grub
  echo "Updated GRUB with: ${FRAGMENT}"
  echo "Reboot required: sudo reboot"
  return 0
}

apply_grub() {
  # Exit codes: 0 already OK, 3 updated (reboot), 1 error.
  if [[ ! -e "$GRUB_FILE" ]]; then
    echo "Missing $GRUB_FILE" >&2
    exit 1
  fi

  if grub_has_profile_fragment; then
    echo "/etc/default/grub already has isolation fragment for ${CPUS}."
    exit 0
  fi

  if [[ ! -w "$GRUB_FILE" ]]; then
    if [[ "$(id -u)" -eq 0 ]]; then
      echo "Need write access to $GRUB_FILE." >&2
      exit 1
    fi
    echo "Elevating privileges to update GRUB isolation…"
    exec sudo bash "$ROOT/scripts/apply_rt_isolcpus.sh" --apply-root
  fi

  if write_grub_fragment; then
    exit 3
  fi
  exit 1
}

cmd="${1:-}"
case "$cmd" in
  "" | --dry-run | dry-run)
    print_dry_run
    echo "Dry run only. To write GRUB_CMDLINE_LINUX extras, re-run:"
    echo "  sudo $0 --apply"
    echo "Then reboot and check: grep -E 'isolcpus|nohz_full' /proc/cmdline"
    ;;
  --apply | apply)
    print_dry_run
    apply_grub
    ;;
  --apply-root)
    # Internal: already elevated (or writable); no dry-run banner.
    apply_grub
    ;;
  --ensure | ensure)
    if grub_has_profile_fragment; then
      echo "GRUB isolation fragment OK for ${CPUS}."
      exit 0
    fi
    apply_grub
    ;;
  -h | --help)
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unknown argument: $cmd" >&2
    exit 2
    ;;
esac
