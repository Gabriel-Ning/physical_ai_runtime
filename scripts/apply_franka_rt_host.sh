#!/usr/bin/env bash
# Franka RT host (beta): activate Franka CPU profile, rewrite GRUB isolcpus,
# and install Franka NIC IRQ / coalesce service. Run ONCE with sudo, then reboot.
#
#   sudo bash scripts/apply_franka_rt_host.sh
#
# See docs/FRANKA_RT_COMMUNICATION.md.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run as: sudo bash $0" >&2
  exit 1
fi

FRANKA_PROFILE="$ROOT/scripts/rt_cpu_profile.franka_beta.env"
ACTIVE_PROFILE="$ROOT/scripts/rt_cpu_profile.env"
if [[ ! -f "$FRANKA_PROFILE" ]]; then
  echo "Missing $FRANKA_PROFILE" >&2
  exit 1
fi

echo "=== 0/3 Activate Franka CPU profile ==="
if [[ -f "$ACTIVE_PROFILE" ]] && ! cmp -s "$FRANKA_PROFILE" "$ACTIVE_PROFILE"; then
  cp -a "$ACTIVE_PROFILE" "${ACTIVE_PROFILE}.bak.$(date +%Y%m%d%H%M%S)"
fi
cp -a "$FRANKA_PROFILE" "$ACTIVE_PROFILE"
echo "Installed $ACTIVE_PROFILE from rt_cpu_profile.franka_beta.env"

echo "=== 1/3 GRUB isolcpus (profile) ==="
set +e
bash scripts/apply_rt_isolcpus.sh --apply --replace
rc=$?
set -e
if [[ "$rc" -ne 0 && "$rc" -ne 3 ]]; then
  echo "GRUB apply failed (exit $rc)" >&2
  exit "$rc"
fi

echo "=== 2/3 Franka NIC IRQ / coalesce (boot service) ==="
bash scripts/apply_franka_rt_networking.sh --install

echo "=== 3/3 current status ==="
bash scripts/apply_rt_isolcpus.sh --dry-run || true
bash scripts/apply_franka_rt_networking.sh --status || true

echo
echo "NEXT: sudo reboot"
echo "After reboot verify:"
echo "  cat /proc/cmdline | tr ' ' '\n' | grep -E 'isolcpus|nohz_full|rcu_nocbs'"
echo "  cat /sys/devices/system/cpu/isolated   # expect 12-15"
echo "  bash scripts/apply_franka_rt_networking.sh --status"
echo "  # then start rt_stack; ros2_control should taskset to 14,15"
