#!/usr/bin/env bash
# Pin Franka FCI NIC IRQs onto isolated IRQ CPUs, keep ROS NIC IRQs off isolcpus,
# and disable interrupt coalescing on the Franka NIC.
#
# Usage:
#   sudo scripts/apply_franka_rt_networking.sh              # apply now
#   sudo scripts/apply_franka_rt_networking.sh --install    # apply + enable boot service
#   scripts/apply_franka_rt_networking.sh --status
#   sudo scripts/apply_franka_rt_networking.sh --uninstall
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Prefer the Franka beta profile; fall back to the active host profile.
# shellcheck disable=SC1091
if [[ -f "$ROOT/scripts/rt_cpu_profile.franka_beta.env" ]]; then
  source "$ROOT/scripts/rt_cpu_profile.franka_beta.env"
else
  source "$ROOT/scripts/rt_cpu_profile.env"
fi

UNIT_PATH=/etc/systemd/system/pai-franka-rt-networking.service
WRAPPER=/usr/local/sbin/pai-franka-rt-networking

expand_cpu_list() {
  local spec="${1// /}" part a b i
  local -a out=()
  IFS=',' read -ra parts <<< "$spec"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    if [[ "$part" == *-* ]]; then
      a="${part%-*}"; b="${part#*-}"
      for ((i = a; i <= b; i++)); do out+=("$i"); done
    else
      out+=("$part")
    fi
  done
  ((${#out[@]} == 0)) && { echo ""; return 0; }
  printf '%s\n' "${out[@]}" | sort -n | uniq | paste -sd,
}

cpu_list_to_mask() {
  # decimal bitmask for smp_affinity (CPU0 = LSB)
  local list="$1" bit mask=0
  IFS=',' read -ra ids <<< "$(expand_cpu_list "$list")"
  for bit in "${ids[@]}"; do
    mask=$((mask | (1 << bit)))
  done
  printf '%x' "$mask"
}

iface_irqs() {
  local iface="$1"
  local irq
  if [[ -d "/sys/class/net/${iface}/device/msi_irqs" ]]; then
    for irq in "/sys/class/net/${iface}/device/msi_irqs"/*; do
      [[ -e "$irq" ]] || continue
      basename "$irq"
    done
    return 0
  fi
  # fallback: /proc/interrupts name match
  awk -v iface="$iface" '
    $0 ~ iface {
      gsub(/:/,"",$1);
      print $1
    }' /proc/interrupts
}

set_irq_affinity() {
  local iface="$1" cpus="$2" irq mask list
  list="$(expand_cpu_list "$cpus")"
  mask="$(cpu_list_to_mask "$cpus")"
  echo "  ${iface}: IRQs -> CPUs ${list} (mask 0x${mask})"
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "    skip IRQ affinity (need root)" >&2
    return 0
  fi
  # Do not use test -w on procfs — it lies even for root.
  for irq in $(iface_irqs "$iface"); do
    if echo "$list" >"/proc/irq/${irq}/smp_affinity_list" 2>/dev/null \
      || echo "$mask" >"/proc/irq/${irq}/smp_affinity" 2>/dev/null; then
      echo "    irq ${irq}: $(cat /proc/irq/${irq}/smp_affinity_list)"
    else
      echo "    irq ${irq}: write failed" >&2
    fi
  done
}

tune_coalesce() {
  local iface="$1"
  if ! command -v ethtool >/dev/null 2>&1; then
    echo "  ethtool missing — skip coalesce for ${iface}" >&2
    return 0
  fi
  # Best-effort: zero rx coalescing for FCI latency.
  ethtool -C "$iface" rx-usecs 0 tx-usecs 0 2>/dev/null \
    || ethtool -C "$iface" rx-usecs 0 2>/dev/null \
    || echo "  ${iface}: coalesce not fully supported (continuing)"
  echo "  ${iface}: coalesce:"
  ethtool -C "$iface" 2>/dev/null | grep -E 'rx-usecs:|tx-usecs:' || true
}

print_status() {
  echo "Franka RT networking status (profile=${RT_CPU_PROFILE_NAME})"
  echo "  isolated: $(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo none)"
  echo "  CM aff:   ${RT_CM_CPU_AFFINITY}"
  local iface irq
  for iface in "${RT_FRANKA_NIC}" "${RT_ROS_NIC}"; do
    [[ -e "/sys/class/net/${iface}" ]] || { echo "  ${iface}: DOWN/missing"; continue; }
    echo "  ${iface}:"
    for irq in $(iface_irqs "$iface"); do
      echo "    irq ${irq}: $(cat /proc/irq/${irq}/smp_affinity_list 2>/dev/null || echo '?')"
    done
  done
  if systemctl list-unit-files pai-franka-rt-networking.service >/dev/null 2>&1; then
    echo "  service: $(systemctl is-enabled pai-franka-rt-networking.service 2>/dev/null || echo n/a) / $(systemctl is-active pai-franka-rt-networking.service 2>/dev/null || echo n/a)"
  fi
}

apply_now() {
  local franka_nic="${RT_FRANKA_NIC:?}"
  local franka_cpus="${RT_FRANKA_IRQ_CPUS:?}"
  local ros_nic="${RT_ROS_NIC:?}"
  local ros_cpus="${RT_ROS_IRQ_CPUS:?}"

  if [[ "$(id -u)" -ne 0 ]]; then
    echo "Need root to write /proc/irq/*/smp_affinity_list" >&2
    exit 1
  fi

  echo "Applying Franka RT networking…"
  if [[ ! -e "/sys/class/net/${franka_nic}" ]]; then
    echo "Franka NIC ${franka_nic} missing" >&2
    exit 1
  fi
  set_irq_affinity "$franka_nic" "$franka_cpus"
  tune_coalesce "$franka_nic"

  if [[ -e "/sys/class/net/${ros_nic}" ]]; then
    set_irq_affinity "$ros_nic" "$ros_cpus"
  else
    echo "  ROS NIC ${ros_nic} missing — skip"
  fi

  # irqbalance fights manual affinity; keep it off on this RT host.
  if systemctl list-unit-files irqbalance.service >/dev/null 2>&1; then
    systemctl disable --now irqbalance.service 2>/dev/null || true
    echo "  irqbalance: disabled"
  fi
  echo "Done."
}

install_service() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "Need root for --install" >&2
    exit 1
  fi
  apply_now
  install -d /usr/local/sbin
  cat >"$WRAPPER" <<WRAP
#!/usr/bin/env bash
exec bash "$ROOT/scripts/apply_franka_rt_networking.sh"
WRAP
  chmod 755 "$WRAPPER"

  cat >"$UNIT_PATH" <<UNIT
[Unit]
Description=Physical AI Runtime Franka FCI NIC IRQ / coalesce tuning
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=root
ExecStart=$WRAPPER
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
  systemctl daemon-reload
  systemctl enable --now pai-franka-rt-networking.service
  echo "Installed and enabled $UNIT_PATH"
}

uninstall_service() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "Need root for --uninstall" >&2
    exit 1
  fi
  systemctl disable --now pai-franka-rt-networking.service 2>/dev/null || true
  rm -f "$UNIT_PATH" "$WRAPPER"
  systemctl daemon-reload
  echo "Removed Franka RT networking service"
}

case "${1:-}" in
  ""|apply|--apply)
    apply_now
    ;;
  --install|install)
    install_service
    ;;
  --status|status)
    print_status
    ;;
  --uninstall|uninstall)
    uninstall_service
    ;;
  -h|--help)
    sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unknown argument: $1" >&2
    exit 2
    ;;
esac
