#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "This script needs sudo privileges. Re-running with sudo..."
  exec sudo "$0" "$@"
fi

if [[ $# -gt 0 ]]; then
  interfaces=("$@")
else
  interfaces=()
  for iface in can0 can1; do
    if ip link show "$iface" >/dev/null 2>&1; then
      interfaces+=("$iface")
    fi
  done

  if [[ ${#interfaces[@]} -eq 0 ]]; then
    echo "Error: no Piper teleop CAN interface found (checked can0, can1)."
    echo "Please connect the USB-CAN adapter(s) or pass interface names explicitly."
    exit 1
  fi

  echo "Auto-detected teleop CAN interface(s): ${interfaces[*]}"
fi

bitrate=1000000
txqueuelen=10

for iface in "${interfaces[@]}"; do
  echo "Resetting Piper teleop CAN interface: $iface"

  if ! ip link show "$iface" >/dev/null 2>&1; then
    echo "Error: interface '$iface' was not found."
    exit 1
  fi

  ip link set "$iface" down || true
  ip link set "$iface" type can bitrate "$bitrate"
  ip link set "$iface" txqueuelen "$txqueuelen"
  ip link set "$iface" up

  echo "OK: $iface is configured (bitrate=${bitrate}, txqueuelen=${txqueuelen}) and UP."
done
