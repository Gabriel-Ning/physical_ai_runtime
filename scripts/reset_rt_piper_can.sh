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
  for iface in piper0 piper1; do
    if ip link show "$iface" >/dev/null 2>&1; then
      interfaces+=("$iface")
    fi
  done

  if [[ ${#interfaces[@]} -eq 0 ]]; then
    for iface in can0 can1; do
      if ip link show "$iface" >/dev/null 2>&1; then
        interfaces+=("$iface")
      fi
    done
  fi

  if [[ ${#interfaces[@]} -eq 0 ]]; then
    echo "Error: no Piper CAN interface was found (checked piper0, piper1, can0, can1)."
    echo "Connect the USB-CAN adapter or pass its interface name explicitly."
    exit 1
  fi

  echo "Auto-detected CAN interface(s): ${interfaces[*]}"
fi

bitrate=1000000
txqueuelen=10

for iface in "${interfaces[@]}"; do
  echo "Bringing up CAN interface: $iface"

  if ! ip link show "$iface" >/dev/null 2>&1; then
    echo "Error: interface '$iface' was not found."
    exit 1
  fi

  ip link set "$iface" down || true
  ip link set "$iface" type can bitrate "$bitrate"
  ip link set "$iface" txqueuelen "$txqueuelen"
  ip link set "$iface" up

  echo "OK: $iface is up"
done
