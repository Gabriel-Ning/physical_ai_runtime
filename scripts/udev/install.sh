#!/usr/bin/env bash
# Install all workspace udev rules from scripts/udev/rules.d/ into
# /etc/udev/rules.d/ (idempotent, explicit, privileged).
#
# Usage:
#   sudo bash scripts/udev/install.sh
#
# Ordinary build/launch never writes /etc/udev/rules.d — operators run this
# once per host (or after pulling new rules).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SRC_DIR="$ROOT/scripts/udev/rules.d"
DEST_DIR="/etc/udev/rules.d"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Need root. Run: sudo bash $0" >&2
  exit 1
fi

if [[ ! -d "$SRC_DIR" ]]; then
  echo "Missing ${SRC_DIR}" >&2
  exit 1
fi

shopt -s nullglob
rules=("$SRC_DIR"/*.rules)
if ((${#rules[@]} == 0)); then
  echo "No *.rules files in ${SRC_DIR}." >&2
  exit 1
fi

for src in "${rules[@]}"; do
  base="$(basename "$src")"
  install -m 0644 "$src" "${DEST_DIR}/${base}"
  echo "Installed ${DEST_DIR}/${base}"
done

udevadm control --reload-rules
echo "Reloaded udev rules."
echo "If a device name did not change yet, unplug/replug it or reboot."
