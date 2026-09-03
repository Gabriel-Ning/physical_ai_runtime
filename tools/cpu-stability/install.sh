#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SERVICE_NAME=cpu-stability.service
GUARD_PATH=/usr/local/sbin/cpu-stability-guard
UNIT_PATH=/etc/systemd/system/$SERVICE_NAME

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  printf 'Run this installer with sudo.\n' >&2
  exit 1
fi

if [[ ${1:-} == --uninstall ]]; then
  systemctl disable --now "$SERVICE_NAME" 2>/dev/null || true
  if [[ -x "$GUARD_PATH" ]]; then
    "$GUARD_PATH" rollback
  fi
  rm -f -- "$UNIT_PATH" "$GUARD_PATH"
  systemctl daemon-reload
  printf 'CPU stability guard removed and balanced defaults restored.\n'
  exit 0
fi

install -D -m 0755 "$SCRIPT_DIR/cpu-stability-guard" "$GUARD_PATH"
install -D -m 0644 "$SCRIPT_DIR/cpu-stability.service" "$UNIT_PATH"
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"
systemctl --no-pager --full status "$SERVICE_NAME"

printf '\nInstalled. To remove and restore defaults:\n'
printf '  sudo %q --uninstall\n' "$SCRIPT_DIR/install.sh"
