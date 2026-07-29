#!/usr/bin/env bash
# Set CPU frequency governor to performance (Franka / ros2_control RT hosts).
#
# Franka docs: Disabling CPU frequency scaling — powersave increases RT latency
# and can trigger communication_constraints_violation.
#
# Usage:
#   scripts/enable_cpu_performance_governor.sh           # apply now
#   scripts/enable_cpu_performance_governor.sh --install  # apply + enable at boot
#   scripts/enable_cpu_performance_governor.sh --status
set -euo pipefail

SERVICE_NAME="pai-cpu-performance-governor.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

usage() {
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
}

run_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif sudo -n true 2>/dev/null; then
    sudo "$@"
  elif [[ -t 0 ]]; then
    sudo "$@"
  else
    echo "Need root to change CPU governor (non-interactive sudo unavailable)." >&2
    echo "Run: sudo $0 $*" >&2
    return 1
  fi
}

current_governors() {
  if compgen -G '/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor' >/dev/null; then
    cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c
  else
    echo "(no cpufreq scaling_governor sysfs nodes)" >&2
    return 1
  fi
}

apply_now() {
  if command -v cpupower >/dev/null 2>&1; then
    run_root cpupower frequency-set -g performance >/dev/null
  else
    run_root bash -c '
      for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        printf performance > "$gov"
      done
    '
  fi
  echo "CPU governor set to performance:"
  current_governors
}

install_boot() {
  local unit
  unit=$(cat <<'EOF'
[Unit]
Description=Physical AI Runtime: set CPU frequency governor to performance
Documentation=https://frankaemika.github.io/docs/
DefaultDependencies=no
After=sysinit.target local-fs.target
Before=multi-user.target

[Service]
Type=oneshot
# Prefer cpupower when installed; fall back to sysfs (Franka-recommended performance governor).
ExecStart=/usr/bin/bash -c 'if command -v cpupower >/dev/null 2>&1; then cpupower frequency-set -g performance >/dev/null; else for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do printf performance > "$g"; done; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
)

  echo "${unit}" | run_root tee "${SERVICE_PATH}" >/dev/null
  run_root systemctl daemon-reload
  run_root systemctl enable --now "${SERVICE_NAME}"
  echo "Installed and enabled ${SERVICE_NAME} (applies performance governor at boot)."
}

status() {
  echo "Current governors:"
  current_governors || true
  if [[ -f "${SERVICE_PATH}" ]]; then
    systemctl is-enabled "${SERVICE_NAME}" 2>/dev/null || true
    systemctl --no-pager --full status "${SERVICE_NAME}" 2>/dev/null | head -n 12 || true
  else
    echo "Boot service not installed (${SERVICE_PATH} missing)."
  fi
}

ensure_boot() {
  # Idempotent path used by `pixi run -e cpu setup`.
  apply_now
  if [[ -f "${SERVICE_PATH}" ]] && systemctl is-enabled --quiet "${SERVICE_NAME}" 2>/dev/null; then
    echo "Boot service already enabled: ${SERVICE_NAME}"
    return 0
  fi
  install_boot
}

cmd="${1:-apply}"
case "${cmd}" in
  -h|--help)
    usage
    ;;
  --status|status)
    status
    ;;
  --install|install)
    apply_now
    install_boot
    ;;
  --ensure-boot|ensure-boot)
    ensure_boot
    ;;
  apply|"")
    apply_now
    ;;
  *)
    echo "Unknown argument: ${cmd}" >&2
    usage >&2
    exit 2
    ;;
esac
