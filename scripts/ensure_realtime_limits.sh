#!/usr/bin/env bash
# Ensure PAM limits + group membership so ros2_control can take SCHED_FIFO.
#
# Without this, controller_manager logs:
#   Could not enable FIFO RT scheduling policy ... Operation not permitted
# and `ulimit -r` stays 0.
#
# Usage:
#   scripts/ensure_realtime_limits.sh --status
#   scripts/ensure_realtime_limits.sh --ensure   # idempotent (used by cpu setup)
#   sudo scripts/ensure_realtime_limits.sh --apply
#
# --ensure / --apply exit codes:
#   0  already configured on disk (group + limits file)
#   3  updated this run — re-login or reboot before ulimit -r becomes 99
#   1  failure
set -euo pipefail

LIMITS_FILE="/etc/security/limits.d/99-pai-realtime.conf"
GROUP_NAME="realtime"
TARGET_USER="${SUDO_USER:-${USER:-}}"

LIMITS_BODY=$(cat <<EOF
# Physical AI Runtime — ros2_control SCHED_FIFO / mlock
@${GROUP_NAME}   soft    rtprio     99
@${GROUP_NAME}   hard    rtprio     99
@${GROUP_NAME}   soft    memlock    unlimited
@${GROUP_NAME}   hard    memlock    unlimited
EOF
)

run_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif sudo -n true 2>/dev/null; then
    sudo "$@"
  elif [[ -t 0 ]]; then
    sudo "$@"
  else
    echo "Need root for realtime limits (non-interactive sudo unavailable)." >&2
    echo "Run: sudo $0 --apply" >&2
    return 1
  fi
}

print_status() {
  local in_group=no file=missing rtprio
  if getent group "$GROUP_NAME" >/dev/null 2>&1; then
    if id -nG "${TARGET_USER}" 2>/dev/null | tr ' ' '\n' | grep -qx "$GROUP_NAME"; then
      in_group=yes
    fi
  fi
  [[ -f "$LIMITS_FILE" ]] && file=present
  rtprio="$(ulimit -r 2>/dev/null || echo unknown)"
  echo "Realtime limits status:"
  echo "  user:       ${TARGET_USER:-"(unknown)"}"
  echo "  group:      ${GROUP_NAME} (member=${in_group})"
  echo "  limits file:${LIMITS_FILE} (${file})"
  echo "  ulimit -r:  ${rtprio}  (99 after re-login once configured)"
}

limits_file_ok() {
  [[ -f "$LIMITS_FILE" ]] || return 1
  grep -Eq "^@${GROUP_NAME}[[:space:]]+soft[[:space:]]+rtprio[[:space:]]+99[[:space:]]*$" "$LIMITS_FILE" \
    && grep -Eq "^@${GROUP_NAME}[[:space:]]+hard[[:space:]]+rtprio[[:space:]]+99[[:space:]]*$" "$LIMITS_FILE" \
    && grep -Eq "^@${GROUP_NAME}[[:space:]]+soft[[:space:]]+memlock[[:space:]]+unlimited[[:space:]]*$" "$LIMITS_FILE" \
    && grep -Eq "^@${GROUP_NAME}[[:space:]]+hard[[:space:]]+memlock[[:space:]]+unlimited[[:space:]]*$" "$LIMITS_FILE"
}

user_in_group() {
  [[ -n "$TARGET_USER" ]] || return 1
  getent group "$GROUP_NAME" >/dev/null 2>&1 || return 1
  id -nG "$TARGET_USER" 2>/dev/null | tr ' ' '\n' | grep -qx "$GROUP_NAME"
}

apply_or_ensure() {
  local changed=0

  if [[ -z "$TARGET_USER" || "$TARGET_USER" == "root" ]]; then
    echo "Refusing to configure realtime group for user '${TARGET_USER:-}'." >&2
    echo "Run as the robot operator account, or: sudo -u <user> …" >&2
    exit 1
  fi

  if ! getent group "$GROUP_NAME" >/dev/null 2>&1; then
    run_root groupadd -f "$GROUP_NAME"
    changed=1
    echo "Created group ${GROUP_NAME}."
  fi

  if ! user_in_group; then
    run_root usermod -aG "$GROUP_NAME" "$TARGET_USER"
    changed=1
    echo "Added ${TARGET_USER} to ${GROUP_NAME}."
  fi

  if ! limits_file_ok; then
    echo "$LIMITS_BODY" | run_root tee "$LIMITS_FILE" >/dev/null
    run_root chmod 644 "$LIMITS_FILE"
    changed=1
    echo "Wrote ${LIMITS_FILE}."
  fi

  print_status

  if ((changed)); then
    echo
    echo "Realtime limits updated. Re-login (or reboot) so PAM applies rtprio/memlock."
    echo "Then verify: ulimit -r   # expect 99"
    exit 3
  fi

  echo "Realtime limits already configured for ${TARGET_USER}."
  exit 0
}

cmd="${1:-}"
case "$cmd" in
  "" | --status | status)
    print_status
    ;;
  --ensure | ensure | --apply | apply)
    apply_or_ensure
    ;;
  -h | --help | help)
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unknown argument: $cmd" >&2
    exit 1
    ;;
esac
