#!/usr/bin/env bash
# Push this machine's wall clock onto a remote SSH host.
#
# Cross-host ROS demos (marker / leader → remote EM) reject stamps when skew
# exceeds EM max_future_s (often 0.1 s). Typical symptom:
#   future pose stamp rejected: NNN ms ahead
#
# Usage (run on the workstation / AA):
#   scripts/sync_remote_clock.sh --status [user@host]
#   scripts/sync_remote_clock.sh --apply  [user@host]   # needs remote sudo
#
# Default host: delta@192.168.1.101
#
# --apply:
#   1) interactive `sudo -v` on the remote (enter password once)
#   2) immediately set remote UTC from *this* machine (avoids stale time
#      while typing the password)
#   3) leaves remote NTP disabled so broken fake-ip chrony sources do not
#      pull the clock away again
set -euo pipefail

DEFAULT_HOST="delta@192.168.1.101"
MODE=""
HOST=""

usage() {
  sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --status|--apply) MODE="${1#--}"; shift ;;
    -h|--help) usage 0 ;;
    -*)
      echo "Unknown option: $1" >&2
      usage 1
      ;;
    *)
      if [[ -n "$HOST" ]]; then
        echo "Unexpected argument: $1" >&2
        usage 1
      fi
      HOST="$1"
      shift
      ;;
  esac
done

HOST="${HOST:-$DEFAULT_HOST}"
if [[ -z "$MODE" ]]; then
  echo "Choose --status or --apply." >&2
  usage 1
fi

ssh_base=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST")
ssh_tty=(ssh -tt -o ConnectTimeout=15 "$HOST")

measure_skew_ms() {
  # Positive => remote ahead of local. Corrects for half RTT.
  python3 - "$HOST" <<'PY'
import subprocess, sys, time
host = sys.argv[1]
samples = []
for _ in range(5):
    t0 = time.time()
    out = subprocess.check_output(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, "date -u +%s.%N"],
        text=True,
    ).strip()
    t1 = time.time()
    remote = float(out)
    mid = 0.5 * (t0 + t1)
    samples.append(((remote - mid) * 1000.0, (t1 - t0) * 1000.0))
samples.sort(key=lambda x: x[0])
skew = samples[len(samples) // 2][0]
rtt = sum(s[1] for s in samples) / len(samples)
print(f"{skew:.1f} {rtt:.1f}")
PY
}

print_status() {
  local skew rtt
  read -r skew rtt < <(measure_skew_ms)
  echo "host:              $HOST"
  echo "local UTC:         $(date -u +'%Y-%m-%d %H:%M:%S.%N %Z')"
  echo "remote UTC:        $("${ssh_base[@]}" "date -u +'%Y-%m-%d %H:%M:%S.%N %Z'")"
  echo "skew (remote-local, median): ${skew} ms"
  echo "ssh RTT (mean):    ${rtt} ms"
  if awk -v s="$skew" 'BEGIN { exit (s < 0 ? -s : s) > 50 }'; then
    echo "verdict:           OK (|skew| <= 50 ms)"
  else
    echo "verdict:           HIGH — EM may reject cross-host stamps (max_future_s often 0.1 s)"
  fi
  "${ssh_base[@]}" 'timedatectl 2>/dev/null | sed -n "1,8p"' || true
}

apply_remote() {
  local local_utc skew_before skew_after rtt
  read -r skew_before rtt < <(measure_skew_ms)
  echo "Before: skew ${skew_before} ms (RTT ${rtt} ms)"

  echo
  echo "Step 1/2: cache sudo on $HOST (enter password, then this returns)."
  "${ssh_tty[@]}" 'sudo -v && echo SUDO_CACHED_OK'

  # Capture time only AFTER sudo is cached so typing delay does not stale it.
  local_utc="$(date -u +'%Y-%m-%d %H:%M:%S')"
  echo
  echo "Step 2/2: set $HOST UTC to ${local_utc} (should not re-prompt)."
  "${ssh_tty[@]}" "sudo timedatectl set-ntp false \
    && sudo date -u -s '${local_utc}' \
    && (command -v hwclock >/dev/null && sudo hwclock -w || true) \
    && echo REMOTE_CLOCK_SET_OK \
    && date -u \
    && timedatectl | sed -n '1,8p'"

  sleep 0.3
  read -r skew_after rtt < <(measure_skew_ms)
  echo
  echo "After:  skew ${skew_after} ms (RTT ${rtt} ms)"
  if awk -v s="$skew_after" 'BEGIN { exit (s < 0 ? -s : s) > 50 }'; then
    echo "verdict: OK"
  else
    echo "verdict: still HIGH — re-run --apply quickly after sudo -v, or check for another clock setter"
  fi
  echo "Remote NTP left disabled (delta chrony sources were fake-ip / unreachable)."
}

case "$MODE" in
  status) print_status ;;
  apply) apply_remote ;;
esac
