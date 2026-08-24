#!/usr/bin/env bash
# Print / apply / ensure GRUB isolcpus from scripts/rt_cpu_profile.env.
# Does NOT reboot.
#
# Usage:
#   scripts/apply_rt_isolcpus.sh
#   sudo scripts/apply_rt_isolcpus.sh --apply --replace
#   scripts/apply_rt_isolcpus.sh --ensure
#
# Exit: 0 ok, 3 GRUB updated (reboot), 1 failure
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/rt_cpu_profile.env"

CPUS="$RT_ISOL_CPUS"
FRAGMENT="isolcpus=${CPUS} nohz_full=${CPUS} rcu_nocbs=${CPUS}"
REPLACE=0
CMD=""

for arg in "$@"; do
  case "$arg" in
    --replace|replace) REPLACE=1 ;;
    --apply|apply|--ensure|ensure|--dry-run|dry-run|-h|--help) CMD="$arg" ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

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

grub_has_profile_fragment() {
  local grub="/etc/default/grub"
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
  local active
  active="$(expand_cpu_list "$(tr -d '[:space:]' </sys/devices/system/cpu/isolated 2>/dev/null || true)")"
  echo "Active isolcpus (this boot): ${active:-"(none)"}"
  echo "Expected isolcpus:           $(expand_cpu_list "$CPUS")"
  if [[ -n "${RT_FRANKA_NIC:-}" ]]; then
    echo "Franka NIC IRQ plan: ${RT_FRANKA_NIC} -> CPUs ${RT_FRANKA_IRQ_CPUS:-?}"
  fi
  if [[ -n "${RT_ROS_NIC:-}" ]]; then
    echo "ROS NIC IRQ plan:    ${RT_ROS_NIC} -> CPUs ${RT_ROS_IRQ_CPUS:-?}"
  fi
}

apply_grub() {
  if [[ ! -w /etc/default/grub ]]; then
    echo "Need write access to /etc/default/grub (use sudo)." >&2
    exit 1
  fi
  if grub_has_profile_fragment; then
    echo "/etc/default/grub already has isolation fragment for ${CPUS}."
    exit 0
  fi
  if grep -q 'isolcpus=' /etc/default/grub && [[ "$REPLACE" -ne 1 ]]; then
    echo "/etc/default/grub already contains isolcpus= but not the profile fragment:" >&2
    echo "  want: $FRAGMENT" >&2
    echo "Re-run with --replace to rewrite." >&2
    exit 1
  fi

  FRAGMENT="$FRAGMENT" CPUS="$CPUS" python3 - <<'PY'
from pathlib import Path
import os, re
grub = Path("/etc/default/grub")
text = grub.read_text(encoding="utf-8")
fragment = os.environ["FRAGMENT"]
cpus = os.environ["CPUS"]

def strip_tokens(s: str) -> str:
    s = re.sub(r"\s*isolcpus=\S+", "", s)
    s = re.sub(r"\s*nohz_full=\S+", "", s)
    s = re.sub(r"\s*rcu_nocbs=\S+", "", s)
    return re.sub(r"\s+", " ", s).strip()

out = []
found = False
for line in text.splitlines(keepends=True):
    nl = "\n" if line.endswith("\n") else ""
    raw = line[:-1] if nl else line
    m = re.match(r'^(GRUB_CMDLINE_LINUX_DEFAULT=")(.*)("\s*)$', raw)
    if not m:
        out.append(line)
        continue
    body = strip_tokens(m.group(2))
    body = (body + " " + fragment).strip() if body else fragment
    out.append(f'{m.group(1)}{body}"{nl}')
    found = True
if not found:
    raise SystemExit("GRUB_CMDLINE_LINUX_DEFAULT not found")
grub.write_text("".join(out), encoding="utf-8")
print(f"Patched GRUB with isolcpus={cpus}")
PY

  update-grub
  echo "Updated GRUB with: ${FRAGMENT}"
  echo "Reboot required: sudo reboot"
  exit 3
}

case "${CMD:-}" in
  ""|--dry-run|dry-run)
    print_dry_run
    echo "Dry run only. To write GRUB:"
    echo "  sudo $0 --apply --replace"
    ;;
  --apply|apply)
    print_dry_run
    apply_grub
    ;;
  --ensure|ensure)
    if grub_has_profile_fragment; then
      echo "GRUB isolation fragment OK for ${CPUS}."
      exit 0
    fi
    REPLACE=1
    apply_grub
    ;;
  -h|--help)
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unknown argument: ${CMD}" >&2
    exit 2
    ;;
esac
