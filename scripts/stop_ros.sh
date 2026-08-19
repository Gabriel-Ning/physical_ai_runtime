#!/usr/bin/env bash
# Stop leftover ROS 2 processes from this workspace that weren't cleanly
# Ctrl-C'd (launch parents, spawners, CM, RViz, JSP GUI, ...).
#
# Optionally also clears orphan publishers on the RT host (beta) when SSH
# works — those often survive a partial kill and steal /joint_states on
# ROS_DOMAIN_ID=1, which makes local joint_state_publisher_gui appear stuck.
#
#   pixi run stop                         # local only (default)
#   STOP_ROS_REMOTE=1 pixi run stop       # also clear beta via SSH
#
# Safe with nothing running.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_HOST="${STOP_ROS_REMOTE_HOST:-beta@192.168.1.100}"
DO_REMOTE="${STOP_ROS_REMOTE:-0}"

patterns=(
  "ros2 launch"
  "rt_stack.launch.py"
  "visualize_franka.launch.py"
  "visualize_piper.launch.py"
  "controller_bringup.launch.py"
  "piper_leader.launch.py"
  "piper_leader_node"
  "piper_leader"
  "piper_leader_teleop"
  "franka_manipulation_controller_bringup"
  "piper_manipulation_controller_bringup"
  "marvin_manipulation_controller_bringup"
  "ros2_control_node"
  "controller_manager/spawner"
  "controller_manager/hardware_spawner"
  "robot_state_publisher"
  "joint_state_publisher_gui"
  "joint_state_publisher"
  "rviz2"
  "plotjuggler"
  "execution_manager_server"
  "execution_manager"
  "target_marker"
  "pyroki_global_setpoint_planner"
  "ros2-daemon"
)

_kill_patterns() {
  local label="$1"
  shift
  local stopped=0
  local pattern
  for pattern in "$@"; do
    if pkill -f "${pattern}" 2>/dev/null; then
      echo "[${label}] stopped: ${pattern}"
      stopped=1
    fi
  done
  # Second pass: force leftovers that ignored SIGTERM.
  sleep 0.4
  for pattern in "$@"; do
    if pkill -9 -f "${pattern}" 2>/dev/null; then
      echo "[${label}] killed: ${pattern}"
      stopped=1
    fi
  done
  return "${stopped}"
}

echo "Stopping local ROS leftovers..."
if _kill_patterns "local" "${patterns[@]}"; then
  :
else
  echo "[local] no matching processes"
fi

# Drop CLI discovery cache so ghost names clear faster after remote kill.
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

if [[ "${DO_REMOTE}" == "1" ]]; then
  echo "Stopping remote ROS leftovers on ${REMOTE_HOST}..."
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_HOST}" \
    'bash -s' <<'EOS'
set -uo pipefail
patterns=(
  "ros2 launch"
  "rt_stack.launch.py"
  "visualize_franka.launch.py"
  "visualize_piper.launch.py"
  "controller_bringup.launch.py"
  "piper_leader.launch.py"
  "piper_leader_node"
  "piper_leader"
  "piper_leader_teleop"
  "franka_manipulation_controller_bringup"
  "piper_manipulation_controller_bringup"
  "marvin_manipulation_controller_bringup"
  "ros2_control_node"
  "controller_manager/spawner"
  "controller_manager/hardware_spawner"
  "robot_state_publisher"
  "joint_state_publisher_gui"
  "joint_state_publisher"
  "rviz2"
  "execution_manager_server"
  "execution_manager"
  "target_marker"
  "ros2-daemon"
)
stopped=0
for pattern in "${patterns[@]}"; do
  if pkill -f "${pattern}" 2>/dev/null; then
    echo "[remote] stopped: ${pattern}"
    stopped=1
  fi
done
sleep 0.4
for pattern in "${patterns[@]}"; do
  if pkill -9 -f "${pattern}" 2>/dev/null; then
    echo "[remote] killed: ${pattern}"
    stopped=1
  fi
done
if [[ "${stopped}" -eq 0 ]]; then
  echo "[remote] no matching processes"
fi
EOS
  then
    :
  else
    echo "[remote] skip (${REMOTE_HOST} unreachable or SSH failed)"
  fi
else
  echo "[remote] skipped (STOP_ROS_REMOTE=0)"
fi

# Brief settle + report what Domain still sees (best-effort).
sleep 0.8
if command -v ros2 >/dev/null 2>&1; then
  # Prefer workspace env if already activated; otherwise just try.
  nodes="$(ros2 node list 2>/dev/null || true)"
  if [[ -n "${nodes}" ]]; then
    echo "Remaining ROS nodes on this domain:"
    echo "${nodes}"
  else
    echo "ROS graph clear (or daemon not ready yet)."
  fi
fi

echo "Done. Re-launch visualize / rt_stack after stop."
