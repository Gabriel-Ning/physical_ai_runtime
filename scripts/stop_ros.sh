#!/usr/bin/env bash
# Stop leftover ROS 2 processes from this workspace that weren't cleanly
# Ctrl-C'd (launch parents, spawners, CM, cameras, RViz, ...).
#
#   pixi run stop                         # this machine only (default)
#   STOP_ROS_REMOTE=1 pixi run stop       # also run the same script on RT host
#
# Safe with nothing running. Always restarts the ros2 daemon afterwards.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DO_REMOTE="${STOP_ROS_REMOTE:-0}"
REMOTE_HOST="${STOP_ROS_REMOTE_HOST:-gamma@192.168.1.102}"
REMOTE_WS="${STOP_ROS_REMOTE_WS:-/home/gamma/Documents/physical_ai_runtime}"

_ensure_ros2() {
  if command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  local cand
  for cand in \
    "${ROOT}/.pixi/envs/cpu/bin" \
    "${ROOT}/.pixi/envs/runtime/bin" \
    "${ROOT}/.pixi/envs/default/bin"
  do
    if [[ -x "${cand}/ros2" ]]; then
      PATH="${cand}:${PATH}"
      return 0
    fi
  done
  return 1
}

# Launch / CLI cmdline fragments.
launch_patterns=(
  "ros2 launch"
  "rt_stack.launch.py"
  "pika_camera_bringup.launch.py"
  "visualize_franka.launch.py"
  "visualize_piper.launch.py"
  "visualize_marvin_manipulation.launch.py"
  "controller_bringup.launch.py"
  "piper_leader.launch.py"
  "piper_leader_node"
  "piper_leader"
  "piper_leader_teleop"
  "franka_manipulation_rt_launch"
  "franka_manipulation_workstation_launch"
  "franka_manipulation_controller_bringup"
  "piper_manipulation_rt_launch"
  "piper_manipulation_workstation_launch"
  "piper_manipulation_controller_bringup"
  "marvin_manipulation_rt_launch"
  "marvin_manipulation_workstation_launch"
  "marvin_manipulation_controller_bringup"
)

# Node cmdline fragments. Cameras are started by rt_stack and often
# survive SIGKILL of the launch parent.
node_patterns=(
  "ros2_control_node"
  "controller_manager/spawner"
  "controller_manager/hardware_spawner"
  "mjpeg_cam_node"
  "realsense2_camera_node"
  "jtc_guard_node"
  "robot_state_publisher"
  "joint_state_publisher_gui"
  "joint_state_publisher"
  "rviz2"
  "plotjuggler"
  "execution_manager_server"
  "execution_manager"
  "episode_recorder"
  "target_marker"
  "pyroki_global_setpoint_planner"
)

# killall matches /proc/pid/comm (Linux truncates to 15 chars).
binaries=(
  ros2_control_node
  mjpeg_cam_node
  realsense2_camera_node
  realsense2_came
  jtc_guard_node
  robot_state_publisher
  joint_state_publisher_gui
  joint_state_publisher
  rviz2
  plotjuggler
  episode_recorder
)

_is_protected() {
  local pid="$1"
  [[ "$pid" -eq "$$" || "$pid" -eq "$PPID" ]] && return 0
  local cmd
  cmd="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "$cmd" == *stop_ros.sh* ]] && return 0
  [[ "$cmd" == *"pixi run stop"* ]] && return 0
  return 1
}

_kill_pattern() {
  local sig="$1"
  local pattern="$2"
  local pid killed=0
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    _is_protected "$pid" && continue
    if kill "-${sig}" "$pid" 2>/dev/null; then
      killed=1
    fi
  done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  return $((1 - killed))
}

_sweep_patterns() {
  local label="$1"
  local sig="$2"
  shift 2
  local pattern
  for pattern in "$@"; do
    if _kill_pattern "$sig" "$pattern"; then
      echo "[${label}] SIG${sig}: ${pattern}"
    fi
  done
}

_sweep_binaries() {
  local sig="$1"
  local name
  for name in "${binaries[@]}"; do
    killall "-${sig}" "$name" 2>/dev/null || true
  done
}

_stop_local() {
  echo "Stopping local ROS leftovers..."
  _sweep_patterns "local" TERM "${launch_patterns[@]}"
  sleep 1.2
  _sweep_patterns "local" TERM "${node_patterns[@]}"
  _sweep_binaries TERM
  sleep 0.6
  _sweep_patterns "local" KILL "${launch_patterns[@]}" "${node_patterns[@]}"
  _sweep_binaries KILL
}

_restart_daemon() {
  if ! _ensure_ros2; then
    echo "ros2 not on PATH; skip daemon restart."
    return 0
  fi
  echo "Restarting ros2 daemon..."
  timeout 8 ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.2
  timeout 8 ros2 daemon start >/dev/null 2>&1 || true
}

_report_leftovers() {
  local leftover=0
  local pattern pid
  echo "Checking leftover processes..."
  for pattern in \
    ros2_control_node \
    mjpeg_cam_node \
    realsense2_camera_node \
    jtc_guard_node \
    robot_state_publisher \
    "ros2 launch" \
    rt_stack.launch.py
  do
    while read -r pid; do
      [[ -z "$pid" ]] && continue
      _is_protected "$pid" && continue
      echo "  STILL RUNNING (${pattern}): pid ${pid} $(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null)"
      leftover=1
    done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  done
  if [[ "${leftover}" -eq 0 ]]; then
    echo "  none"
  fi
  if _ensure_ros2; then
    sleep 0.4
    local nodes
    nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
    if [[ -n "${nodes}" ]]; then
      echo "Remaining ROS nodes on this domain:"
      echo "${nodes}"
    else
      echo "ROS graph clear (or discovery still settling)."
    fi
  fi
}

_stop_local
_restart_daemon

if [[ "${DO_REMOTE}" == "1" ]]; then
  echo "Stopping remote ROS leftovers on ${REMOTE_HOST}..."
  if ssh -o BatchMode=yes -o ConnectTimeout=8 "${REMOTE_HOST}" \
    "STOP_ROS_REMOTE=0 bash '${REMOTE_WS}/scripts/stop_ros.sh'"; then
    :
  else
    echo "[remote] skip (${REMOTE_HOST} unreachable, SSH failed, or script missing)"
  fi
else
  echo "[remote] skipped (STOP_ROS_REMOTE=0)"
fi

_report_leftovers
echo "Done. Re-launch visualize / rt_stack after stop."
