#!/usr/bin/env bash
# Stop leftover background ROS 2 processes from any `ros2 launch` in this
# workspace that wasn't cleanly Ctrl-C'd (nodes, spawners, controller_manager,
# rviz, etc.). Safe to run with nothing left running: every pkill is allowed
# to match zero processes, so this never fails the pixi task.
set -uo pipefail

patterns=(
  "ros2 launch"
  "ros2_control_node"
  "controller_manager/spawner"
  "controller_manager/hardware_spawner"
  "robot_state_publisher"
  "rviz2"
  "execution_manager"
  "target_marker_node"
  "joint_state_publisher"
)

stopped_any=0
for pattern in "${patterns[@]}"; do
  if pkill -f "${pattern}" 2>/dev/null; then
    echo "stopped: ${pattern}"
    stopped_any=1
  fi
done

if [[ "${stopped_any}" -eq 0 ]]; then
  echo "No matching ROS 2 processes were running."
fi
