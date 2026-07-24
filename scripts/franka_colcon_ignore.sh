#!/usr/bin/env bash
# Idempotently COLCON_IGNORE non-core packages inside the franka_ros2 checkout.
#
# Interim compromise while this workspace vendors upstream franka_ros2 (jazzy).
# Target embodiment shape is Marvin-like: description + hardware_interface only
# (see docs/ARCHITECTURE.md, Franka maintenance stance). Do not treat this
# filter as the long-term ownership model.
#
# Core arm stack (msgs, hardware, gripper, bringup, example controllers, MoveIt
# config, …) stays buildable; Gazebo/mobile/vision and vendored realtime_tools
# are skipped so colcon does not need official dependency.repos overlays.
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
franka_root="${workspace_root}/src/embodiments/robots/franka/franka_ros2"

if [[ ! -d "${franka_root}" ]]; then
  echo "franka_ros2 checkout missing: ${franka_root}" >&2
  echo "Run: vcs import src < repos/embodiment.repos" >&2
  exit 1
fi

ignore_paths=(
  "franka_gazebo/franka_gazebo_bringup"
  "franka_gazebo/franka_gazebo_hardware"
  "franka_mobile"
  "franka_mobile_fr3_duo_moveit_config"
  "franka_mobile_sensors"
  "mobile_fr3_duo_trajectory_controller"
  "franka_spine/franka_spine_examples"
  "franka_spine/franka_spine_msgs"
  "franka_spine/franka_spine_server"
  "franka_vision_and_manipulation_kit"
  "franka_ros2"
  "realtime_tools/realtime_tools"
)

created=0
for rel in "${ignore_paths[@]}"; do
  dir="${franka_root}/${rel}"
  if [[ ! -d "${dir}" ]]; then
    echo "skip (missing): ${rel}"
    continue
  fi
  marker="${dir}/COLCON_IGNORE"
  if [[ -f "${marker}" ]]; then
    continue
  fi
  touch "${marker}"
  echo "COLCON_IGNORE: ${rel}"
  created=$((created + 1))
done

echo "franka_colcon_ignore: ${created} new marker(s) under ${franka_root}"
