#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_root}"

mkdir -p build install log/ros data "${CLOUDXR_DIR}"

for command in python ros2 colcon vcs rosdep cmake ninja; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "Missing required command in Pixi environment: ${command}" >&2
    exit 1
  fi
done

echo "Physical AI Runtime development environment is ready."
echo "  workspace: ${WORKSPACE_ROOT}"
echo "  python:    $(command -v python)"
echo "  ros2:      $(command -v ros2)"
echo "  colcon:    $(command -v colcon)"
echo "  cloudxr:   ${CLOUDXR_DIR}"
