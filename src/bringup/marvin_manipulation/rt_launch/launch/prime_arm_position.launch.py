# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Prime Marvin CCS position mode after real-hardware RT controller spawn.

Waits until left/right_arm_jtc are loaded (inactive), then activates both so
``perform_command_mode_switch`` runs cold ``enter_position_modes`` during RT
bringup — before the first workstation EM/JTC goal.

Why: vendor CCS can race the first cold position-mode enter with the first
FollowJointTrajectory claim (example 17 first-run ABORT). EM never fully
unloads route controllers after the first claim, so leaving JTC active avoids
re-entering on later switches.

Real hardware only. Prefer removing this workaround when official CCS /
libmarvin makes cold enter reliable under concurrent claim.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


_PRIME_SCRIPT = r"""
set -euo pipefail
manager="$1"
echo "[prime_arm_position] waiting for left/right_arm_jtc under ${manager}..."
for _ in $(seq 1 90); do
  listing="$(ros2 control list_controllers --controller-manager "${manager}" 2>/dev/null || true)"
  left_state="$(printf '%s\n' "${listing}" | awk '$1=="left_arm_jtc" {print $NF; exit}')"
  right_state="$(printf '%s\n' "${listing}" | awk '$1=="right_arm_jtc" {print $NF; exit}')"
  if [[ -n "${left_state}" && -n "${right_state}" ]]; then
    if [[ "${left_state}" == "active" && "${right_state}" == "active" ]]; then
      echo "[prime_arm_position] left/right_arm_jtc already active — skip"
      exit 0
    fi
    echo "[prime_arm_position] activating left_arm_jtc + right_arm_jtc (was L=${left_state} R=${right_state})"
    ros2 control switch_controllers \
      --activate left_arm_jtc right_arm_jtc \
      --controller-manager "${manager}"
    exit $?
  fi
  sleep 1
done
echo "[prime_arm_position] timed out waiting for arm JTCs" >&2
exit 1
"""


def generate_launch_description() -> LaunchDescription:
    controller_manager = LaunchConfiguration("controller_manager")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="controller_manager service namespace.",
            ),
            LogInfo(
                msg=(
                    "Priming Marvin CCS position mode via left_arm_jtc + "
                    "right_arm_jtc (real-hardware workaround; see BRINGUP.md)."
                )
            ),
            ExecuteProcess(
                cmd=["bash", "-c", _PRIME_SCRIPT, "prime_arm_position", controller_manager],
                output="screen",
                name="prime_arm_position",
            ),
        ]
    )
