# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""Unified RT-Host stack bringup for dual Piper.

Dispatches by ``backend`` to a sub-launch module via IncludeLaunchDescription:
  - real / fake → ``controller_bringup.launch.py``
  - mujoco → ``mujoco_bringup.launch.py``
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_CONTROL_MODULES = {
    "mujoco": "mujoco_bringup.launch.py",
    "real": "controller_bringup.launch.py",
    "fake": "controller_bringup.launch.py",
}


def _include_launch(launch_dir: str, module: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, module)),
        launch_arguments=arguments.items(),
    )


def _include_control_stack(context, *args, **kwargs):
    backend = LaunchConfiguration("backend").perform(context).lower().strip()
    use_sim_mujoco = (
        LaunchConfiguration("use_sim_mujoco").perform(context).lower().strip()
    )
    if backend == "mujoco" or use_sim_mujoco in ("true", "1", "yes"):
        module_key = "mujoco"
    elif backend in ("real", "fake"):
        module_key = backend
    elif backend:
        raise RuntimeError(
            f"'backend' must be real, fake, or mujoco, got '{backend}'"
        )
    else:
        fake = LaunchConfiguration("use_fake_hardware").perform(context).lower().strip()
        module_key = "fake" if fake in ("true", "1", "yes") else "real"

    launch_dir = os.path.join(
        get_package_share_directory("piper_manipulation_rt_launch"), "launch"
    )
    module = _CONTROL_MODULES[module_key]

    if module_key == "mujoco":
        return [
            _include_launch(
                launch_dir,
                module,
                {
                    "arms": "both",
                    "task": LaunchConfiguration("task"),
                    "headless": LaunchConfiguration("headless"),
                    "mujoco_plugins_yaml": LaunchConfiguration("mujoco_plugins_yaml"),
                    "load_gripper_hardware": LaunchConfiguration(
                        "load_gripper_hardware"
                    ),
                    "left_can_interface": LaunchConfiguration("left_can_interface"),
                    "right_can_interface": LaunchConfiguration("right_can_interface"),
                    "left_end_effector": "piper_gripper",
                    "right_end_effector": "piper_gripper",
                    "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                        "jtc_guard_heartbeat_timeout_s"
                    ),
                    "jtc_guard_cancel_response_timeout_s": LaunchConfiguration(
                        "jtc_guard_cancel_response_timeout_s"
                    ),
                },
            )
        ]

    return [
        _include_launch(
            launch_dir,
            module,
            {
                "arms": "both",
                "left_can_interface": LaunchConfiguration("left_can_interface"),
                "right_can_interface": LaunchConfiguration("right_can_interface"),
                "left_end_effector": "piper_gripper",
                "right_end_effector": "piper_gripper",
                "load_gripper_hardware": LaunchConfiguration("load_gripper_hardware"),
                "use_fake_hardware": "true" if module_key == "fake" else "false",
                "backend": module_key,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "cpu_affinity": LaunchConfiguration("cpu_affinity"),
                "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                    "jtc_guard_heartbeat_timeout_s"
                ),
                "jtc_guard_cancel_response_timeout_s": LaunchConfiguration(
                    "jtc_guard_cancel_response_timeout_s"
                ),
            },
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("piper_manipulation_rt_launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "left_can_interface",
                default_value="piper0",
                description="SocketCAN name for the left follower (default: piper0).",
            ),
            DeclareLaunchArgument(
                "right_can_interface",
                default_value="piper1",
                description="SocketCAN name for the right follower (default: piper1).",
            ),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument(
                "backend",
                default_value="",
                description="real, fake, or mujoco. Empty falls back to use_fake_hardware.",
            ),
            DeclareLaunchArgument(
                "use_sim_mujoco",
                default_value="false",
                description="Compat alias for backend:=mujoco.",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="table_pick_cube",
                description=(
                    "RoboTwin / piper_description task name or absolute MJCF path "
                    "(MuJoCo only; default: table_pick_cube)."
                ),
            ),
            DeclareLaunchArgument(
                "mujoco_plugins_yaml",
                default_value=os.path.join(
                    bringup_share, "config", "mujoco_plugins.yaml"
                ),
                description="Shared CameraPlugin and SHM bridge configuration.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo simulation in headless mode (no GUI window).",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "load_gripper_hardware",
                default_value="true",
                description=(
                    "Load both native Piper grippers and their controllers. "
                    "Set false to omit both sides."
                ),
            ),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node. Empty uses "
                    "RT_CM_CPU_AFFINITY from the RT host profile. Pass none to disable."
                ),
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "jtc_guard_cancel_response_timeout_s", default_value="0.5"
            ),
            OpaqueFunction(function=_include_control_stack),
        ]
    )
