"""Launch gamepad teleop from this package's Franka-specific config."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "franka_manipulation_workstation_launch"
    )
    gamepad_share = get_package_share_directory("gamepad_teleop")
    default_config = os.path.join(
        workstation_share, "config", "teleop", "gamepad.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gamepad_config", default_value=default_config
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        gamepad_share, "launch", "gamepad_teleop.launch.py"
                    )
                ),
                launch_arguments={
                    "config": LaunchConfiguration("gamepad_config")
                }.items(),
            ),
        ]
    )
