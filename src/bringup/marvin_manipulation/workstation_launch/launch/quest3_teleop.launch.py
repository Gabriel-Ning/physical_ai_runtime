"""Launch Quest 3 teleop with Marvin-specific EM and frame overrides."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "marvin_manipulation_workstation_launch"
    )
    toolbox_share = get_package_share_directory("isaacteleop_toolbox")
    default_config = os.path.join(
        workstation_share,
        "config",
        "teleop",
        "quest3_bimanual_relative.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "quest3_config",
                default_value=default_config,
                description="Marvin-specific Quest 3 parameter overrides.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        toolbox_share,
                        "launch",
                        "bimanual_target_live.launch.py",
                    )
                ),
                launch_arguments={
                    "profile_config": LaunchConfiguration("quest3_config"),
                    "left_base_frame": "Base_L",
                    "right_base_frame": "Base_R",
                }.items(),
            ),
        ]
    )
