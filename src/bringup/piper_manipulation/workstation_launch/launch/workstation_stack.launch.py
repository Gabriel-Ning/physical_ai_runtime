"""Launch the Piper workstation stack from package-owned child launches."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("piper_manipulation_workstation_launch")
    launch_dir = os.path.join(share, "launch")

    def include(name: str, condition: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            condition=IfCondition(LaunchConfiguration(condition)),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("with_execution_manager", default_value="true"),
            DeclareLaunchArgument("with_recorder", default_value="true"),
            DeclareLaunchArgument("with_orbbec", default_value="true"),
            DeclareLaunchArgument("with_realsense", default_value="true"),
            DeclareLaunchArgument("with_leaders", default_value="true"),
            include("execution_manager.launch.py", "with_execution_manager"),
            include("recorder.launch.py", "with_recorder"),
            include("piper_orbbec.launch.py", "with_orbbec"),
            include("piper_realsense.launch.py", "with_realsense"),
            include("piper_leaders.launch.py", "with_leaders"),
        ]
    )
