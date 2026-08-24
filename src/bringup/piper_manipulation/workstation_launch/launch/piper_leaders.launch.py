"""Launch the two Piper Leader Arm workstation drivers."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    robot_share = get_package_share_directory(
        "piper_manipulation_workstation_launch"
    )
    leader_share = get_package_share_directory("piper_leader_teleop")
    launch_path = os.path.join(leader_share, "launch", "piper_leader.launch.py")
    default_config = os.path.join(
        robot_share, "config", "teleop", "piper_leaders.yaml"
    )

    def include(side: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                "config": LaunchConfiguration("config"),
                "node_name": f"piper_leader_{side}",
                "can_interface": LaunchConfiguration(f"{side}_can_interface"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "default_mode": LaunchConfiguration("default_mode"),
                "fallback_mode": LaunchConfiguration("fallback_mode"),
            }.items(),
            condition=IfCondition(LaunchConfiguration(f"enable_{side}")),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("enable_left", default_value="true"),
            DeclareLaunchArgument("enable_right", default_value="true"),
            DeclareLaunchArgument("left_can_interface", default_value=""),
            DeclareLaunchArgument("right_can_interface", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value=""),
            DeclareLaunchArgument("default_mode", default_value=""),
            DeclareLaunchArgument("fallback_mode", default_value=""),
            include("left"),
            include("right"),
        ]
    )
