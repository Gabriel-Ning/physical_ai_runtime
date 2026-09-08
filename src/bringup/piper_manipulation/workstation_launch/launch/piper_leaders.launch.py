"""Launch both Piper leaders from the package-owned bimanual config."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("piper_manipulation_workstation_launch")
    leader_share = get_package_share_directory("piper_leader_teleop")
    leader_launch = os.path.join(
        leader_share, "launch", "piper_leader.launch.py"
    )
    config = os.path.join(share, "config", "teleop", "piper_leaders.yaml")

    def include(side: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(leader_launch),
            launch_arguments={
                "config": config,
                "node_name": f"piper_leader_{side}",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock); required for MuJoCo RT.",
            ),
            include("left"),
            include("right"),
        ]
    )
