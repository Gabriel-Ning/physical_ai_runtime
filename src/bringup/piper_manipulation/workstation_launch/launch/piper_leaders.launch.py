"""Launch both Piper leaders from the package-owned bimanual config."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
            }.items(),
        )

    return LaunchDescription([include("left"), include("right")])
