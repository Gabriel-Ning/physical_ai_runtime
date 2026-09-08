"""Launch Piper leaders from the package-owned teleop configuration."""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    config = LaunchConfiguration("leader_config").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip()
    leader_share = get_package_share_directory("piper_leader_teleop")
    leader_launch = os.path.join(leader_share, "launch", "piper_leader.launch.py")
    leaders = yaml.safe_load(Path(config).read_text(encoding="utf-8"))
    if not isinstance(leaders, dict) or not leaders:
        raise ValueError(f"{config}: expected a non-empty mapping of leader nodes")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(leader_launch),
            launch_arguments={
                "config": config,
                "node_name": node_name,
                "use_sim_time": use_sim_time,
            }.items(),
        )
        for node_name in leaders
    ]


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "piper_manipulation_workstation_launch"
    )
    default_config = os.path.join(
        workstation_share, "config", "teleop", "piper_leaders.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock); required for MuJoCo RT.",
            ),
            DeclareLaunchArgument("leader_config", default_value=default_config),
            OpaqueFunction(function=_setup),
        ]
    )
