"""Launch the complete Franka workstation stack."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "franka_manipulation_workstation_launch"
    )
    launch_dir = os.path.join(workstation_share, "launch")
    default_em = os.path.join(workstation_share, "config", "execution_manager.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock).",
            ),
            DeclareLaunchArgument(
                "em_config",
                default_value=default_em,
                description="Execution Manager routing table.",
            ),
            DeclareLaunchArgument(
                "with_recorder",
                default_value="true",
                description=(
                    "Launch episode_recorder. Set false to keep EM/gamepad only "
                    "(e.g. RViz camera A/B without recorder image subscriptions)."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "execution_manager.launch.py")
                ),
                launch_arguments={
                    "config": LaunchConfiguration("em_config"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "gamepad_teleop.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "recorder.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_recorder")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
