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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock); required for MuJoCo RT.",
            ),
            DeclareLaunchArgument("with_execution_manager", default_value="true"),
            DeclareLaunchArgument("with_recorder", default_value="true"),
            DeclareLaunchArgument(
                "with_orbbec",
                default_value="true",
                description="Start static Orbbec (Femto Bolt) on the workstation.",
            ),
            DeclareLaunchArgument(
                "with_realsense",
                default_value="true",
                description="Start dual wrist RealSense D435i on the workstation.",
            ),
            DeclareLaunchArgument(
                "with_leaders",
                default_value="false",
                description=(
                    "Start Piper leader teleop arms. Default off; enable when "
                    "leaders are connected (with_leaders:=true)."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "execution_manager.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_execution_manager")),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "piper_orbbec.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_orbbec")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "piper_realsense.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_realsense")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "piper_leaders.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_leaders")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
