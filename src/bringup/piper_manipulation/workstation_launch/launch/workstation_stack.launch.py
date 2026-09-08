"""Launch the Piper workstation stack from package-owned child launches.

Each child launch reads its own file under config/. This stack only declares
whether to start a child and forwards use_sim_time.
"""

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
        "piper_manipulation_workstation_launch"
    )
    launch_dir = os.path.join(workstation_share, "launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock); required for MuJoCo RT.",
            ),
            DeclareLaunchArgument(
                "with_execution_manager",
                default_value="true",
                description="Launch Execution Manager from config/execution_manager.yaml.",
            ),
            DeclareLaunchArgument(
                "with_recorder",
                default_value="true",
                description="Launch episode_recorder.",
            ),
            DeclareLaunchArgument(
                "with_orbbec",
                default_value="true",
                description="Start Femto Bolt cameras from config/camera/femto_bolt.yaml.",
            ),
            DeclareLaunchArgument(
                "with_realsense",
                default_value="true",
                description="Start D435 cameras from config/camera/realsense_d435.yaml.",
            ),
            DeclareLaunchArgument(
                "with_leaders",
                default_value="true",
                description=(
                    "Start Piper leader teleop arms. Default on for real hardware; "
                    "disable for MuJoCo (with_leaders:=false)."
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
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "recorder.launch.py")),
                condition=IfCondition(LaunchConfiguration("with_recorder")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "piper_orbbec.launch.py")),
                condition=IfCondition(LaunchConfiguration("with_orbbec")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "piper_realsense.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_realsense")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "piper_leaders.launch.py")),
                condition=IfCondition(LaunchConfiguration("with_leaders")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
