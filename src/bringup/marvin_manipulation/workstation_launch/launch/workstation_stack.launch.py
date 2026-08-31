"""Unified Workstation stack bringup for Marvin.

Aggregates Workstation Host bringup:
  1. execution_manager.launch.py
  2. realsense_camera.launch.py
  3. recorder.launch.py
  4. quest3_teleop.launch.py

Each child launch owns its package-local config. Camera bringup is optional;
recording stream requirements remain application-owned.
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
        "marvin_manipulation_workstation_launch"
    )
    launch_dir = os.path.join(workstation_share, "launch")
    default_em = os.path.join(workstation_share, "config", "execution_manager.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "em_config",
                default_value=default_em,
                description="Execution Manager routing table. Default is the dual-Pika table.",
            ),
            DeclareLaunchArgument(
                "with_cameras",
                default_value="true",
                description="Launch workstation RealSense cameras from their config.",
            ),
            DeclareLaunchArgument(
                "with_teleop",
                default_value="false",
                description="Launch the workstation Quest 3 teleop stack.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "execution_manager.launch.py")
                ),
                launch_arguments={
                    "config": LaunchConfiguration("em_config"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "realsense_camera.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_cameras")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "recorder.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "quest3_teleop.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("with_teleop")),
            ),
        ]
    )
