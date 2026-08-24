"""Launch FR3 workstation Execution Manager and recorder infrastructure.

Defaults come only from ``apps/profiles/fr3_pika_single_arm.yaml``.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))
from workstation_defaults import workstation_launch_defaults


def generate_launch_description() -> LaunchDescription:
    defaults = workstation_launch_defaults("fr3_pika_single_arm.yaml")
    recorder_share = get_package_share_directory("episode_recorder")

    execution_manager = Node(
        package="execution_manager",
        executable="execution_manager",
        name="execution_manager",
        output="screen",
        parameters=[
            {
                "profile": LaunchConfiguration("embodiment_profile"),
                "max_command_age_s": LaunchConfiguration("max_command_age_s"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_execution_manager")),
    )
    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(recorder_share, "launch", "recorder.launch.py")
        ),
        launch_arguments={
            "stream_config_uri": LaunchConfiguration("recording_stream_config"),
            "root_dir": LaunchConfiguration("root_dir"),
            "experiment_name": LaunchConfiguration("experiment_name"),
            "task": LaunchConfiguration("task"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_recorder")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "embodiment_profile",
                default_value=defaults["embodiment_profile"],
            ),
            DeclareLaunchArgument(
                "with_execution_manager",
                default_value=defaults["with_execution_manager"],
            ),
            DeclareLaunchArgument(
                "with_recorder", default_value=defaults["with_recorder"]
            ),
            DeclareLaunchArgument(
                "max_command_age_s",
                default_value=defaults["max_command_age_s"],
            ),
            DeclareLaunchArgument(
                "recording_stream_config",
                default_value=defaults["recording_stream_config"],
            ),
            DeclareLaunchArgument("root_dir", default_value=defaults["root_dir"]),
            DeclareLaunchArgument(
                "experiment_name", default_value=defaults["experiment_name"]
            ),
            DeclareLaunchArgument("task", default_value=defaults["task"]),
            execution_manager,
            recorder,
        ]
    )
