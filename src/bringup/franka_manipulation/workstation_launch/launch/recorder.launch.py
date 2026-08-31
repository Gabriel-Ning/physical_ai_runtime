"""Launch the workstation-wide recorder daemon."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    recorder_share = get_package_share_directory("episode_recorder")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(recorder_share, "launch", "recorder.launch.py")
                )
            )
        ]
    )
