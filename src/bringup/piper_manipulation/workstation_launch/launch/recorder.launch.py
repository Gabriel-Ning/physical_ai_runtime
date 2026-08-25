"""Launch the workstation-wide episode recorder daemon.

RMI applications supply the Piper stream contract and episode metadata. The
daemon only owns the recording service and its bounded ingestion queue.
"""

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
                ),
                launch_arguments={
                    "queue_capacity_bytes": str(4 * 1024 * 1024 * 1024),
                    "queue_capacity_messages": "16384",
                }.items(),
            )
        ]
    )
