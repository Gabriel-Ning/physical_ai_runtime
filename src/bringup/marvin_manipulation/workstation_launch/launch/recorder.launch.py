"""Launch the workstation-wide recorder daemon.

The application supplies its recording contract through RMI.

Raw RGB-D on this cell is ~400 MB/s. The episode_recorder default 1 GiB
queue overflowed on a 5 s capture, so this launch raises the byte and
message caps. Lifecycle recycle does not apply new values; relaunch the node.
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
            ),
        ]
    )
