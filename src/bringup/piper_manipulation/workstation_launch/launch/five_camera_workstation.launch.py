"""Workstation half of the Piper five-camera collection topology.

Starts leader teleoperation, Execution Manager, the recorder daemon, the local
Orbbec external camera, and both local wrist cameras on 192.168.1.18.  The two
new external RealSense cameras are launched on the NUC at 192.168.1.101.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("piper_manipulation_workstation_launch")
    launch_dir = os.path.join(share, "launch")

    def include(name: str, arguments=None) -> GroupAction:
        return GroupAction(
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
                    launch_arguments=(arguments or {}).items(),
                )
            ],
        )

    return LaunchDescription(
        [
            include("execution_manager.launch.py"),
            include(
                "piper_leaders.launch.py",
                {"autostart": "true"},
            ),
            include(
                "piper_orbbec.launch.py",
                {
                    "camera_name": "orbbec",
                    "serial_number": "CL8384201CG",
                },
            ),
            include("piper_realsense.launch.py"),
            include("recorder.launch.py"),
        ]
    )
