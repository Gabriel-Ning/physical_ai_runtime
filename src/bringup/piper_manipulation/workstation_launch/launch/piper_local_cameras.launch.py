"""Launch only the three cameras attached to the workstation.

This is the safe visualization/validation entry point for the standard
three-camera setup: local Orbbec is ``orbbec`` and both wrist RealSense cameras
keep their canonical names. It does not start leaders, controllers, EM, or the
recorder.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    launch_dir = os.path.join(
        get_package_share_directory("piper_manipulation_workstation_launch"),
        "launch",
    )

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
            include(
                "piper_orbbec.launch.py",
                {
                    "camera_name": "orbbec",
                    "serial_number": "CL8384201CG",
                },
            ),
            include("piper_realsense.launch.py"),
        ]
    )
