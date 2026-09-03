"""Unified Piper workstation bringup.

Starts both leader arms, one selectable static camera, two wrist RealSense
cameras, the episode recorder daemon, and Execution Manager. Robot controllers
remain on the RT host.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    workstation_share = get_package_share_directory(
        "piper_manipulation_workstation_launch"
    )
    launch_dir = os.path.join(workstation_share, "launch")
    default_em = os.path.join(workstation_share, "config", "execution_manager.yaml")

    def include(name: str, arguments=None, condition=None) -> GroupAction:
        # Each included launch declares common names such as ``config`` and
        # ``camera_name``.  Scope them so an execution-manager config cannot be
        # injected into leader/camera nodes (and vice versa).
        return GroupAction(
            # Keep top-level arguments (for example em_config and autostart)
            # visible long enough to resolve launch_arguments, while retaining
            # each child's declarations inside this GroupAction scope.
            forwarding=True,
            condition=condition,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
                    launch_arguments=(arguments or {}).items(),
                )
            ]
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "em_config",
                default_value=default_em,
                description="Execution Manager capability and command routing table.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="",
                description="Optional Leader hardware activation override.",
            ),
            DeclareLaunchArgument(
                "static_camera",
                default_value="orbbec",
                description="Static third camera: 'orbbec' (default) or 'd435i'.",
            ),
            DeclareLaunchArgument(
                "static_realsense_serial_no",
                default_value="_310222078614",
                description="Serial number for the fixed D435i; keep the leading underscore.",
            ),
            include(
                "execution_manager.launch.py",
                {"config": LaunchConfiguration("em_config")},
            ),
            include(
                "piper_leaders.launch.py",
                {"autostart": LaunchConfiguration("autostart")},
            ),
            include(
                "piper_orbbec.launch.py",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("static_camera"), "' == 'orbbec'"]
                    )
                ),
            ),
            include(
                "piper_static_realsense.launch.py",
                {
                    "serial_no": LaunchConfiguration("static_realsense_serial_no"),
                },
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("static_camera"), "' == 'd435i'"]
                    )
                ),
            ),
            include("piper_realsense.launch.py"),
            include("recorder.launch.py"),
        ]
    )
