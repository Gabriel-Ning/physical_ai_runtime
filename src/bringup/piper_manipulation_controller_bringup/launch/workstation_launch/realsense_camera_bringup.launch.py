# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the two wrist RealSense D435i cameras.

Publishes color images to:

* ``/observation/left_hand_realsense/color/image_raw``
* ``/observation/right_hand_realsense/color/image_raw``
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    ResetLaunchConfigurations,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _camera_group(camera_name_arg: str, serial_no_arg: str) -> GroupAction:
    """Isolate each upstream rs_launch.py invocation's launch arguments."""
    return GroupAction(
        scoped=True,
        actions=[
            ResetLaunchConfigurations(
                {
                    "config_file": LaunchConfiguration("realsense_config_file"),
                    "camera_name": LaunchConfiguration(camera_name_arg),
                    "camera_namespace": "/observation",
                    "serial_no": LaunchConfiguration(serial_no_arg),
                }
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realsense2_camera"),
                            "launch",
                            "rs_launch.py",
                        ]
                    )
                )
            ),
        ],
    )


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("piper_manipulation_controller_bringup"),
            "config",
            "camera",
            "d435i_dual.yaml",
        ]
    )
    left_camera = _camera_group("left_camera_name", "left_serial_no")
    right_camera = _camera_group("right_camera_name", "right_serial_no")

    return LaunchDescription(
        [
            # Suppress benign librealsense USB HID warnings not covered by
            # ROS log levels.
            SetEnvironmentVariable("LRS_LOG_LEVEL", "ERROR"),
            DeclareLaunchArgument(
                "realsense_config_file",
                default_value=config_file,
                description="Absolute path to the shared D435i parameter YAML.",
            ),
            DeclareLaunchArgument(
                "left_camera_name",
                default_value="left_hand_realsense",
                description="Left wrist camera topic and frame-name prefix.",
            ),
            DeclareLaunchArgument(
                "right_camera_name",
                default_value="right_hand_realsense",
                description="Right wrist camera topic and frame-name prefix.",
            ),
            DeclareLaunchArgument(
                "left_serial_no",
                default_value="_332522075913",
                description="Left D435i serial number.",
            ),
            DeclareLaunchArgument(
                "right_serial_no",
                default_value="_332322073584",
                description="Right D435i serial number.",
            ),
            DeclareLaunchArgument(
                "right_camera_delay",
                default_value="10.0",
                description="Seconds to wait before starting the right D435i.",
            ),
            left_camera,
            TimerAction(
                period=LaunchConfiguration("right_camera_delay"),
                actions=[right_camera],
            ),
        ]
    )
