# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the static Orbbec Femto Bolt camera.

Publishes RGB-D streams below ``/observation/static_orbbec``; in particular:

* ``/observation/static_orbbec/color/image_raw``
* ``/observation/static_orbbec/depth/image_raw``
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("piper_manipulation_controller_bringup"),
            "config",
            "camera",
            "femto_bolt.yaml",
        ]
    )
    femto_bolt_launch = PathJoinSubstitution(
        [FindPackageShare("orbbec_camera"), "launch", "femto_bolt.launch.py"]
    )

    camera = GroupAction(
        actions=[
            PushRosNamespace("observation"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(femto_bolt_launch),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "serial_number": LaunchConfiguration("serial_number"),
                    "config_file": LaunchConfiguration("orbbec_config_file"),
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "orbbec_config_file",
                default_value=config_file,
                description="Absolute path to the Femto Bolt parameter YAML.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="static_orbbec",
                description="Camera topic and frame-name prefix.",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="",
                description="Optional Orbbec camera serial-number override.",
            ),
            camera,
        ]
    )
