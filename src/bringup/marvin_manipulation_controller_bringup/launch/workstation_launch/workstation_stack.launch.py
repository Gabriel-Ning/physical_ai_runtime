# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the Workstation Peripherals Server Stack for Marvin Bimanual.

Aggregates Workstation services:
1. Optional C++ MCAP Episode Recorder backend (recorder_bringup.launch.py)
2. Optional workstation RealSense bringup (camera_bringup.launch.py)
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
    bringup_share = get_package_share_directory("marvin_manipulation_controller_bringup")
    default_stream_cfg = os.path.join(
        bringup_share, "config", "recording", "rmi_marvin_bimanual.yaml"
    )
    default_camera_cfg = os.path.join(
        bringup_share, "config", "camera", "workstation_realsense.yaml"
    )

    # 1. Workstation RealSense
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "workstation_launch", "camera_bringup.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("with_cameras")),
        launch_arguments={
            "camera_config": LaunchConfiguration("camera_config"),
        }.items(),
    )

    # 2. C++ MCAP Episode Recorder Server
    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "workstation_launch", "recorder_bringup.launch.py")
        ),
        launch_arguments={
            "stream_config_uri": LaunchConfiguration("recording_stream_config"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_recorder")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_cameras",
                default_value="false",
                description="Whether to start the workstation RealSense.",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=default_camera_cfg,
                description="Path to workstation RealSense params YAML.",
            ),
            DeclareLaunchArgument(
                "with_recorder",
                default_value="true",
                description="Whether to start the C++ MCAP episode_recorder server.",
            ),
            DeclareLaunchArgument(
                "recording_stream_config",
                default_value=default_stream_cfg,
                description="Stream contract YAML configuration for MCAP recording.",
            ),
            camera_launch,
            recorder_launch,
        ]
    )
