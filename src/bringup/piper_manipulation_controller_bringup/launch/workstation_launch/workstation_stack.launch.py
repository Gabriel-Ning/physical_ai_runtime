# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the Workstation Peripherals Server Stack for Piper Bimanual.

Modularly aggregates Workstation hardware bringup:
1. Orbbec Femto Bolt static camera (orbbec_camera_bringup.launch.py)
2. Dual RealSense D435 wrist cameras (realsense_camera_bringup.launch.py)
3. Dual Piper Leader teleoperation arms (piper_teleop_leader_bringup.launch.py)
4. Optional C++ MCAP Episode Recorder backend (episode_recorder/recorder.launch.py)
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
    bringup_share = get_package_share_directory("piper_manipulation_controller_bringup")
    default_stream_cfg = os.path.join(
        bringup_share, "config", "recording", "rmi_piper_bimanual.yaml"
    )

    # 1. Orbbec Camera Bringup
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "workstation_launch", "orbbec_camera_bringup.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("with_orbbec")),
    )

    # 2. RealSense Wrist Cameras Bringup
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "workstation_launch", "realsense_camera_bringup.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("with_realsense")),
    )

    # 3. Piper Leader Teleoperation Bringup
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "workstation_launch", "piper_teleop_leader_bringup.launch.py")
        ),
        launch_arguments={
            "left_can_interface": LaunchConfiguration("left_leader_can"),
            "right_can_interface": LaunchConfiguration("right_leader_can"),
            "publish_rate_hz": LaunchConfiguration("leader_publish_rate_hz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_leaders")),
    )

    # 4. Optional C++ MCAP Episode Recorder Server
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
                "with_orbbec",
                default_value="true",
                description="Whether to start the static Orbbec camera.",
            ),
            DeclareLaunchArgument(
                "with_realsense",
                default_value="true",
                description="Whether to start the dual RealSense wrist cameras.",
            ),
            DeclareLaunchArgument(
                "with_leaders",
                default_value="true",
                description="Whether to start the master-slave Piper leader arms.",
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
            DeclareLaunchArgument(
                "left_leader_can",
                default_value="can1",
                description="SocketCAN interface for left leader arm.",
            ),
            DeclareLaunchArgument(
                "right_leader_can",
                default_value="can0",
                description="SocketCAN interface for right leader arm.",
            ),
            DeclareLaunchArgument(
                "leader_publish_rate_hz",
                default_value="200.0",
                description="Leader arm publication frequency in Hz.",
            ),
            orbbec_launch,
            realsense_launch,
            leader_launch,
            recorder_launch,
        ]
    )
