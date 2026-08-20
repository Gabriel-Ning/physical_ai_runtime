# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the C++ MCAP episode recorder server configured for Marvin Bimanual."""

from __future__ import annotations

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_controller_bringup")
    default_stream_cfg = os.path.join(
        bringup_share, "config", "recording", "rmi_marvin_bimanual.yaml"
    )

    try:
        recorder_share = get_package_share_directory("episode_recorder")
        launch_path = os.path.join(recorder_share, "launch", "recorder.launch.py")
        recorder = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                "stream_config_uri": LaunchConfiguration("stream_config_uri"),
                "root_dir": LaunchConfiguration("root_dir"),
                "experiment_name": LaunchConfiguration("experiment_name"),
                "task": LaunchConfiguration("task"),
            }.items(),
        )
    except Exception:
        recorder = None

    actions = [
        DeclareLaunchArgument(
            "stream_config_uri",
            default_value=default_stream_cfg,
            description="Stream contract YAML configuration for MCAP recording.",
        ),
        DeclareLaunchArgument(
            "root_dir",
            default_value="data/episodes",
            description="Root output directory for recorded episodes.",
        ),
        DeclareLaunchArgument(
            "experiment_name",
            default_value="marvin_bimanual",
            description="Dataset/experiment identifier.",
        ),
        DeclareLaunchArgument(
            "task",
            default_value="bimanual_manipulation",
            description="Task description default label.",
        ),
    ]

    if recorder is not None:
        actions.append(recorder)

    return LaunchDescription(actions)
