# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the dual Piper Leader Arm teleoperation hardware drivers.

Defaults (CAN, joints, topics, modes, rate) come from
``config/teleop/piper_leaders.yaml``. Launch arguments are optional overrides
only — leave them empty to keep the YAML values.
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
    default_config = os.path.join(bringup_share, "config", "teleop", "piper_leaders.yaml")

    leader_share = get_package_share_directory("piper_leader_teleop")
    launch_path = os.path.join(leader_share, "launch", "piper_leader.launch.py")

    left_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "node_name": "piper_leader_left",
            "can_interface": LaunchConfiguration("left_can_interface"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "default_mode": LaunchConfiguration("default_mode"),
            "fallback_mode": LaunchConfiguration("fallback_mode"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_left")),
    )

    right_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "node_name": "piper_leader_right",
            "can_interface": LaunchConfiguration("right_can_interface"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "default_mode": LaunchConfiguration("default_mode"),
            "fallback_mode": LaunchConfiguration("fallback_mode"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_right")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to piper dual leaders configuration YAML file.",
            ),
            DeclareLaunchArgument(
                "enable_left",
                default_value="true",
                description="Whether to start the left leader arm driver.",
            ),
            DeclareLaunchArgument(
                "enable_right",
                default_value="true",
                description="Whether to start the right leader arm driver.",
            ),
            DeclareLaunchArgument(
                "left_can_interface",
                default_value="",
                description="Optional left SocketCAN override (empty = use config).",
            ),
            DeclareLaunchArgument(
                "right_can_interface",
                default_value="",
                description="Optional right SocketCAN override (empty = use config).",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="",
                description="Optional publish rate override in Hz (empty = use config).",
            ),
            DeclareLaunchArgument(
                "default_mode",
                default_value="",
                description="Optional startup mode override: shadow | passive (empty = use config).",
            ),
            DeclareLaunchArgument(
                "fallback_mode",
                default_value="",
                description="Optional fallback mode override: shadow | passive (empty = use config).",
            ),
            left_leader,
            right_leader,
        ]
    )
