# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the dual Piper Leader Arm teleoperation hardware drivers.

Publishes to:
- /action_sources/piper_leader_left/arm/joint_reference
- /action_sources/piper_leader_left/end_effector/joint_reference
- /action_sources/piper_leader_right/arm/joint_reference
- /action_sources/piper_leader_right/end_effector/joint_reference
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
            "joint_names": "left_joint1,left_joint2,left_joint3,left_joint4,left_joint5,left_joint6",
            "gripper_joint_name": "left_gripper_joint1",
            "joint_reference_topic": "/action_sources/piper_leader_left/arm/joint_reference",
            "gripper_reference_topic": "/action_sources/piper_leader_left/end_effector/joint_reference",
            "status_topic": "/teleop/piper_leader_left/status",
            "pendant_state_topic": "/teleop/piper_leader_left/pendant_state",
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
            "joint_names": "right_joint1,right_joint2,right_joint3,right_joint4,right_joint5,right_joint6",
            "gripper_joint_name": "right_gripper_joint1",
            "joint_reference_topic": "/action_sources/piper_leader_right/arm/joint_reference",
            "gripper_reference_topic": "/action_sources/piper_leader_right/end_effector/joint_reference",
            "status_topic": "/teleop/piper_leader_right/status",
            "pendant_state_topic": "/teleop/piper_leader_right/pendant_state",
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
                default_value="can0",
                description="SocketCAN interface for the left leader arm.",
            ),
            DeclareLaunchArgument(
                "right_can_interface",
                default_value="can1",
                description="SocketCAN interface for the right leader arm.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="200.0",
                description="Leader arm publication frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "default_mode",
                default_value="shadow",
                description="Startup mode for leader arms: shadow | passive.",
            ),
            DeclareLaunchArgument(
                "fallback_mode",
                default_value="shadow",
                description="Preempt release fallback mode for leader arms: shadow | passive.",
            ),
            left_leader,
            right_leader,
        ]
    )
