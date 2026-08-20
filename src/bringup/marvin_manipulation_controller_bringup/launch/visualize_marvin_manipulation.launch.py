# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Description-only visualization for Marvin + dual Pika assembly tuning.

Starts robot_state_publisher, joint_state_publisher(_gui), and RViz2.
No ros2_control, no hardware, no controllers.
"""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
import sys
import xacro


def _spawn_publishers(context: LaunchContext):
    share = get_package_share_directory("marvin_manipulation_controller_bringup")
    xacro_path = f"{share}/urdf/marvin_manipulation.urdf.xacro"
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    mappings = {"ros2_control": "false"}
    for key in (
        "connected_to",
        "xyz",
        "rpy",
        "mounts_file",
        "left_base_xyz",
        "left_base_rpy",
        "right_base_xyz",
        "right_base_rpy",
    ):
        value = context.perform_substitution(LaunchConfiguration(key)).strip()
        if value:
            mappings[key] = value

    robot_description = xacro.process_file(
        xacro_path, mappings=mappings
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", joint_states_topic)],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(use_joint_state_gui),
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", joint_states_topic)],
            # python_qt_binding expects CONDA_PREFIX, while pixi exposes the
            # environment through sys.prefix instead.
            additional_env={"CONDA_PREFIX": os.environ.get("CONDA_PREFIX", sys.prefix)},
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(use_joint_state_gui),
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", joint_states_topic)],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution(
        [
            get_package_share_directory("marvin_description"),
            "rviz",
            "visualize_marvin.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "connected_to",
                default_value="",
                description="Override connected_to; empty keeps xacro default.",
            ),
            DeclareLaunchArgument(
                "xyz",
                default_value="",
                description="Override stand xyz; empty keeps xacro default.",
            ),
            DeclareLaunchArgument(
                "rpy",
                default_value="",
                description="Override stand rpy; empty keeps xacro default.",
            ),
            DeclareLaunchArgument(
                "mounts_file",
                default_value="",
                description="Override arm mounts YAML; empty keeps xacro default.",
            ),
            DeclareLaunchArgument(
                "left_base_xyz",
                default_value="",
                description="Override left arm mount xyz; empty keeps mounts YAML.",
            ),
            DeclareLaunchArgument(
                "left_base_rpy",
                default_value="",
                description="Override left arm mount rpy; empty keeps mounts YAML.",
            ),
            DeclareLaunchArgument(
                "right_base_xyz",
                default_value="",
                description="Override right arm mount xyz; empty keeps mounts YAML.",
            ),
            DeclareLaunchArgument(
                "right_base_rpy",
                default_value="",
                description="Override right arm mount rpy; empty keeps mounts YAML.",
            ),
            DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/marvin_manipulation_description/joint_states",
            ),
            OpaqueFunction(function=_spawn_publishers),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
