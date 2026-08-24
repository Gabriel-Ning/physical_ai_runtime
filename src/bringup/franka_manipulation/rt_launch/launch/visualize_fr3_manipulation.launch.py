# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Description-only visualization for FR3 + Pika assembly tuning.

Starts robot_state_publisher, joint_state_publisher(_gui), and RViz2.
No ros2_control, no hardware, no controllers.

Use adaptor_* / gripper_* args to iterate flange and Pika mount poses.
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
    share = get_package_share_directory("franka_manipulation_rt_launch")
    xacro_path = f"{share}/urdf/fr3_manipulation.urdf.xacro"
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    # Only override poses when launch args are non-empty; otherwise keep
    # fr3_manipulation.urdf.xacro defaults (edit that file to tune assembly).
    mappings = {"ros2_control": "false"}
    for key in ("adaptor_xyz", "adaptor_rpy", "gripper_xyz", "gripper_rpy"):
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
            get_package_share_directory("franka_description"),
            "rviz",
            "visualize_franka.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "adaptor_xyz",
                default_value="",
                description="Override adaptor xyz on fr3_link8; "
                "empty keeps urdf/fr3_manipulation.urdf.xacro default.",
            ),
            DeclareLaunchArgument(
                "adaptor_rpy",
                default_value="",
                description="Override adaptor rpy on fr3_link8; "
                "empty keeps xacro default.",
            ),
            DeclareLaunchArgument(
                "gripper_xyz",
                default_value="",
                description="Override Pika xyz on pika_adaptor_link; "
                "empty keeps xacro default.",
            ),
            DeclareLaunchArgument(
                "gripper_rpy",
                default_value="",
                description="Override Pika rpy on pika_adaptor_link; "
                "empty keeps xacro default.",
            ),
            DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/franka_manipulation_description/joint_states",
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
