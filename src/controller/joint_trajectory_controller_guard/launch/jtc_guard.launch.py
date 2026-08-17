# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("action_name"),
            DeclareLaunchArgument("heartbeat_topic", default_value="~/heartbeat"),
            DeclareLaunchArgument("heartbeat_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("cancel_response_timeout_s", default_value="0.5"),
            Node(
                package="joint_trajectory_controller_guard",
                executable="jtc_guard_node",
                name="jtc_guard",
                output="screen",
                parameters=[
                    {
                        "action_name": LaunchConfiguration("action_name"),
                        "heartbeat_topic": LaunchConfiguration("heartbeat_topic"),
                        "heartbeat_timeout_s": ParameterValue(
                            LaunchConfiguration("heartbeat_timeout_s"),
                            value_type=float,
                        ),
                        "cancel_response_timeout_s": ParameterValue(
                            LaunchConfiguration("cancel_response_timeout_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
