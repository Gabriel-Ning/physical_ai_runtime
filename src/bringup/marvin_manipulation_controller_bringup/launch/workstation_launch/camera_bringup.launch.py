# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Workstation RealSense bringup for Marvin bimanual cell.

Wrist Pika cameras stay on the RT host
(``launch/rt_launch/pika_camera_bringup.launch.py``).

This launch starts the external / third-person RealSense attached to the
workstation (site default: D430 ``_309422070502`` in
``config/camera/workstation_realsense.yaml``).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("camera_config").perform(context).strip()
    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace="workstation_realsense",
            parameters=[config_file],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_controller_bringup")
    default_config = os.path.join(
        bringup_share, "config", "camera", "workstation_realsense.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=default_config,
                description="ROS params YAML for the workstation RealSense.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
