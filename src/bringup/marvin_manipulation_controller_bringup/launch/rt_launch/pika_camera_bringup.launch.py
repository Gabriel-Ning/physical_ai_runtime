# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Perception camera bringup for Marvin bimanual + dual Pika grippers on RT Host.

The RealSense D405 + Sunplus DECXIN fisheye cameras on each Pika wrist are physically
connected to the RT Host via a combined cable harness.

Stream parameters are sourced from ``config/camera/marvin_cameras.yaml``.

Launches (per arm):
  1. RealSense D405 (wrist RGB-D): realsense2_camera
  2. Sunplus DECXIN fisheye (wrist RGB): usb_cam
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1")


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("camera_config").perform(context).strip()
    with_left = _truthy(LaunchConfiguration("with_left").perform(context))
    with_right = _truthy(LaunchConfiguration("with_right").perform(context))
    with_d405 = _truthy(LaunchConfiguration("with_d405").perform(context))
    with_fisheye = _truthy(LaunchConfiguration("with_fisheye").perform(context))

    actions = []

    def _add_arm(side: str) -> None:
        if with_d405:
            actions.append(
                Node(
                    package="realsense2_camera",
                    executable="realsense2_camera_node",
                    name="camera",
                    namespace=f"{side}_pika_d405",
                    parameters=[config_file],
                    output="screen",
                )
            )
        if with_fisheye:
            actions.append(
                Node(
                    package="usb_cam",
                    executable="usb_cam_node_exe",
                    name="camera",
                    namespace=f"{side}_pika_fisheye",
                    parameters=[config_file],
                    output="screen",
                )
            )

    if with_left:
        _add_arm("left")
    if with_right:
        _add_arm("right")

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_controller_bringup")
    default_config = os.path.join(bringup_share, "config", "camera", "marvin_cameras.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=default_config,
                description="ROS params YAML for D405 + fisheye nodes.",
            ),
            DeclareLaunchArgument(
                "with_left",
                default_value="true",
                description="Start left Pika D405 + fisheye.",
            ),
            DeclareLaunchArgument(
                "with_right",
                default_value="true",
                description="Start right Pika D405 + fisheye.",
            ),
            DeclareLaunchArgument(
                "with_d405",
                default_value="true",
                description="Start RealSense D405 nodes when an arm is enabled.",
            ),
            DeclareLaunchArgument(
                "with_fisheye",
                default_value="true",
                description="Start Sunplus fisheye nodes when an arm is enabled.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
