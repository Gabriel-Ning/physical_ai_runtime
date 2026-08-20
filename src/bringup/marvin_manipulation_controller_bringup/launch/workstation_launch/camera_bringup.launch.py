# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Perception camera bringup for Marvin Bimanual Robot.

Launches:
- Head Camera: /sensors/head_cam/color/image_raw
- Left Wrist Camera: /sensors/left_wrist_cam/color/image_raw
- Right Wrist Camera: /sensors/right_wrist_cam/color/image_raw

All physical parameters (resolution, rate, serials) default to config/camera/marvin_cameras.yaml.
"""

from __future__ import annotations

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config").perform(context).strip()
    enable_head = LaunchConfiguration("enable_head").perform(context).strip().lower() in ("true", "1")
    enable_left = LaunchConfiguration("enable_left").perform(context).strip().lower() in ("true", "1")
    enable_right = LaunchConfiguration("enable_right").perform(context).strip().lower() in ("true", "1")
    color_profile = LaunchConfiguration("color_profile").perform(context).strip()

    actions = []
    try:
        rs_share = get_package_share_directory("realsense2_camera")
        launch_file = os.path.join(rs_share, "launch", "rs_launch.py")

        if enable_head:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments={
                        "camera_name": "head_cam",
                        "camera_namespace": "sensors",
                        "rgb_camera.color_profile": color_profile,
                    }.items(),
                )
            )
        if enable_left:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments={
                        "camera_name": "left_wrist_cam",
                        "camera_namespace": "sensors",
                        "rgb_camera.color_profile": color_profile,
                    }.items(),
                )
            )
        if enable_right:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments={
                        "camera_name": "right_wrist_cam",
                        "camera_namespace": "sensors",
                        "rgb_camera.color_profile": color_profile,
                    }.items(),
                )
            )
        return actions
    except Exception:
        pass

    # Direct node fallbacks
    if enable_head:
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="head_cam",
                namespace="sensors",
                parameters=[{"rgb_camera.color_profile": color_profile}],
                output="screen",
            )
        )
    if enable_left:
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="left_wrist_cam",
                namespace="sensors",
                parameters=[{"rgb_camera.color_profile": color_profile}],
                output="screen",
            )
        )
    if enable_right:
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="right_wrist_cam",
                namespace="sensors",
                parameters=[{"rgb_camera.color_profile": color_profile}],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_controller_bringup")
    default_config = os.path.join(bringup_share, "config", "camera", "marvin_cameras.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to camera configuration YAML file.",
            ),
            DeclareLaunchArgument(
                "enable_head",
                default_value="true",
                description="Whether to start head camera.",
            ),
            DeclareLaunchArgument(
                "enable_left",
                default_value="true",
                description="Whether to start left wrist camera.",
            ),
            DeclareLaunchArgument(
                "enable_right",
                default_value="true",
                description="Whether to start right wrist camera.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640x480x30",
                description="Color stream resolution and rate (WIDTHxHEIGHTxFPS).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
