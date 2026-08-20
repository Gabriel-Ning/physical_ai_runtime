# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the dual RealSense D435 wrist cameras bringup.

Publishes to:
- /observation/left_hand_realsense/color/image_raw
- /observation/right_hand_realsense/color/image_raw
"""

from __future__ import annotations

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    enable_left = LaunchConfiguration("enable_left").perform(context).strip().lower() in ("true", "1")
    enable_right = LaunchConfiguration("enable_right").perform(context).strip().lower() in ("true", "1")
    left_serial = LaunchConfiguration("left_serial_no").perform(context).strip()
    right_serial = LaunchConfiguration("right_serial_no").perform(context).strip()
    color_profile = LaunchConfiguration("color_profile").perform(context).strip()

    actions = []
    try:
        rs_share = get_package_share_directory("realsense2_camera")
        launch_file = os.path.join(rs_share, "launch", "rs_launch.py")

        if enable_left:
            left_args = {
                "camera_name": "left_hand_realsense",
                "camera_namespace": "observation",
                "rgb_camera.color_profile": color_profile,
            }
            if left_serial:
                left_args["serial_no"] = left_serial
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments=left_args.items(),
                )
            )

        if enable_right:
            right_args = {
                "camera_name": "right_hand_realsense",
                "camera_namespace": "observation",
                "rgb_camera.color_profile": color_profile,
            }
            if right_serial:
                right_args["serial_no"] = right_serial
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments=right_args.items(),
                )
            )
        return actions
    except Exception:
        pass

    # Fallback to direct nodes if launch package unavailable
    if enable_left:
        params = {"rgb_camera.color_profile": color_profile}
        if left_serial:
            params["serial_no"] = left_serial
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="left_hand_realsense",
                namespace="observation",
                parameters=[params],
                output="screen",
            )
        )
    if enable_right:
        params = {"rgb_camera.color_profile": color_profile}
        if right_serial:
            params["serial_no"] = right_serial
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="right_hand_realsense",
                namespace="observation",
                parameters=[params],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("piper_manipulation_controller_bringup")
    default_config = os.path.join(bringup_share, "config", "camera", "piper_cameras.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to camera configuration YAML file.",
            ),
            DeclareLaunchArgument(
                "enable_left",
                default_value="true",
                description="Whether to start left wrist RealSense camera.",
            ),
            DeclareLaunchArgument(
                "enable_right",
                default_value="true",
                description="Whether to start right wrist RealSense camera.",
            ),
            DeclareLaunchArgument(
                "left_serial_no",
                default_value="",
                description="Optional serial number for left wrist RealSense.",
            ),
            DeclareLaunchArgument(
                "right_serial_no",
                default_value="",
                description="Optional serial number for right wrist RealSense.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640x480x30",
                description="Color stream resolution and rate (WIDTHxHEIGHTxFPS).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
