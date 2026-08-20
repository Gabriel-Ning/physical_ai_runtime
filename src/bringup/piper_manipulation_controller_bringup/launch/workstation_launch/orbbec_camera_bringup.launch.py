# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the static Orbbec Femto Bolt / Gemini RGBD camera bringup.

Publishes to:
- /observation/static_orbbec/color/image_raw
- /observation/static_orbbec/depth/image_raw
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
    config_path = LaunchConfiguration("config").perform(context).strip()
    camera_name = LaunchConfiguration("camera_name").perform(context).strip()
    serial_number = LaunchConfiguration("serial_number").perform(context).strip()
    rgb_width = LaunchConfiguration("rgb_width").perform(context).strip()
    rgb_height = LaunchConfiguration("rgb_height").perform(context).strip()
    rgb_fps = LaunchConfiguration("rgb_fps").perform(context).strip()
    enable_pointcloud = LaunchConfiguration("enable_pointcloud").perform(context).strip().lower() in ("true", "1")

    # Try standard orbbec_camera package launch
    try:
        orbbec_share = get_package_share_directory("orbbec_camera")
        launch_path = os.path.join(orbbec_share, "launch", "gemini_330_series.launch.py")
        if os.path.isfile(launch_path):
            launch_args = {
                "camera_name": camera_name,
                "rgb_width": rgb_width,
                "rgb_height": rgb_height,
                "rgb_fps": rgb_fps,
                "enable_colored_point_cloud": "true" if enable_pointcloud else "false",
            }
            if serial_number:
                launch_args["serial_number"] = serial_number

            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                    launch_arguments=launch_args.items(),
                )
            ]
    except Exception:
        pass

    # Fallback to direct orbbec_camera_node using config file or params
    params = [config_path] if (config_path and os.path.isfile(config_path)) else []
    node_params = {
        "camera_name": camera_name,
        "rgb_width": int(rgb_width),
        "rgb_height": int(rgb_height),
        "rgb_fps": int(rgb_fps),
        "enable_color": True,
        "enable_depth": True,
    }
    if serial_number:
        node_params["serial_number"] = serial_number
    params.append(node_params)

    node = Node(
        package="orbbec_camera",
        executable="orbbec_camera_node",
        name=camera_name,
        namespace="observation",
        parameters=params,
        output="screen",
    )
    return [node]


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
                "camera_name",
                default_value="static_orbbec",
                description="Camera namespace and node name.",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="",
                description="Optional Orbbec camera serial number override.",
            ),
            DeclareLaunchArgument(
                "rgb_width",
                default_value="640",
                description="Color image stream width.",
            ),
            DeclareLaunchArgument(
                "rgb_height",
                default_value="480",
                description="Color image stream height.",
            ),
            DeclareLaunchArgument(
                "rgb_fps",
                default_value="30",
                description="Color image stream framerate (Hz).",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description="Whether to generate and publish colored 3D pointclouds.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
