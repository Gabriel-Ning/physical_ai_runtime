# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch one mjpeg_cam_node from the package template (or a site YAML)."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    params = [LaunchConfiguration("params_file").perform(context).strip()]
    overrides = {}
    for key in ("video_device", "frame_id", "camera_name", "compressed_topic"):
        value = LaunchConfiguration(key).perform(context).strip()
        if value:
            overrides[key] = value
    if overrides:
        params.append(overrides)

    return [
        Node(
            package="mjpeg_cam",
            executable="mjpeg_cam_node",
            name=LaunchConfiguration("name").perform(context).strip() or "camera",
            namespace=LaunchConfiguration("namespace").perform(context).strip(),
            parameters=params,
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("mjpeg_cam"), "config", "mjpeg_cam.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS params YAML. Default is this package's template.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Node namespace (empty = none).",
            ),
            DeclareLaunchArgument(
                "name",
                default_value="camera",
                description="Node name.",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="",
                description="Optional V4L2 device override. Empty uses params_file.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional header.frame_id override. Empty uses params_file.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="",
                description="Optional camera_name override. Empty uses params_file.",
            ),
            DeclareLaunchArgument(
                "compressed_topic",
                default_value="",
                description="Optional topic override. Empty uses params_file.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
