# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""Perception camera bringup for Franka FR3 + Pika Gripper setup.

Loads camera configurations (resolution, framerate, stream formats) from
``config/camera/pika_cameras.yaml``.

Launches:
  1. RealSense D405 (Wrist Depth/RGB): realsense2_camera
  2. Sunplus DECXIN Fisheye (Wrist RGB): mjpeg_cam (original JPEG)

Frames match URDF pika_d405_link and pika_fisheye_link.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("camera_config").perform(context).strip()
    with_d405 = LaunchConfiguration("with_d405").perform(
        context
    ).strip().lower() in ("true", "1")
    with_fisheye = LaunchConfiguration("with_fisheye").perform(
        context
    ).strip().lower() in ("true", "1")
    d405_serial_arg = LaunchConfiguration("d405_serial").perform(context).strip()
    fisheye_device_arg = (
        LaunchConfiguration("fisheye_device").perform(context).strip()
    )

    actions = []

    # 1. RealSense D405 Node
    if with_d405:
        d405_params = [config_file]
        # CLI overrides if provided and non-empty
        overrides = {}
        if d405_serial_arg:
            overrides["serial_no"] = d405_serial_arg
        if overrides:
            d405_params.append(overrides)

        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="camera",
                namespace="pika_d405",
                parameters=d405_params,
                output="screen",
            )
        )

    # 2. Sunplus Fisheye Camera Node
    if with_fisheye:
        fisheye_params = [config_file]
        overrides = {}
        if fisheye_device_arg:
            overrides["video_device"] = fisheye_device_arg
        if overrides:
            fisheye_params.append(overrides)

        actions.append(
            Node(
                package="mjpeg_cam",
                executable="mjpeg_cam_node",
                name="camera",
                namespace="pika_fisheye",
                parameters=fisheye_params,
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("franka_manipulation_rt_launch")
    default_config = PathJoinSubstitution(
        [bringup_share, "config", "camera", "pika_cameras.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=default_config,
                description="Path to camera YAML configuration file.",
            ),
            DeclareLaunchArgument(
                "with_d405",
                default_value="true",
                description="Whether to start the RealSense D405 camera node.",
            ),
            DeclareLaunchArgument(
                "with_fisheye",
                default_value="true",
                description="Whether to start the Sunplus Fisheye USB camera node.",
            ),
            DeclareLaunchArgument(
                "d405_serial",
                default_value="",
                description=(
                    "Optional RealSense D405 serial override. "
                    "Empty uses value from camera_config."
                ),
            ),
            DeclareLaunchArgument(
                "fisheye_device",
                default_value="",
                description=(
                    "Optional Fisheye device node override (e.g. /dev/video12). "
                    "Empty uses value from camera_config."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
