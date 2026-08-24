# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Perception camera bringup for Marvin bimanual + dual Pika grippers on RT Host.

D405 and fisheye share a Pika cable physically, but configs are per camera
model so they can be reused:

  config/camera/pika_d405.yaml
  config/camera/pika_fisheye.yaml

Launches:
  1. Both fisheye nodes immediately (mjpeg_cam, original JPEG)
  2. Left D405 immediately, right D405 after right_d405_delay
     (two D405s claiming USB3 at once can return RS2_USB_STATUS_BUSY)
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1")


def _launch_setup(context, *args, **kwargs):
    d405_config = LaunchConfiguration("d405_config").perform(context).strip()
    fisheye_config = LaunchConfiguration("fisheye_config").perform(context).strip()
    with_left = _truthy(LaunchConfiguration("with_left").perform(context))
    with_right = _truthy(LaunchConfiguration("with_right").perform(context))
    with_d405 = _truthy(LaunchConfiguration("with_d405").perform(context))
    with_fisheye = _truthy(LaunchConfiguration("with_fisheye").perform(context))
    right_d405_delay = float(
        LaunchConfiguration("right_d405_delay").perform(context).strip() or "0"
    )

    actions = []

    def _d405(side: str) -> Node:
        return Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace=f"{side}_pika_d405",
            parameters=[d405_config],
            output="screen",
        )

    def _fisheye(side: str) -> Node:
        return Node(
            package="mjpeg_cam",
            executable="mjpeg_cam_node",
            name="camera",
            namespace=f"{side}_pika_fisheye",
            parameters=[
                fisheye_config,
                {
                    "video_device": f"/dev/pika_{side}_fisheye",
                    "frame_id": f"{side}_pika_fisheye_link",
                    "camera_name": f"{side}_pika_fisheye",
                    "compressed_topic": "image/compressed",
                    "format": "jpeg",
                },
            ],
            output="screen",
        )

    if with_fisheye:
        if with_left:
            actions.append(_fisheye("left"))
        if with_right:
            actions.append(_fisheye("right"))

    if with_d405:
        if with_left:
            actions.append(_d405("left"))
        if with_right:
            right = _d405("right")
            if with_left and right_d405_delay > 0.0:
                actions.append(TimerAction(period=right_d405_delay, actions=[right]))
            else:
                actions.append(right)

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_rt_launch")
    default_d405 = os.path.join(bringup_share, "config", "camera", "pika_d405.yaml")
    default_fisheye = os.path.join(
        bringup_share, "config", "camera", "pika_fisheye.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "d405_config",
                default_value=default_d405,
                description="ROS params YAML for Pika wrist D405 nodes.",
            ),
            DeclareLaunchArgument(
                "fisheye_config",
                default_value=default_fisheye,
                description="ROS params YAML for Pika wrist fisheye (mjpeg_cam) nodes.",
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
            DeclareLaunchArgument(
                "right_d405_delay",
                default_value="2.0",
                description=(
                    "Seconds to wait after left D405 before starting right D405. "
                    "Ignored when only one D405 is enabled. Set 0 to start together."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
