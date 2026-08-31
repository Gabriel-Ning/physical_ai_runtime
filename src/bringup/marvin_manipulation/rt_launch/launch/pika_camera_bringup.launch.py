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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configured_cameras(config_file: str) -> list[str]:
    with open(config_file, encoding="utf-8") as stream:
        configured = yaml.safe_load(stream) or {}
    if not isinstance(configured, dict):
        raise TypeError(f"Camera config must be a mapping: {config_file}")
    return list(configured)


def _launch_setup(context, *args, **kwargs):
    d405_config = LaunchConfiguration("d405_config").perform(context).strip()
    fisheye_config = LaunchConfiguration("fisheye_config").perform(context).strip()
    right_d405_delay = float(
        LaunchConfiguration("right_d405_delay").perform(context).strip() or "0"
    )

    actions = []

    def _d405(namespace: str) -> Node:
        return Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace=namespace,
            parameters=[d405_config],
            output="screen",
        )

    def _fisheye(namespace: str) -> Node:
        return Node(
            package="mjpeg_cam",
            executable="mjpeg_cam_node",
            name="camera",
            namespace=namespace,
            parameters=[fisheye_config],
            output="screen",
        )

    for namespace in _configured_cameras(fisheye_config):
        actions.append(_fisheye(namespace))

    for index, namespace in enumerate(_configured_cameras(d405_config)):
        camera = _d405(namespace)
        if index > 0 and right_d405_delay > 0.0:
            actions.append(
                TimerAction(period=index * right_d405_delay, actions=[camera])
            )
        else:
            actions.append(camera)

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
