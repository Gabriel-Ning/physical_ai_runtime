"""Launch Marvin workstation RealSense cameras: head and third-person D435I."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("camera_config").perform(context).strip()
    second_delay = float(
        LaunchConfiguration("second_camera_delay").perform(context).strip() or "0"
    )

    with open(config_file, encoding="utf-8") as stream:
        configured_cameras = yaml.safe_load(stream) or {}
    if not isinstance(configured_cameras, dict) or not configured_cameras:
        raise ValueError(f"No camera namespaces declared in {config_file}")

    def _realsense(namespace: str) -> Node:
        return Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace=namespace,
            parameters=[config_file],
            output="screen",
        )

    actions = []
    for index, namespace in enumerate(configured_cameras):
        camera = _realsense(namespace)
        if index > 0 and second_delay > 0.0:
            actions.append(TimerAction(period=index * second_delay, actions=[camera]))
        else:
            actions.append(camera)
    return actions


def generate_launch_description() -> LaunchDescription:
    robot_share = get_package_share_directory(
        "marvin_manipulation_workstation_launch"
    )
    default_config = os.path.join(
        robot_share, "config", "camera", "workstation_realsense.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_config", default_value=default_config),
            DeclareLaunchArgument(
                "second_camera_delay",
                default_value="2.0",
                description="Seconds to stagger each additional configured camera.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
