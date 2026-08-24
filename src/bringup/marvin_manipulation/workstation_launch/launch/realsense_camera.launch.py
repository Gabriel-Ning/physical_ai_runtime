"""Launch Marvin workstation RealSense cameras: head and third-person D435I."""

from __future__ import annotations

import os

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

    def _realsense(namespace: str) -> Node:
        return Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace=namespace,
            parameters=[config_file],
            output="screen",
        )

    head = _realsense("head_d435")
    third_person = _realsense("third_person_d435")
    if second_delay > 0.0:
        return [head, TimerAction(period=second_delay, actions=[third_person])]
    return [head, third_person]


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
                description="Seconds to wait after head D435 before starting third-person D435.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
