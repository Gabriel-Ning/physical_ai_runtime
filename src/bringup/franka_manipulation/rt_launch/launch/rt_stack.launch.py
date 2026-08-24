# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""Unified RT-Host stack bringup for Franka FR3 + Pika setup.

Always launches:
  - controller_bringup.launch.py (ros2_control, FCI, Pika gripper, safety guard)

Optionally launches (when ``with_cameras:=true``):
  - camera_bringup.launch.py     (RealSense D405 + Sunplus Fisheye cameras)
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory(
        "franka_manipulation_rt_launch"
    )

    # 1. Real-time Controller & Hardware Stack (Always launched)
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "controller_bringup.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "cpu_affinity": LaunchConfiguration("cpu_affinity"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "load_pika_hardware": LaunchConfiguration("load_pika_hardware"),
            "gripper_serial_port": LaunchConfiguration("gripper_serial_port"),
            "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                "jtc_guard_heartbeat_timeout_s"
            ),
        }.items(),
    )

    # 2. Camera Perception Stack (Optional, default false)
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "camera_bringup.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("with_cameras")),
        launch_arguments={
            "camera_config": LaunchConfiguration("camera_config"),
            "d405_serial": LaunchConfiguration("d405_serial"),
            "fisheye_device": LaunchConfiguration("fisheye_device"),
            "with_d405": LaunchConfiguration("with_d405"),
            "with_fisheye": LaunchConfiguration("with_fisheye"),
        }.items(),
    )

    return LaunchDescription(
        [
            # Controller & Hardware parameters
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node. Empty uses "
                    "RT_CM_CPU_AFFINITY from the cpu RT profile. Pass none to disable."
                ),
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.2.101"),
            DeclareLaunchArgument(
                "load_pika_hardware",
                default_value="true",
                description="Load Pika ros2_control. Set false when gripper is absent.",
            ),
            DeclareLaunchArgument(
                "gripper_serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device for the attached Pika gripper.",
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),

            # Optional Perception Cameras parameters
            DeclareLaunchArgument(
                "with_cameras",
                default_value="false",
                description="Whether to launch Pika wrist perception cameras (D405 + Fisheye).",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=os.path.join(
                    bringup_share, "config", "camera", "pika_cameras.yaml"
                ),
                description="Path to camera YAML configuration file.",
            ),
            DeclareLaunchArgument(
                "with_d405",
                default_value="true",
                description="Start D405 node when with_cameras is true.",
            ),
            DeclareLaunchArgument(
                "with_fisheye",
                default_value="true",
                description="Start Fisheye node when with_cameras is true.",
            ),
            DeclareLaunchArgument(
                "d405_serial",
                default_value="",
                description="Optional RealSense D405 serial override.",
            ),
            DeclareLaunchArgument(
                "fisheye_device",
                default_value="",
                description="Optional Fisheye device override.",
            ),

            controller,
            cameras,
        ]
    )
