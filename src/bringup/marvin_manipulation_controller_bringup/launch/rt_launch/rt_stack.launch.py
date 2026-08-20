# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Unified RT-Host stack bringup for Marvin Bimanual + Dual Pika Grippers.

Aggregates RT Host bringup:
  1. Sub-launch 1: controller_bringup.launch.py (ros2_control, Marvin M6 controller, Pika grippers, safety guards)
  2. Sub-launch 2: pika_camera_bringup.launch.py (Dual Pika wrist RealSense D405 + Sunplus fisheye cameras)

On the physical Marvin RT Host, both Pika grippers and wrist perception cameras default to enabled.
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
        "marvin_manipulation_controller_bringup"
    )
    default_camera_cfg = os.path.join(
        bringup_share, "config", "camera", "marvin_cameras.yaml"
    )

    # 1. Real-time Controller & Hardware Interface Stack
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "rt_launch", "controller_bringup.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "cpu_affinity": LaunchConfiguration("cpu_affinity"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "load_pika_hardware": LaunchConfiguration("load_pika_hardware"),
            "load_left_pika_hardware": LaunchConfiguration("load_left_pika_hardware"),
            "load_right_pika_hardware": LaunchConfiguration(
                "load_right_pika_hardware"
            ),
            "left_gripper_serial_port": LaunchConfiguration("left_gripper_serial_port"),
            "right_gripper_serial_port": LaunchConfiguration(
                "right_gripper_serial_port"
            ),
            "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                "jtc_guard_heartbeat_timeout_s"
            ),
        }.items(),
    )

    # 2. Dual Pika Wrist Perception Cameras Stack (D405 + Fisheye per wrist)
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "rt_launch", "pika_camera_bringup.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("with_cameras")),
        launch_arguments={
            "camera_config": LaunchConfiguration("camera_config"),
            "with_left": LaunchConfiguration("with_left"),
            "with_right": LaunchConfiguration("with_right"),
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
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.19.0.191",
                description="Marvin CCS controller IP.",
            ),
            DeclareLaunchArgument(
                "load_pika_hardware",
                default_value="true",
                description="Load Pika ros2_control. Set false when grippers are absent or in workstation fake hardware.",
            ),
            DeclareLaunchArgument(
                "load_left_pika_hardware",
                default_value="true",
                description="Load left Pika ros2_control (requires load_pika_hardware:=true).",
            ),
            DeclareLaunchArgument(
                "load_right_pika_hardware",
                default_value="true",
                description="Load right Pika ros2_control (requires load_pika_hardware:=true).",
            ),
            DeclareLaunchArgument(
                "left_gripper_serial_port",
                default_value="/dev/ttyUSB1",
                description="Left Pika gripper serial (gamma default).",
            ),
            DeclareLaunchArgument(
                "right_gripper_serial_port",
                default_value="/dev/ttyUSB0",
                description="Right Pika gripper serial (gamma default).",
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),

            # Pika Wrist Perception Cameras parameters
            DeclareLaunchArgument(
                "with_cameras",
                default_value="true",
                description="Whether to launch Pika wrist perception cameras (D405 + Fisheye) on RT Host.",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=default_camera_cfg,
                description="Path to camera YAML configuration file.",
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

            controller,
            cameras,
        ]
    )
