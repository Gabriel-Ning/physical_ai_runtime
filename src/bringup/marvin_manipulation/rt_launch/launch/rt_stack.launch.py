# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Unified RT-Host stack bringup for Marvin Bimanual + Dual Pika Grippers.

Aggregates RT Host bringup:
  1. Sub-launch 1: controller_bringup.launch.py (ros2_control, Marvin M6 controller, Pika grippers, safety guards)
  2. On real hardware: prime_arm_position.launch.py (activate left/right_arm_jtc once)
  3. Sub-launch 2: pika_camera_bringup.launch.py (Dual Pika wrist RealSense D405 + Sunplus fisheye cameras)

On the physical Marvin RT Host, both Pika grippers and wrist perception cameras default to enabled.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _prime_actions(context: LaunchContext):
    """Real hardware only: after controller_bringup, activate JTCs once."""

    def _as_bool(name: str) -> bool:
        return LaunchConfiguration(name).perform(context).strip().lower() in (
            "true",
            "1",
        )

    if _as_bool("use_fake_hardware") or not _as_bool("prime_arm_position"):
        return []

    bringup_share = get_package_share_directory("marvin_manipulation_rt_launch")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "prime_arm_position.launch.py")
            )
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_rt_launch")
    default_d405_cfg = os.path.join(bringup_share, "config", "camera", "pika_d405.yaml")
    default_fisheye_cfg = os.path.join(
        bringup_share, "config", "camera", "pika_fisheye.yaml"
    )

    # 1. Real-time Controller & Hardware Interface Stack
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
            os.path.join(bringup_share, "launch", "pika_camera_bringup.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("with_cameras")),
        launch_arguments={
            "d405_config": LaunchConfiguration("d405_config"),
            "fisheye_config": LaunchConfiguration("fisheye_config"),
            "right_d405_delay": LaunchConfiguration("right_d405_delay"),
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
                description=(
                    "Load both Pika grippers (fake or real follows "
                    "use_fake_hardware). Set false to omit both sides."
                ),
            ),
            DeclareLaunchArgument(
                "left_gripper_serial_port",
                default_value="/dev/pika_left_gripper",
                description="Left Pika gripper serial (udev /dev/pika_left_gripper).",
            ),
            DeclareLaunchArgument(
                "right_gripper_serial_port",
                default_value="/dev/pika_right_gripper",
                description="Right Pika gripper serial (udev /dev/pika_right_gripper).",
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "prime_arm_position",
                default_value="true",
                description=(
                    "Real hardware only (ignored when use_fake_hardware:=true): "
                    "after controller_bringup, activate left/right_arm_jtc once "
                    "so CCS position mode is entered before the first EM claim."
                ),
            ),
            # Pika Wrist Perception Cameras parameters
            DeclareLaunchArgument(
                "with_cameras",
                default_value="true",
                description="Whether to launch Pika wrist perception cameras (D405 + Fisheye) on RT Host.",
            ),
            DeclareLaunchArgument(
                "d405_config",
                default_value=default_d405_cfg,
                description="Pika wrist D405 ROS params YAML.",
            ),
            DeclareLaunchArgument(
                "fisheye_config",
                default_value=default_fisheye_cfg,
                description="Pika wrist fisheye (mjpeg_cam) ROS params YAML.",
            ),
            DeclareLaunchArgument(
                "right_d405_delay",
                default_value="2.0",
                description=(
                    "Seconds to wait after left D405 before starting right D405."
                ),
            ),
            controller,
            OpaqueFunction(function=_prime_actions),
            cameras,
        ]
    )
