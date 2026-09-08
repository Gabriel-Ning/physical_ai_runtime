# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""Unified RT-Host stack bringup for Franka FR3 + Pika setup.

Dispatches by ``backend`` to a sub-launch module via IncludeLaunchDescription:
  - real / fake → ``controller_bringup.launch.py``
  - mujoco → ``mujoco_bringup.launch.py``

Optionally includes ``camera_bringup.launch.py`` when ``with_cameras:=true``.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Sub-launch modules under share/.../launch/, selected by backend.
_CONTROL_MODULES = {
    "mujoco": "mujoco_bringup.launch.py",
    "real": "controller_bringup.launch.py",
    "fake": "controller_bringup.launch.py",
}


def _include_launch(launch_dir: str, module: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, module)),
        launch_arguments=arguments.items(),
    )


def _include_control_stack(context, *args, **kwargs):
    backend = LaunchConfiguration("backend").perform(context).lower().strip()
    use_sim_mujoco = (
        LaunchConfiguration("use_sim_mujoco").perform(context).lower().strip()
    )
    if backend == "mujoco" or use_sim_mujoco in ("true", "1", "yes"):
        module_key = "mujoco"
    elif backend in ("real", "fake"):
        module_key = backend
    elif backend:
        raise RuntimeError(
            f"'backend' must be real, fake, or mujoco, got '{backend}'"
        )
    else:
        fake = LaunchConfiguration("use_fake_hardware").perform(context).lower().strip()
        module_key = "fake" if fake in ("true", "1", "yes") else "real"

    launch_dir = os.path.join(
        get_package_share_directory("franka_manipulation_rt_launch"), "launch"
    )
    module = _CONTROL_MODULES[module_key]

    if module_key == "mujoco":
        return [
            _include_launch(
                launch_dir,
                module,
                {
                    "task": LaunchConfiguration("task"),
                    "headless": LaunchConfiguration("headless"),
                    "mujoco_plugins_yaml": LaunchConfiguration("mujoco_plugins_yaml"),
                    "load_pika_hardware": LaunchConfiguration("load_pika_hardware"),
                    "gripper_serial_port": LaunchConfiguration("gripper_serial_port"),
                    "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                        "jtc_guard_heartbeat_timeout_s"
                    ),
                    "jtc_guard_cancel_response_timeout_s": LaunchConfiguration(
                        "jtc_guard_cancel_response_timeout_s"
                    ),
                },
            )
        ]

    return [
        _include_launch(
            launch_dir,
            module,
            {
                "use_fake_hardware": "true" if module_key == "fake" else "false",
                "backend": module_key,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "cpu_affinity": LaunchConfiguration("cpu_affinity"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "load_pika_hardware": LaunchConfiguration("load_pika_hardware"),
                "gripper_serial_port": LaunchConfiguration("gripper_serial_port"),
                "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                    "jtc_guard_heartbeat_timeout_s"
                ),
                "jtc_guard_cancel_response_timeout_s": LaunchConfiguration(
                    "jtc_guard_cancel_response_timeout_s"
                ),
            },
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("franka_manipulation_rt_launch")
    launch_dir = os.path.join(bringup_share, "launch")

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "camera_bringup.launch.py")
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
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument(
                "backend",
                default_value="",
                description="real, fake, or mujoco. Empty falls back to use_fake_hardware.",
            ),
            DeclareLaunchArgument(
                "use_sim_mujoco",
                default_value="false",
                description="Compat alias for backend:=mujoco.",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="",
                description=(
                    "LIBERO task name or absolute MJCF path (MuJoCo only). "
                    "Empty uses mujoco_bringup's default LIBERO task."
                ),
            ),
            DeclareLaunchArgument(
                "mujoco_plugins_yaml",
                default_value=os.path.join(bringup_share, "config", "mujoco_plugins.yaml"),
                description="Shared CameraPlugin and SHM bridge configuration.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo simulation headless without rendering window.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node. Empty uses "
                    "RT_CM_CPU_AFFINITY from the RT host profile. Pass none to disable."
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
                default_value="/dev/pika_left_gripper",
                description="Serial device for the attached Pika gripper.",
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "jtc_guard_cancel_response_timeout_s", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "with_cameras",
                default_value="false",
                description=(
                    "Launch Pika wrist perception cameras (D405 + Fisheye) on RT host."
                ),
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
            OpaqueFunction(function=_include_control_stack),
            cameras,
        ]
    )
