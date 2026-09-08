# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""Unified RT-Host stack bringup for Marvin bimanual + dual Pika.

Dispatches by ``backend`` like Franka / Piper:
  - real / fake → ``controller_bringup.launch.py``
  - mujoco → ``mujoco_bringup.launch.py``

On real hardware, optionally primes ``left/right_arm_jtc`` once (CCS
``enter_position``). Wrist cameras stay on this host via
``pika_camera_bringup.launch.py`` when ``with_cameras:=true``.
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


def _module_key(context: LaunchContext) -> str:
    backend = LaunchConfiguration("backend").perform(context).lower().strip()
    use_sim_mujoco = (
        LaunchConfiguration("use_sim_mujoco").perform(context).lower().strip()
    )
    if backend == "mujoco" or use_sim_mujoco in ("true", "1", "yes"):
        return "mujoco"
    if backend in ("real", "fake"):
        return backend
    if backend:
        raise RuntimeError(
            f"'backend' must be real, fake, or mujoco, got '{backend}'"
        )
    fake = LaunchConfiguration("use_fake_hardware").perform(context).lower().strip()
    return "fake" if fake in ("true", "1", "yes") else "real"


def _include_control_stack(context: LaunchContext, *args, **kwargs):
    module_key = _module_key(context)
    launch_dir = os.path.join(
        get_package_share_directory("marvin_manipulation_rt_launch"), "launch"
    )
    if module_key == "mujoco":
        return [
            _include_launch(
                launch_dir,
                _CONTROL_MODULES[module_key],
                {
                    "task": LaunchConfiguration("task"),
                    "mujoco_model": LaunchConfiguration("mujoco_model"),
                    "headless": LaunchConfiguration("headless"),
                    "mujoco_plugins_yaml": LaunchConfiguration("mujoco_plugins_yaml"),
                    "load_pika_hardware": LaunchConfiguration("load_pika_hardware"),
                    "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                        "jtc_guard_heartbeat_timeout_s"
                    ),
                    "jtc_guard_cancel_response_timeout_s": LaunchConfiguration(
                        "jtc_guard_cancel_response_timeout_s"
                    ),
                },
            )
        ]
    actions = [
        _include_launch(
            launch_dir,
            _CONTROL_MODULES[module_key],
            {
                "use_fake_hardware": "true" if module_key == "fake" else "false",
                "backend": module_key,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "cpu_affinity": LaunchConfiguration("cpu_affinity"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "load_pika_hardware": LaunchConfiguration("load_pika_hardware"),
                "left_gripper_serial_port": LaunchConfiguration(
                    "left_gripper_serial_port"
                ),
                "right_gripper_serial_port": LaunchConfiguration(
                    "right_gripper_serial_port"
                ),
                "jtc_guard_heartbeat_timeout_s": LaunchConfiguration(
                    "jtc_guard_heartbeat_timeout_s"
                ),
                "jtc_guard_cancel_response_timeout_s": LaunchConfiguration(
                    "jtc_guard_cancel_response_timeout_s"
                ),
            },
        )
    ]
    prime = LaunchConfiguration("prime_arm_position").perform(context).strip().lower()
    if module_key == "real" and prime in ("true", "1", "yes"):
        actions.append(
            _include_launch(launch_dir, "prime_arm_position.launch.py", {})
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_manipulation_rt_launch")
    launch_dir = os.path.join(bringup_share, "launch")
    default_d405_cfg = os.path.join(bringup_share, "config", "camera", "pika_d405.yaml")
    default_fisheye_cfg = os.path.join(
        bringup_share, "config", "camera", "pika_fisheye.yaml"
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "pika_camera_bringup.launch.py")
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
                default_value="test_marvin",
                description=(
                    "test_marvin, a DuoBench task id under duobench_tasks, "
                    "or an absolute MJCF path (MuJoCo only)."
                ),
            ),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="",
                description="Absolute MJCF path. When empty, resolved from task:=.",
            ),
            DeclareLaunchArgument(
                "mujoco_plugins_yaml",
                default_value=os.path.join(bringup_share, "config", "mujoco_plugins.yaml"),
                description="CameraPlugin and SHM bridge configuration.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without a rendering window.",
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
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.19.0.191",
                description="Marvin CCS controller IP.",
            ),
            DeclareLaunchArgument(
                "load_pika_hardware",
                default_value="true",
                description=(
                    "Load both Pika grippers (fake or real follows backend). "
                    "Set false to omit both sides."
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
                "jtc_guard_cancel_response_timeout_s", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "prime_arm_position",
                default_value="true",
                description=(
                    "Real hardware only (ignored when backend:=fake): after "
                    "controller_bringup, activate left/right_arm_jtc once so "
                    "CCS position mode is entered before the first EM claim."
                ),
            ),
            DeclareLaunchArgument(
                "with_cameras",
                default_value="true",
                description=(
                    "Launch Pika wrist perception cameras (D405 + Fisheye) on RT Host."
                ),
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
            OpaqueFunction(function=_include_control_stack),
            cameras,
        ]
    )
