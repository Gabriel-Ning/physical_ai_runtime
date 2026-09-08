# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""MuJoCo simulation bringup for Marvin bimanual + dual Pika.

Owns the sim control path:
  task → MJCF resolution, MujocoSystemInterface URDF, mujoco ros2_control_node,
  inactive route controllers, optional mujoco_image_bridge (SHM → Image).

Real / fake hardware: ``controller_bringup.launch.py``.
"""

from __future__ import annotations

import os
import shlex

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_DEFAULT_TASK = "test_marvin"


def _resolve_mujoco_model(task: str) -> str:
    """Map task to an absolute MJCF path.

    Order: absolute path, this package's ``mjcf/robot``, then
    ``duobench_tasks`` (DuoBench IDs on Marvin). LIBERO / RoboTwin names
    are rejected — those pair with Franka / Piper.
    """
    task_name = (task or "").strip() or _DEFAULT_TASK
    if os.path.isabs(task_name) and os.path.exists(task_name):
        return task_name

    share = get_package_share_directory("marvin_manipulation_rt_launch")
    candidates = [
        os.path.join(share, "mjcf", "robot", f"{task_name}.xml"),
        os.path.join(share, "mjcf", f"{task_name}.xml"),
    ]
    try:
        duo_share = get_package_share_directory("duobench_tasks")
        candidates.extend(
            [
                os.path.join(duo_share, "mjcf", f"{task_name}.xml"),
                os.path.join(duo_share, "mjcf", "tasks", f"{task_name}.xml"),
            ]
        )
    except Exception:
        pass
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    for suite, robot in (("libero_tasks", "Franka"), ("robotwin_tasks", "Piper")):
        try:
            other = get_package_share_directory(suite)
        except Exception:
            continue
        other_xml = os.path.join(other, "mjcf", f"{task_name}.xml")
        if os.path.exists(other_xml):
            raise RuntimeError(
                f"Task '{task_name}' belongs to {suite} ({robot}). "
                f"Marvin uses local test_marvin or duobench_tasks. "
                f"Use rt-franka / rt-piper for that suite."
            )
    raise RuntimeError(
        f"MuJoCo task MJCF not found for task={task_name!r}. Tried: {candidates}"
    )


def _camera_bridge_parameters(config_path: str) -> dict | None:
    """SHM output requires an image bridge (same contract as Franka / Piper)."""
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    camera = config["/**"]["ros__parameters"]["mujoco_plugins"]["mujoco_camera_plugin"]
    output = camera.get("output", "ros")
    if output == "ros":
        return None
    if output not in ("shm", "both"):
        raise ValueError("Camera output must be ros, shm, or both")
    if camera.get("default_policy") != "disabled":
        raise ValueError(
            "SHM bringup requires default_policy: disabled so all active cameras are bridged"
        )
    names = [
        name
        for name, entry in camera.items()
        if isinstance(entry, dict) and entry.get("policy") in ("streaming", "polled")
    ]
    return {
        "camera_names": names,
        "shm_prefix": camera.get("shm_prefix", "/pai_mj_cam_"),
        "poll_hz": 120.0,
        "use_sim_time": False,
    }


def _camera_bridge_environment() -> dict[str, str]:
    """Bridge-only Cyclone AllowMulticast=spdp; keep inherited NIC/peer settings."""
    inherited = os.environ.get("CYCLONEDDS_URI", "").strip()
    override = (
        "<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast>"
        "</General></Domain></CycloneDDS>"
    )
    return {
        "CYCLONEDDS_URI": os.environ.get(
            "MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI",
            ",".join(part for part in (inherited, override) if part),
        )
    }


def _after(event, actions, stage):
    if event.returncode == 0:
        return actions
    return [EmitEvent(event=Shutdown(reason=f"{stage} failed ({event.returncode})"))]


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("marvin_manipulation_rt_launch")

    headless = LaunchConfiguration("headless").perform(context).lower().strip() in (
        "true",
        "1",
        "yes",
    )
    load_pika_hardware = LaunchConfiguration("load_pika_hardware").perform(
        context
    ).strip().lower() in ("true", "1", "yes")

    mujoco_model_path = LaunchConfiguration("mujoco_model").perform(context).strip()
    if not mujoco_model_path:
        mujoco_model_path = _resolve_mujoco_model(
            LaunchConfiguration("task").perform(context)
        )

    description = xacro.process_file(
        os.path.join(bringup_share, "urdf", "marvin_manipulation.urdf.xacro"),
        mappings={
            "ros2_control": "true",
            "connected_to": LaunchConfiguration("connected_to").perform(context),
            "xyz": LaunchConfiguration("xyz").perform(context),
            "rpy": LaunchConfiguration("rpy").perform(context),
            "mounts_file": LaunchConfiguration("mounts_file").perform(context),
            "use_fake_hardware": "false",
            "use_sim_mujoco": "true",
            "headless": "true" if headless else "false",
            "mujoco_model": mujoco_model_path,
            "load_pika_hardware": "true" if load_pika_hardware else "false",
        },
    ).toprettyxml(indent="  ")

    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    plugins_yaml = LaunchConfiguration("mujoco_plugins_yaml").perform(context)
    robot_description = {
        "robot_description": ParameterValue(description, value_type=str)
    }
    params = [
        robot_description,
        controllers_yaml,
        {"use_sim_time": True},
        {"headless": headless},
    ]
    bridge_parameters = None
    if os.path.exists(plugins_yaml):
        params.append(plugins_yaml)
        bridge_parameters = _camera_bridge_parameters(plugins_yaml)

    heartbeat_timeout_s = float(
        LaunchConfiguration("jtc_guard_heartbeat_timeout_s").perform(context)
    )
    cancel_response_timeout_s = float(
        LaunchConfiguration("jtc_guard_cancel_response_timeout_s").perform(context)
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        # use_sim_time false: publish /robot_description before /clock exists.
        # Jazzy CM waits on that topic; waiting on sim time deadlocks the viewer.
        parameters=[{**robot_description, "use_sim_time": False}],
    )
    cm = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=params,
    )
    jsb = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                "ros2 run controller_manager spawner joint_state_broadcaster "
                "--controller-manager /controller_manager --controller-manager-timeout 30 "
                "|| ros2 control set_controller_state joint_state_broadcaster active 2>/dev/null "
                "|| true"
            ),
        ],
        name="joint_state_broadcaster_spawner",
        output="screen",
    )

    route_controllers = [
        f"{side}_arm_{route}"
        for side in ("left", "right")
        for route in ("jspc", "tskpc", "jtc")
    ]
    remaps = [
        (
            f"--remap {side}_arm_jtc/follow_joint_trajectory:="
            f"/execution/{side}_arm/follow_joint_trajectory"
        )
        for side in ("left", "right")
    ]
    if load_pika_hardware:
        route_controllers.extend(
            [
                "left_pika_gripper_fwd",
                "left_pika_gripper_action",
                "right_pika_gripper_fwd",
                "right_pika_gripper_action",
            ]
        )
        remaps.extend(
            [
                "--remap left_pika_gripper_fwd/commands:=/execution/left_gripper/joint_reference",
                "--remap left_pika_gripper_action/gripper_cmd:=/execution/left_gripper/gripper_command",
                "--remap right_pika_gripper_fwd/commands:=/execution/right_gripper/joint_reference",
                "--remap right_pika_gripper_action/gripper_cmd:=/execution/right_gripper/gripper_command",
            ]
        )

    controller_remaps = " ".join(remaps)
    route_args = [
        *route_controllers,
        "--inactive",
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "30",
    ]
    if controller_remaps:
        route_args.extend(["--controller-ros-args", controller_remaps])
    routes = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "ros2 run controller_manager spawner "
            f"{' '.join(shlex.quote(arg) for arg in route_args)} || true",
        ],
        name="routes_spawner",
        output="screen",
    )

    actions = [
        rsp,
        cm,
        *[
            Node(
                package="joint_trajectory_controller_guard",
                executable="jtc_guard_node",
                name=f"{side}_arm_jtc_guard",
                parameters=[
                    {
                        "action_name": (
                            f"/execution/{side}_arm/follow_joint_trajectory"
                        ),
                        "heartbeat_topic": (
                            f"/execution/{side}_arm/trajectory_guard_heartbeat"
                        ),
                        "heartbeat_timeout_s": heartbeat_timeout_s,
                        "cancel_response_timeout_s": cancel_response_timeout_s,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            )
            for side in ("left", "right")
        ],
    ]
    if bridge_parameters is not None:
        actions.append(
            Node(
                package="mujoco_ros2_control_plugins",
                executable="mujoco_image_bridge",
                name="mujoco_image_bridge",
                output="screen",
                parameters=[bridge_parameters],
                additional_env=_camera_bridge_environment(),
            )
        )
    actions.extend(
        [
            jsb,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=jsb,
                    on_exit=lambda event, context: _after(
                        event, [routes], "joint-state broadcaster"
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=routes,
                    on_exit=lambda event, context: _after(
                        event, [], "route controllers"
                    ),
                )
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("marvin_manipulation_rt_launch")
    marvin_share = get_package_share_directory("marvin_description")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task",
                default_value=_DEFAULT_TASK,
                description=(
                    "test_marvin, a DuoBench task id (bin_sort, transfer_cube, "
                    "...), or an absolute .xml path."
                ),
            ),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="",
                description="Absolute MJCF path. When empty, resolved from task:=.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without a rendering window.",
            ),
            DeclareLaunchArgument(
                "mujoco_plugins_yaml",
                default_value=os.path.join(share, "config", "mujoco_plugins.yaml"),
                description="CameraPlugin and SHM bridge configuration.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [share, "config", "controller", "controllers.yaml"]
                ),
            ),
            DeclareLaunchArgument("connected_to", default_value="world"),
            DeclareLaunchArgument("xyz", default_value="0 0 0"),
            DeclareLaunchArgument("rpy", default_value="0 0 0"),
            DeclareLaunchArgument(
                "mounts_file",
                default_value=PathJoinSubstitution(
                    [marvin_share, "config", "arm_mounts.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "load_pika_hardware",
                default_value="true",
                description="Include both Pika grippers in the MuJoCo system.",
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "jtc_guard_cancel_response_timeout_s", default_value="0.5"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
