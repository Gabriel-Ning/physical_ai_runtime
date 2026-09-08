# Copyright 2026 physical_ai_runtime
# SPDX-License-Identifier: Apache-2.0
"""MuJoCo simulation bringup for dual Piper.

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

_DEFAULT_TASK = "table_pick_cube"
VALID_ARMS = {"left", "right", "both"}
VALID_END_EFFECTORS = {"none", "piper_gripper"}
_END_EFFECTOR_WIRING = {
    "piper_gripper": ("gripper_joint1", "gripper_fwd"),
}


def _resolve_mujoco_model(task: str) -> str:
    """Map task launch arg to an absolute MJCF path under ``robotwin_tasks``."""
    task_name = (task or "").strip() or _DEFAULT_TASK
    if os.path.isabs(task_name) and os.path.exists(task_name):
        return task_name

    candidates: list[str] = []
    try:
        tasks_share = get_package_share_directory("robotwin_tasks")
        candidates.extend(
            [
                os.path.join(tasks_share, "mjcf", f"{task_name}.xml"),
                os.path.join(tasks_share, "mjcf", "tasks", f"{task_name}.xml"),
            ]
        )
    except Exception:
        pass
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        f"MuJoCo task MJCF not found for task={task_name!r}. Tried: {candidates}"
    )


def _camera_bridge_parameters(config_path: str) -> dict | None:
    """SHM output requires an image bridge (same contract as Franka)."""
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


def _optional_xacro_args(context, *names):
    mappings = {}
    for name in names:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            mappings[name] = value
    return mappings


def _after(event, actions, stage):
    if event.returncode == 0:
        return actions
    return [EmitEvent(event=Shutdown(reason=f"{stage} failed ({event.returncode})"))]


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("piper_manipulation_rt_launch")
    arms = LaunchConfiguration("arms").perform(context).lower()
    if arms not in VALID_ARMS:
        raise RuntimeError("'arms' must be left, right, or both")
    active = [side for side in ("left", "right") if arms in (side, "both")]

    end_effectors = {
        side: LaunchConfiguration(f"{side}_end_effector").perform(context).lower()
        for side in active
    }
    load_gripper_hardware = LaunchConfiguration("load_gripper_hardware").perform(
        context
    ).lower()
    if load_gripper_hardware not in ("true", "false"):
        raise RuntimeError("'load_gripper_hardware' must be true or false")
    if load_gripper_hardware == "false":
        end_effectors = {side: "none" for side in active}
    for side, end_effector in end_effectors.items():
        if end_effector not in VALID_END_EFFECTORS:
            raise RuntimeError(
                f"'{side}_end_effector' must be one of {sorted(VALID_END_EFFECTORS)}"
            )

    headless = LaunchConfiguration("headless").perform(context).lower().strip() in (
        "true",
        "1",
        "yes",
    )
    mujoco_model_path = LaunchConfiguration("mujoco_model").perform(context).strip()
    if not mujoco_model_path:
        mujoco_model_path = _resolve_mujoco_model(
            LaunchConfiguration("task").perform(context)
        )

    mappings = {
        "enable_left": str("left" in active).lower(),
        "enable_right": str("right" in active).lower(),
        "connected_to": LaunchConfiguration("connected_to").perform(context),
        "enable_table": LaunchConfiguration("enable_table").perform(context),
        "use_fake_hardware": "false",
        "use_sim_mujoco": "true",
        "headless": "true" if headless else "false",
        "mujoco_model": mujoco_model_path,
    }
    mappings.update(
        _optional_xacro_args(
            context,
            "table_xyz",
            "table_rpy",
            "left_xyz",
            "right_xyz",
            "left_rpy",
            "right_rpy",
        )
    )
    for side in active:
        mappings.update(
            {
                f"{side}_can_interface": LaunchConfiguration(
                    f"{side}_can_interface"
                ).perform(context),
                f"{side}_mit_kd_effort_damping": LaunchConfiguration(
                    f"{side}_mit_kd_effort_damping"
                ).perform(context),
                f"enable_{side}_gripper": str(
                    end_effectors[side] == "piper_gripper"
                ).lower(),
                f"{side}_gripper_home_on_activate": LaunchConfiguration(
                    f"{side}_gripper_home_on_activate"
                ).perform(context),
            }
        )

    description = xacro.process_file(
        os.path.join(
            bringup_share, "urdf", "piper_bimanual_manipulation.urdf.xacro"
        ),
        mappings=mappings,
    ).toprettyxml(indent="  ")

    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    plugins_yaml = LaunchConfiguration("mujoco_plugins_yaml").perform(context)
    params = [
        {"robot_description": description},
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
        parameters=[{"robot_description": description, "use_sim_time": True}],
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
    gripper_controllers = [
        controller
        for side in active
        if end_effectors[side] != "none"
        for controller in (
            f"{side}_{_END_EFFECTOR_WIRING[end_effectors[side]][1]}",
            f"{side}_gripper_action",
        )
    ]
    route_remaps = [
        (
            f"--remap {side}_arm_jtc/follow_joint_trajectory:="
            f"/execution/{side}_arm/follow_joint_trajectory"
        )
        for side in active
    ]
    for side in active:
        if end_effectors[side] == "none":
            continue
        fwd = f"{side}_{_END_EFFECTOR_WIRING[end_effectors[side]][1]}"
        route_remaps.extend(
            [
                f"--remap {fwd}/commands:=/execution/{side}_gripper/joint_reference",
                (
                    f"--remap {side}_gripper_action/gripper_cmd:="
                    f"/execution/{side}_gripper/gripper_command"
                ),
            ]
        )
    route_args = [
        *[
            f"{side}_arm_{route}"
            for side in active
            for route in ("jspc", "tskpc", "jtc")
        ],
        *gripper_controllers,
        "--inactive",
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "30",
    ]
    if route_remaps:
        route_args.extend(["--controller-ros-args", " ".join(route_remaps)])
    # Quote: --controller-ros-args is one argv; unquoted join splits remaps
    # and the spawner exits before loading left_arm_jtc.
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
            for side in active
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
    share = get_package_share_directory("piper_manipulation_rt_launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arms", default_value="both", description="left, right, or both."
            ),
            DeclareLaunchArgument(
                "task",
                default_value=_DEFAULT_TASK,
                description=(
                    "RoboTwin / piper_description task name or absolute MJCF path."
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
            DeclareLaunchArgument(
                "load_gripper_hardware",
                default_value="true",
                description="Include native Piper grippers in the MuJoCo system.",
            ),
            DeclareLaunchArgument("connected_to", default_value="world"),
            DeclareLaunchArgument("enable_table", default_value="true"),
            DeclareLaunchArgument("table_xyz", default_value=""),
            DeclareLaunchArgument("table_rpy", default_value=""),
            DeclareLaunchArgument("left_can_interface", default_value="piper0"),
            DeclareLaunchArgument("right_can_interface", default_value="piper1"),
            DeclareLaunchArgument("left_mit_kd_effort_damping", default_value="0.0"),
            DeclareLaunchArgument("right_mit_kd_effort_damping", default_value="0.0"),
            DeclareLaunchArgument("left_xyz", default_value=""),
            DeclareLaunchArgument("right_xyz", default_value=""),
            DeclareLaunchArgument("left_rpy", default_value=""),
            DeclareLaunchArgument("right_rpy", default_value=""),
            DeclareLaunchArgument("left_end_effector", default_value="piper_gripper"),
            DeclareLaunchArgument("right_end_effector", default_value="piper_gripper"),
            DeclareLaunchArgument(
                "left_gripper_home_on_activate", default_value="true"
            ),
            DeclareLaunchArgument(
                "right_gripper_home_on_activate", default_value="true"
            ),
            DeclareLaunchArgument("jtc_guard_heartbeat_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "jtc_guard_cancel_response_timeout_s", default_value="0.5"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
