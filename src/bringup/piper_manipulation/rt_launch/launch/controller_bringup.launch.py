"""Bring up one or two Piper followers from validated launch arguments."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


VALID_ARMS = {"left", "right", "both"}
VALID_END_EFFECTORS = {"none", "piper_gripper"}

# Per end-effector choice: (xacro-enabled joint name, forward controller name).
_END_EFFECTOR_WIRING = {
    "piper_gripper": ("gripper_joint1", "gripper_fwd"),
}


def _resolve_cpu_affinity(context) -> str:
    """Prefer launch arg; else RT_CM_CPU_AFFINITY from the RT host profile."""
    explicit = LaunchConfiguration("cpu_affinity").perform(context).strip()
    if explicit in ("none", "off", "-"):
        return ""
    if explicit:
        return explicit
    return os.environ.get("RT_CM_CPU_AFFINITY", "").strip()


def _optional_xacro_args(context, *names):
    """Forward launch args to xacro only when the user set them."""
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


def _nodes(context):
    description_share = get_package_share_directory("piper_description")
    arms = LaunchConfiguration("arms").perform(context).lower()
    if arms not in VALID_ARMS:
        raise RuntimeError("'arms' must be left, right, or both")
    active = [side for side in ("left", "right") if arms in (side, "both")]

    use_sim_mujoco_arg = LaunchConfiguration("use_sim_mujoco").perform(context).lower().strip()
    backend_arg = LaunchConfiguration("backend").perform(context).lower().strip()
    fake_arg = LaunchConfiguration("use_fake_hardware").perform(context).lower().strip()
    if backend_arg:
        if backend_arg not in ("real", "fake", "mujoco"):
            raise RuntimeError(f"'backend' must be real, fake, or mujoco, got '{backend_arg}'")
        is_mujoco = (backend_arg == "mujoco")
        fake = "true" if backend_arg == "fake" else "false"
    elif use_sim_mujoco_arg in ("true", "1", "yes"):
        is_mujoco = True
        fake = "false"
    else:
        is_mujoco = False
        fake = fake_arg

    can_interfaces = {
        side: LaunchConfiguration(f"{side}_can_interface").perform(context)
        for side in active
    }
    if fake == "false" and not is_mujoco and arms == "both" and len(set(can_interfaces.values())) != 2:
        raise RuntimeError(
            "real dual-arm profile must use two different CAN interfaces"
        )

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

    task_name = LaunchConfiguration("task").perform(context).strip()
    headless_str = LaunchConfiguration("headless").perform(context).lower().strip()
    headless = headless_str in ("true", "1", "yes")

    mujoco_model_path = ""
    if is_mujoco:
        if os.path.isabs(task_name) and os.path.exists(task_name):
            mujoco_model_path = task_name
        else:
            candidates = []
            try:
                tasks_share = get_package_share_directory("robotwin_tasks")
                candidates.extend([
                    os.path.join(tasks_share, "mjcf", f"{task_name}.xml"),
                    os.path.join(tasks_share, "mjcf", "tasks", f"{task_name}.xml"),
                ])
            except Exception:
                pass
            candidates.extend([
                os.path.join(description_share, "mjcf", "tasks", f"{task_name}.xml"),
                os.path.join(description_share, "mjcf", f"{task_name}.xml"),
            ])
            for candidate in candidates:
                if os.path.exists(candidate):
                    mujoco_model_path = candidate
                    break
            if not mujoco_model_path:
                mujoco_model_path = candidates[0] if candidates else os.path.join(description_share, "mjcf", "tasks", "table_pick_cube.xml")

    mappings = {
        "enable_left": str("left" in active).lower(),
        "enable_right": str("right" in active).lower(),
        "connected_to": LaunchConfiguration("connected_to").perform(context),
        "enable_table": LaunchConfiguration("enable_table").perform(context),
        "use_fake_hardware": fake,
        "use_sim_mujoco": "true" if is_mujoco else "false",
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
                f"{side}_can_interface": can_interfaces[side],
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
            description_share, "urdf", "piper_bimanual_manipulation.urdf.xacro"
        ),
        mappings=mappings,
    ).toprettyxml(indent="  ")
    params = [
        {"robot_description": description},
        LaunchConfiguration("controllers_yaml"),
    ]
    if is_mujoco:
        params.append({"use_sim_time": True})
        params.append({"headless": headless})
        plugins_yaml = os.path.join(description_share, "config", "mujoco_plugins.yaml")
        if os.path.exists(plugins_yaml):
            params.append(plugins_yaml)

    heartbeat_timeout_s = float(
        LaunchConfiguration("jtc_guard_heartbeat_timeout_s").perform(context)
    )
    try:
        cancel_response_timeout_s = float(
            LaunchConfiguration("jtc_guard_cancel_response_timeout_s").perform(context)
        )
    except Exception:
        cancel_response_timeout_s = 0.5
    cpu_affinity = _resolve_cpu_affinity(context)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": description, "use_sim_time": is_mujoco}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(description_share, "rviz", "visualize_piper.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    cm = Node(
        package="mujoco_ros2_control" if is_mujoco else "controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=params,
        # launch joins prefix substitutions without spaces, then shlex.splits;
        # pass one shell-like string (same requirement as launch-prefix).
        prefix=f"taskset -c {cpu_affinity}" if cpu_affinity else None,
    )
    # Spawner can race with itself under slow HW init (already-active then
    # re-configure). Treat "already active" as success via a small shell guard.
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
    gripper_remaps = " ".join(
        remap
        for side in active
        if end_effectors[side] != "none"
        for remap in (
            f"--remap {side}_{_END_EFFECTOR_WIRING[end_effectors[side]][1]}"
            f"/commands:=/execution/{side}_gripper/joint_reference",
            f"--remap {side}_gripper_action/gripper_cmd:="
            f"/execution/{side}_gripper/gripper_command",
        )
    )
    jtc_remaps = " ".join(
        (
            f"--remap {side}_arm_jtc/follow_joint_trajectory:="
            f"/execution/{side}_arm/follow_joint_trajectory"
        )
        for side in active
    )
    controller_remaps = " ".join(
        part for part in (jtc_remaps, gripper_remaps) if part
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
    route_cmd_str = (
        f"ros2 run controller_manager spawner {' '.join(route_args)} || true"
    )
    routes = ExecuteProcess(
        cmd=["bash", "-c", route_cmd_str],
        name="routes_spawner",
        output="screen",
    )
    actions = []
    if cpu_affinity:
        actions.append(
            LogInfo(
                msg=(
                    f"Pinning ros2_control_node to CPUs {cpu_affinity} "
                    "(taskset; from cpu_affinity or RT_CM_CPU_AFFINITY)."
                )
            )
        )
    actions.extend(
        [
            rsp,
            rviz,
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
                        }
                    ],
                    output="screen",
                )
                for side in active
            ],
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


def generate_launch_description():
    share = get_package_share_directory("piper_manipulation_rt_launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arms", default_value="both", description="left, right, or both."
            ),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument(
                "load_gripper_hardware",
                default_value="true",
                description=(
                    "Load all selected native Piper grippers. Set false to "
                    "omit gripper URDF, hardware, and controllers."
                ),
            ),
            DeclareLaunchArgument(
                "connected_to",
                default_value="world",
                description="Root frame for the Piper workcell assembly.",
            ),
            DeclareLaunchArgument(
                "enable_table",
                default_value="true",
                description="Include the visual-only experiment table model.",
            ),
            DeclareLaunchArgument("table_xyz", default_value=""),
            DeclareLaunchArgument("table_rpy", default_value=""),
            DeclareLaunchArgument(
                "left_can_interface",
                default_value="piper0",
                description="SocketCAN name for the left follower (site alias).",
            ),
            DeclareLaunchArgument(
                "right_can_interface",
                default_value="piper1",
                description="SocketCAN name for the right follower (site alias).",
            ),
            DeclareLaunchArgument(
                "left_mit_kd_effort_damping", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "right_mit_kd_effort_damping", default_value="0.0"
            ),
            # Empty defers to piper_description xacro (site-calibrated mounts).
            DeclareLaunchArgument("left_xyz", default_value=""),
            DeclareLaunchArgument("right_xyz", default_value=""),
            DeclareLaunchArgument("left_rpy", default_value=""),
            DeclareLaunchArgument("right_rpy", default_value=""),
            DeclareLaunchArgument(
                "left_end_effector",
                default_value="piper_gripper",
                description="none or piper_gripper.",
            ),
            DeclareLaunchArgument(
                "right_end_effector",
                default_value="piper_gripper",
                description="none or piper_gripper.",
            ),
            DeclareLaunchArgument(
                "left_gripper_home_on_activate", default_value="true"
            ),
            DeclareLaunchArgument(
                "right_gripper_home_on_activate", default_value="true"
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [share, "config", "controller", "controllers.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz using the Piper description configuration.",
            ),
            DeclareLaunchArgument(
                "cpu_affinity",
                default_value="",
                description=(
                    "Comma-separated CPUs for ros2_control_node taskset. "
                    "Empty uses RT_CM_CPU_AFFINITY from the RT host profile "
                    "(see docs/CPU_HOST_SETUP.md). Pass none to disable."
                ),
            ),
            DeclareLaunchArgument(
                "jtc_guard_heartbeat_timeout_s",
                default_value="0.5",
                description=(
                    "Cancel an armed JTC goal after this many seconds without "
                    "a workstation heartbeat."
                ),
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="",
                description="real, fake, or mujoco. Empty falls back to use_fake_hardware.",
            ),
            DeclareLaunchArgument(
                "use_sim_mujoco",
                default_value="false",
                description="Run MuJoCo simulation backend (alias for backend:=mujoco).",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="table_pick_cube",
                description="Task name for MuJoCo simulation (e.g. table_pick_cube).",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo simulation in headless mode (no GUI window).",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
