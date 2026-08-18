"""Shared support utilities for RMI example applications."""

from __future__ import annotations

import math
from pathlib import Path

from curobo_planner_adapter import (
    CuroboIkResolverBackend,
    CuroboIkSolverConfig,
    CuroboJointStreamerBackend,
    CuroboMpcSolverConfig,
    CuroboRobotConfig,
    CuroboTrajectoryPlannerBackend,
)
from motion_planner_core.contracts import CartesianState

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
CUROBO_CONFIGS_DIR = (
    WORKSPACE_ROOT
    / "src"
    / "motion_planning"
    / "motion_planners"
    / "curobo_robot_models"
    / "config"
)

PROFILE_TO_CUROBO = {
    "fr3_pika_single_arm.yaml": {
        "curobo_yaml": "fr3_manipulation.yml",
        "target_link": "pika_gripper_tcp",
        "arm_part": "arm",
        "home_joint_positions": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        "default_target_pose": {
            "position": [0.45, 0.0, 0.35],
            "orientation": [0.0, 1.0, 0.0, 0.0],  # w, x, y, z (z pointing downwards)
        },
    },
    "marvin_bimanual.yaml": {
        "curobo_yaml": "marvin_manipulation.yml",
        "target_link": "left_pika_gripper_tcp",
        "arm_part": "left_arm",
        "home_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "default_target_pose": {
            "position": [0.4, 0.2, 0.3],
            "orientation": [0.0, 1.0, 0.0, 0.0],
        },
    },
    "piper_bimanual.yaml": {
        "curobo_yaml": "piper_bimanual_manipulation.yml",
        "target_link": "left_gripper_tcp",
        "arm_part": "left_arm",
        "home_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "default_target_pose": {
            "position": [0.3, 0.15, 0.2],
            "orientation": [0.0, 1.0, 0.0, 0.0],
        },
    },
}


def get_curobo_config_path(profile_name: str) -> Path:
    key = Path(profile_name).name
    info = PROFILE_TO_CUROBO.get(key)
    if not info:
        raise ValueError(f"Unknown profile {profile_name!r}")
    yaml_file = CUROBO_CONFIGS_DIR / info["curobo_yaml"]
    if not yaml_file.is_file():
        raise FileNotFoundError(f"cuRobo robot config not found: {yaml_file}")
    return yaml_file


def create_curobo_robot_config(
    profile_name: str, device: str = "cuda"
) -> CuroboRobotConfig:
    key = Path(profile_name).name
    info = PROFILE_TO_CUROBO[key]
    config_path = get_curobo_config_path(profile_name)
    return CuroboRobotConfig(
        robot=str(config_path),
        target_link_name=info["target_link"],
        device=device,
        use_cuda_graph=False,
    )


def create_curobo_trajectory_planner(
    profile_name: str, device: str = "cuda"
) -> CuroboTrajectoryPlannerBackend:
    robot_config = create_curobo_robot_config(profile_name, device)
    return CuroboTrajectoryPlannerBackend(robot_config=robot_config)


def create_curobo_ik_resolver(
    profile_name: str, device: str = "cuda"
) -> CuroboIkResolverBackend:
    robot_config = create_curobo_robot_config(profile_name, device)
    return CuroboIkResolverBackend(
        robot_config=robot_config,
        ik_config=CuroboIkSolverConfig(num_seeds=32, max_iterations=100),
    )


def create_curobo_joint_streamer(
    profile_name: str, horizon: int = 8, dt: float = 0.033, device: str = "cuda"
) -> CuroboJointStreamerBackend:
    robot_config = create_curobo_robot_config(profile_name, device)
    mpc_config = CuroboMpcSolverConfig(
        optimization_dt=dt,
        horizon_points=horizon,
    )
    return CuroboJointStreamerBackend(
        robot_config=robot_config,
        solver_config=mpc_config,
    )


def make_circular_target(
    center: list[float], radius: float, t: float, speed: float = 1.0
) -> CartesianState:
    """Generate a continuous circular target pose around center in xy plane."""
    x = center[0] + radius * math.cos(speed * t)
    y = center[1] + radius * math.sin(speed * t)
    z = center[2]
    return CartesianState(
        position_xyz=(float(x), float(y), float(z)),
        orientation_wxyz=(0.0, 1.0, 0.0, 0.0),
    )
