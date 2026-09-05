"""ROS parameter declaration, resolution, and validation for Quest3 Bimanual Target Nodes."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

from isaacteleop.deviceio import McapReplayConfig
from isaacteleop.teleop_session_manager import SessionMode
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node


@dataclass(frozen=True)
class CloudXRParams:
    install_dir: str
    env_config: str | None
    accept_eula: bool
    setup_oob: bool
    usb_local: bool
    host_client: bool
    device_profile: str


@dataclass(frozen=True)
class NodeParameters:
    profile_name: str
    sleep_period_s: float
    session_mode: SessionMode
    mcap_config: McapReplayConfig | None
    cloudxr_params: CloudXRParams

    # Target topic names
    left_output_topic: str
    right_output_topic: str
    debug_output_topic: str
    status_topic: str
    left_clutch_topic: str
    right_clutch_topic: str

    # Clutch/Calibration snapshot topic names
    left_snapshot_controller_topic: str
    left_snapshot_ee_topic: str
    right_snapshot_controller_topic: str
    right_snapshot_ee_topic: str

    # TF frame names
    left_base_frame: str
    right_base_frame: str
    left_tcp_frame: str
    right_tcp_frame: str
    output_frame: str
    left_target_frame: str
    right_target_frame: str

    # Retargeting config parameters
    pose_source: str
    deadman_source: str
    deadman_threshold: float
    require_both_deadman: bool
    linear_scale: float
    angular_scale: float
    enable_dynamic_scale: bool
    scale_step: float
    min_scale: float
    max_scale: float
    filter_type: str
    one_euro_min_cutoff: float
    one_euro_beta: float
    one_euro_d_cutoff: float
    lowpass_alpha: float
    max_linear_step_m: float
    max_angular_step_rad: float
    openxr_to_base_rotation_xyzw: list[float]

    # Absolute mode specific parameters
    calibrate_source: str
    calibrate_threshold: float
    enable_deadman: bool
    use_live_ee_as_home: bool
    left_home_xyz: list[float]
    left_home_xyzw: list[float]
    right_home_xyz: list[float]
    right_home_xyzw: list[float]

    # Gripper parameters
    left_gripper_output_topic: str
    right_gripper_output_topic: str
    left_gripper_joint_name: str
    right_gripper_joint_name: str
    gripper_min_width: float
    gripper_max_width: float
    gripper_speed_mps: float
    require_clutch_for_gripper: bool

    @property
    def cloudxr(self) -> CloudXRParams:
        return self.cloudxr_params


def create_node_parameters(node: Node, default_profile: str = "quest3_bimanual_relative") -> NodeParameters:
    profile_name = node.declare_parameter("profile_name", default_profile).value

    # Rate and MCAP
    rate_hz = _load_rate_hz(node)
    session_mode, mcap_config = _load_mcap_replay(node)
    cloudxr_params = _load_cloudxr(node)

    # Topics
    left_output_topic = node.declare_parameter("left_output_topic", "").value
    right_output_topic = node.declare_parameter("right_output_topic", "").value
    left_gripper_output_topic = node.declare_parameter(
        "left_gripper_output_topic", "~/left_gripper_target"
    ).value
    right_gripper_output_topic = node.declare_parameter(
        "right_gripper_output_topic", "~/right_gripper_target"
    ).value
    left_gripper_joint_name = node.declare_parameter(
        "left_gripper_joint_name", "left_gripper_joint1"
    ).value
    right_gripper_joint_name = node.declare_parameter(
        "right_gripper_joint_name", "right_gripper_joint1"
    ).value
    debug_output_topic = node.declare_parameter(
        "debug_output_topic", "/teleop/quest3_bimanual/pose_chunk"
    ).value
    status_topic = node.declare_parameter(
        "status_topic", "/teleop/quest3_bimanual/status"
    ).value
    left_clutch_topic = node.declare_parameter(
        "left_clutch_topic", "/teleop/quest3_bimanual/left_clutch"
    ).value
    right_clutch_topic = node.declare_parameter(
        "right_clutch_topic", "/teleop/quest3_bimanual/right_clutch"
    ).value
    left_snapshot_controller_topic = node.declare_parameter(
        "left_snapshot_controller_topic",
        "/teleop/quest3_bimanual/left_snapshot/controller_pose",
    ).value
    left_snapshot_ee_topic = node.declare_parameter(
        "left_snapshot_ee_topic", "/teleop/quest3_bimanual/left_snapshot/ee_pose"
    ).value
    right_snapshot_controller_topic = node.declare_parameter(
        "right_snapshot_controller_topic",
        "/teleop/quest3_bimanual/right_snapshot/controller_pose",
    ).value
    right_snapshot_ee_topic = node.declare_parameter(
        "right_snapshot_ee_topic", "/teleop/quest3_bimanual/right_snapshot/ee_pose"
    ).value

    # TF Frames
    left_base_frame = node.declare_parameter("left_base_frame", "").value
    right_base_frame = node.declare_parameter("right_base_frame", "").value
    left_tcp_frame = node.declare_parameter("left_tcp_frame", "").value
    right_tcp_frame = node.declare_parameter("right_tcp_frame", "").value
    output_frame = node.declare_parameter("output_frame", "world").value
    left_target_frame = node.declare_parameter(
        "left_target_frame", "teleop_left_ee_target"
    ).value
    right_target_frame = node.declare_parameter(
        "right_target_frame", "teleop_right_ee_target"
    ).value

    # Retargeting parameters
    pose_source = node.declare_parameter("pose_source", "grip").value
    deadman_source = node.declare_parameter("deadman_source", "squeeze").value
    deadman_threshold = float(node.declare_parameter("deadman_threshold", 0.5).value)
    require_both_deadman = bool(
        node.declare_parameter("require_both_deadman", False).value
    )
    linear_scale = float(node.declare_parameter("linear_scale", 1.0).value)
    angular_scale = float(node.declare_parameter("angular_scale", 1.0).value)
    enable_dynamic_scale = bool(
        node.declare_parameter("enable_dynamic_scale", True).value
    )
    scale_step = float(node.declare_parameter("scale_step", 0.1).value)
    min_scale = float(node.declare_parameter("min_scale", 0.1).value)
    max_scale = float(node.declare_parameter("max_scale", 3.0).value)

    filter_type = node.declare_parameter("filter_type", "one_euro").value
    one_euro_min_cutoff = float(
        node.declare_parameter("one_euro_min_cutoff", 1.0).value
    )
    one_euro_beta = float(node.declare_parameter("one_euro_beta", 0.007).value)
    one_euro_d_cutoff = float(
        node.declare_parameter("one_euro_d_cutoff", 1.0).value
    )
    lowpass_alpha = float(node.declare_parameter("lowpass_alpha", 0.85).value)
    max_linear_step_m = float(node.declare_parameter("max_linear_step_m", 0.05).value)
    max_angular_step_rad = float(
        node.declare_parameter("max_angular_step_rad", 0.25).value
    )
    gripper_min_width = float(node.declare_parameter("gripper_min_width", 0.0).value)
    gripper_max_width = float(node.declare_parameter("gripper_max_width", 0.08).value)
    gripper_speed_mps = float(node.declare_parameter("gripper_speed_mps", 0.05).value)
    require_clutch_for_gripper = bool(
        node.declare_parameter("require_clutch_for_gripper", False).value
    )

    openxr_to_base_quat = node.declare_parameter(
        "openxr_to_base_rotation_xyzw", [0.5, -0.5, -0.5, 0.5]
    ).value

    # Absolute mode specific parameters
    calibrate_source = node.declare_parameter("calibrate_source", "menu").value
    calibrate_threshold = float(
        node.declare_parameter("calibrate_threshold", 0.5).value
    )
    enable_deadman = bool(node.declare_parameter("enable_deadman", False).value)
    use_live_ee_as_home = bool(
        node.declare_parameter("use_live_ee_as_home", True).value
    )
    left_home_xyz = list(
        node.declare_parameter("left_home_xyz", [0.3, 0.2, 0.2]).value
    )
    left_home_xyzw = list(
        node.declare_parameter("left_home_xyzw", [0.0, 0.0, 0.0, 1.0]).value
    )
    right_home_xyz = list(
        node.declare_parameter("right_home_xyz", [0.3, -0.2, 0.2]).value
    )
    right_home_xyzw = list(
        node.declare_parameter("right_home_xyzw", [0.0, 0.0, 0.0, 1.0]).value
    )

    _validate_retarget_parameters(
        pose_source=pose_source,
        deadman_source=deadman_source,
        deadman_threshold=deadman_threshold,
        linear_scale=linear_scale,
        angular_scale=angular_scale,
        lowpass_alpha=lowpass_alpha,
        max_linear_step_m=max_linear_step_m,
        max_angular_step_rad=max_angular_step_rad,
        openxr_to_base_quat=openxr_to_base_quat,
        gripper_min_width=gripper_min_width,
        gripper_max_width=gripper_max_width,
        gripper_speed_mps=gripper_speed_mps,
    )

    return NodeParameters(
        profile_name=profile_name,
        sleep_period_s=1.0 / rate_hz,
        session_mode=session_mode,
        mcap_config=mcap_config,
        cloudxr_params=cloudxr_params,
        left_output_topic=left_output_topic,
        right_output_topic=right_output_topic,
        left_gripper_output_topic=left_gripper_output_topic,
        right_gripper_output_topic=right_gripper_output_topic,
        left_gripper_joint_name=left_gripper_joint_name,
        right_gripper_joint_name=right_gripper_joint_name,
        gripper_min_width=gripper_min_width,
        gripper_max_width=gripper_max_width,
        gripper_speed_mps=gripper_speed_mps,
        require_clutch_for_gripper=require_clutch_for_gripper,
        debug_output_topic=debug_output_topic,
        status_topic=status_topic,
        left_clutch_topic=left_clutch_topic,
        right_clutch_topic=right_clutch_topic,
        left_snapshot_controller_topic=left_snapshot_controller_topic,
        left_snapshot_ee_topic=left_snapshot_ee_topic,
        right_snapshot_controller_topic=right_snapshot_controller_topic,
        right_snapshot_ee_topic=right_snapshot_ee_topic,
        left_base_frame=left_base_frame,
        right_base_frame=right_base_frame,
        left_tcp_frame=left_tcp_frame,
        right_tcp_frame=right_tcp_frame,
        output_frame=output_frame,
        left_target_frame=left_target_frame,
        right_target_frame=right_target_frame,
        pose_source=pose_source,
        deadman_source=deadman_source,
        deadman_threshold=deadman_threshold,
        require_both_deadman=require_both_deadman,
        linear_scale=linear_scale,
        angular_scale=angular_scale,
        enable_dynamic_scale=enable_dynamic_scale,
        scale_step=scale_step,
        min_scale=min_scale,
        max_scale=max_scale,
        filter_type=filter_type,
        one_euro_min_cutoff=one_euro_min_cutoff,
        one_euro_beta=one_euro_beta,
        one_euro_d_cutoff=one_euro_d_cutoff,
        lowpass_alpha=lowpass_alpha,
        max_linear_step_m=max_linear_step_m,
        max_angular_step_rad=max_angular_step_rad,
        openxr_to_base_rotation_xyzw=list(openxr_to_base_quat),
        calibrate_source=calibrate_source,
        calibrate_threshold=calibrate_threshold,
        enable_deadman=enable_deadman,
        use_live_ee_as_home=use_live_ee_as_home,
        left_home_xyz=left_home_xyz,
        left_home_xyzw=left_home_xyzw,
        right_home_xyz=right_home_xyz,
        right_home_xyzw=right_home_xyzw,
    )


def _load_rate_hz(node: Node) -> float:
    node.declare_parameter("rate_hz", 60.0)
    rate_hz = node.get_parameter("rate_hz").get_parameter_value().double_value
    if rate_hz <= 0 or not math.isfinite(rate_hz):
        raise ValueError("Parameter 'rate_hz' must be > 0")
    return rate_hz


def _load_mcap_replay(
    node: Node,
) -> tuple[SessionMode, McapReplayConfig | None]:
    node.declare_parameter(
        "mcap_replay_path",
        "",
        ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Optional MCAP file to replay through TeleopSession.",
        ),
    )
    mcap_replay_path = (
        node.get_parameter("mcap_replay_path")
        .get_parameter_value()
        .string_value.strip()
    )
    if not mcap_replay_path:
        return SessionMode.LIVE, None

    replay_path = Path(mcap_replay_path).expanduser().resolve()
    if not replay_path.is_file():
        raise FileNotFoundError(f"mcap_replay_path file not found: {replay_path}")
    node.get_logger().info(f"Replaying MCAP input: {replay_path}")
    return SessionMode.REPLAY, McapReplayConfig(str(replay_path))


def _get_bool_param(node: Node, name: str) -> bool:
    val = node.get_parameter(name).value
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.lower() in ("true", "1", "yes")
    return bool(val)


def _validate_retarget_parameters(
    *,
    pose_source,
    deadman_source,
    deadman_threshold,
    linear_scale,
    angular_scale,
    lowpass_alpha,
    max_linear_step_m,
    max_angular_step_rad,
    openxr_to_base_quat,
    gripper_min_width=0.0,
    gripper_max_width=0.04,
    gripper_speed_mps=0.025,
) -> None:
    if pose_source not in {"aim", "grip"}:
        raise ValueError("pose_source must be 'aim' or 'grip'")
    valid_deadman = {"none", "squeeze", "trigger", "primary", "secondary", "thumbstick", "menu"}
    if deadman_source not in valid_deadman:
        raise ValueError(f"deadman_source must be one of {sorted(valid_deadman)}")
    if not math.isfinite(deadman_threshold) or not 0.0 <= deadman_threshold <= 1.0:
        raise ValueError("deadman_threshold must be finite and in [0, 1]")

    nonnegative = {
        "linear_scale": linear_scale,
        "angular_scale": angular_scale,
        "max_linear_step_m": max_linear_step_m,
        "max_angular_step_rad": max_angular_step_rad,
    }
    for name, value in nonnegative.items():
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
    if not math.isfinite(lowpass_alpha) or not 0.0 <= lowpass_alpha <= 1.0:
        raise ValueError("lowpass_alpha must be finite and in [0, 1]")

    if not math.isfinite(gripper_min_width) or gripper_min_width < 0.0:
        raise ValueError("gripper_min_width must be finite and >= 0")
    if not math.isfinite(gripper_max_width) or gripper_max_width <= gripper_min_width:
        raise ValueError("gripper_max_width must be finite and > gripper_min_width")
    if not math.isfinite(gripper_speed_mps) or gripper_speed_mps <= 0.0:
        raise ValueError("gripper_speed_mps must be finite and > 0")

    if len(openxr_to_base_quat) != 4 or not all(
        math.isfinite(float(v)) for v in openxr_to_base_quat
    ):
        raise ValueError("openxr_to_base_rotation_xyzw must contain 4 finite values")
    norm = math.sqrt(sum(float(v) ** 2 for v in openxr_to_base_quat))
    if not math.isclose(norm, 1.0, rel_tol=1e-5, abs_tol=1e-5):
        raise ValueError(
            "openxr_to_base_rotation_xyzw must be a normalized nonzero quaternion"
        )


def _load_cloudxr(node: Node) -> CloudXRParams:
    default_cloudxr_dir = os.environ.get("CLOUDXR_DIR", "")
    node.declare_parameter("cloudxr_install_dir", default_cloudxr_dir)
    node.declare_parameter("cloudxr_env_config", "")
    node.declare_parameter("cloudxr_accept_eula", False)
    node.declare_parameter("cloudxr_setup_oob", False)
    node.declare_parameter("cloudxr_usb_local", False)
    node.declare_parameter("cloudxr_host_client", False)
    node.declare_parameter("cloudxr_device_profile", "Quest3")

    install_dir = (
        node.get_parameter("cloudxr_install_dir")
        .get_parameter_value()
        .string_value.strip()
    )
    if not install_dir:
        raise ValueError(
            "cloudxr_install_dir must be set explicitly outside the Physical AI Runtime "
            "Pixi environment"
        )
    install_dir = str(Path(install_dir).expanduser().resolve())

    env_config_str = (
        node.get_parameter("cloudxr_env_config")
        .get_parameter_value()
        .string_value.strip()
    )
    env_config = None
    if env_config_str:
        env_config_path = Path(env_config_str).expanduser()
        if env_config_path.is_file():
            env_config = str(env_config_path)
        else:
            node.get_logger().warn(
                f"cloudxr_env_config file not found at {env_config_path}, ignoring."
            )

    setup_oob = _get_bool_param(node, "cloudxr_setup_oob")
    usb_local = _get_bool_param(node, "cloudxr_usb_local")
    if usb_local and not setup_oob:
        raise ValueError(
            "Parameter 'cloudxr_usb_local' requires 'cloudxr_setup_oob' to be true"
        )

    device_profile = (
        node.get_parameter("cloudxr_device_profile")
        .get_parameter_value()
        .string_value.strip()
    ) or "Quest3"

    return CloudXRParams(
        install_dir=install_dir,
        env_config=env_config,
        accept_eula=_get_bool_param(node, "cloudxr_accept_eula"),
        setup_oob=setup_oob,
        usb_local=usb_local,
        host_client=_get_bool_param(node, "cloudxr_host_client"),
        device_profile=device_profile,
    )
