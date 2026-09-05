"""Reusable live/replay TeleopSession runner for ROS 2 application nodes."""

from __future__ import annotations

import inspect
import os
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from isaacteleop.cloudxr import CloudXRLauncher
from isaacteleop.teleop_session_manager import SessionMode, TeleopSession
from rclpy.node import Node

from .cloudxr_host_client import (
    apply_static_asset_compatibility_patch,
    validate_workspace_cloudxr,
)


@dataclass(frozen=True)
class OutputMetadata:
    """Small per-output envelope for replay and pipelined-frame diagnostics."""

    output_seq: int
    returned_frame_id: int | None
    submitted_frame_id: int | None
    returned_age_frames: int | None
    execution_state: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "output_seq": self.output_seq,
            "returned_frame_id": self.returned_frame_id,
            "submitted_frame_id": self.submitted_frame_id,
            "returned_age_frames": self.returned_age_frames,
            "execution_state": self.execution_state,
        }


StepCallback = Callable[[dict, object, OutputMetadata], None]


def run_teleop_session_loop(node: Node, config, step_callback: StepCallback) -> int:
    """Run one application pipeline with identical live and MCAP replay handling."""
    runtime = node.runtime_params
    if runtime.session_mode != SessionMode.LIVE:
        return _run_loop(node, config, step_callback, launcher=None)

    cloudxr_dir = Path(runtime.cloudxr.install_dir)
    if runtime.cloudxr.host_client:
        static_dir = validate_workspace_cloudxr(cloudxr_dir)
        os.environ["TELEOP_WEB_CLIENT_STATIC_DIR"] = str(static_dir)
        apply_static_asset_compatibility_patch()

    launcher_kwargs = {
        "install_dir": runtime.cloudxr.install_dir,
        "env_config": runtime.cloudxr.env_config,
        "accept_eula": runtime.cloudxr.accept_eula,
        "setup_oob": runtime.cloudxr.setup_oob,
        "usb_local": runtime.cloudxr.usb_local,
        "host_client": runtime.cloudxr.host_client,
    }
    sig_params = inspect.signature(CloudXRLauncher.__init__).parameters
    if "device_profile" in sig_params:
        launcher_kwargs["device_profile"] = runtime.cloudxr.device_profile
    if "run_embedded" in sig_params:
        launcher_kwargs["run_embedded"] = True

    with CloudXRLauncher(**launcher_kwargs) as launcher:
        node.get_logger().info(
            f"CloudXR runtime started (WSS log: {launcher.wss_log_path})"
        )
        return _run_loop(node, config, step_callback, launcher)


def _run_loop(node: Node, config, step_callback: StepCallback, launcher) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    # TF and other ROS subscriptions can run much faster than the teleop loop.
    # A single spin_once() per teleop frame lets callbacks accumulate, making
    # Time(0) return the newest transform *processed by the stale buffer*.
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin,
        name=f"{node.get_name()}-ros-executor",
        daemon=True,
    )
    spin_thread.start()
    try:
        while rclpy.ok():
            if launcher is not None:
                launcher.health_check()
            try:
                with TeleopSession(config) as session:
                    node.get_logger().info("TeleopSession started successfully")
                    output_seq = 0
                    while rclpy.ok():
                        if launcher is not None:
                            launcher.health_check()
                        result = session.step()
                        stamp = node.get_clock().now()
                        output_seq += 1
                        step_callback(
                            result,
                            stamp.to_msg(),
                            _capture_output_metadata(session, output_seq),
                        )
                        time.sleep(node.runtime_params.sleep_period_s)
            except RuntimeError as exc:
                if "Failed to get OpenXR system" not in str(exc):
                    raise
                node.get_logger().warning(
                    f"No XR client connected ({exc}), retrying in 2s..."
                )
                time.sleep(2.0)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        executor.remove_node(node)
    return 0


def _capture_output_metadata(session, output_seq: int) -> OutputMetadata:
    context = session.last_context
    step_info = session.last_step_info
    execution_state = "unknown"
    if context is not None:
        execution_state = getattr(
            context.execution_events.execution_state,
            "name",
            str(context.execution_events.execution_state),
        ).lower()
    return OutputMetadata(
        output_seq=output_seq,
        returned_frame_id=getattr(step_info, "returned_frame_id", None),
        submitted_frame_id=getattr(step_info, "submitted_frame_id", None),
        returned_age_frames=getattr(step_info, "returned_age_frames", None),
        execution_state=execution_state,
    )
