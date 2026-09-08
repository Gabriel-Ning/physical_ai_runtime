"""Pluggable Reset Handler for physical AI embodiments and simulation environments.

Supports:
- SIM / SIM_SERVICE: Programmatic MuJoCo ResetWorld.srv call (resets joints, bodies, and PID).
- HOMING: Smooth quintic spline motion returning joints to profile homing configurations.
- INTERACTIVE / MANUAL: Homing followed by interactive operator confirmation in CLI.
- CUSTOM: Invocation of application-provided reset callback.
- AUTO: Auto-detects simulation service; falls back to homing or interactive.
"""

from __future__ import annotations

import logging
import sys
import time
from collections.abc import Callable
from enum import Enum
from typing import Any

logger = logging.getLogger("rmi.reset")

try:
    from mujoco_ros2_control_msgs.srv import ResetWorld
except ImportError:
    ResetWorld = None


class ResetMode(str, Enum):
    """Supported reset behaviors."""

    AUTO = "auto"
    SIM = "sim"
    HOMING = "homing"
    INTERACTIVE = "manual"
    CUSTOM = "custom"
    NONE = "none"

    @classmethod
    def parse(cls, value: ResetMode | str) -> ResetMode:
        if isinstance(value, cls):
            return value
        raw = str(value).strip().lower()
        for member in cls:
            if member.value == raw or member.name.lower() == raw:
                return member
        raise ValueError(f"Unknown reset mode: {value!r}. Options: {[m.value for m in cls]}")


class ResetHandler:
    """Dispatches embodiment and environment resets according to configured mode."""

    def __init__(self, context: Any) -> None:
        self.context = context

    def reset(
        self,
        mode: ResetMode | str = ResetMode.AUTO,
        *,
        keyframe: str = "",
        custom_fn: Callable[[Any], bool] | None = None,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Execute reset procedure matching the selected mode."""
        parsed_mode = ResetMode.parse(mode)

        if parsed_mode == ResetMode.NONE:
            return True

        if parsed_mode == ResetMode.CUSTOM:
            if custom_fn is None:
                raise ValueError("ResetMode.CUSTOM requested but no custom_fn provided.")
            return bool(custom_fn(self.context))

        if parsed_mode in (ResetMode.AUTO, ResetMode.SIM):
            sim_ok = self._try_sim_reset(keyframe=keyframe, timeout_sec=timeout_sec)
            if sim_ok:
                return True
            if parsed_mode == ResetMode.SIM:
                raise RuntimeError(
                    "ResetMode.SIM failed: MuJoCo ResetWorld service not available or failed."
                )

        if parsed_mode in (ResetMode.AUTO, ResetMode.HOMING, ResetMode.INTERACTIVE):
            homing_ok = self._try_homing_reset(timeout_sec=timeout_sec)
            if parsed_mode == ResetMode.INTERACTIVE:
                self._wait_for_user_interaction()
            return homing_ok

        return True

    def _try_sim_reset(self, *, keyframe: str = "", timeout_sec: float = 3.0) -> bool:
        """Attempt to call MuJoCo ResetWorld ROS 2 service."""
        if ResetWorld is None:
            logger.debug("mujoco_ros2_control_msgs not importable, skipping sim reset.")
            return False

        node = getattr(self.context, "node", None)
        if node is None:
            return False

        candidate_services = [
            "/controller_manager/reset_world",
            "/mujoco_ros2_control_node/reset_world",
        ]

        client = None
        for srv_name in candidate_services:
            c = node.create_client(ResetWorld, srv_name)
            if c.wait_for_service(timeout_sec=0.2):
                client = c
                break
            try:
                node.destroy_client(c)
            except Exception:
                pass

        if client is None:
            return False

        try:
            req = ResetWorld.Request()
            if keyframe:
                req.keyframe = keyframe
            future = client.call_async(req)

            t0 = time.monotonic()
            import rclpy

            while time.monotonic() - t0 < timeout_sec:
                if future.done():
                    resp = future.result()
                    if resp is not None and getattr(resp, "success", True):
                        logger.info(
                            "MuJoCo world reset succeeded (keyframe=%r, message=%r)",
                            keyframe,
                            getattr(resp, "message", ""),
                        )
                        return True
                    logger.warning("MuJoCo ResetWorld failed: %s", getattr(resp, "message", ""))
                    return False
                time.sleep(0.02)

            logger.warning("MuJoCo ResetWorld service timed out after %.1fs", timeout_sec)
            return False
        except Exception as exc:
            logger.warning("Exception calling ResetWorld service: %s", exc)
            return False
        finally:
            try:
                node.destroy_client(client)
            except Exception:
                pass

    def _try_homing_reset(self, *, duration_s: float = 2.5, rate_hz: float = 50.0, timeout_sec: float = 5.0) -> bool:
        """Smooth quintic spline staging motion to home poses defined in profile."""
        profile = getattr(self.context, "profile", None)
        if profile is None:
            return False

        homing_cfg = getattr(profile, "raw_data", {}).get("homing")
        if not homing_cfg:
            homing_cfg = getattr(profile, "homing", None)
        if not homing_cfg or not isinstance(homing_cfg, dict):
            logger.debug("No homing configuration found in profile.")
            return True

        duration_s = float(homing_cfg.get("duration_s", duration_s))
        joint_targets = homing_cfg.get("joint_positions")
        if not joint_targets or not isinstance(joint_targets, dict):
            return True

        robot = getattr(self.context, "robot", None)
        if robot is None:
            return False

        try:
            obs = robot.get_observation()
            name_to_pos = dict(zip(obs.joint_names, obs.joint_positions))

            parts_to_home = {}
            for part_name, target in joint_targets.items():
                part_spec = profile.parts.get(part_name)
                if part_spec and all(j in name_to_pos for j in part_spec.joint_names):
                    parts_to_home[part_name] = (
                        [name_to_pos[j] for j in part_spec.joint_names],
                        list(target),
                    )

            if not parts_to_home:
                return True

            from .contracts import Action

            steps = int(max(duration_s * rate_hz, 10))
            dt = duration_s / steps

            policy_node = self.context.make_node("ResetHoming")
            with policy_node.activate():
                for i in range(steps + 1):
                    s = i / steps
                    # Quintic polynomial: 10s^3 - 15s^4 + 6s^5
                    h = 10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5)

                    actions = [
                        Action(
                            part=part_name,
                            command="joint_reference",
                            value=[
                                q_start[j] + h * (q_goal[j] - q_start[j])
                                for j in range(len(q_goal))
                            ],
                        )
                        for part_name, (q_start, q_goal) in parts_to_home.items()
                    ]
                    policy_node.submit(actions)
                    time.sleep(dt)

            logger.info("Smooth homing reset completed.")
            return True
        except Exception as exc:
            logger.warning("Error during smooth homing reset: %s", exc)
            return False

    def _wait_for_user_interaction(self) -> None:
        """Prompt operator to reset physical workpiece and press Enter."""
        prompt = (
            "\n"
            + "=" * 60
            + "\n[Reset Handler] Please arrange environment objects / workpiece.\n"
            + "Press [ENTER] to start next episode (or Ctrl+C to abort)..."
            + "\n"
            + "=" * 60
            + "\n"
        )
        try:
            input(prompt)
        except (EOFError, KeyboardInterrupt):
            logger.info("Interactive reset aborted by user.")
            raise
