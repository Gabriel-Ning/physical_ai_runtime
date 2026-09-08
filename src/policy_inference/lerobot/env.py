"""Gymnasium-compatible Environment interfaces for Physical AI Runtime.

Provides:
- RmiEnv: Direct Gymnasium environment for RLPD, SAC, PPO, LoRA, and full VLA online rollouts.
- ResidualRmiEnv: Residual RL wrapper (e.g. ResFiT / Amazon FAR) augmenting a frozen or
  pretrained base policy with online learned residual actions.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from rmi.reset import ResetHandler, ResetMode
from rmi.selection import SourceRole

from .bridges.action import ActionDecoder, CartesianActionDecoder, JointActionDecoder
from .bridges.observation import ObservationEncoder
from .features import PolicyLayout

logger = logging.getLogger("policy_inference.env")


class RmiEnv(gym.Env):
    """Standard Gymnasium environment wrapping physical_ai_runtime RMI stack."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        context: Any,
        layout: PolicyLayout,
        *,
        resource: str = "arm",
        node_name: str = "Policy",
        task: Any | None = None,
        action_space_type: str = "rel",
        max_steps: int = 300,
        control_freq: float = 30.0,
        reset_mode: ResetMode | str = ResetMode.AUTO,
        custom_reset_fn: Callable[[Any], bool] | None = None,
        position_scale: float = 1.0,
        orientation_scale: float = 1.0,
        gripper_max_width: float = 0.08,
    ) -> None:
        super().__init__()
        self.context = context
        self.layout = layout
        self.resource = resource
        self.node_name = node_name
        self.task = task
        self.action_space_type = action_space_type
        self.max_steps = max_steps
        self.control_freq = control_freq
        self.dt = 1.0 / max(control_freq, 1.0)
        self.reset_mode = ResetMode.parse(reset_mode)
        self.custom_reset_fn = custom_reset_fn
        self.position_scale = position_scale
        self.orientation_scale = orientation_scale
        self.gripper_max_width = gripper_max_width

        # Bridge: Encoder & Decoder
        self.encoder = ObservationEncoder(layout)
        self.decoder: ActionDecoder = self._init_decoder()

        # Action and Observation Spaces
        self.action_space = self._build_action_space()
        self.observation_space = self._build_observation_space()

        # RMI Policy Action Node
        self.policy_node = self.context.make_node(self.node_name)
        self._node_activation = None

        # Episode state
        self.current_step = 0
        self._next_tick = 0.0
        self._last_raw_obs = None
        self._last_encoded_obs: dict[str, np.ndarray] = {}

    def _init_decoder(self) -> ActionDecoder:
        if self.action_space_type == "joint":
            return JointActionDecoder(self.layout)
        return CartesianActionDecoder(
            self.layout,
            action_space=self.action_space_type,
            position_scale=self.position_scale,
            orientation_scale=self.orientation_scale,
            gripper_max_width=self.gripper_max_width,
        )

    def _build_action_space(self) -> spaces.Box:
        dim = int(self.layout.action_dimension)
        return spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(dim,),
            dtype=np.float32,
        )

    def _build_observation_space(self) -> spaces.Dict:
        obs_spaces: dict[str, spaces.Space] = {}

        # Cameras (keyed by short camera name, matching ObservationEncoder output)
        for feature_name, cam_source in self.layout.camera_sources.items():
            cam_key = feature_name.removeprefix("observation.images.")
            shape = self.layout.camera_shapes.get(feature_name, (480, 640, 3))
            obs_spaces[cam_key] = spaces.Box(
                low=0,
                high=255,
                shape=shape,
                dtype=np.uint8,
            )

        # State features (keyed by feature name, matching ObservationEncoder output)
        for name in self.layout.state_feature_names:
            obs_spaces[name] = spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(),
                dtype=np.float32,
            )

        return spaces.Dict(obs_spaces)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        """Reset environment, physical simulation, and step counters."""
        super().reset(seed=seed)
        self.current_step = 0

        # Execute physical reset (MuJoCo ResetWorld, Homing, or Custom)
        self.context.reset(
            mode=self.reset_mode,
            custom_fn=self.custom_reset_fn,
        )

        # Task reset (domain randomization / predicate reset)
        if self.task is not None and hasattr(self.task, "reset"):
            try:
                self.task.reset()
            except Exception as exc:
                logger.warning("Error during task.reset(): %s", exc)

        # Ensure node is active
        if self._node_activation is None:
            self._node_activation = self.policy_node.activate()
            self._node_activation.__enter__()

        # Retrieve initial observation
        raw_obs = self._get_robot_observation()
        self._last_raw_obs = raw_obs
        self._last_encoded_obs = self.encoder.encode(raw_obs)

        # Anchor decoder TCP pose
        pose_part = self.layout.pose_part
        if pose_part and pose_part in raw_obs.sensors:
            pose = raw_obs.sensors[pose_part].value
            if hasattr(pose, "position_xyz") and hasattr(pose, "orientation_wxyz"):
                self.decoder.set_current_pose(pose.position_xyz, pose.orientation_wxyz)

        self._next_tick = time.monotonic()
        info = {
            "step": 0,
            "is_intervention": False,
        }
        return self._last_encoded_obs, info

    def step(
        self,
        action: np.ndarray,
    ) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        """Step environment by submitting action, waiting 1 tick, and evaluating reward."""
        self.current_step += 1
        self._next_tick += self.dt

        # 1. Action translation and submission
        raw_action = np.asarray(action, dtype=np.float32)
        pose_part = self.layout.pose_part
        if pose_part and self._last_raw_obs and pose_part in self._last_raw_obs.sensors:
            pose = self._last_raw_obs.sensors[pose_part].value
            if hasattr(pose, "position_xyz") and hasattr(pose, "orientation_wxyz"):
                self.decoder.set_current_pose(pose.position_xyz, pose.orientation_wxyz)

        native_actions = self.decoder.decode(raw_action)
        if native_actions:
            try:
                self.policy_node.submit(native_actions)
            except Exception as exc:
                logger.warning("Error submitting actions to RMI: %s", exc)

        # 2. Wait 1 control tick
        sleep_s = self._next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            self._next_tick = time.monotonic()

        # 3. Retrieve new observation
        raw_obs = self._get_robot_observation()
        self._last_raw_obs = raw_obs
        self._last_encoded_obs = self.encoder.encode(raw_obs)
        if pose_part and pose_part in raw_obs.sensors:
            pose = raw_obs.sensors[pose_part].value
            if hasattr(pose, "position_xyz") and hasattr(pose, "orientation_wxyz"):
                self.decoder.set_current_pose(pose.position_xyz, pose.orientation_wxyz)

        # 4. Check success / reward
        success = False
        if self.task is not None and hasattr(self.task, "check_success"):
            try:
                obs_dict = (
                    dict(raw_obs.data)
                    if hasattr(raw_obs, "data")
                    else self._last_encoded_obs
                )
                success = bool(self.task.check_success(obs_dict))
            except Exception as exc:
                logger.warning("Error in task.check_success: %s", exc)

        reward = 1.0 if success else 0.0
        terminated = success
        truncated = self.current_step >= self.max_steps

        # 5. Check human intervention
        is_intervened = self._check_intervention()

        info = {
            "step": self.current_step,
            "success": success,
            "is_intervention": is_intervened,
            "timeout": truncated and not terminated,
        }
        return self._last_encoded_obs, reward, terminated, truncated, info

    def _get_robot_observation(self) -> Any:
        try:
            return self.context.robot[self.resource].get_observation()
        except Exception:
            return self.context.robot.get_observation()

    def _check_intervention(self) -> bool:
        """Inspect if human teleoperation has claimed control via Execution Manager."""
        authority = getattr(self.context, "authority_client", None)
        if authority is not None:
            try:
                status = authority.describe_authority()
                resources = {group.part for group in self.layout.joints.groups}
                if self.layout.pose_part:
                    resources.add(self.layout.pose_part)
                for resource in resources & set(status.owned):
                    if status.resources[resource].get("source_role") == SourceRole.TELEOP:
                        return True
            except Exception as exc:
                logger.warning("Error checking RMI intervention: %s", exc)
        return False

    def close(self) -> None:
        if self._node_activation is not None:
            self._node_activation.close()
            self._node_activation = None


class ResidualRmiEnv(gym.Env):
    """Residual RL Environment wrapper (ResFiT / Amazon FAR paradigm).

    Augments a base policy (e.g. pretrained VLA) with an online residual policy:
        a_exec = clip(a_base + residual_scale * residual_action)
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        env: RmiEnv,
        base_policy: Any,
        *,
        residual_scale: float | np.ndarray = 0.1,
    ) -> None:
        super().__init__()
        self.env = env
        self.base_policy = base_policy
        self.residual_scale = residual_scale

        # Residual action space matches base env action space limits [-1, 1]
        self.action_space = env.action_space
        self.observation_space = env.observation_space

        self._last_obs: dict[str, np.ndarray] | None = None

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        """Reset base policy internal queues and underlying environment."""
        if hasattr(self.base_policy, "reset"):
            self.base_policy.reset()

        obs, info = self.env.reset(seed=seed, options=options)
        self._last_obs = obs
        return obs, info

    def step(
        self,
        residual_action: np.ndarray,
    ) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        """Compute base action, fuse with residual action, and step environment."""
        if self._last_obs is None:
            raise RuntimeError("Cannot call step() before reset().")

        # 1. Base policy selects action
        raw_base_action = self.base_policy.select_action(self._last_obs)
        if raw_base_action is None:
            base_act = np.zeros(self.action_space.shape, dtype=np.float32)
        else:
            base_act = np.asarray(raw_base_action, dtype=np.float32)

        # 2. Residual action fusion
        res_act = np.asarray(residual_action, dtype=np.float32)
        fused_act = np.clip(
            base_act + self.residual_scale * res_act,
            self.action_space.low,
            self.action_space.high,
        )

        # 3. Step underlying RMI environment
        next_obs, reward, terminated, truncated, info = self.env.step(fused_act)
        self._last_obs = next_obs

        # Ingest action telemetry
        info["base_action"] = base_act
        info["residual_action"] = res_act
        info["executed_action"] = fused_act

        return next_obs, reward, terminated, truncated, info

    def close(self) -> None:
        self.env.close()
        if hasattr(self.base_policy, "close"):
            self.base_policy.close()
