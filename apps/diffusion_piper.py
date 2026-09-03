"""Run a LeRobot Diffusion policy on the dual Piper RMI control plane."""

from __future__ import annotations

import argparse
import logging
import threading
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
from act_piper import (
    IMAGE_SHAPES,
    JOINT_NAMES,
    POLICY_IMAGE_KEYS,
    _raw_from_arrays,
    _resolve_checkpoint,
    _resolve_device,
    _run_ros,
)

LOGGER = logging.getLogger("diffusion_piper")


def _configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


class DiffusionRunner:
    """Load Diffusion Policy and preserve its consecutive observation history."""

    def __init__(
        self, checkpoint: Path, device: str, action_steps: int | None = None
    ) -> None:
        import torch
        from lerobot.policies import get_policy_class, make_pre_post_processors
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import hw_to_dataset_features

        checkpoint = _resolve_checkpoint(checkpoint)
        self.device = _resolve_device(device)
        self.checkpoint = checkpoint
        LOGGER.info("loading Diffusion checkpoint: %s", checkpoint)
        self.policy = get_policy_class("diffusion").from_pretrained(str(checkpoint))
        self.policy.to(self.device)
        self.policy.eval()
        self.policy.reset()

        expected_images = set(POLICY_IMAGE_KEYS)
        actual_images = set(self.policy.config.image_features)
        if actual_images != expected_images:
            raise ValueError(
                "checkpoint image features do not match dual Piper training data: "
                f"expected={sorted(expected_images)}, actual={sorted(actual_images)}"
            )
        state_feature = self.policy.config.input_features.get("observation.state")
        action_feature = self.policy.config.output_features.get("action")
        if state_feature is None or tuple(state_feature.shape) != (len(JOINT_NAMES),):
            raise ValueError(
                "checkpoint must consume a 14-dimensional observation.state; "
                f"got {getattr(state_feature, 'shape', None)}"
            )
        if action_feature is None or tuple(action_feature.shape) != (len(JOINT_NAMES),):
            raise ValueError(
                "checkpoint must produce a 14-dimensional action; "
                f"got {getattr(action_feature, 'shape', None)}"
            )

        hardware_features: dict[str, type | tuple[int, int, int]] = {
            name: float for name in JOINT_NAMES
        }
        hardware_features.update(IMAGE_SHAPES)
        self.lerobot_features = hw_to_dataset_features(
            hardware_features, OBS_STR, use_video=False
        )
        device_override = {"device": self.device}
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": device_override},
            postprocessor_overrides={"device_processor": device_override},
        )
        self.n_obs_steps = int(self.policy.config.n_obs_steps)
        self.chunk_size = int(self.policy.config.horizon)
        predicted_steps = int(self.policy.config.n_action_steps)
        self.action_steps = predicted_steps if action_steps is None else action_steps
        if not 0 < self.action_steps <= predicted_steps:
            raise ValueError(
                "invalid Diffusion action steps: "
                f"{self.action_steps}; expected 1..{predicted_steps}"
            )
        self._history: deque[dict[str, Any]] = deque(maxlen=self.n_obs_steps)
        self._history_lock = threading.Lock()
        self._torch = torch
        LOGGER.info(
            "Diffusion ready: device=%s, observations=%d, horizon=%d, "
            "predicted=%d, execute=%d",
            self.device,
            self.n_obs_steps,
            self.chunk_size,
            predicted_steps,
            self.action_steps,
        )

    def observe(self, raw_observation: dict[str, Any]) -> None:
        """Retain the latest raw observation while a predicted chunk executes."""
        with self._history_lock:
            self._history.append(raw_observation)

    def reset_history(self) -> None:
        """Discard observations captured before a stream or control discontinuity."""
        with self._history_lock:
            self._history.clear()

    def _prepare_observation(self, raw_observation: dict[str, Any]) -> dict[str, Any]:
        from lerobot.utils.constants import OBS_STATE
        from lerobot.utils.feature_utils import build_dataset_frame
        from torch.nn import functional

        frame = build_dataset_frame(
            self.lerobot_features, raw_observation, "observation"
        )
        observation: dict[str, Any] = {
            OBS_STATE: self._torch.as_tensor(frame[OBS_STATE]).unsqueeze(0)
        }
        for key in POLICY_IMAGE_KEYS:
            image = self._torch.as_tensor(frame[key]).permute(2, 0, 1).unsqueeze(0)
            target_shape = self.policy.config.image_features[key].shape
            image = functional.interpolate(
                image.float(),
                size=(target_shape[1], target_shape[2]),
                mode="bilinear",
                align_corners=False,
            )
            observation[key] = image / 255.0
        return self.preprocessor(observation)

    def predict(self, raw_observation: dict[str, Any]) -> np.ndarray:
        """Return a de-normalized action chunk with shape ``[T, 14]``."""
        from lerobot.utils.constants import OBS_STATE

        self.observe(raw_observation)
        with self._history_lock:
            history = list(self._history)
        while len(history) < self.n_obs_steps:
            history.insert(0, history[0])
        processed = [self._prepare_observation(item) for item in history]
        temporal_keys = (OBS_STATE, *POLICY_IMAGE_KEYS)
        temporal_observation = {
            key: self._torch.stack([frame[key] for frame in processed], dim=1)
            for key in temporal_keys
        }

        with self._torch.inference_mode():
            chunk = self.policy.predict_action_chunk(temporal_observation)
            if chunk.ndim == 2:
                chunk = chunk.unsqueeze(0)
            chunk = chunk[:, : self.action_steps, :]
            actions = [
                self.postprocessor(chunk[:, index, :])
                for index in range(chunk.shape[1])
            ]
            action_tensor = self._torch.stack(actions, dim=1).squeeze(0)
        action = action_tensor.detach().cpu().numpy().astype(np.float32, copy=False)
        expected_shape = (self.action_steps, len(JOINT_NAMES))
        if action.shape != expected_shape:
            raise ValueError(
                f"Diffusion returned {action.shape}, expected {expected_shape}"
            )
        if not np.isfinite(action).all():
            raise ValueError("Diffusion returned NaN or Inf")
        return action


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a LeRobot Diffusion checkpoint on the dual Piper control plane",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--profile", default="apps/profiles/piper_bimanual.yaml")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--task", default="pick_corner")
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument(
        "--top-camera-topic", default="/observation/orbbec/color/image_raw"
    )
    parser.add_argument(
        "--left-wrist-topic",
        default="/observation/left_hand_realsense/color/image_raw",
    )
    parser.add_argument(
        "--right-wrist-topic",
        default="/observation/right_hand_realsense/color/image_raw",
    )
    parser.add_argument(
        "--action-steps",
        type=int,
        default=None,
        help="execute this many actions from each predicted Diffusion chunk",
    )
    parser.add_argument("--input-timeout-s", type=float, default=10.0)
    parser.add_argument("--max-input-age-s", type=float, default=0.5)
    parser.add_argument(
        "--record-episodes",
        type=int,
        default=0,
        help="record this many successful Diffusion-assisted MCAP episodes; 0 keeps continuous inference",
    )
    parser.add_argument(
        "--layout-ids",
        default=None,
        help="comma-separated layout IDs, one for each recorded episode",
    )
    parser.add_argument(
        "--hold-grippers-open",
        action="store_true",
        help="hard-override both policy gripper channels during approach-only tests",
    )
    parser.add_argument(
        "--open-gripper-position",
        type=float,
        default=0.020,
        help="per-finger position used by --hold-grippers-open, in metres",
    )
    parser.add_argument(
        "--teleop-takeover",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="enable local Piper leader takeover through T or /diffusion_piper/set_teleop",
    )
    parser.add_argument(
        "--teleop-hotkey",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="toggle teleop takeover with raw T in this terminal",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="explicitly acknowledge that the connected RT stack may control real hardware",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="load the checkpoint and run one synthetic inference without ROS or motion",
    )
    return parser


def main() -> None:
    _configure_logging()
    args = _build_parser().parse_args()
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.record_episodes < 0:
        raise SystemExit("--record-episodes must be non-negative")

    runner = DiffusionRunner(args.checkpoint, args.device, args.action_steps)
    if args.dry_run:
        state = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        images = {
            name: np.zeros(shape, dtype=np.uint8)
            for name, shape in IMAGE_SHAPES.items()
        }
        action = runner.predict(_raw_from_arrays(state, images))
        LOGGER.info(
            "dry-run passed: action shape=%s, finite=%s",
            action.shape,
            np.isfinite(action).all(),
        )
        return

    args.policy_type = "diffusion"
    _run_ros(args, runner)


if __name__ == "__main__":
    main()
