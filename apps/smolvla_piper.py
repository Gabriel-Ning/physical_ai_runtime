"""Run a LeRobot SmolVLA policy on the dual Piper RMI control plane."""

from __future__ import annotations

import argparse
import logging
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

LOGGER = logging.getLogger("smolvla_piper")


def _configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


class SmolVlaRunner:
    """Load a SmolVLA checkpoint and run one processor-faithful inference."""

    def __init__(self, checkpoint: Path, device: str, task: str) -> None:
        import torch
        from lerobot.policies import get_policy_class, make_pre_post_processors
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import hw_to_dataset_features

        checkpoint = _resolve_checkpoint(checkpoint)

        self.device = _resolve_device(device)
        self.checkpoint = checkpoint
        self.task = task
        LOGGER.info("loading SmolVLA checkpoint: %s", checkpoint)
        self.policy = get_policy_class("smolvla").from_pretrained(str(checkpoint))
        self.policy.to(self.device)
        self.policy.eval()

        expected_images = set(POLICY_IMAGE_KEYS)
        actual_images = set(self.policy.config.image_features)
        missing_images = expected_images - actual_images
        unexpected_images = {
            key for key in actual_images - expected_images if "empty_camera_" not in key
        }
        if missing_images or unexpected_images:
            raise ValueError(
                "checkpoint image features do not match dual Piper training data: "
                f"missing={sorted(missing_images)}, unexpected={sorted(unexpected_images)}"
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
        self.action_steps = int(self.policy.config.n_action_steps)
        self.chunk_size = int(self.policy.config.chunk_size)
        if not 0 < self.action_steps <= self.chunk_size:
            raise ValueError(
                f"invalid SmolVLA action steps: {self.action_steps} / {self.chunk_size}"
            )
        self._torch = torch
        LOGGER.info(
            "SmolVLA ready: device=%s, state/action_dim=%d, chunk=%d, execute=%d",
            self.device,
            len(JOINT_NAMES),
            self.chunk_size,
            self.action_steps,
        )

    def predict(self, raw_observation: dict[str, Any]) -> np.ndarray:
        """Return a de-normalized action chunk after conditioning on task text."""
        from lerobot.utils.constants import OBS_STATE
        from lerobot.utils.feature_utils import build_dataset_frame

        frame = build_dataset_frame(
            self.lerobot_features, raw_observation, "observation"
        )
        observation: dict[str, Any] = {
            OBS_STATE: self._torch.as_tensor(frame[OBS_STATE]).unsqueeze(0),
            "task": self.task,
        }
        for key in POLICY_IMAGE_KEYS:
            image = self._torch.as_tensor(frame[key]).permute(2, 0, 1).unsqueeze(0)
            observation[key] = image.float() / 255.0
        with self._torch.inference_mode():
            processed = self.preprocessor(observation)
            chunk = self.policy.predict_action_chunk(processed)
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
                f"SmolVLA returned {action.shape}, expected {expected_shape}"
            )
        if not np.isfinite(action).all():
            raise ValueError("SmolVLA returned NaN or Inf")
        return action


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a SmolVLA policy on the dual Piper control plane",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--profile", default="apps/profiles/piper_bimanual.yaml")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--task",
        required=True,
        help="exact natural-language task string used in the SmolVLA demonstrations",
    )
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument(
        "--top-camera-topic", default="/observation/static_orbbec/color/image_raw"
    )
    parser.add_argument(
        "--left-wrist-topic",
        default="/observation/left_hand_realsense/color/image_raw",
    )
    parser.add_argument(
        "--right-wrist-topic",
        default="/observation/right_hand_realsense/color/image_raw",
    )
    parser.add_argument("--input-timeout-s", type=float, default=10.0)
    parser.add_argument("--max-input-age-s", type=float, default=0.5)
    parser.add_argument(
        "--record-episodes",
        type=int,
        default=0,
        help="record this many successful SmolVLA-assisted MCAP episodes; 0 keeps continuous inference",
    )
    parser.add_argument(
        "--teleop-takeover",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="enable local Piper leader takeover through T or /smolvla_piper/set_teleop",
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
        help="load the checkpoint and run one synthetic task-conditioned inference without ROS or motion",
    )
    return parser


def main() -> None:
    _configure_logging()
    args = _build_parser().parse_args()
    if not args.task.strip():
        raise SystemExit(
            "--task must be the non-empty task sentence used during training"
        )
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.record_episodes < 0:
        raise SystemExit("--record-episodes must be non-negative")

    runner = SmolVlaRunner(args.checkpoint, args.device, args.task)
    if args.dry_run:
        state = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        images = {
            name: np.zeros(shape, dtype=np.uint8)
            for name, shape in IMAGE_SHAPES.items()
        }
        action = runner.predict(_raw_from_arrays(state, images))
        LOGGER.info(
            "dry-run passed: task=%r, action shape=%s, finite=%s",
            args.task,
            action.shape,
            np.isfinite(action).all(),
        )
        return

    args.policy_type = "smolvla"
    _run_ros(args, runner)


if __name__ == "__main__":
    main()
