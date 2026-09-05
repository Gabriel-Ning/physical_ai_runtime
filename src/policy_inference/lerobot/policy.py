"""Thin loader around LeRobot's native policy and processor factories."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from rmi import PolicyLayout

from .bridge import LeRobotToRmiActionBridge, RmiToLeRobotObservationBridge


@dataclass(frozen=True)
class LeRobotPolicyBundle:
    policy: Any
    preprocessor: Any
    postprocessor: Any
    config: Any


class LeRobotPolicy:
    """Local LeRobot policy with the standard ``select_action`` boundary."""

    def __init__(
        self,
        layout: PolicyLayout,
        checkpoint: str,
        *,
        task: str,
        device: str = "cuda",
        revision: str | None = None,
        rename_map: dict[str, str] | None = None,
        expected_policy_type: str | None = None,
        max_stream_skew_s: float = 0.5,
    ) -> None:
        if not task.strip():
            raise ValueError("task must not be empty")

        from lerobot.rollout.inference.sync import SyncInferenceEngine

        self.layout = layout
        self.bundle, self.compatibility = load_validated_policy_bundle(
            layout,
            checkpoint,
            device=device,
            revision=revision,
            rename_map=rename_map,
            expected_policy_type=expected_policy_type,
        )
        self._observation_bridge = RmiToLeRobotObservationBridge(
            layout,
            max_stream_skew_s=max_stream_skew_s,
        )
        self._action_bridge = LeRobotToRmiActionBridge(layout)

        from .utils import make_dataset_features

        self._dataset_features = make_dataset_features(layout)
        self._engine = SyncInferenceEngine(
            policy=self.bundle.policy,
            preprocessor=self.bundle.preprocessor,
            postprocessor=self.bundle.postprocessor,
            dataset_features=self._dataset_features,
            ordered_action_keys=list(layout.action_feature_names),
            task=task,
            device=device,
            robot_type="rmi",
        )
        self._closed = False
        self._engine.start()

    def select_action(self, observation: Any) -> Any:
        """Return native RMI actions for one RMI observation."""
        if self._closed:
            raise RuntimeError("policy is closed")

        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import build_dataset_frame

        raw = self._observation_bridge.encode(observation)
        frame = build_dataset_frame(self._dataset_features, raw, prefix=OBS_STR)
        action = self._engine.get_action(frame)
        return None if action is None else self._action_bridge.decode(action)

    def reset(self) -> None:
        """Reset policy, processor, and buffered action state."""
        if self._closed:
            raise RuntimeError("policy is closed")
        self._engine.reset()

    def close(self) -> None:
        """Release inference resources; safe to call more than once."""
        if not self._closed:
            self._closed = True
            self._engine.stop()

def _load_pretrained_config(checkpoint: str, revision: str | None) -> Any:
    # Importing the native policy package registers every config subclass with
    # PreTrainedConfig before it resolves the checkpoint's serialized type.
    import lerobot.policies  # noqa: F401
    from lerobot.configs import PreTrainedConfig

    return PreTrainedConfig.from_pretrained(checkpoint, revision=revision)


def load_policy_bundle(
    checkpoint: str,
    *,
    device: str,
    revision: str | None = None,
    rename_map: dict[str, str] | None = None,
) -> LeRobotPolicyBundle:
    """Load checkpoint-native model and processors without importing LeRobot at module import."""
    from .compatibility import resolve_checkpoint

    checkpoint = resolve_checkpoint(checkpoint)
    from lerobot.policies import get_policy_class, make_pre_post_processors

    config = _load_pretrained_config(checkpoint, revision)
    policy_class = get_policy_class(config.type)
    policy = policy_class.from_pretrained(
        checkpoint,
        config=config,
        revision=revision,
    )
    policy = policy.to(device)
    policy.eval()
    preprocessor_overrides = {"device_processor": {"device": device}}
    if rename_map is not None:
        preprocessor_overrides["rename_observations_processor"] = {
            "rename_map": rename_map
        }
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=checkpoint,
        pretrained_revision=revision,
        preprocessor_overrides=preprocessor_overrides,
    )
    return LeRobotPolicyBundle(policy, preprocessor, postprocessor, config)


def load_validated_policy_bundle(
    layout: PolicyLayout,
    checkpoint: str,
    *,
    device: str,
    revision: str | None = None,
    rename_map: dict[str, str] | None = None,
    expected_policy_type: str | None = None,
) -> tuple[LeRobotPolicyBundle, Any]:
    """Validate semantics before loading weights, then install native image resize."""
    from .compatibility import (
        load_contract_manifest,
        resolve_checkpoint,
        validate_policy_compatibility,
    )
    from .utils import install_native_resize_step, make_native_resize_step

    resolved = resolve_checkpoint(checkpoint)
    config = _load_pretrained_config(resolved, revision)
    report = validate_policy_compatibility(
        layout,
        config,
        checkpoint=resolved,
        expected_policy_type=expected_policy_type,
        rename_map=rename_map,
        manifest=load_contract_manifest(resolved),
    )
    bundle = load_policy_bundle(
        resolved, device=device, revision=revision, rename_map=rename_map
    )
    resize_step = make_native_resize_step(
        layout, config.image_features, rename_map=rename_map
    )
    install_native_resize_step(bundle.preprocessor, resize_step)
    return bundle, report


def supports_native_rtc(policy: Any) -> bool:
    """Use LeRobot's capability and call-signature check as the only RTC gate."""
    from lerobot.rollout.inference.rtc import supports_rtc_inference

    return supports_rtc_inference(policy)
