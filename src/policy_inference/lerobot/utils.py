"""Small builders around native LeRobot feature and processor utilities."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from ..common.contract import PolicyIOContract


def make_dataset_features(
    contract: PolicyIOContract, *, use_video: bool = False
) -> dict[str, dict]:
    """Build the one dataset schema shared by conversion and inference."""
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import hw_to_dataset_features

    observations: dict[str, type | tuple[int, int, int]] = {
        name: float for name in contract.state_feature_names
    }
    observations.update(
        {
            name.removeprefix("observation.images."): shape
            for name, shape in contract.camera_shapes.items()
        }
    )
    actions = {name: float for name in contract.action_feature_names}
    return {
        **hw_to_dataset_features(observations, OBS_STR, use_video=use_video),
        **hw_to_dataset_features(actions, ACTION, use_video=use_video),
    }


def make_native_resize_step(
    contract: PolicyIOContract,
    checkpoint_image_features: Mapping[str, Any],
    *,
    rename_map: Mapping[str, str] | None = None,
) -> Any | None:
    """Return LeRobot's resize step only when every checkpoint camera needs one size."""
    targets: set[tuple[int, int]] = set()
    needs_resize = False
    for profile_name, (height, width, channels) in contract.camera_shapes.items():
        policy_name = (rename_map or {}).get(profile_name, profile_name)
        feature = checkpoint_image_features.get(policy_name)
        if feature is None:
            continue
        target_channels, target_height, target_width = tuple(feature.shape)
        if target_channels != channels:
            raise ValueError(
                f"cannot resize {policy_name}: channel count {channels} != {target_channels}"
            )
        targets.add((target_height, target_width))
        needs_resize |= (height, width) != (target_height, target_width)
    if not needs_resize:
        return None
    if len(targets) != 1:
        raise ValueError(
            f"native LeRobot resize requires one common image size, got {sorted(targets)}"
        )

    from lerobot.processor import ImageCropResizeProcessorStep

    return ImageCropResizeProcessorStep(resize_size=next(iter(targets)))


def install_native_resize_step(preprocessor: Any, resize_step: Any | None) -> None:
    """Insert resize after device conversion and before checkpoint normalization."""
    if resize_step is None:
        return
    from lerobot.processor import NormalizerProcessorStep

    steps = list(preprocessor.steps)
    try:
        index = next(
            index
            for index, step in enumerate(steps)
            if isinstance(step, NormalizerProcessorStep)
        )
    except StopIteration as exc:
        raise ValueError(
            "checkpoint preprocessor has no normalization insertion point"
        ) from exc
    steps.insert(index, resize_step)
    preprocessor.steps = steps
