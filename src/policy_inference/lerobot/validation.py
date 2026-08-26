"""No-ROS, no-motion validation of a loaded LeRobot policy bundle."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..common.contract import PolicyIOContract
from .policy import LeRobotPolicyBundle
from .utils import make_dataset_features


@dataclass(frozen=True)
class DryRunResult:
    action_shape: tuple[int, ...]
    finite: bool


def synthetic_sync_dry_run(
    bundle: LeRobotPolicyBundle,
    contract: PolicyIOContract,
    *,
    task: str,
    device: str,
) -> DryRunResult:
    """Execute one native synchronous inference without ROS or an RMI session."""
    if not task.strip():
        raise ValueError("synthetic dry-run requires a non-empty task")

    from lerobot.rollout.inference.sync import SyncInferenceEngine
    from lerobot.utils.constants import OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    dataset_features = make_dataset_features(contract)
    raw_observation = {name: 0.0 for name in contract.state_feature_names}
    raw_observation.update(
        {
            name.removeprefix("observation.images."): np.zeros(shape, dtype=np.uint8)
            for name, shape in contract.camera_shapes.items()
        }
    )
    frame = build_dataset_frame(dataset_features, raw_observation, prefix=OBS_STR)
    engine = SyncInferenceEngine(
        policy=bundle.policy,
        preprocessor=bundle.preprocessor,
        postprocessor=bundle.postprocessor,
        dataset_features=dataset_features,
        ordered_action_keys=list(contract.action_feature_names),
        task=task,
        device=device,
        robot_type="rmi",
    )
    action = engine.get_action(frame)
    if action is None:
        raise RuntimeError("policy returned no action during synthetic dry-run")
    values = action.detach().cpu().numpy()
    finite = bool(np.isfinite(values).all())
    if values.shape != (contract.action_dim,):
        raise ValueError(
            f"dry-run action shape {values.shape} != profile ({contract.action_dim},)"
        )
    if not finite:
        raise ValueError("dry-run action contains NaN or Inf")
    return DryRunResult(tuple(values.shape), finite)
