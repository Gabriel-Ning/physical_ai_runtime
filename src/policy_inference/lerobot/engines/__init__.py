"""Inference backends: local Sync/RTC and remote transport adapters."""

from __future__ import annotations

from typing import Any

from rmi import PolicyLayout

from ..loader import LeRobotPolicyBundle
from .base import InferenceBackend, InferenceMode, normalize_inference_mode
from .learner_client import LeRobotLearnerClient
from .local import LocalAsyncBackend, LocalSyncBackend, make_local_backend
from .remote import (
    LeRobotRemoteTransport,
    PolicyTransport,
    RemoteBackend,
    RemoteBackendConfig,
)

__all__ = [
    "InferenceBackend",
    "InferenceMode",
    "LeRobotLearnerClient",
    "LeRobotRemoteTransport",
    "LocalAsyncBackend",
    "LocalSyncBackend",
    "PolicyTransport",
    "RemoteBackend",
    "RemoteBackendConfig",
    "make_inference_backend",
    "make_local_backend",
    "normalize_inference_mode",
]


def make_inference_backend(
    *,
    inference: InferenceMode | str,
    layout: PolicyLayout,
    task: str,
    device: str,
    dataset_features: dict[str, dict],
    observation_features: dict[str, dict],
    bundle: LeRobotPolicyBundle | None = None,
    rtc_config: Any | None = None,
    rtc_queue_threshold: int = 30,
    remote_transport: Any | None = None,
    remote_grpc_address: str | None = None,
    remote_config: RemoteBackendConfig | None = None,
    expected_policy_type: str | None = None,
    checkpoint: str | None = None,
    rename_map: dict[str, str] | None = None,
) -> InferenceBackend:
    """Factory for local and remote backends behind ``LeRobotPolicy``."""
    mode = normalize_inference_mode(inference)
    if mode in {"sync", "async"}:
        if bundle is None:
            raise ValueError(f"local inference={mode!r} requires a loaded policy bundle")
        return make_local_backend(
            bundle,
            layout,
            inference=mode,
            task=task,
            device=device,
            dataset_features=dataset_features,
            observation_features=observation_features,
            rtc_config=rtc_config,
            rtc_queue_threshold=rtc_queue_threshold,
        )
    cfg = remote_config or RemoteBackendConfig()
    if remote_transport is None:
        if remote_grpc_address:
            policy_type = expected_policy_type or "diffusion"
            # Keep client/server chunk aligned with each policy's n_action_steps.
            actions_per_chunk = cfg.actions_per_chunk
            if policy_type in {"fastwam", "pi05"} and remote_config is None:
                actions_per_chunk = 10
                cfg = RemoteBackendConfig(
                    chunk_size_threshold=cfg.chunk_size_threshold,
                    actions_per_chunk=actions_per_chunk,
                )
            elif policy_type == "groot" and remote_config is None:
                actions_per_chunk = 16
                cfg = RemoteBackendConfig(
                    chunk_size_threshold=cfg.chunk_size_threshold,
                    actions_per_chunk=actions_per_chunk,
                )
            remote_transport = LeRobotRemoteTransport(
                remote_grpc_address,
                policy_type=policy_type,
                pretrained_name_or_path=checkpoint or "",
                actions_per_chunk=actions_per_chunk,
                device=device,
                lerobot_features=dataset_features,
                rename_map=rename_map,
            )
        else:
            raise ValueError(
                "inference='remote' requires remote_transport or remote_grpc_address "
                "(connecting to LeRobot policy_server via gRPC)"
            )
    return RemoteBackend(remote_transport, config=cfg)

