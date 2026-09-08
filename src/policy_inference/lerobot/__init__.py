"""LeRobot backend adapter for RMI, with lazy model/runtime imports.

Module map::

    policy.py         LeRobotPolicy — the select_action producer boundary
    bridges/          RMI <-> LeRobot data boundary (encode / decode pair)
    engines/          InferenceBackend implementations (sync / async / remote)
    loader.py         checkpoint -> weights + processors
    compatibility.py  Profile <-> checkpoint contract checks
    features.py       LeRobot dataset / observation feature schemas
    geometry.py       SO(3) + LIBERO EE state packing
    telemetry.py      action-chunk bookkeeping for eval logging
    dry_run.py        no-ROS synthetic inference smoke test
"""

from __future__ import annotations

from typing import Any

__all__ = [
    "ActionDecoder",
    "CartesianActionDecoder",
    "ChunkTelemetry",
    "DryRunResult",
    "InferenceBackend",
    "JointActionDecoder",
    "LeRobotPolicy",
    "LeRobotPolicyBundle",
    "ObservationEncoder",
    "PolicyCompatibilityError",
    "PolicyContractManifest",
    "PolicyTransport",
    "RemoteBackend",
    "RemoteBackendConfig",
    "ResidualRmiEnv",
    "RmiEnv",
    "gripper_joint_values",
    "load_policy_bundle",
    "load_validated_policy_bundle",
    "make_action_decoder",
    "make_dataset_features",
    "make_inference_backend",
    "make_observation_features",
    "normalize_inference_mode",
    "pack_libero_ee_state",
    "resolve_checkpoint",
    "supports_native_rtc",
    "synthetic_dry_run",
    "validate_policy_compatibility",
    "write_contract_manifest",
]


_LAZY = {
    "LeRobotPolicy": (".policy", "LeRobotPolicy"),
    "ActionDecoder": (".bridges", "ActionDecoder"),
    "CartesianActionDecoder": (".bridges", "CartesianActionDecoder"),
    "JointActionDecoder": (".bridges", "JointActionDecoder"),
    "ObservationEncoder": (".bridges", "ObservationEncoder"),
    "make_action_decoder": (".bridges", "make_action_decoder"),
    "InferenceBackend": (".engines", "InferenceBackend"),
    "PolicyTransport": (".engines", "PolicyTransport"),
    "RemoteBackend": (".engines", "RemoteBackend"),
    "RemoteBackendConfig": (".engines", "RemoteBackendConfig"),
    "RmiEnv": (".env", "RmiEnv"),
    "ResidualRmiEnv": (".env", "ResidualRmiEnv"),
    "make_inference_backend": (".engines", "make_inference_backend"),
    "normalize_inference_mode": (".engines", "normalize_inference_mode"),
    "LeRobotPolicyBundle": (".loader", "LeRobotPolicyBundle"),
    "load_policy_bundle": (".loader", "load_policy_bundle"),
    "load_validated_policy_bundle": (".loader", "load_validated_policy_bundle"),
    "supports_native_rtc": (".loader", "supports_native_rtc"),
    "PolicyCompatibilityError": (".compatibility", "PolicyCompatibilityError"),
    "PolicyContractManifest": (".compatibility", "PolicyContractManifest"),
    "resolve_checkpoint": (".compatibility", "resolve_checkpoint"),
    "validate_policy_compatibility": (
        ".compatibility",
        "validate_policy_compatibility",
    ),
    "write_contract_manifest": (".compatibility", "write_contract_manifest"),
    "make_dataset_features": (".features", "make_dataset_features"),
    "make_observation_features": (".features", "make_observation_features"),
    "gripper_joint_values": (".geometry", "gripper_joint_values"),
    "pack_libero_ee_state": (".geometry", "pack_libero_ee_state"),
    "ChunkTelemetry": (".telemetry", "ChunkTelemetry"),
    "DryRunResult": (".dry_run", "DryRunResult"),
    "synthetic_dry_run": (".dry_run", "synthetic_dry_run"),
}


def __getattr__(name: str) -> Any:
    try:
        module_name, attr = _LAZY[name]
    except KeyError as exc:
        raise AttributeError(name) from exc
    from importlib import import_module

    module = import_module(module_name, __name__)
    value = getattr(module, attr)
    globals()[name] = value
    return value
