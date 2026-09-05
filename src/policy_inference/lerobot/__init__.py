"""LeRobot-specific integrations with lazy model/runtime imports."""

from __future__ import annotations

from typing import Any

__all__ = [
    "DryRunResult",
    "LeRobotPolicy",
    "LeRobotPolicyBundle",
    "LeRobotToRmiActionBridge",
    "PolicyCompatibilityError",
    "PolicyContractManifest",
    "RmiToLeRobotObservationBridge",
    "load_policy_bundle",
    "load_validated_policy_bundle",
    "resolve_checkpoint",
    "ros_image_to_numpy",
    "supports_native_rtc",
    "synthetic_sync_dry_run",
    "validate_policy_compatibility",
    "write_contract_manifest",
]


def __getattr__(name: str) -> Any:
    if name in {
        "LeRobotPolicy",
        "LeRobotPolicyBundle",
        "load_policy_bundle",
        "load_validated_policy_bundle",
        "supports_native_rtc",
    }:
        from .policy import (
            LeRobotPolicy,
            LeRobotPolicyBundle,
            load_policy_bundle,
            load_validated_policy_bundle,
            supports_native_rtc,
        )

        return {
            "LeRobotPolicy": LeRobotPolicy,
            "LeRobotPolicyBundle": LeRobotPolicyBundle,
            "load_policy_bundle": load_policy_bundle,
            "load_validated_policy_bundle": load_validated_policy_bundle,
            "supports_native_rtc": supports_native_rtc,
        }[name]
    if name in {
        "PolicyCompatibilityError",
        "PolicyContractManifest",
        "resolve_checkpoint",
        "validate_policy_compatibility",
        "write_contract_manifest",
    }:
        from . import compatibility

        return getattr(compatibility, name)
    if name in {
        "LeRobotToRmiActionBridge",
        "RmiToLeRobotObservationBridge",
        "ros_image_to_numpy",
    }:
        from .bridge import (
            LeRobotToRmiActionBridge,
            RmiToLeRobotObservationBridge,
            ros_image_to_numpy,
        )

        return {
            "LeRobotToRmiActionBridge": LeRobotToRmiActionBridge,
            "RmiToLeRobotObservationBridge": RmiToLeRobotObservationBridge,
            "ros_image_to_numpy": ros_image_to_numpy,
        }[name]
    if name in {"DryRunResult", "synthetic_sync_dry_run"}:
        from .validation import DryRunResult, synthetic_sync_dry_run

        return {
            "DryRunResult": DryRunResult,
            "synthetic_sync_dry_run": synthetic_sync_dry_run,
        }[name]
    raise AttributeError(name)
