"""Offline checks between an RMI profile contract and a LeRobot checkpoint."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ..common.contract import PolicyIOContract

CONTRACT_MANIFEST = "policy_contract.json"


class PolicyCompatibilityError(ValueError):
    """Raised before inference when profile and checkpoint semantics disagree."""


@dataclass(frozen=True)
class PolicyContractManifest:
    profile: str
    profile_hash: str
    state_names: tuple[str, ...]
    action_names: tuple[str, ...]
    image_features: tuple[str, ...]

    @classmethod
    def from_contract(cls, contract: PolicyIOContract) -> PolicyContractManifest:
        return cls(
            profile=contract.profile_name,
            profile_hash=contract.profile_hash,
            state_names=contract.state_feature_names,
            action_names=contract.action_feature_names,
            image_features=tuple(contract.camera_shapes),
        )

    @classmethod
    def from_json(cls, path: str | Path) -> PolicyContractManifest:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(
            profile=str(data["profile"]),
            profile_hash=str(data["profile_hash"]),
            state_names=tuple(data["state_names"]),
            action_names=tuple(data["action_names"]),
            image_features=tuple(data["image_features"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "profile": self.profile,
            "profile_hash": self.profile_hash,
            "state_names": list(self.state_names),
            "action_names": list(self.action_names),
            "image_features": list(self.image_features),
        }


@dataclass(frozen=True)
class CompatibilityReport:
    policy_type: str
    checkpoint: str
    warnings: tuple[str, ...] = ()


def resolve_checkpoint(checkpoint: str | Path) -> str:
    """Resolve common local LeRobot run layouts, leaving Hub repository IDs intact."""
    raw = str(checkpoint)
    path = Path(raw).expanduser()
    if path.is_dir():
        candidates = (
            path,
            path / "pretrained_model",
            path / "checkpoints" / "last" / "pretrained_model",
        )
        for candidate in candidates:
            if (candidate / "config.json").is_file():
                return str(candidate.resolve())
        raise FileNotFoundError(f"no LeRobot config.json found below {path}")
    if path.exists():
        raise ValueError(f"checkpoint must be a directory, got {path}")
    if path.is_absolute() or raw.startswith(("./", "../", "~")):
        raise FileNotFoundError(path)
    return raw


def load_contract_manifest(checkpoint: str) -> PolicyContractManifest | None:
    path = Path(checkpoint)
    manifest = path / CONTRACT_MANIFEST
    return PolicyContractManifest.from_json(manifest) if manifest.is_file() else None


def write_contract_manifest(output_dir: str | Path, contract: PolicyIOContract) -> Path:
    """Write the semantic vector/image order beside a dataset or trained checkpoint."""
    path = Path(output_dir) / CONTRACT_MANIFEST
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = PolicyContractManifest.from_contract(contract).to_dict()
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path


def validate_policy_compatibility(
    contract: PolicyIOContract,
    config: Any,
    *,
    checkpoint: str,
    expected_policy_type: str | None = None,
    rename_map: Mapping[str, str] | None = None,
    manifest: PolicyContractManifest | None = None,
    allow_spatial_resize: bool = True,
) -> CompatibilityReport:
    """Fail on feature/shape mismatches and verify element order when a manifest exists."""
    errors: list[str] = []
    warnings: list[str] = []
    policy_type = str(config.type)
    if expected_policy_type is not None and policy_type != expected_policy_type:
        errors.append(
            f"policy type {policy_type!r} != requested {expected_policy_type!r}"
        )

    mapped_images = {
        (rename_map or {}).get(name, name): shape
        for name, shape in contract.camera_shapes.items()
    }
    checkpoint_images = {
        name: tuple(feature.shape)
        for name, feature in config.input_features.items()
        if name.startswith("observation.image")
        and not name.startswith("observation.images.empty_camera_")
    }
    if set(mapped_images) != set(checkpoint_images):
        errors.append(
            "image features differ: "
            f"profile={sorted(mapped_images)}, checkpoint={sorted(checkpoint_images)}"
        )
    for name in mapped_images.keys() & checkpoint_images.keys():
        height, width, channels = mapped_images[name]
        profile_shape = (channels, height, width)
        if profile_shape != checkpoint_images[name]:
            if channels != checkpoint_images[name][0] or not allow_spatial_resize:
                errors.append(
                    f"image shape {name}: profile={profile_shape}, "
                    f"checkpoint={checkpoint_images[name]}"
                )
            else:
                warnings.append(
                    f"{name} will be resized from {profile_shape[1:]} "
                    f"to {checkpoint_images[name][1:]}"
                )

    _check_vector_shape(
        errors, config.input_features, "observation.state", contract.action_dim
    )
    _check_vector_shape(errors, config.output_features, "action", contract.action_dim)

    if manifest is None:
        warnings.append(
            f"{CONTRACT_MANIFEST} is absent; vector dimensions were checked but joint order cannot be proven"
        )
    else:
        if manifest.state_names != contract.state_feature_names:
            errors.append("checkpoint state_names do not match profile joint order")
        if manifest.action_names != contract.action_feature_names:
            errors.append("checkpoint action_names do not match profile joint order")
        expected_images = tuple(
            (rename_map or {}).get(name, name) for name in contract.camera_shapes
        )
        if manifest.image_features != expected_images:
            errors.append(
                "checkpoint image_features do not match profile feature order"
            )

    if errors:
        raise PolicyCompatibilityError("; ".join(errors))
    return CompatibilityReport(policy_type, checkpoint, tuple(warnings))


def _check_vector_shape(
    errors: list[str], features: Mapping[str, Any], name: str, dimension: int
) -> None:
    feature = features.get(name)
    if feature is None:
        errors.append(f"checkpoint has no {name}")
    elif tuple(feature.shape) != (dimension,):
        errors.append(f"{name} shape {tuple(feature.shape)} != profile ({dimension},)")
