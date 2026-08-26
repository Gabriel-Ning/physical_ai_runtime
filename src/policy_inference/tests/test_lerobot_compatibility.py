from __future__ import annotations

from types import SimpleNamespace

import pytest
from test_common_runtime import _Profile

from policy_inference.common.contract import PolicyIOContract
from policy_inference.lerobot.compatibility import (
    PolicyCompatibilityError,
    PolicyContractManifest,
    resolve_checkpoint,
    validate_policy_compatibility,
    write_contract_manifest,
)
from policy_inference.lerobot.utils import (
    make_dataset_features,
    make_native_resize_step,
)


def _feature(*shape: int) -> SimpleNamespace:
    return SimpleNamespace(shape=shape)


def _config(*, image_shape=(3, 8, 8), policy_type="act") -> SimpleNamespace:
    return SimpleNamespace(
        type=policy_type,
        input_features={
            "observation.state": _feature(5),
            "observation.images.wrist": _feature(*image_shape),
        },
        output_features={"action": _feature(5)},
    )


def test_resolve_checkpoint_accepts_run_layout_and_hub_id(tmp_path) -> None:
    pretrained = tmp_path / "checkpoints" / "last" / "pretrained_model"
    pretrained.mkdir(parents=True)
    (pretrained / "config.json").write_text("{}", encoding="utf-8")

    assert resolve_checkpoint(tmp_path) == str(pretrained.resolve())
    assert resolve_checkpoint("organization/policy") == "organization/policy"


def test_compatibility_requires_exact_features_and_warns_without_manifest() -> None:
    contract = PolicyIOContract.from_profile(_Profile())

    report = validate_policy_compatibility(
        contract, _config(), checkpoint="organization/policy"
    )

    assert report.policy_type == "act"
    assert "joint order cannot be proven" in report.warnings[0]


def test_compatibility_rejects_semantic_joint_order_from_manifest() -> None:
    contract = PolicyIOContract.from_profile(_Profile())
    manifest = PolicyContractManifest.from_contract(contract)
    wrong = PolicyContractManifest(
        profile=manifest.profile,
        profile_hash=manifest.profile_hash,
        state_names=tuple(reversed(manifest.state_names)),
        action_names=manifest.action_names,
        image_features=manifest.image_features,
    )

    with pytest.raises(PolicyCompatibilityError, match="state_names"):
        validate_policy_compatibility(
            contract, _config(), checkpoint="organization/policy", manifest=wrong
        )


def test_manifest_round_trip(tmp_path) -> None:
    manifest = PolicyContractManifest.from_contract(
        PolicyIOContract.from_profile(_Profile())
    )
    path = write_contract_manifest(tmp_path, PolicyIOContract.from_profile(_Profile()))

    assert PolicyContractManifest.from_json(path) == manifest


def test_compatibility_allows_native_spatial_resize_but_not_channel_change() -> None:
    contract = PolicyIOContract.from_profile(_Profile())

    report = validate_policy_compatibility(
        contract, _config(image_shape=(3, 4, 6)), checkpoint="organization/policy"
    )
    assert "will be resized" in report.warnings[0]

    with pytest.raises(PolicyCompatibilityError, match="image shape"):
        validate_policy_compatibility(
            contract, _config(image_shape=(1, 4, 6)), checkpoint="organization/policy"
        )


def test_dataset_schema_preserves_joint_order_and_policy_camera_name() -> None:
    contract = PolicyIOContract.from_profile(_Profile())

    features = make_dataset_features(contract)

    assert features["observation.state"]["names"] == list(contract.state_feature_names)
    assert features["action"]["names"] == list(contract.action_feature_names)
    assert features["observation.images.wrist"]["shape"] == (8, 8, 3)


def test_native_resize_step_is_inserted_only_when_needed() -> None:
    contract = PolicyIOContract.from_profile(_Profile())

    assert (
        make_native_resize_step(
            contract, {"observation.images.wrist": _feature(3, 8, 8)}
        )
        is None
    )
    step = make_native_resize_step(
        contract, {"observation.images.wrist": _feature(3, 4, 6)}
    )

    assert step.resize_size == (4, 6)


def test_empty_smolvla_camera_does_not_count_as_required_profile_input() -> None:
    contract = PolicyIOContract.from_profile(_Profile())
    config = _config(policy_type="smolvla")
    config.input_features["observation.images.empty_camera_0"] = _feature(3, 8, 8)

    report = validate_policy_compatibility(
        contract, config, checkpoint="organization/policy"
    )

    assert report.policy_type == "smolvla"
