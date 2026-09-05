from __future__ import annotations

import subprocess
import sys

import numpy as np
from test_common_runtime import _layout

from policy_inference.lerobot.policy import LeRobotPolicy


def test_lerobot_policy_loader_import_is_lazy() -> None:
    code = """
import sys
import policy_inference.lerobot.policy
assert 'torch' not in sys.modules
assert 'lerobot.policies' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True)


class _ObservationBridge:
    def encode(self, observation):
        assert observation is _OBSERVATION
        return {"encoded": observation}


class _ActionBridge:
    def decode(self, action):
        return ("decoded", action)


class _Engine:
    def __init__(self, action):
        self.action = action
        self.frames = []
        self.reset_count = 0
        self.stop_count = 0

    def get_action(self, frame):
        self.frames.append(frame)
        return self.action

    def reset(self):
        self.reset_count += 1

    def stop(self):
        self.stop_count += 1


_OBSERVATION = object()


def _policy(action):
    policy = LeRobotPolicy.__new__(LeRobotPolicy)
    policy.layout = _layout()
    policy._observation_bridge = _ObservationBridge()
    policy._action_bridge = _ActionBridge()
    policy._dataset_features = {}
    policy._engine = _Engine(action)
    policy._closed = False
    return policy


def test_policy_exposes_one_select_action_boundary(monkeypatch) -> None:
    monkeypatch.setattr(
        "lerobot.utils.feature_utils.build_dataset_frame",
        lambda features, raw, *, prefix: {
            "features": features,
            "raw": raw,
            "prefix": prefix,
        },
    )
    action = np.arange(5.0)
    policy = _policy(action)

    result = policy.select_action(_OBSERVATION)

    assert result == ("decoded", action)
    assert policy._engine.frames[0]["raw"] == {"encoded": _OBSERVATION}


def test_policy_preserves_no_action_and_owns_only_engine_lifecycle(monkeypatch) -> None:
    monkeypatch.setattr(
        "lerobot.utils.feature_utils.build_dataset_frame",
        lambda features, raw, *, prefix: raw,
    )
    policy = _policy(None)

    assert policy.select_action(_OBSERVATION) is None
    policy.reset()
    policy.close()
    policy.close()

    assert policy._engine.reset_count == 1
    assert policy._engine.stop_count == 1
