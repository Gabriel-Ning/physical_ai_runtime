from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).parents[2]


def _load_app(name: str):
    spec = importlib.util.spec_from_file_location(
        name, REPO_ROOT / "apps" / f"{name}.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


record = _load_app("record")
teleop = _load_app("teleop")


class _Future:
    def __init__(self, response):
        self._response = response

    def done(self):
        return True

    def result(self):
        return self._response


class _Client:
    def __init__(self, response, available=True):
        self._response = response
        self._available = available

    def wait_for_service(self, timeout_sec):
        assert timeout_sec > 0.0
        return self._available

    def call_async(self, request):
        return _Future(self._response)


class _Node:
    def __init__(self, responses):
        self.context = SimpleNamespace(ok=lambda: True)
        self._responses = iter(responses)
        self.created = []
        self.destroyed = []

    def create_client(self, message_type, service_name):
        del message_type
        client = _Client(next(self._responses))
        self.created.append((service_name, client))
        return client

    def destroy_client(self, client):
        self.destroyed.append(client)


TELEOPERATORS = {
    "left": {"preempt_service": "/left/preempt"},
    "right": {"preempt_service": "/right/preempt"},
}


@pytest.mark.parametrize("app", [record, teleop])
def test_preempt_requires_every_leader_and_destroys_clients(app):
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=False, message="rejected"),
        ]
    )

    assert app.set_teleop_preempt(node, TELEOPERATORS, True) is False
    assert [name for name, _ in node.created] == ["/left/preempt", "/right/preempt"]
    assert node.destroyed == [client for _, client in node.created]


@pytest.mark.parametrize("app", [record, teleop])
def test_preempt_succeeds_only_when_every_leader_confirms(app):
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=True, message="ready"),
        ]
    )

    assert app.set_teleop_preempt(node, TELEOPERATORS, True) is True


@pytest.mark.parametrize(
    "value",
    ["", "   ", ".", "..", "task/name", "task\\name", "task\0name"],
)
def test_task_name_rejects_unsafe_directory_values(value):
    with pytest.raises(ValueError, match="task name"):
        record._normalize_task_name(value)


def test_task_recorder_config_aligns_task_and_dataset_directory():
    config = record._task_recorder_config(
        {
            "root_dir": "data/episodes",
            "experiment_name": "profile_default",
            "task": "profile_default",
            "max_duration_s": 42.0,
            "episodes": 10,
        },
        "pick_bread",
        "operator-a",
    )

    assert config.root_dir == "data/episodes"
    assert config.experiment_name == "pick_bread"
    assert config.task == "pick_bread"
    assert config.operator_name == "operator-a"
    assert config.max_episode_duration == 42.0


def test_finalized_episode_directory_accepts_directory_and_mcap(tmp_path):
    episode_dir = tmp_path / "episode_000001"
    episode_dir.mkdir()
    directory_scope = SimpleNamespace(
        final_status=SimpleNamespace(episode_path=str(episode_dir))
    )
    file_scope = SimpleNamespace(
        final_status=SimpleNamespace(episode_path=str(episode_dir / "episode.mcap"))
    )

    assert record._finalized_episode_directory(directory_scope) == episode_dir
    assert record._finalized_episode_directory(file_scope) == episode_dir
