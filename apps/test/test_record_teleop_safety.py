from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

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
PREEMPT_SERVICES = ["/left/preempt", "/right/preempt"]


def test_load_teleoperators_from_workstation_yaml_not_profile():
    profile = SimpleNamespace(
        raw_data={
            "execution_manager_config": {
                "package": "piper_manipulation_workstation_launch",
                "file": "config/execution_manager.yaml",
            }
        }
    )
    loaded = record.load_teleoperators(profile)
    left = loaded["piper_leader_left"]
    assert left["preempt_service"] == "/piper_leader_left/preempt"
    assert left["arm_part"] == "left_arm"
    assert left["gripper_part"] == "left_gripper"
    assert left["target_node"] == "TeleopJoint_Left"
    assert loaded["piper_leader_right"]["target_node"] == "TeleopJoint_Right"
    assert loaded["piper_leader_right"]["preempt_service"] == (
        "/piper_leader_right/preempt"
    )


def test_load_teleoperators_follows_profile_package_and_skips_franka():
    piper = SimpleNamespace(raw_data=_profile_raw("piper_bimanual.yaml"))
    real = SimpleNamespace(raw_data=_profile_raw("site/piper_bimanual_real.yaml"))
    assert set(record.load_teleoperators(piper)) == {
        "piper_leader_left",
        "piper_leader_right",
    }
    assert set(record.load_teleoperators(real)) == {
        "piper_leader_left",
        "piper_leader_right",
    }
    assert (
        record.load_teleoperators(
            SimpleNamespace(raw_data=_profile_raw("fr3_pika_single_arm.yaml"))
        )
        == {}
    )
    assert (
        record.load_teleoperators(
            SimpleNamespace(raw_data=_profile_raw("marvin_bimanual.yaml"))
        )
        == {}
    )


def _profile_raw(name: str) -> dict:
    return yaml.safe_load(
        (REPO_ROOT / "apps" / "profiles" / name).read_text(encoding="utf-8")
    )


def test_teleop_preempt_services_come_from_workstation_teleop_yaml():
    assert teleop.load_preempt_services(SimpleNamespace(raw_data={})) == []
    assert "teleop" not in _profile_raw("piper_bimanual.yaml")
    assert "teleop" not in _profile_raw("site/piper_bimanual_real.yaml")
    leaders = yaml.safe_load(
        (
            REPO_ROOT
            / "src/bringup/piper_manipulation/workstation_launch/config/teleop/piper_leaders.yaml"
        ).read_text(encoding="utf-8")
    )
    assert leaders["piper_leader_left"]["ros__parameters"]["preempt_service"] == (
        "/piper_leader_left/preempt"
    )
    assert leaders["piper_leader_right"]["ros__parameters"]["preempt_service"] == (
        "/piper_leader_right/preempt"
    )
    assert teleop.load_preempt_services(
        SimpleNamespace(raw_data=_profile_raw("piper_bimanual.yaml"))
    ) == [
        "/piper_leader_left/preempt",
        "/piper_leader_right/preempt",
    ]
    assert teleop.load_preempt_services(
        SimpleNamespace(raw_data=_profile_raw("site/piper_bimanual_real.yaml"))
    ) == [
        "/piper_leader_left/preempt",
        "/piper_leader_right/preempt",
    ]
    assert (
        teleop.load_preempt_services(
            SimpleNamespace(raw_data=_profile_raw("fr3_pika_single_arm.yaml"))
        )
        == []
    )
    assert (
        teleop.load_preempt_services(
            SimpleNamespace(raw_data=_profile_raw("marvin_bimanual.yaml"))
        )
        == []
    )


def test_teleop_node_names_are_profile_teleop_sources():
    nodes = {
        "JointPolicy": SimpleNamespace(source_role="POLICY"),
        "TeleopJoint_Left": SimpleNamespace(source_role="TELEOP"),
        "TeleopTwist": SimpleNamespace(source_role="TELEOP"),
    }
    assert teleop.teleop_node_names(SimpleNamespace(nodes=nodes)) == [
        "TeleopJoint_Left",
        "TeleopTwist",
    ]


def test_record_preempt_requires_every_leader_and_destroys_clients():
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=False, message="rejected"),
        ]
    )

    assert record.set_teleop_preempt(node, TELEOPERATORS, True) is False
    assert [name for name, _ in node.created] == ["/left/preempt", "/right/preempt"]
    assert node.destroyed == [client for _, client in node.created]


def test_teleop_preempt_requires_every_service_and_destroys_clients():
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=False, message="rejected"),
        ]
    )

    assert teleop.set_teleop_preempt(node, PREEMPT_SERVICES, True) is False
    assert [name for name, _ in node.created] == ["/left/preempt", "/right/preempt"]
    assert node.destroyed == [client for _, client in node.created]


def test_record_verify_leader_preempt_services_requires_both():
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=True, message="ready"),
        ]
    )
    assert record.verify_leader_preempt_services(node, TELEOPERATORS) is True
    assert [name for name, _ in node.created] == ["/left/preempt", "/right/preempt"]
    assert node.destroyed == [client for _, client in node.created]


def test_teleop_verify_leader_preempt_services_requires_both():
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=True, message="ready"),
        ]
    )
    assert teleop.verify_leader_preempt_services(node, PREEMPT_SERVICES) is True
    assert [name for name, _ in node.created] == ["/left/preempt", "/right/preempt"]
    assert node.destroyed == [client for _, client in node.created]


def test_record_preempt_succeeds_only_when_every_leader_confirms():
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=True, message="ready"),
        ]
    )

    assert record.set_teleop_preempt(node, TELEOPERATORS, True) is True


def test_teleop_preempt_succeeds_only_when_every_service_confirms():
    node = _Node(
        [
            SimpleNamespace(success=True, message="ready"),
            SimpleNamespace(success=True, message="ready"),
        ]
    )

    assert teleop.set_teleop_preempt(node, PREEMPT_SERVICES, True) is True


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


def test_record_and_teleop_expose_use_sim_time_flag():
    source = (REPO_ROOT / "apps" / "record.py").read_text(encoding="utf-8")
    teleop_source = (REPO_ROOT / "apps" / "teleop.py").read_text(encoding="utf-8")
    assert "--use-sim-time" in source
    assert "--use-sim-time" in teleop_source
    assert "def _open_context(" in source
    assert "def _open_context(" in teleop_source


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
