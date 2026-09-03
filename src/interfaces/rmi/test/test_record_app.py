import ast
import importlib.util
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[4]


def _record_module():
    path = ROOT / "apps" / "record.py"
    spec = importlib.util.spec_from_file_location("piper_record_app", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_multi_episode_loop_keeps_recorder_prepared_until_session_shutdown():
    """STOP preserves subscriptions; per-episode RELEASE breaks the next START."""
    tree = ast.parse((ROOT / "apps" / "record.py").read_text(encoding="utf-8"))
    episode_loops = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.While)
        and ast.unparse(node.test) == "current_ep_idx <= target_episodes"
    ]
    assert len(episode_loops) == 1
    recorder_close_calls = [
        node
        for node in ast.walk(episode_loops[0])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "recorder"
        and node.func.attr == "close"
    ]

    assert recorder_close_calls == []


def test_task_controls_both_episode_label_and_dataset_directory(tmp_path):
    record = _record_module()
    config = record._task_recorder_config(
        {
            "root_dir": str(tmp_path),
            "contract_path": str(tmp_path / "contract.yaml"),
            "experiment_name": "old_fixed_name",
            "task": "old_fixed_task",
            "max_duration_s": 45.0,
        },
        "pick_bread",
        "alpha",
    )

    assert config.task == "pick_bread"
    assert config.experiment_name == "pick_bread"
    assert Path(config.root_dir) / config.experiment_name == tmp_path / "pick_bread"
    assert config.operator_name == "alpha"
    assert config.max_episode_duration == 45.0


@pytest.mark.parametrize("task", ["", "   ", ".", "..", "pick/bread", "pick\\bread"])
def test_task_rejects_empty_or_path_like_values(task):
    record = _record_module()

    with pytest.raises(ValueError, match="task name"):
        record._normalize_task_name(task)


def test_finalized_episode_path_comes_from_recorder_status(tmp_path):
    record = _record_module()
    episode_dir = tmp_path / "pick_bread" / "episode_000001"
    episode_dir.mkdir(parents=True)
    scope = SimpleNamespace(final_status=SimpleNamespace(episode_path=str(episode_dir)))

    assert record._finalized_episode_directory(scope) == episode_dir


def test_missing_finalized_episode_path_is_an_error():
    record = _record_module()

    with pytest.raises(RuntimeError, match="episode_path"):
        record._finalized_episode_directory(SimpleNamespace(final_status=None))


def test_demonstration_label_is_human_verified_and_traceable(tmp_path):
    record = _record_module()
    trial_id = record._trial_id("towel_corner_grasp_v1")

    record._write_demonstration_label(
        tmp_path, trial_id, "alpha", "accepted_success", True
    )

    payload = __import__("json").loads(
        (tmp_path / "demonstration_label.json").read_text()
    )
    assert trial_id.startswith("towel_corner_grasp_v1_")
    assert payload["trial_id"] == trial_id
    assert payload["demonstration_quality"] == "accepted_success"
    assert payload["stable_corner_grasp_success"] is True
    assert payload["human_verified"] is True


def test_record_cli_accepts_five_second_duration_override(monkeypatch):
    record = _record_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["record", "--episodes", "1", "--max-duration-s", "5"],
    )

    args = record.parse_args()

    assert args.episodes == 1
    assert args.max_duration_s == 5.0


def test_finalized_episode_gate_rejects_checksum_mismatch_before_mcap_read(tmp_path):
    record = _record_module()
    mcap = tmp_path / "episode_000001.mcap"
    mcap.write_bytes(b"truncated")
    (tmp_path / "checksums.sha256").write_text(
        f"{'0' * 64}  {mcap.name}\n", encoding="utf-8"
    )

    with pytest.raises(RuntimeError, match="checksum mismatch"):
        record._verify_finalized_episode(tmp_path, {"/camera/image_raw"})


class _CameraNode:
    def __init__(self, frames_by_topic):
        self.frames_by_topic = frames_by_topic
        self.destroyed = []

    def create_subscription(self, _message_type, topic, callback, _qos):
        subscription = object()
        if topic in self.frames_by_topic:
            callback(self.frames_by_topic[topic])
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


class _GraphNode:
    def __init__(self, topics):
        self.topics = topics

    def get_topic_names_and_types(self):
        return [(topic, []) for topic in self.topics]


CAMERAS = {
    "top": {"ros_topic": "/top/image_raw"},
    "left": {"ros_topic": "/left/image_raw"},
    "right": {"ros_topic": "/right/image_raw"},
}
FRAME = SimpleNamespace(width=640, height=480, encoding="rgb8", data=b"frame")


def test_camera_gate_allows_recording_only_after_every_camera_has_a_frame():
    record = _record_module()
    node = _CameraNode({cfg["ros_topic"]: FRAME for cfg in CAMERAS.values()})

    record.verify_cameras(node, CAMERAS, timeout_sec=0.0)

    assert len(node.destroyed) == 3


def test_camera_gate_rejects_missing_camera_before_recorder_start():
    record = _record_module()
    node = _CameraNode(
        {
            "/top/image_raw": FRAME,
            "/left/image_raw": FRAME,
        }
    )

    with pytest.raises(RuntimeError, match=r"right \(/right/image_raw\)"):
        record.verify_cameras(node, CAMERAS, timeout_sec=0.0)

    assert len(node.destroyed) == 3


def test_camera_liveness_guard_reports_a_stream_that_stops_mid_episode():
    record = _record_module()
    node = _CameraNode({cfg["ros_topic"]: FRAME for cfg in CAMERAS.values()})
    guard = record.CameraFrameGuard(node, CAMERAS)
    guard.start()

    with guard._lock:
        guard._last_frame_s["right"] = time.monotonic() - 1.1

    try:
        assert guard.stalled(timeout_sec=1.0) == ["right (/right/image_raw)"]
    finally:
        guard.close()


def test_recording_camera_selection_keeps_only_orbbec_and_wrists():
    record = _record_module()
    cameras = {
        camera_id: {"ros_topic": topic}
        for camera_id, topic in record.RECORD_CAMERA_TOPICS.items()
    }
    cameras["d435i1"] = {"ros_topic": "/observation/d435i1/color/image_raw"}

    assert record._select_recording_cameras(cameras) == {
        camera_id: cameras[camera_id] for camera_id in record.RECORD_CAMERA_TOPICS
    }


def test_recording_camera_selection_rejects_missing_wrist():
    record = _record_module()
    cameras = {
        camera_id: {"ros_topic": topic}
        for camera_id, topic in record.RECORD_CAMERA_TOPICS.items()
        if camera_id != "right_wrist_cam"
    }

    with pytest.raises(RuntimeError, match="right_wrist_cam"):
        record._select_recording_cameras(cameras)
