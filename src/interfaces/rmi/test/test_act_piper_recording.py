import argparse
import importlib.util
import json
import sys
import threading
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "apps"))

from act_piper import (
    ALL3_CAMERA_TOPICS,
    ALL3_IMAGE_SHAPES,
    JOINT_NAMES,
    ActPiperNode,
    AssistEpisode,
    RecordingState,
    _apply_gripper_guard,
    _decode_rgb_image,
    _parse_layout_ids,
    _policy_image_shapes,
    _policy_image_topics,
    _recording_key_action,
    _resolve_action_steps,
    _resolve_checkpoint,
    _switch_teleop,
)


def test_all3_policy_schema_and_topics_match_converter_contract():
    args = SimpleNamespace(
        camera_view="all3",
        top_camera_topic="/unused-in-all3",
        left_wrist_topic="/left",
        right_wrist_topic="/right",
    )

    assert _policy_image_shapes("all3") == ALL3_IMAGE_SHAPES
    assert _policy_image_topics(args) == {
        **ALL3_CAMERA_TOPICS,
        "left_wrist": "/left",
        "right_wrist": "/right",
    }


def test_single_policy_topic_contract_remains_backward_compatible():
    args = SimpleNamespace(
        top_camera_topic="/top",
        left_wrist_topic="/left",
        right_wrist_topic="/right",
    )

    assert _policy_image_topics(args) == {
        "top": "/top",
        "left_wrist": "/left",
        "right_wrist": "/right",
    }


def _corner_analyzer_module():
    path = ROOT / "scripts" / "analyze_corner_following.py"
    scripts = str(ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("corner_analyzer", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _converter_module():
    path = ROOT / "scripts" / "convert_episode_to_lerobot.py"
    spec = importlib.util.spec_from_file_location("piper_converter", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_policy_image_decoder_handles_bgr_and_row_padding_without_cv_bridge():
    message = SimpleNamespace(
        encoding="bgr8",
        height=1,
        width=2,
        step=8,
        data=bytes([3, 2, 1, 6, 5, 4, 99, 99]),
    )

    image = _decode_rgb_image(message)

    assert image.shape == (1, 2, 3)
    assert image.tolist() == [[[1, 2, 3], [4, 5, 6]]]


def test_checkpoint_resolver_accepts_training_output_root(tmp_path):
    pretrained = tmp_path / "checkpoints" / "004000" / "pretrained_model"
    pretrained.mkdir(parents=True)
    (pretrained / "config.json").write_text("{}", encoding="utf-8")
    (tmp_path / "checkpoints" / "last").symlink_to("004000")

    assert _resolve_checkpoint(tmp_path) == pretrained.resolve()


def test_action_steps_can_override_checkpoint_without_changing_prediction_horizon():
    assert _resolve_action_steps(100, 100, None) == 100
    assert _resolve_action_steps(100, 100, 10) == 10


def test_async_inference_does_not_repeat_controller_reference_while_waiting():
    import numpy as np

    class Node:
        def create_subscription(self, *_args, **_kwargs):
            return object()

        def create_timer(self, _period, callback, **_kwargs):
            self.callback = callback
            return object()

        def destroy_timer(self, _timer):
            return None

    class Runner:
        action_steps = 2

        def __init__(self):
            self.calls = 0
            self.second_started = threading.Event()
            self.release_second = threading.Event()

        def predict(self, _observation):
            self.calls += 1
            if self.calls == 2:
                self.second_started.set()
                assert self.release_second.wait(timeout=1.0)
            return np.full((2, 14), self.calls, dtype=np.float32)

    args = SimpleNamespace(
        state_topic="/joint_states",
        top_camera_topic="/top",
        left_wrist_topic="/left",
        right_wrist_topic="/right",
        rate_hz=30.0,
        max_input_age_s=1.0,
        hold_grippers_open=False,
        open_gripper_position=0.02,
    )
    runner = Runner()
    bridge = ActPiperNode(Node(), runner, args)
    raw_observation = {name: 0.0 for name in JOINT_NAMES}
    bridge.inputs = SimpleNamespace(snapshot=lambda: (raw_observation, 0.0))
    bridge.set_session(SimpleNamespace(active=True, act=lambda _action: None))
    bridge.start_episode_trace()
    bridge.start()
    try:
        bridge._tick()
        deadline = time.monotonic() + 1.0
        while not bridge._prediction_future.done() and time.monotonic() < deadline:
            time.sleep(0.001)
        assert bridge._prediction_future.done()

        bridge._tick()
        bridge._tick()
        bridge._tick()
        assert runner.second_started.wait(timeout=1.0)

        published = bridge._episode_trace["published_actions"]
        assert [item["source"] for item in published] == ["policy", "policy"]
    finally:
        runner.release_second.set()
        bridge.stop()
        trace = bridge.finish_episode_trace()
        bridge.close()

    assert trace["summary"]["policy_action_count"] == 2
    assert trace["summary"]["hold_action_count"] == 0
    assert trace["summary"]["actual_publish_rate_hz"] is not None


@pytest.mark.parametrize("requested", [0, -1, 101])
def test_action_steps_rejects_values_outside_the_predicted_chunk(requested):
    with pytest.raises(ValueError, match="expected 1..100"):
        _resolve_action_steps(100, 100, requested)


def test_approach_only_guard_overrides_both_grippers_without_changing_arms():
    import numpy as np

    action = np.arange(14, dtype=np.float32)
    guarded = _apply_gripper_guard(action, hold_open=True, open_position=0.02)

    assert guarded[6] == pytest.approx(0.02)
    assert guarded[13] == pytest.approx(0.02)
    assert guarded[[*range(6), *range(7, 13)]].tolist() == action[
        [*range(6), *range(7, 13)]
    ].tolist()
    assert action[6] == 6.0 and action[13] == 13.0


def test_approach_only_guard_rejects_position_outside_hardware_contract():
    import numpy as np

    with pytest.raises(ValueError, match="within"):
        _apply_gripper_guard(
            np.zeros(14, dtype=np.float32), hold_open=True, open_position=0.05
        )


def test_recording_layout_ids_are_explicit_and_counted():
    assert _parse_layout_ids("C,L,R", 3) == ["C", "L", "R"]
    assert _parse_layout_ids("L1,L2,L1", 3) == ["L1", "L2", "L1"]
    assert _parse_layout_ids(None, 2) == ["layout_00", "layout_01"]
    with pytest.raises(ValueError, match="expected 3"):
        _parse_layout_ids("C,L", 3)


def test_recording_key_actions_keep_enter_and_teleop_separate():
    assert _recording_key_action(RecordingState.READY, "enter") == "start"
    assert _recording_key_action(RecordingState.READY, "t") is None
    assert _recording_key_action(RecordingState.RECORDING, "t") == "toggle_teleop"
    assert _recording_key_action(RecordingState.RECORDING, "enter") == "finish"
    assert _recording_key_action(RecordingState.REVIEW, "enter") == "save"
    assert _recording_key_action(RecordingState.REVIEW, "d") == "discard"


def test_teleop_switch_stops_policy_before_activation_and_restarts_after_release():
    events = []

    class Control:
        def start(self):
            events.append("policy_start")

        def stop(self):
            events.append("policy_stop")

    class Teleop:
        active = False

        def activate(self):
            events.append("teleop_activate")
            self.active = True

        def deactivate(self):
            events.append("teleop_deactivate")
            self.active = False

    control = Control()
    teleop = Teleop()
    _switch_teleop(control, teleop, None, True)
    _switch_teleop(control, teleop, None, False)
    assert events == [
        "policy_stop",
        "teleop_activate",
        "teleop_deactivate",
        "policy_start",
    ]


def test_assist_episode_writes_sidecar_after_recorder_finalizes(tmp_path):
    episode_dir = tmp_path / "episode_000001"
    episode_dir.mkdir()

    class Scope:
        final_status = SimpleNamespace(episode_path=str(episode_dir))

        def __init__(self):
            self.closed = False

        def __exit__(self, *_args):
            self.closed = True

        def discard(self):
            raise AssertionError("successful episode must not be discarded")

    scope = Scope()
    episode = AssistEpisode(
        scope=scope,
        checkpoint=tmp_path / "checkpoint",
        task="new_task",
        episode_index=3,
        started_time_ns=100,
        intervals=[],
        prediction_horizon_k=100,
        executed_steps_m=30,
        rate_hz=30.0,
        hold_grippers_open=True,
        layout_id="C",
    )
    episode.teleop_started()
    path = episode.finish(discard=False)
    payload = json.loads((episode_dir / "episode_assist.json").read_text())

    assert path == episode_dir
    assert scope.closed
    assert payload["policy"]["type"] == "act"
    assert payload["task"] == "new_task"
    assert payload["teleop_count"] == 1
    assert payload["layout_id"] == "C"
    assert payload["deployment"] == {
        "prediction_horizon_k": 100,
        "executed_steps_m": 30,
        "control_rate_hz": 30.0,
        "nominal_query_rate_hz": 1.0,
        "hold_grippers_open": True,
    }


def test_assist_episode_keeps_the_deploying_policy_type(tmp_path):
    episode_dir = tmp_path / "episode_000002"
    episode_dir.mkdir()

    class Scope:
        final_status = SimpleNamespace(episode_path=str(episode_dir))

        def __exit__(self, *_args):
            return None

        def discard(self):
            raise AssertionError("successful episode must not be discarded")

    episode = AssistEpisode(
        scope=Scope(),
        checkpoint=tmp_path / "checkpoint",
        task="pick_bread",
        episode_index=4,
        started_time_ns=100,
        intervals=[],
        policy_type="smolvla",
    )
    episode.finish(discard=False)

    payload = json.loads((episode_dir / "episode_assist.json").read_text())
    assert payload["policy"]["type"] == "smolvla"


def test_assist_episode_discard_does_not_write_sidecar(tmp_path):
    class Scope:
        final_status = None

        def __init__(self):
            self.discarded = False
            self.closed = False

        def __exit__(self, *_args):
            self.closed = True

        def discard(self):
            self.discarded = True

    scope = Scope()
    episode = AssistEpisode(
        scope=scope,
        checkpoint=tmp_path / "checkpoint",
        task="new_task",
        episode_index=4,
        started_time_ns=100,
        intervals=[],
    )
    assert episode.finish(discard=True) is None
    assert scope.discarded and scope.closed
    assert not (tmp_path / "episode_assist.json").exists()


def test_converter_reads_optional_assist_sidecar(tmp_path):
    converter = _converter_module()
    assert converter._load_assist_metadata(tmp_path) is None
    payload = {"schema_version": "1.0", "teleop_intervals": [], "teleop_count": 0}
    (tmp_path / "episode_assist.json").write_text(json.dumps(payload))
    assert converter._load_assist_metadata(tmp_path) == payload
    converter._write_conversion_manifest(
        tmp_path,
        {
            "dataset_episode_index": 0,
            "repo_id": "local/test",
            "fps": 30,
            "policy_assist": payload,
        },
    )
    manifest = json.loads((tmp_path / "piper_conversion_manifest.json").read_text())
    assert manifest["episodes"][0]["policy_assist"] == payload


def test_converter_rejects_episode_with_empty_required_camera_stream():
    converter = _converter_module()

    with pytest.raises(RuntimeError, match="right_hand_realsense"):
        converter._require_image_samples(
            {
                converter.IMAGE_STATIC_REALSENSE: [1],
                converter.IMAGE_LEFT: [1],
                converter.IMAGE_RIGHT: [],
            }
        )


def test_converter_defines_three_camera_features_for_static_realsense():
    converter = _converter_module()

    features = converter._build_features(converter.IMAGE_STATIC_REALSENSE)

    assert set(features) >= {
        converter.IMAGE_TOP_KEY,
        converter.IMAGE_LEFT_KEY,
        converter.IMAGE_RIGHT_KEY,
    }
    assert features[converter.IMAGE_TOP_KEY]["shape"] == [480, 640, 3]


def test_converter_accepts_the_renamed_static_d435i_stream():
    converter = _converter_module()

    selected = converter._require_image_samples(
        {
            converter.IMAGE_STATIC_D435I: [1],
            converter.IMAGE_LEFT: [1],
            converter.IMAGE_RIGHT: [1],
        }
    )

    assert selected == converter.IMAGE_STATIC_D435I
    assert converter._build_features(selected)[converter.IMAGE_TOP_KEY]["shape"] == [
        480,
        640,
        3,
    ]


def test_converter_center_crops_orbbec_before_resizing_to_shared_image_shape():
    import numpy as np

    converter = _converter_module()
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    image[:, :160] = (255, 0, 0)
    image[:, 160:1120] = (0, 255, 0)
    image[:, 1120:] = (0, 0, 255)

    normalized = converter._normalize_model_image(image)

    assert normalized.shape == (480, 640, 3)
    assert np.all(normalized == (0, 255, 0))


def test_converter_five_camera_views_share_one_complete_capture_contract():
    converter = _converter_module()
    timestamps = {
        topic: [1, 2] for topic in converter.FIVE_CAMERA_IMAGE_TOPICS
    }

    e1, sync_topics = converter._camera_view_bindings(timestamps, "e1")
    orbbec, canonical_sync_topics = converter._camera_view_bindings(
        timestamps, "orbbec"
    )
    all3, all3_sync_topics = converter._camera_view_bindings(timestamps, "all3")

    assert [key for _topic, key in e1] == [
        converter.IMAGE_TOP_KEY,
        converter.IMAGE_LEFT_KEY,
        converter.IMAGE_RIGHT_KEY,
    ]
    assert [key for _topic, key in all3] == [
        converter.IMAGE_EXTERNAL_1_KEY,
        converter.IMAGE_EXTERNAL_2_KEY,
        converter.IMAGE_EXTERNAL_3_KEY,
        converter.IMAGE_LEFT_KEY,
        converter.IMAGE_RIGHT_KEY,
    ]
    assert sync_topics == converter.FIVE_CAMERA_IMAGE_TOPICS
    assert orbbec == e1
    assert canonical_sync_topics == (
        converter.IMAGE_ORBBEC,
        converter.IMAGE_LEFT,
        converter.IMAGE_RIGHT,
    )
    assert all3_sync_topics == converter.FIVE_CAMERA_IMAGE_TOPICS
    assert converter._build_features(all3)[converter.IMAGE_EXTERNAL_1_KEY][
        "shape"
    ] == [480, 640, 3]
    for view in converter.PAIRED3_VIEWS:
        bindings, _ = converter._camera_view_bindings(timestamps, view)
        assert [key for _topic, key in bindings] == [
            converter.IMAGE_TOP_KEY,
            converter.IMAGE_LEFT_KEY,
            converter.IMAGE_RIGHT_KEY,
        ]
        assert converter._camera_sources(bindings)[converter.IMAGE_TOP_KEY][
            "camera_id"
        ] == view
    with pytest.raises(ValueError, match="E4/Mix-1"):
        converter._camera_view_bindings(timestamps, "e4")
    with pytest.raises(ValueError, match="paired3"):
        converter._camera_view_bindings(timestamps, "paired3")


def test_converter_rejects_any_incomplete_five_camera_capture():
    converter = _converter_module()
    timestamps = {
        topic: [1] for topic in converter.FIVE_CAMERA_IMAGE_TOPICS
    }
    timestamps[converter.IMAGE_EXTERNAL_3] = []

    with pytest.raises(RuntimeError, match="同一份完整五路采集"):
        converter._camera_view_bindings(timestamps, "e1")


def test_converter_task_name_derives_dataset_defaults(monkeypatch):
    converter = _converter_module()
    monkeypatch.setattr(sys, "argv", ["convert", "--all", "--task", "pick_bread"])

    args = converter.parse_args()

    assert args.episodes_root == Path("data/episodes/pick_bread")
    assert args.output == Path.home() / "lerobot_train" / "pick_bread"
    assert args.repo_id == "pick_bread"


def test_converter_episode_list_resolves_ordered_unique_subset(tmp_path):
    converter = _converter_module()
    episodes_root = tmp_path / "episodes"
    first = episodes_root / "episode_000002"
    second = episodes_root / "episode_000001"
    first.mkdir(parents=True)
    second.mkdir()
    selection = tmp_path / "selection.txt"
    selection.write_text(
        "# fixed random selection\nepisode_000002\n\nepisode_000001\n",
        encoding="utf-8",
    )

    assert converter._episodes_from_list(selection, episodes_root) == [first, second]

    selection.write_text("episode_000001\nepisode_000001\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="重复"):
        converter._episodes_from_list(selection, episodes_root)


@pytest.mark.parametrize("historical_view", ["e3", "e4", "paired3", "all3"])
def test_converter_cli_rejects_historical_camera_views(monkeypatch, historical_view):
    converter = _converter_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["convert", "--all", "--task", "towel", "--camera-view", historical_view],
    )

    with pytest.raises(SystemExit):
        converter.parse_args()


def test_converter_validates_reloaded_images_are_channel_first_640x480():
    import numpy as np

    converter = _converter_module()
    bindings = converter.CAMERA_VIEW_BINDINGS["e1"]
    frame = {
        key: np.zeros((3, 480, 640), dtype=np.float32)
        for _topic, key in bindings
    }

    converter._validate_reloaded_images(frame, bindings)
    frame[converter.IMAGE_TOP_KEY] = np.zeros((3, 720, 1280), dtype=np.float32)
    with pytest.raises(RuntimeError, match="observation.images.top"):
        converter._validate_reloaded_images(frame, bindings)


def test_converter_verifies_every_recorder_checksum(tmp_path):
    converter = _converter_module()
    payload = tmp_path / "episode.mcap"
    payload.write_bytes(b"complete")
    import hashlib

    digest = hashlib.sha256(payload.read_bytes()).hexdigest()
    (tmp_path / "checksums.sha256").write_text(
        f"{digest}  episode.mcap\n", encoding="utf-8"
    )

    assert converter._verify_checksums(tmp_path) == {"episode.mcap": digest}

    payload.write_bytes(b"truncated")
    with pytest.raises(RuntimeError, match="checksum mismatch"):
        converter._verify_checksums(tmp_path)


def test_converter_hashes_each_source_only_once_per_invocation(monkeypatch, tmp_path):
    converter = _converter_module()
    calls = []
    args = SimpleNamespace(_verified_source_dirs=set())
    monkeypatch.setattr(
        converter, "_verify_checksums", lambda episode: calls.append(episode)
    )

    converter._verify_checksums_once(args, tmp_path)
    converter._verify_checksums_once(args, tmp_path)

    assert calls == [tmp_path.resolve()]


def test_converter_matches_orbbec_and_wrist_counts_to_sidecars(tmp_path):
    import yaml

    converter = _converter_module()
    required_topics = (
        converter.IMAGE_ORBBEC,
        converter.IMAGE_LEFT,
        converter.IMAGE_RIGHT,
    )
    timestamps = {topic: [1, 2, 3] for topic in required_topics}
    stream_ids = {
        converter.IMAGE_ORBBEC: "orbbec_color",
        converter.IMAGE_EXTERNAL_2: "d435i1_color",
        converter.IMAGE_EXTERNAL_3: "d435i2_color",
        converter.IMAGE_LEFT: "left_wrist_realsense_color",
        converter.IMAGE_RIGHT: "right_wrist_realsense_color",
    }
    metadata = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": len(values),
                }
                for topic, values in timestamps.items()
            ]
        }
    }
    (tmp_path / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    (tmp_path / "episode_health.json").write_text(
        json.dumps(
            {
                "capture_health": {
                    "streams": {
                        stream_ids[topic]: {"written": len(values)}
                        for topic, values in timestamps.items()
                    }
                }
            }
        )
    )

    converter._validate_recorded_image_counts(
        tmp_path, timestamps, required_topics
    )
    timestamps[converter.IMAGE_RIGHT].pop()
    with pytest.raises(RuntimeError, match="sidecar"):
        converter._validate_recorded_image_counts(
            tmp_path, timestamps, required_topics
        )


def test_converter_all_quarantines_invalid_episode_and_continues(
    monkeypatch, tmp_path, capsys
):
    converter = _converter_module()
    bad = tmp_path / "episode_000000"
    good = tmp_path / "episode_000001"
    bad.mkdir()
    good.mkdir()
    for episode in (bad, good):
        (episode / f"{episode.name}.mcap").write_bytes(b"mcap")

    args = argparse.Namespace(
        all=True,
        episodes_root=tmp_path,
        camera_view="legacy",
    )
    converted = []

    def verify(episode):
        if episode == bad:
            raise RuntimeError("checksum mismatch")
        return {"episode_000001.mcap": "0" * 64}

    def convert(_args, episode, source_camera_view=None):
        converted.append((episode, source_camera_view))
        return True

    monkeypatch.setattr(converter, "parse_args", lambda: args)
    monkeypatch.setattr(converter, "_verify_checksums", verify)
    monkeypatch.setattr(converter, "convert_one", convert)

    assert converter.main() == 0
    assert converted == [(good, None)]
    captured = capsys.readouterr()
    assert "[SKIP INVALID]" in captured.err
    assert "隔离损坏 episode 1 个" in captured.out


def test_converter_all_fails_when_every_episode_is_invalid(
    monkeypatch, tmp_path
):
    converter = _converter_module()
    bad = tmp_path / "episode_000000"
    bad.mkdir()
    (bad / "episode_000000.mcap").write_bytes(b"mcap")
    args = argparse.Namespace(
        all=True,
        episodes_root=tmp_path,
        camera_view="legacy",
    )

    monkeypatch.setattr(converter, "parse_args", lambda: args)
    monkeypatch.setattr(
        converter,
        "_verify_checksums",
        lambda _episode: (_ for _ in ()).throw(RuntimeError("checksum mismatch")),
    )
    monkeypatch.setattr(
        converter,
        "convert_one",
        lambda *_args, **_kwargs: pytest.fail("invalid episode must not convert"),
    )

    with pytest.raises(RuntimeError, match="全部未通过完整性校验"):
        converter.main()


def test_act_readiness_does_not_require_authority_hardware_diagnostics():
    import act_piper

    calls = []

    class Context:
        def wait_until_ready(self, **kwargs):
            calls.append(kwargs)

    class Bridge:
        def wait_for_inputs(self, timeout):
            calls.append({"input_timeout": timeout})

    args = SimpleNamespace(input_timeout_s=7.5)
    context = Context()
    bridge = Bridge()

    act_piper._wait_for_deployment_ready(context, bridge, args.input_timeout_s)

    assert calls == [{"timeout": 7.5}, {"input_timeout": 7.5}]


def test_stale_act_inputs_hold_output_without_stopping_deployment():
    bridge = object.__new__(ActPiperNode)
    bridge.fatal_error = None
    bridge.session = SimpleNamespace(active=True)
    bridge.inputs = SimpleNamespace(snapshot=lambda: ({}, 1.01))
    bridge.args = SimpleNamespace(max_input_age_s=1.0)
    bridge.actions = deque([[0.0] * 14])
    bridge._last_stale_log = 0.0
    bridge._prediction_generation = 0
    bridge._control_lock = threading.Lock()
    bridge._running = True

    bridge._tick()

    assert not bridge.actions
    assert bridge.fatal_error is None


def test_policy_control_stops_callbacks_before_releasing_session():
    from act_piper import PolicyControl

    events = []
    bridge = SimpleNamespace(
        clear_session=lambda: events.append("clear"),
        stop=lambda: events.append("stop"),
    )
    session = SimpleNamespace(
        __exit__=lambda *_args: events.append("release"),
    )
    control = PolicyControl(None, None, bridge, 30.0)
    control.session = session

    control.stop()
    control.stop()

    assert control.session is None
    assert events == ["stop", "clear", "release", "stop", "clear"]


def test_blue_cloth_detector_returns_pixel_geometry_for_largest_component():
    import cv2
    import numpy as np

    analyzer = _corner_analyzer_module()
    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    rectangle = ((170.0, 120.0), (130.0, 80.0), 18.0)
    points = np.rint(cv2.boxPoints(rectangle)).astype(np.int32)
    cv2.fillConvexPoly(rgb, points, (50, 150, 220))
    cv2.circle(rgb, (20, 20), 4, (50, 150, 220), -1)

    observed = analyzer.detect_blue_cloth(rgb)

    assert observed is not None
    assert observed.centroid_uv == pytest.approx((170.0, 120.0), abs=1.0)
    assert observed.area_fraction > 0.1
    assert observed.corners_uv.shape == (4, 2)


def test_pixel_to_tcp_regression_requires_multiple_layouts_and_is_labelled_exploratory():
    analyzer = _corner_analyzer_module()
    results = []
    for index in range(4):
        u = float(index * 10)
        v = float(index * index + 3)
        results.append(
            {
                "cloth": {
                    "start_corners_uv_px": {
                        "top_left": [u, v],
                        "top_right": [u, v],
                    }
                },
                "tcp": {
                    side: {"end_xyz_mm": [2.0 * u + v, u - v, 100.0]}
                    for side in ("left", "right")
                },
            }
        )

    fit = analyzer.fit_pixel_to_tcp_regression(results)

    assert fit["available"] is True
    assert "not calibrated G" in fit["interpretation"]
    assert fit["fits"]["left"]["r_squared_xyz"][0] == pytest.approx(1.0)
