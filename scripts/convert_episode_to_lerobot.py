#!/usr/bin/env python3
"""Convert Piper MCAP episodes into a LeRobot Dataset v3.0 dataset.

The recorder stores asynchronous ROS topics in one MCAP directory.  This
converter creates a regular ``fps`` timeline from the Orbbec-plus-wrists setup,
then synchronizes state and absolute joint-reference actions onto that timeline.
Historical five-camera view bindings remain available only for reproducibility;
new recordings and the default conversion path use the Orbbec single view. It can create a new
dataset or append episodes to an existing one; converted source episodes are
tracked in ``piper_conversion_manifest.json`` so rerunning the command is safe.

The script intentionally does not clamp or otherwise alter joint values.  The
recorded action topic is the downstream absolute ``joint_reference`` command
and is preserved as-is.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image as PILImage

JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_gripper_joint1",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_gripper_joint1",
]

IMAGE_ORBBEC = "/observation/orbbec/color/image_raw"
IMAGE_STATIC_D435I = "/observation/static_d435i/color/image_raw"
# Kept for episodes recorded before the static camera was renamed to
# ``static_d435i``.
IMAGE_STATIC_REALSENSE = "/observation/static_realsense/color/image_raw"
IMAGE_LEFT = "/observation/left_hand_realsense/color/image_raw"
IMAGE_RIGHT = "/observation/right_hand_realsense/color/image_raw"
IMAGE_EXTERNAL_1 = IMAGE_ORBBEC
IMAGE_EXTERNAL_2 = "/observation/d435i1/color/image_raw"
IMAGE_EXTERNAL_3 = "/observation/d435i2/color/image_raw"
IMAGE_TOP_KEY = "observation.images.top"
IMAGE_LEFT_KEY = "observation.images.left_wrist"
IMAGE_RIGHT_KEY = "observation.images.right_wrist"
IMAGE_EXTERNAL_1_KEY = "observation.images.external_1"
IMAGE_EXTERNAL_2_KEY = "observation.images.external_2"
IMAGE_EXTERNAL_3_KEY = "observation.images.external_3"
STATIC_IMAGE_TOPICS = (
    IMAGE_ORBBEC,
    IMAGE_STATIC_D435I,
    IMAGE_STATIC_REALSENSE,
    "/observation/static_orbbec/color/image_raw",
)
WRIST_IMAGE_TOPICS = (IMAGE_LEFT, IMAGE_RIGHT)
EXTERNAL_IMAGE_TOPICS = (IMAGE_EXTERNAL_1, IMAGE_EXTERNAL_2, IMAGE_EXTERNAL_3)
FIVE_CAMERA_IMAGE_TOPICS = (*EXTERNAL_IMAGE_TOPICS, *WRIST_IMAGE_TOPICS)
KNOWN_IMAGE_TOPICS = (*STATIC_IMAGE_TOPICS, *FIVE_CAMERA_IMAGE_TOPICS)
MODEL_IMAGE_HEIGHT = 480
MODEL_IMAGE_WIDTH = 640
CAMERA_IDS_BY_TOPIC = {
    IMAGE_ORBBEC: "orbbec",
    IMAGE_STATIC_D435I: "static_d435i",
    IMAGE_STATIC_REALSENSE: "static_realsense",
    "/observation/static_orbbec/color/image_raw": "static_orbbec",
    IMAGE_EXTERNAL_2: "d435i1",
    IMAGE_EXTERNAL_3: "d435i2",
    IMAGE_LEFT: "left_wrist_realsense",
    IMAGE_RIGHT: "right_wrist_realsense",
}

CAMERA_VIEW_BINDINGS = {
    "e1": (
        (IMAGE_EXTERNAL_1, IMAGE_TOP_KEY),
        (IMAGE_LEFT, IMAGE_LEFT_KEY),
        (IMAGE_RIGHT, IMAGE_RIGHT_KEY),
    ),
    "e2": (
        (IMAGE_EXTERNAL_2, IMAGE_TOP_KEY),
        (IMAGE_LEFT, IMAGE_LEFT_KEY),
        (IMAGE_RIGHT, IMAGE_RIGHT_KEY),
    ),
    "e3": (
        (IMAGE_EXTERNAL_3, IMAGE_TOP_KEY),
        (IMAGE_LEFT, IMAGE_LEFT_KEY),
        (IMAGE_RIGHT, IMAGE_RIGHT_KEY),
    ),
    "all3": (
        (IMAGE_EXTERNAL_1, IMAGE_EXTERNAL_1_KEY),
        (IMAGE_EXTERNAL_2, IMAGE_EXTERNAL_2_KEY),
        (IMAGE_EXTERNAL_3, IMAGE_EXTERNAL_3_KEY),
        (IMAGE_LEFT, IMAGE_LEFT_KEY),
        (IMAGE_RIGHT, IMAGE_RIGHT_KEY),
    ),
}
CAMERA_VIEW_ALIASES = {
    "orbbec": "e1",
    "d435i1": "e2",
    "d435i2": "e3",
}
PAIRED3_VIEWS = tuple(CAMERA_VIEW_ALIASES)

STATE_TOPIC = "/joint_states"
LEFT_ARM_TOPIC = "/execution/left_arm/joint_reference"
RIGHT_ARM_TOPIC = "/execution/right_arm/joint_reference"
LEFT_GRIPPER_TOPIC = "/execution/left_gripper/joint_reference"
RIGHT_GRIPPER_TOPIC = "/execution/right_gripper/joint_reference"


@dataclass
class Sample:
    timestamp_ns: int
    value: Any


def _require_ros_and_lerobot() -> tuple[Any, Any, Any, Any]:
    """Import optional runtime dependencies with an actionable error."""
    try:
        import rosbag2_py
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "需要同时使用 ROS 2 和 LeRobot 的 Python 环境。请通过 "
            "`pixi run -e lerobot lerobot-convert -- ...` 运行。"
        ) from exc
    return rosbag2_py, deserialize_message, get_message, LeRobotDataset


def _episode_mcap(episode: Path) -> Path:
    if episode.is_file() and episode.suffix == ".mcap":
        return episode
    if not episode.is_dir():
        raise FileNotFoundError(f"episode 不存在: {episode}")
    candidates = sorted(episode.glob("*.mcap"))
    if len(candidates) != 1:
        raise RuntimeError(
            f"{episode} 中应有且仅有一个 .mcap，实际找到 {len(candidates)} 个"
        )
    return candidates[0]


def _open_reader(rosbag2_py: Any, mcap: Path) -> Any:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap.parent), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def _source_fingerprint(episode: Path, mcap: Path) -> str:
    """Use the already-verified recorder checksum as the source identity."""
    checksum_file = episode / "checksums.sha256"
    if checksum_file.is_file():
        for line in checksum_file.read_text(encoding="utf-8").splitlines():
            digest, _, filename = line.partition("  ")
            if filename == mcap.name and len(digest) == 64:
                return digest
    stat = mcap.stat()
    return f"size={stat.st_size};mtime_ns={stat.st_mtime_ns}"


def _verify_checksums(episode: Path) -> dict[str, str]:
    """Verify every recorder checksum before a dataset directory is created."""
    checksum_file = episode / "checksums.sha256"
    if not checksum_file.is_file():
        raise RuntimeError(f"episode 缺少 checksums.sha256: {episode}")

    verified: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        checksum_file.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise RuntimeError(
                f"checksums.sha256 第 {line_number} 行格式错误: {checksum_file}"
            )
        expected, filename = parts
        filename = filename.lstrip("*")
        relative = Path(filename)
        if relative.is_absolute() or ".." in relative.parts:
            raise RuntimeError(f"checksums.sha256 包含不安全路径: {filename}")
        target = episode / relative
        if not target.is_file():
            raise RuntimeError(f"checksum 文件不存在: {target}")
        digest = hashlib.sha256()
        with target.open("rb") as stream:
            for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(chunk)
        actual = digest.hexdigest()
        if actual != expected.lower():
            raise RuntimeError(
                f"checksum mismatch: {target.name}，expected={expected.lower()}，"
                f"actual={actual}"
            )
        verified[filename] = actual
    if not verified:
        raise RuntimeError(f"checksums.sha256 没有任何条目: {checksum_file}")
    return verified


def _verify_checksums_once(args: argparse.Namespace, episode: Path) -> None:
    """Hash one source episode at most once during a converter invocation."""
    verified = getattr(args, "_verified_source_dirs", None)
    if verified is None:
        verified = set()
        args._verified_source_dirs = verified
    episode = episode.resolve()
    if episode not in verified:
        _verify_checksums(episode)
        verified.add(episode)


def _validate_recorded_image_counts(
    episode: Path,
    image_timestamps: dict[str, list[int]],
    required_topics: tuple[str, ...] = FIVE_CAMERA_IMAGE_TOPICS,
) -> None:
    """Match the selected image counts against recorder metadata and health."""
    import yaml

    metadata_path = episode / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeError(f"episode 缺少 metadata.yaml: {episode}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    bag_info = metadata.get("rosbag2_bagfile_information", {})
    metadata_counts = {
        item.get("topic_metadata", {}).get("name"): int(item.get("message_count", 0))
        for item in bag_info.get("topics_with_message_count", [])
    }
    health = _load_json(episode / "episode_health.json")
    health_streams = health.get("capture_health", {}).get("streams", {})
    stream_ids = {
        IMAGE_ORBBEC: "orbbec_color",
        IMAGE_EXTERNAL_2: "d435i1_color",
        IMAGE_EXTERNAL_3: "d435i2_color",
        IMAGE_LEFT: "left_wrist_realsense_color",
        IMAGE_RIGHT: "right_wrist_realsense_color",
    }
    problems = []
    for topic in required_topics:
        actual = len(image_timestamps.get(topic, []))
        expected_metadata = metadata_counts.get(topic)
        expected_health = health_streams.get(stream_ids[topic], {}).get("written")
        if actual <= 0:
            problems.append(f"{topic}: actual=0")
        if expected_metadata is None or actual != int(expected_metadata):
            problems.append(
                f"{topic}: actual={actual}, metadata={expected_metadata}"
            )
        if expected_health is None or actual != int(expected_health):
            problems.append(f"{topic}: actual={actual}, health={expected_health}")
    if problems:
        raise RuntimeError(
            f"{episode} 的相机 MCAP 计数不完整或与 sidecar 不一致: "
            + "; ".join(problems)
        )


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _load_assist_metadata(episode: Path) -> dict[str, Any] | None:
    """Return optional ACT-assist provenance without changing dataset features."""
    path = episode / "episode_assist.json"
    if not path.is_file():
        return None
    payload = _load_json(path)
    if payload.get("schema_version") != "1.0":
        raise RuntimeError(f"不支持的辅助采集标记版本: {path}")
    intervals = payload.get("teleop_intervals", [])
    if not isinstance(intervals, list):
        raise TypeError(f"辅助采集标记 teleop_intervals 必须是列表: {path}")
    return payload


def _load_demonstration_label(episode: Path) -> dict[str, Any] | None:
    path = episode / "demonstration_label.json"
    if not path.is_file():
        return None
    payload = _load_json(path)
    if payload.get("schema_version") != "1.0":
        raise RuntimeError(f"不支持的示范标签版本: {path}")
    return payload


def _validate_capture(episode: Path, allow_unhealthy: bool) -> dict[str, Any]:
    health = _load_json(episode / "episode_health.json")
    manifest = _load_json(episode / "episode_manifest.json")
    capture = health.get("capture_health", {})
    problems = list(health.get("errors", []))
    if capture.get("writer_failed"):
        problems.append("writer_failed=true")
    if capture.get("total_recorder_drops", 0):
        problems.append(f"recorder_drops={capture['total_recorder_drops']}")
    if problems and not allow_unhealthy:
        raise RuntimeError(
            f"{episode} 采集健康检查失败: {', '.join(map(str, problems))}。"
            "如确认要继续，请加 --allow-unhealthy。"
        )
    if problems:
        print(f"[WARN] 忽略采集健康问题: {', '.join(map(str, problems))}")
    if manifest.get("finalizer_result") == "WARN":
        print(
            "[WARN] episode_manifest finalizer_result=WARN，但未发现 drop/error；继续转换。"
        )
    return manifest


def _decode_rgb_image(msg: Any) -> np.ndarray:
    """Decode the ROS Image encodings used by the current Piper camera setup."""
    encoding = str(msg.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if encoding in {"mono8", "8uc1"}:
        channels = 1
    if encoding in {"8uc3", "8sc3"}:
        channels = 3
    if channels is None:
        raise ValueError(f"不支持的图像 encoding={msg.encoding!r}")

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    required = int(msg.height) * int(msg.step)
    if raw.size < required:
        raise ValueError(f"图像数据长度不足: {raw.size} < {required}")
    rows = raw[:required].reshape(int(msg.height), int(msg.step))
    pixels = rows[:, : int(msg.width) * channels]
    pixels = pixels.reshape(int(msg.height), int(msg.width), channels)

    if channels == 1:
        return np.repeat(pixels, 3, axis=2).copy()
    if encoding in {"bgr8", "bgra8"}:
        return pixels[:, :, :3][:, :, ::-1].copy()
    return pixels[:, :, :3].copy()


def _normalize_model_image(image: np.ndarray) -> np.ndarray:
    """Normalize camera geometry before assigning a shared model feature key."""
    if image.shape[:2] == (MODEL_IMAGE_HEIGHT, MODEL_IMAGE_WIDTH):
        return image
    height, width = image.shape[:2]
    target_ratio = MODEL_IMAGE_WIDTH / MODEL_IMAGE_HEIGHT
    current_ratio = width / height
    if current_ratio > target_ratio:
        crop_width = round(height * target_ratio)
        left = (width - crop_width) // 2
        image = image[:, left : left + crop_width]
    elif current_ratio < target_ratio:
        crop_height = round(width / target_ratio)
        top = (height - crop_height) // 2
        image = image[top : top + crop_height, :]
    resized = PILImage.fromarray(image, mode="RGB").resize(
        (MODEL_IMAGE_WIDTH, MODEL_IMAGE_HEIGHT), PILImage.Resampling.BILINEAR
    )
    return np.asarray(resized, dtype=np.uint8)


def _deserialize_type(type_name: str, get_message: Any) -> Any:
    return get_message(type_name)


def _collect_samples(
    rosbag2_py: Any,
    deserialize_message: Any,
    get_message: Any,
    mcap: Path,
) -> tuple[dict[str, list[Sample]], dict[str, list[int]], dict[str, str]]:
    """Read numeric topics and image timestamps; defer image decoding to pass two."""
    reader = _open_reader(rosbag2_py, mcap)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required_numeric = {
        STATE_TOPIC,
        LEFT_ARM_TOPIC,
        RIGHT_ARM_TOPIC,
        LEFT_GRIPPER_TOPIC,
        RIGHT_GRIPPER_TOPIC,
    }
    missing = sorted(required_numeric - topic_types.keys())
    if missing:
        raise RuntimeError(f"MCAP 缺少必要 topic: {', '.join(missing)}")

    numeric: dict[str, list[Sample]] = {
        topic: [] for topic in required_numeric
    }
    image_timestamps: dict[str, list[int]] = {
        topic: [] for topic in KNOWN_IMAGE_TOPICS if topic in topic_types
    }
    while reader.has_next():
        topic, serialized, record_time_ns = reader.read_next()
        if topic in image_timestamps:
            image_timestamps[topic].append(int(record_time_ns))
            continue
        if topic not in numeric:
            continue
        msg = deserialize_message(
            serialized, _deserialize_type(topic_types[topic], get_message)
        )
        if topic == STATE_TOPIC:
            value = dict(zip(msg.name, msg.position))
        elif "arm/joint_reference" in topic:
            if not msg.points:
                continue
            value = dict(zip(msg.joint_names, msg.points[0].positions))
        else:
            if not msg.data:
                continue
            value = {
                "left_gripper_joint1"
                if "left" in topic
                else "right_gripper_joint1": msg.data[0]
            }
        numeric[topic].append(Sample(int(record_time_ns), value))

    for samples in numeric.values():
        samples.sort(key=lambda sample: sample.timestamp_ns)
    for timestamps in image_timestamps.values():
        timestamps.sort()
    return numeric, image_timestamps, topic_types


def _collect_selected_images(
    rosbag2_py: Any,
    deserialize_message: Any,
    get_message: Any,
    mcap: Path,
    topic_types: dict[str, str],
    image_timestamps: dict[str, list[int]],
    target_timestamps: list[int],
    image_topics: tuple[str, ...],
) -> dict[str, list[np.ndarray]]:
    """Second MCAP pass: decode only the image frames selected for the output timeline."""
    wanted: dict[str, list[int]] = {}
    wanted_indices: dict[str, set[int]] = {}
    for topic in image_topics:
        timestamps = image_timestamps[topic]
        indices = [
            min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - target))
            for target in target_timestamps
        ]
        wanted[topic] = indices
        wanted_indices[topic] = set(indices)

    decoded: dict[str, dict[int, np.ndarray]] = {topic: {} for topic in image_topics}
    counters = {topic: 0 for topic in image_topics}
    reader = _open_reader(rosbag2_py, mcap)
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in decoded:
            continue
        index = counters[topic]
        counters[topic] += 1
        if index not in wanted_indices[topic]:
            continue
        msg = deserialize_message(
            serialized, _deserialize_type(topic_types[topic], get_message)
        )
        decoded[topic][index] = _normalize_model_image(_decode_rgb_image(msg))

    result: dict[str, list[np.ndarray]] = {}
    for topic in image_topics:
        missing = [index for index in wanted[topic] if index not in decoded[topic]]
        if missing:
            raise RuntimeError(
                f"无法解码 {topic} 的选定图像帧，缺少索引: {missing[:5]}"
            )
        result[topic] = [decoded[topic][index] for index in wanted[topic]]
    return result


def _require_image_samples(image_timestamps: dict[str, list[int]]) -> str:
    """Return the selected static topic after validating all three RGB streams."""
    static_topics = [
        topic for topic in STATIC_IMAGE_TOPICS if image_timestamps.get(topic)
    ]
    if len(static_topics) != 1:
        found = ", ".join(static_topics) or "none"
        raise RuntimeError(
            f"MCAP 必须恰有一个可用静态相机流（Orbbec 或固定 D435i），实际为: {found}。"
        )
    missing = [topic for topic in WRIST_IMAGE_TOPICS if not image_timestamps.get(topic)]
    if missing:
        raise RuntimeError(
            "MCAP 缺少可用图像帧，无法转换为当前三相机 LeRobot schema: "
            f"{', '.join(missing)}。请在所有相机出帧后重新录制；"
            "任何一个相机都不能替代缺失视角。"
        )
    return static_topics[0]


def _camera_view_bindings(
    image_timestamps: dict[str, list[int]], camera_view: str
) -> tuple[tuple[tuple[str, str], ...], tuple[str, ...]]:
    """Validate one capture schema and return output bindings plus sync topics."""
    if camera_view == "legacy":
        static_topic = _require_image_samples(image_timestamps)
        return (
            (
                (static_topic, IMAGE_TOP_KEY),
                (IMAGE_LEFT, IMAGE_LEFT_KEY),
                (IMAGE_RIGHT, IMAGE_RIGHT_KEY),
            ),
            (static_topic, *WRIST_IMAGE_TOPICS),
        )

    if camera_view == "e4":
        raise ValueError(
            "E4/Mix-1 必须由 main() 按 e1/e2/e3 展开，不能绑定为单个相机视图"
        )
    if camera_view == "paired3":
        raise ValueError("paired3 必须由 main() 展开，不能绑定为单个相机视图")

    if camera_view == "orbbec":
        bindings = CAMERA_VIEW_BINDINGS[CAMERA_VIEW_ALIASES[camera_view]]
        required_topics = tuple(topic for topic, _key in bindings)
        missing = [
            topic for topic in required_topics if not image_timestamps.get(topic)
        ]
        if missing:
            raise RuntimeError(
                "奥比中光单视角 MCAP 缺少可用图像帧: " + ", ".join(missing)
            )
        return bindings, required_topics

    missing = [
        topic for topic in FIVE_CAMERA_IMAGE_TOPICS if not image_timestamps.get(topic)
    ]
    if missing:
        raise RuntimeError(
            "五相机实验 MCAP 缺少可用图像帧: "
            + ", ".join(missing)
            + "。E1--E4 必须来自同一份完整五路采集。"
        )
    binding_name = CAMERA_VIEW_ALIASES.get(camera_view, camera_view)
    return CAMERA_VIEW_BINDINGS[binding_name], FIVE_CAMERA_IMAGE_TOPICS


def _nearest(samples: list[Sample], timestamp_ns: int) -> Sample:
    if not samples:
        raise RuntimeError("topic 没有可用消息")
    timestamps = [sample.timestamp_ns for sample in samples]
    index = bisect_left(timestamps, timestamp_ns)
    if index == 0:
        return samples[0]
    if index == len(samples):
        return samples[-1]
    before, after = samples[index - 1], samples[index]
    return (
        before
        if timestamp_ns - before.timestamp_ns <= after.timestamp_ns - timestamp_ns
        else after
    )


def _latest_value(samples: list[Sample], timestamp_ns: int) -> dict[str, float] | None:
    if not samples:
        return None
    index = bisect_right([sample.timestamp_ns for sample in samples], timestamp_ns) - 1
    return None if index < 0 else samples[index].value


def _image_shape(topic: str) -> list[int]:
    return [MODEL_IMAGE_HEIGHT, MODEL_IMAGE_WIDTH, 3]


def _build_features(
    image_bindings: tuple[tuple[str, str], ...] | str,
) -> dict[str, dict[str, Any]]:
    if isinstance(image_bindings, str):
        image_bindings = (
            (image_bindings, IMAGE_TOP_KEY),
            (IMAGE_LEFT, IMAGE_LEFT_KEY),
            (IMAGE_RIGHT, IMAGE_RIGHT_KEY),
        )
    image_shape = ["height", "width", "channels"]
    features = {
        key: {"dtype": "video", "shape": _image_shape(topic), "names": image_shape}
        for topic, key in image_bindings
    }
    features.update({
        "observation.state": {"dtype": "float32", "shape": [14], "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": [14], "names": JOINT_NAMES},
    })
    return features


def _camera_sources(
    image_bindings: tuple[tuple[str, str], ...],
) -> dict[str, dict[str, str]]:
    """Keep physical camera provenance out of the model-facing feature names."""
    return {
        key: {"camera_id": CAMERA_IDS_BY_TOPIC[topic], "topic": topic}
        for topic, key in image_bindings
    }


def _validate_reloaded_images(
    frame: dict[str, Any], image_bindings: tuple[tuple[str, str], ...]
) -> None:
    expected_shape = (3, MODEL_IMAGE_HEIGHT, MODEL_IMAGE_WIDTH)
    for _topic, key in image_bindings:
        actual_shape = tuple(frame[key].shape)
        if actual_shape != expected_shape:
            raise RuntimeError(
                f"LeRobot reload 后 {key} shape={actual_shape}，"
                f"expected={expected_shape}"
            )


def _rgb_encoder() -> Any:
    """Use the same H.264 settings as the existing Piper v3 dataset."""
    from lerobot.configs.video import RGBEncoderConfig

    return RGBEncoderConfig(
        vcodec="h264",
        pix_fmt="yuv420p",
        g=2,
        crf=23,
        preset="veryfast",
        fast_decode=0,
        video_backend="pyav",
    )


def _manifest_path(root: Path) -> Path:
    return root / "piper_conversion_manifest.json"


def _already_converted(
    root: Path,
    source: Path,
    fingerprint: str,
    camera_view: str,
    external_camera_id: str | None,
) -> bool:
    manifest = _load_json(_manifest_path(root))
    for entry in manifest.get("episodes", []):
        same_source = entry.get("source_fingerprint") == fingerprint or entry.get(
            "source_episode"
        ) == str(source.resolve())
        entry_view = entry.get("camera_view", "legacy")
        if (
            same_source
            and entry_view == camera_view
            and entry.get("external_camera_id") == external_camera_id
        ):
            return True
    return False


def _write_conversion_manifest(root: Path, entry: dict[str, Any]) -> None:
    path = _manifest_path(root)
    manifest = _load_json(path)
    manifest.setdefault("schema_version", "1.0")
    manifest.setdefault("repo_id", entry["repo_id"])
    manifest.setdefault("fps", entry["fps"])
    manifest.setdefault("episodes", []).append(entry)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def convert_one(
    args: argparse.Namespace,
    episode: Path,
    source_camera_view: str | None = None,
) -> bool:
    episode = episode.resolve()
    mcap = _episode_mcap(episode)
    source_dir = episode if episode.is_dir() else episode.parent
    _verify_checksums_once(args, source_dir)
    fingerprint = _source_fingerprint(source_dir, mcap)
    output_root = args.output.resolve()

    selected_camera_view = source_camera_view or args.camera_view
    external_camera_id = {
        "e1": "external_1",
        "e2": "external_2",
        "e3": "external_3",
        "orbbec": "orbbec",
        "d435i1": "d435i1",
        "d435i2": "d435i2",
    }.get(selected_camera_view)
    manifest_camera_view = (
        args.camera_view if args.camera_view == "e4" else selected_camera_view
    )
    if _already_converted(
        output_root,
        episode,
        fingerprint,
        manifest_camera_view,
        external_camera_id,
    ):
        print(f"[SKIP] 已转换: {episode}")
        return False

    capture_manifest = _validate_capture(source_dir, args.allow_unhealthy)
    assist_metadata = _load_assist_metadata(source_dir)
    demonstration_label = _load_demonstration_label(source_dir)
    if args.require_accepted_demonstration and (
        demonstration_label is None
        or demonstration_label.get("demonstration_quality") != "accepted_success"
        or demonstration_label.get("stable_corner_grasp_success") is not True
        or demonstration_label.get("human_verified") is not True
    ):
        raise RuntimeError(
            f"{source_dir} 没有人工确认的 accepted_success 示范标签；"
            "不得进入首轮行为克隆数据集"
        )
    rosbag2_py, deserialize_message, get_message, LeRobotDataset = (
        _require_ros_and_lerobot()
    )
    numeric, image_timestamps, topic_types = _collect_samples(
        rosbag2_py, deserialize_message, get_message, mcap
    )
    image_bindings, synchronization_topics = _camera_view_bindings(
        image_timestamps, selected_camera_view
    )
    if selected_camera_view != "legacy":
        _validate_recorded_image_counts(
            source_dir, image_timestamps, synchronization_topics
        )
    image_topics = tuple(topic for topic, _key in image_bindings)
    start_ns = max(image_timestamps[topic][0] for topic in synchronization_topics)
    end_ns = min(image_timestamps[topic][-1] for topic in synchronization_topics)
    period_ns = 1_000_000_000 / args.fps
    frame_count = math.floor((end_ns - start_ns) / period_ns) + 1
    if frame_count < 2:
        raise RuntimeError(f"共同图像时间范围过短: {(end_ns - start_ns) / 1e9:.3f}s")
    target_timestamps = [round(start_ns + i * period_ns) for i in range(frame_count)]
    images = _collect_selected_images(
        rosbag2_py,
        deserialize_message,
        get_message,
        mcap,
        topic_types,
        image_timestamps,
        target_timestamps,
        image_topics,
    )

    task = (
        args.task
        or capture_manifest.get("manifest_context", {}).get("task")
        or "bimanual_manipulation"
    )
    info_path = output_root / "meta" / "info.json"
    repo_id = args.repo_id
    rgb_encoder = _rgb_encoder()
    if info_path.exists():
        existing_info = _load_json(info_path)
        if existing_info.get("codebase_version") != "v3.0":
            raise RuntimeError(f"已有输出不是 LeRobot v3.0: {output_root}")
        if existing_info.get("fps") != args.fps:
            raise RuntimeError(
                f"已有数据集 fps={existing_info.get('fps')}，当前要求 fps={args.fps}；"
                "请换一个输出目录，或显式使用与已有数据集相同的 --fps。"
            )
        expected_features = _build_features(image_bindings)
        for key, expected in expected_features.items():
            actual = existing_info.get("features", {}).get(key, {})
            if (
                actual.get("dtype") != expected["dtype"]
                or actual.get("shape") != expected["shape"]
            ):
                raise RuntimeError(f"已有数据集 feature 不兼容: {key}")
        dataset = LeRobotDataset.resume(
            repo_id=repo_id,
            root=output_root,
            video_backend="pyav",
            rgb_encoder=rgb_encoder,
            image_writer_threads=args.image_writer_threads,
        )
    else:
        if output_root.exists():
            unexpected = list(output_root.iterdir())
            if unexpected:
                raise RuntimeError(
                    f"输出目录不是空的新数据集，且没有 meta/info.json: {output_root}"
                )
            output_root.rmdir()
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=args.fps,
            features=_build_features(image_bindings),
            root=output_root,
            robot_type="piper",
            use_videos=True,
            video_backend="pyav",
            rgb_encoder=rgb_encoder,
            image_writer_threads=args.image_writer_threads,
            video_files_size_in_mb=200,
            data_files_size_in_mb=100,
        )

    dataset_episode_index = dataset.meta.total_episodes
    state_samples = numeric[STATE_TOPIC]
    action_topics = (
        LEFT_ARM_TOPIC,
        LEFT_GRIPPER_TOPIC,
        RIGHT_ARM_TOPIC,
        RIGHT_GRIPPER_TOPIC,
    )
    try:
        for index, timestamp_ns in enumerate(target_timestamps):
            state = _nearest(state_samples, timestamp_ns).value
            state_vector = np.asarray(
                [state[name] for name in JOINT_NAMES], dtype=np.float32
            )
            action_values = dict(state)
            for topic in action_topics:
                value = _latest_value(numeric[topic], timestamp_ns)
                if value:
                    action_values.update(value)
            action_vector = np.asarray(
                [action_values[name] for name in JOINT_NAMES], dtype=np.float32
            )
            frame = {
                key: images[topic][index] for topic, key in image_bindings
            }
            frame.update({
                "observation.state": state_vector,
                "action": action_vector,
                "task": task,
            })
            dataset.add_frame(frame)
    except Exception:
        dataset.finalize()
        raise
    else:
        try:
            dataset.save_episode(parallel_encoding=not args.no_parallel_encoding)
        finally:
            dataset.finalize()

    if not args.skip_reload_validation:
        reloaded = LeRobotDataset(
            repo_id=repo_id,
            root=output_root,
            episodes=[dataset_episode_index],
            video_backend="pyav",
        )
        if len(reloaded) != frame_count:
            raise RuntimeError(
                f"LeRobot reload 帧数不一致: {len(reloaded)} != {frame_count}"
            )
        first = reloaded[0]
        if tuple(first["observation.state"].shape) != (14,) or tuple(
            first["action"].shape
        ) != (14,):
            raise RuntimeError("LeRobot reload 后 state/action shape 不是 (14,)")
        _validate_reloaded_images(first, image_bindings)

    manifest_context = capture_manifest.get("manifest_context", {})
    source_episode_id = (
        manifest_context.get("trial_id")
        or capture_manifest.get("trial_id")
        or source_dir.name
    )
    manifest_entry = {
        "dataset_episode_index": dataset_episode_index,
        "repo_id": repo_id,
        "source_episode": str(source_dir),
        "source_episode_id": source_episode_id,
        "source_mcap": str(mcap),
        "source_fingerprint": fingerprint,
        "task": task,
        "fps": args.fps,
        "frames": frame_count,
        "image_time_range_s": [
            (target_timestamps[0] - start_ns) / 1e9,
            (target_timestamps[-1] - start_ns) / 1e9,
        ],
        "source_episode_index": capture_manifest.get("episode_index"),
        "camera_view": manifest_camera_view,
        "external_camera_id": external_camera_id,
        "image_topics": [topic for topic, _key in image_bindings],
        "camera_sources": _camera_sources(image_bindings),
        "synchronization_topics": list(synchronization_topics),
        "model_image_size": [MODEL_IMAGE_HEIGHT, MODEL_IMAGE_WIDTH],
        "image_preprocessing": (
            "center_crop_to_4:3_then_bilinear_resize_if_source_is_not_640x480"
        ),
    }
    if assist_metadata is not None:
        manifest_entry["policy_assist"] = assist_metadata
    if demonstration_label is not None:
        manifest_entry["demonstration_label"] = demonstration_label
    _write_conversion_manifest(output_root, manifest_entry)

    print(
        f"[OK] {episode.name} -> {output_root} | dataset_episode={dataset_episode_index} | "
        f"frames={frame_count} | task={task}"
    )
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert Piper MCAP episodes to LeRobotDataset v3.0",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument(
        "--episode", type=Path, help="一个 episode 目录或 .mcap 文件"
    )
    selection.add_argument(
        "--all", action="store_true", help="转换 --episodes-root 下所有 episode_* 目录"
    )
    selection.add_argument(
        "--episode-list",
        type=Path,
        help="按文本文件逐行列出的 episode 目录转换；相对路径基于 --episodes-root",
    )
    parser.add_argument(
        "--episodes-root",
        type=Path,
        default=None,
        help="--all 扫描目录或 --episode-list 相对路径基准；未指定时由 --task 推导",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="LeRobot 数据集输出目录；未指定时由 --task 推导",
    )
    parser.add_argument(
        "--repo-id", default=None, help="LeRobot repo_id；未指定时由 --task 推导"
    )
    parser.add_argument(
        "--task",
        default=None,
        help=(
            "覆盖 episode metadata 中的 task；并默认使用 "
            "data/episodes/<task>、~/lerobot_train/<task> 和 <task> 作为路径与 repo_id"
        ),
    )
    parser.add_argument("--fps", type=int, default=30, help="输出重采样频率")
    parser.add_argument(
        "--camera-view",
        choices=("orbbec",),
        default="orbbec",
        help="固定转换 Orbbec 顶部视角和左右腕部图像",
    )
    parser.add_argument("--image-writer-threads", type=int, default=4)
    parser.add_argument("--no-parallel-encoding", action="store_true")
    parser.add_argument("--allow-unhealthy", action="store_true")
    parser.add_argument(
        "--require-accepted-demonstration",
        action="store_true",
        help="只转换 demonstration_label.json 中人工确认的成功示范",
    )
    parser.add_argument("--skip-reload-validation", action="store_true")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps 必须大于 0")
    if args.image_writer_threads < 0:
        parser.error("--image-writer-threads 不能小于 0")

    explicit_output = args.output
    explicit_repo_id = args.repo_id
    if args.task:
        args.episodes_root = args.episodes_root or Path("data/episodes") / args.task
        if args.camera_view in {"orbbec", "legacy", "paired3"}:
            dataset_name = args.task
        elif args.camera_view == "e4":
            dataset_name = f"{args.task}_e4_mix1"
        else:
            dataset_name = f"{args.task}_{args.camera_view}"
        args.output = args.output or Path.home() / "lerobot_train" / dataset_name
        args.repo_id = args.repo_id or dataset_name
    else:
        args.episodes_root = args.episodes_root or Path("data/episodes/piper_bimanual")
        args.output = args.output or Path("data/lerobot/piper_bimanual_v3")
        args.repo_id = args.repo_id or "local/piper_bimanual_v3"
    args.paired3_output_base = explicit_output or args.output
    args.paired3_repo_id_base = explicit_repo_id or args.repo_id
    return args


def _paired3_view_args(
    args: argparse.Namespace, camera_view: str
) -> argparse.Namespace:
    """Derive one independently trainable output from a paired3 invocation."""
    if camera_view not in PAIRED3_VIEWS:
        raise ValueError(f"未知 paired3 视角: {camera_view}")
    view_args = argparse.Namespace(**vars(args))
    view_args.camera_view = camera_view
    output_base = Path(args.paired3_output_base)
    view_args.output = output_base.with_name(f"{output_base.name}_{camera_view}")
    view_args.repo_id = f"{args.paired3_repo_id_base}_{camera_view}"
    return view_args


def _episodes_from_list(path: Path, episodes_root: Path) -> list[Path]:
    """Resolve an explicit, ordered episode subset without silently changing it."""
    entries = [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if not entries:
        raise RuntimeError(f"episode 清单为空: {path}")
    if len(entries) != len(set(entries)):
        raise RuntimeError(f"episode 清单包含重复项: {path}")
    episodes = [
        candidate if candidate.is_absolute() else episodes_root / candidate
        for candidate in map(Path, entries)
    ]
    missing = [str(episode) for episode in episodes if not episode.is_dir()]
    if missing:
        raise RuntimeError("episode 清单包含不存在的目录: " + ", ".join(missing))
    return episodes


def main() -> int:
    args = parse_args()
    args._verified_source_dirs = set()
    if args.all:
        episodes = sorted(
            path for path in args.episodes_root.glob("episode_*") if path.is_dir()
        )
        if not episodes:
            raise RuntimeError(f"没有找到 episode_*: {args.episodes_root}")
    elif args.episode_list is not None:
        episodes = _episodes_from_list(args.episode_list, args.episodes_root)
    else:
        episodes = [args.episode]

    if args.camera_view == "paired3":
        jobs = tuple(_paired3_view_args(args, view) for view in PAIRED3_VIEWS)
    else:
        jobs = (args,)
    source_views = ("e1", "e2", "e3") if args.camera_view == "e4" else (None,)
    converted = 0
    total_candidates = len(episodes) * len(source_views) * len(jobs)
    invalid_episodes = 0
    for episode in episodes:
        # An explicit --episode remains fail-fast.  In batch mode, however, one
        # quarantined/corrupt recording must not prevent later healthy episodes
        # from being converted.  Verify it once here before any of the paired
        # outputs are created, report it clearly, and continue with the batch.
        if args.all:
            try:
                source_dir = episode if episode.is_dir() else episode.parent
                _episode_mcap(episode)
                _verify_checksums_once(args, source_dir)
            except Exception as exc:  # noqa: BLE001 - batch quarantine boundary.
                invalid_episodes += 1
                print(f"[SKIP INVALID] {episode}: {exc}", file=sys.stderr)
                continue
        for job_args in jobs:
            for source_view in source_views:
                if convert_one(job_args, episode, source_camera_view=source_view):
                    converted += 1
    if args.all and invalid_episodes == len(episodes):
        raise RuntimeError(
            f"批量扫描的 {len(episodes)} 个 episode 全部未通过完整性校验"
        )
    print(
        f"完成：新增 {converted} 个 dataset episode，"
        f"跳过 {total_candidates - converted} 个；"
        f"隔离损坏 episode {invalid_episodes} 个。"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit("\n已取消。")
    except Exception as exc:  # noqa: BLE001 - CLI must turn conversion failures into a concise error.
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
