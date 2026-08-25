#!/usr/bin/env python3
"""Convert Piper MCAP episodes into a LeRobot Dataset v3.0 dataset.

The recorder stores asynchronous ROS topics in one MCAP directory.  This
converter creates a regular ``fps`` timeline from the selected static RGB
camera and two wrist RGB camera
streams, then synchronizes state and absolute joint-reference actions onto
that timeline.  It can create a new dataset or append episodes to an existing
one; converted source episodes are tracked in ``piper_conversion_manifest.json``
so rerunning the command is safe.

The script intentionally does not clamp or otherwise alter joint values.  The
recorded action topic is the downstream absolute ``joint_reference`` command
and is preserved as-is.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

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

IMAGE_ORBBEC = "/observation/static_orbbec/color/image_raw"
IMAGE_STATIC_REALSENSE = "/observation/static_realsense/color/image_raw"
IMAGE_LEFT = "/observation/left_hand_realsense/color/image_raw"
IMAGE_RIGHT = "/observation/right_hand_realsense/color/image_raw"
IMAGE_TOP_KEY = "observation.images.top"
IMAGE_LEFT_KEY = "observation.images.left_wrist"
IMAGE_RIGHT_KEY = "observation.images.right_wrist"
STATIC_IMAGE_TOPICS = (IMAGE_ORBBEC, IMAGE_STATIC_REALSENSE)
WRIST_IMAGE_TOPICS = (IMAGE_LEFT, IMAGE_RIGHT)

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
    """Use the recorder checksum when available, without hashing a large MCAP again."""
    checksum_file = episode / "checksums.sha256"
    if checksum_file.is_file():
        for line in checksum_file.read_text(encoding="utf-8").splitlines():
            digest, _, filename = line.partition("  ")
            if filename == mcap.name and len(digest) == 64:
                return digest
    stat = mcap.stat()
    return f"size={stat.st_size};mtime_ns={stat.st_mtime_ns}"


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
    required = {
        *WRIST_IMAGE_TOPICS,
        STATE_TOPIC,
        LEFT_ARM_TOPIC,
        RIGHT_ARM_TOPIC,
        LEFT_GRIPPER_TOPIC,
        RIGHT_GRIPPER_TOPIC,
    }
    missing = sorted(required - topic_types.keys())
    if missing:
        raise RuntimeError(f"MCAP 缺少必要 topic: {', '.join(missing)}")

    if not any(topic in topic_types for topic in STATIC_IMAGE_TOPICS):
        raise RuntimeError("MCAP 缺少静态相机 topic: " + ", ".join(STATIC_IMAGE_TOPICS))

    numeric: dict[str, list[Sample]] = {
        topic: [] for topic in required if topic not in WRIST_IMAGE_TOPICS
    }
    image_timestamps: dict[str, list[int]] = {
        topic: []
        for topic in (*STATIC_IMAGE_TOPICS, *WRIST_IMAGE_TOPICS)
        if topic in topic_types
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
        decoded[topic][index] = _decode_rgb_image(msg)

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


def _build_features(static_image_topic: str) -> dict[str, dict[str, Any]]:
    image_shape = ["height", "width", "channels"]
    static_shape = (
        [720, 1280, 3] if static_image_topic == IMAGE_ORBBEC else [480, 640, 3]
    )
    return {
        IMAGE_TOP_KEY: {"dtype": "video", "shape": static_shape, "names": image_shape},
        IMAGE_LEFT_KEY: {
            "dtype": "video",
            "shape": [480, 640, 3],
            "names": image_shape,
        },
        IMAGE_RIGHT_KEY: {
            "dtype": "video",
            "shape": [480, 640, 3],
            "names": image_shape,
        },
        "observation.state": {"dtype": "float32", "shape": [14], "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": [14], "names": JOINT_NAMES},
    }


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


def _already_converted(root: Path, source: Path, fingerprint: str) -> bool:
    manifest = _load_json(_manifest_path(root))
    for entry in manifest.get("episodes", []):
        if entry.get("source_fingerprint") == fingerprint or entry.get(
            "source_episode"
        ) == str(source.resolve()):
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


def convert_one(args: argparse.Namespace, episode: Path) -> bool:
    rosbag2_py, deserialize_message, get_message, LeRobotDataset = (
        _require_ros_and_lerobot()
    )
    episode = episode.resolve()
    mcap = _episode_mcap(episode)
    source_dir = episode if episode.is_dir() else episode.parent
    fingerprint = _source_fingerprint(source_dir, mcap)
    output_root = args.output.resolve()

    if _already_converted(output_root, episode, fingerprint):
        print(f"[SKIP] 已转换: {episode}")
        return False

    capture_manifest = _validate_capture(source_dir, args.allow_unhealthy)
    assist_metadata = _load_assist_metadata(source_dir)
    numeric, image_timestamps, topic_types = _collect_samples(
        rosbag2_py, deserialize_message, get_message, mcap
    )
    static_image_topic = _require_image_samples(image_timestamps)
    image_topics = (static_image_topic, *WRIST_IMAGE_TOPICS)
    start_ns = max(image_timestamps[topic][0] for topic in image_topics)
    end_ns = min(image_timestamps[topic][-1] for topic in image_topics)
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
        expected_features = _build_features(static_image_topic)
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
            features=_build_features(static_image_topic),
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
                IMAGE_TOP_KEY: images[static_image_topic][index],
                IMAGE_LEFT_KEY: images[IMAGE_LEFT][index],
                IMAGE_RIGHT_KEY: images[IMAGE_RIGHT][index],
                "observation.state": state_vector,
                "action": action_vector,
                "task": task,
            }
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

    manifest_entry = {
        "dataset_episode_index": dataset_episode_index,
        "repo_id": repo_id,
        "source_episode": str(source_dir),
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
        "static_image_topic": static_image_topic,
    }
    if assist_metadata is not None:
        manifest_entry["policy_assist"] = assist_metadata
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
    parser.add_argument(
        "--episodes-root",
        type=Path,
        default=None,
        help="--all 扫描的目录；未指定时由 --task 推导",
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
    parser.add_argument("--image-writer-threads", type=int, default=4)
    parser.add_argument("--no-parallel-encoding", action="store_true")
    parser.add_argument("--allow-unhealthy", action="store_true")
    parser.add_argument("--skip-reload-validation", action="store_true")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps 必须大于 0")
    if args.image_writer_threads < 0:
        parser.error("--image-writer-threads 不能小于 0")

    if args.task:
        args.episodes_root = args.episodes_root or Path("data/episodes") / args.task
        args.output = args.output or Path.home() / "lerobot_train" / args.task
        args.repo_id = args.repo_id or args.task
    else:
        args.episodes_root = args.episodes_root or Path("data/episodes/piper_bimanual")
        args.output = args.output or Path("data/lerobot/piper_bimanual_v3")
        args.repo_id = args.repo_id or "local/piper_bimanual_v3"
    return args


def main() -> int:
    args = parse_args()
    if args.all:
        episodes = sorted(
            path for path in args.episodes_root.glob("episode_*") if path.is_dir()
        )
        if not episodes:
            raise RuntimeError(f"没有找到 episode_*: {args.episodes_root}")
    else:
        episodes = [args.episode]

    converted = 0
    for episode in episodes:
        if convert_one(args, episode):
            converted += 1
    print(f"完成：新增 {converted} 个 episode，跳过 {len(episodes) - converted} 个。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit("\n已取消。")
    except Exception as exc:  # noqa: BLE001 - CLI must turn conversion failures into a concise error.
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
