#!/usr/bin/env python3
"""Validate three paired single-external-camera LeRobot datasets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import av
import pyarrow as pa
import pyarrow.parquet as pq
from pyarrow import ipc

IMAGE_KEYS = (
    "observation.images.top",
    "observation.images.left_wrist",
    "observation.images.right_wrist",
)
CORE_COLUMNS = ("observation.state", "action", "timestamp", "frame_index")
EXPECTED_CAMERAS = {
    "orbbec": "orbbec",
    "d435i1": "d435i1",
    "d435i2": "d435i2",
}


def _json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _episode_file(root: Path, kind: str, episode_index: int) -> Path:
    return (
        root
        / kind
        / f"chunk-{episode_index // 1000:03d}"
        / (f"file-{episode_index % 1000:03d}.parquet")
    )


def _video_file(root: Path, key: str, episode_index: int) -> Path:
    return (
        root
        / "videos"
        / key
        / f"chunk-{episode_index // 1000:03d}"
        / (f"file-{episode_index % 1000:03d}.mp4")
    )


def _core_hash(path: Path) -> tuple[str, int]:
    table = pq.read_table(path, columns=list(CORE_COLUMNS)).replace_schema_metadata(
        None
    )
    sink = pa.BufferOutputStream()
    with ipc.new_stream(sink, table.schema) as writer:
        writer.write_table(table)
    return hashlib.sha256(sink.getvalue().to_pybytes()).hexdigest(), table.num_rows


def _decode_video(path: Path) -> dict[str, Any]:
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        width, height = stream.width, stream.height
        fps = float(stream.average_rate) if stream.average_rate else None
        frames = sum(1 for _ in container.decode(stream))
    return {"width": width, "height": height, "fps": fps, "frames": frames}


def _fail(errors: list[str], message: str) -> None:
    errors.append(message)


def validate(roots: dict[str, Path], expected_episodes: int) -> dict[str, Any]:
    errors: list[str] = []
    datasets: dict[str, Any] = {}
    manifests: dict[str, dict[str, dict[str, Any]]] = {}

    for view, root in roots.items():
        info = _json(root / "meta" / "info.json")
        manifest_payload = _json(root / "piper_conversion_manifest.json")
        entries = manifest_payload.get("episodes", [])
        by_source = {entry["source_episode_id"]: entry for entry in entries}
        if len(by_source) != len(entries):
            _fail(errors, f"{view}: duplicate source_episode_id")
        manifests[view] = by_source

        file_counts = {
            "data_parquet": len(list((root / "data").glob("chunk-*/*.parquet"))),
            "episode_parquet": len(
                list((root / "meta" / "episodes").glob("chunk-*/*.parquet"))
            ),
            **{
                f"video:{key}": len(list((root / "videos" / key).glob("chunk-*/*.mp4")))
                for key in IMAGE_KEYS
            },
        }
        for label, actual in file_counts.items():
            if actual != expected_episodes:
                _fail(errors, f"{view}: {label}={actual}, expected={expected_episodes}")
        if info.get("total_episodes") != expected_episodes:
            _fail(
                errors,
                f"{view}: info.total_episodes={info.get('total_episodes')}, "
                f"expected={expected_episodes}",
            )
        if len(entries) != expected_episodes:
            _fail(
                errors, f"{view}: manifest={len(entries)}, expected={expected_episodes}"
            )

        features = info.get("features", {})
        actual_image_keys = sorted(
            key for key, value in features.items() if value.get("dtype") == "video"
        )
        if actual_image_keys != sorted(IMAGE_KEYS):
            _fail(errors, f"{view}: image keys={actual_image_keys}")
        for key in IMAGE_KEYS:
            if features.get(key, {}).get("shape") != [480, 640, 3]:
                _fail(errors, f"{view}: {key} shape is not [480, 640, 3]")
        if features.get("observation.state", {}).get("shape") != [14]:
            _fail(errors, f"{view}: state shape is not [14]")
        if features.get("action", {}).get("shape") != [14]:
            _fail(errors, f"{view}: action shape is not [14]")

        datasets[view] = {
            "root": str(root),
            "repo_id": manifest_payload.get("repo_id"),
            "total_episodes": info.get("total_episodes"),
            "total_frames": info.get("total_frames"),
            "manifest_entries": len(entries),
            "file_counts": file_counts,
        }

    source_sets = {view: set(items) for view, items in manifests.items()}
    reference_view = "orbbec"
    reference_sources = source_sets[reference_view]
    for view, source_ids in source_sets.items():
        if source_ids != reference_sources:
            _fail(
                errors,
                f"{view}: source set differs; missing={sorted(reference_sources - source_ids)}, "
                f"extra={sorted(source_ids - reference_sources)}",
            )

    checked_sources = 0
    checked_videos = 0
    for source_id in sorted(set.intersection(*source_sets.values())):
        entries = {view: manifests[view][source_id] for view in roots}
        for field in ("source_fingerprint", "frames", "fps", "synchronization_topics"):
            values = {
                json.dumps(entry.get(field), sort_keys=True)
                for entry in entries.values()
            }
            if len(values) != 1:
                _fail(errors, f"{source_id}: {field} differs across views")
        for view, entry in entries.items():
            label = entry.get("demonstration_label", {})
            if not (
                label.get("demonstration_quality") == "accepted_success"
                and label.get("stable_corner_grasp_success") is True
                and label.get("human_verified") is True
            ):
                _fail(errors, f"{source_id}/{view}: not a verified accepted success")
            if entry.get("model_image_size") != [480, 640]:
                _fail(errors, f"{source_id}/{view}: model_image_size is not [480, 640]")
            cameras = entry.get("camera_sources", {})
            if (
                cameras.get("observation.images.top", {}).get("camera_id")
                != EXPECTED_CAMERAS[view]
            ):
                _fail(errors, f"{source_id}/{view}: wrong top camera provenance")

        core: dict[str, tuple[str, int]] = {}
        wrists: dict[str, dict[str, str]] = {}
        for view, entry in entries.items():
            episode_index = int(entry["dataset_episode_index"])
            core[view] = _core_hash(_episode_file(roots[view], "data", episode_index))
            wrists[view] = {
                key: _sha256(_video_file(roots[view], key, episode_index))
                for key in IMAGE_KEYS[1:]
            }
            for key in IMAGE_KEYS:
                video = _decode_video(_video_file(roots[view], key, episode_index))
                checked_videos += 1
                if (video["width"], video["height"]) != (640, 480):
                    _fail(errors, f"{source_id}/{view}/{key}: video size={video}")
                if video["fps"] is None or abs(video["fps"] - 30.0) > 1e-6:
                    _fail(errors, f"{source_id}/{view}/{key}: video fps={video['fps']}")
                if video["frames"] != entry["frames"]:
                    _fail(
                        errors,
                        f"{source_id}/{view}/{key}: decoded={video['frames']}, "
                        f"manifest={entry['frames']}",
                    )
        if len({value for value in core.values()}) != 1:
            _fail(errors, f"{source_id}: state/action/timestamp/frame_index differs")
        for key in IMAGE_KEYS[1:]:
            if len({value[key] for value in wrists.values()}) != 1:
                _fail(errors, f"{source_id}: {key} video differs across views")
        checked_sources += 1

    return {
        "schema_version": "1.0",
        "expected_episodes": expected_episodes,
        "passed": not errors,
        "datasets": datasets,
        "paired_sources_checked": checked_sources,
        "videos_fully_decoded": checked_videos,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--orbbec", type=Path, required=True)
    parser.add_argument("--d435i1", type=Path, required=True)
    parser.add_argument("--d435i2", type=Path, required=True)
    parser.add_argument("--expected-episodes", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = validate(
        {"orbbec": args.orbbec, "d435i1": args.d435i1, "d435i2": args.d435i2},
        args.expected_episodes,
    )
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
