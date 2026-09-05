"""Profile-driven MCAP to LeRobot dataset conversion."""

from __future__ import annotations

import json
from collections.abc import Iterable
from pathlib import Path
from typing import Any

from rmi import PolicyLayout

from .episode import ProfileEpisodeReader
from .mcap_reader import McapReader


def convert_episodes(
    episodes: Iterable[str | Path],
    *,
    layout: PolicyLayout,
    output_dir: str | Path,
    repo_id: str,
    task: str,
    use_videos: bool = True,
    dataset_factory: Any | None = None,
) -> Path:
    """Convert episodes with lazy image decoding and native LeRobot writing."""
    if not task.strip():
        raise ValueError("task must not be empty")
    output = Path(output_dir)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"output dataset is not empty: {output}")
    if dataset_factory is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        dataset_factory = LeRobotDataset.create

    dataset = dataset_factory(
        repo_id=repo_id,
        fps=round(layout.frequency),
        features=_make_dataset_features(layout, use_video=use_videos),
        root=output,
        robot_type=layout.profile_name,
        use_videos=use_videos,
    )
    try:
        converted = 0
        for episode in episodes:
            frame_count = 0
            reader = ProfileEpisodeReader(McapReader(episode), layout)
            for sampled in reader.frames():
                dataset.add_frame({**sampled.values, "task": task})
                frame_count += 1
            if frame_count == 0:
                raise ValueError(f"episode produced no complete frames: {episode}")
            dataset.save_episode()
            converted += 1
        if converted == 0:
            raise ValueError("no episodes were provided")
        dataset.finalize()
        _write_contract_manifest(output, layout)
    except BaseException:
        if hasattr(dataset, "clear_episode_buffer"):
            dataset.clear_episode_buffer()
        raise
    return output


def _make_dataset_features(
    layout: PolicyLayout, *, use_video: bool
) -> dict[str, dict]:
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import hw_to_dataset_features

    observations: dict[str, type | tuple[int, int, int]] = {
        name: float for name in layout.state_feature_names
    }
    observations.update(
        {
            name.removeprefix("observation.images."): shape
            for name, shape in layout.camera_shapes.items()
        }
    )
    actions = {name: float for name in layout.action_feature_names}
    return {
        **hw_to_dataset_features(observations, OBS_STR, use_video=use_video),
        **hw_to_dataset_features(actions, ACTION, use_video=use_video),
    }


def _write_contract_manifest(output: Path, layout: PolicyLayout) -> Path:
    path = output / "policy_contract.json"
    payload = {
        "profile": layout.profile_name,
        "profile_hash": layout.profile_hash,
        "state_names": list(layout.state_feature_names),
        "action_names": list(layout.action_feature_names),
        "image_features": list(layout.camera_shapes),
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path
