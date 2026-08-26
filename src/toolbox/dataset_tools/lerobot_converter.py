"""Profile-driven MCAP to LeRobot dataset conversion."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any

from policy_inference.lerobot.compatibility import write_contract_manifest
from policy_inference.lerobot.utils import make_dataset_features

from .contract import DatasetContract
from .episode import ProfileEpisodeReader
from .mcap_reader import McapReader


def convert_episodes(
    episodes: Iterable[str | Path],
    *,
    contract: DatasetContract,
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
        fps=round(contract.policy.frequency),
        features=make_dataset_features(contract.policy, use_video=use_videos),
        root=output,
        robot_type=contract.policy.profile_name,
        use_videos=use_videos,
    )
    try:
        converted = 0
        for episode in episodes:
            frame_count = 0
            for sampled in ProfileEpisodeReader(McapReader(episode), contract).frames():
                dataset.add_frame({**sampled.values, "task": task})
                frame_count += 1
            if frame_count == 0:
                raise ValueError(f"episode produced no complete frames: {episode}")
            dataset.save_episode()
            converted += 1
        if converted == 0:
            raise ValueError("no episodes were provided")
        dataset.finalize()
        write_contract_manifest(output, contract.policy)
    except BaseException:
        if hasattr(dataset, "clear_episode_buffer"):
            dataset.clear_episode_buffer()
        raise
    return output
