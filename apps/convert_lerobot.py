"""Convert C++ recorder MCAP episodes using the unified embodiment Profile."""

from __future__ import annotations

import argparse

from rmi.config import EmbodimentConfig

from toolbox.dataset_tools import DatasetContract, convert_episodes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--episode", action="append", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()

    profile = EmbodimentConfig.from_yaml(args.profile)
    contract = DatasetContract.from_profile(profile)
    output = convert_episodes(
        args.episode,
        contract=contract,
        output_dir=args.output,
        repo_id=args.repo_id,
        task=args.task,
        use_videos=not args.no_video,
    )
    print(f"LeRobot dataset written to {output}")


if __name__ == "__main__":
    main()
