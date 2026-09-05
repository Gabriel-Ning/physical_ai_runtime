#!/usr/bin/env python3
"""Run the validated bimanual homing/replay flow with the Piper profile."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def _has_option(name: str) -> bool:
    return any(arg == name or arg.startswith(f"{name}=") for arg in sys.argv[1:])


def main() -> None:
    if not _has_option("--profile"):
        sys.argv.extend(["--profile", "piper_bimanual.yaml"])
    if not _has_option("--episode") and "--help" not in sys.argv[1:]:
        candidates = list(Path("data/episodes/piper_bimanual").glob("episode_*/*.mcap"))
        if not candidates:
            raise FileNotFoundError(
                "no Piper episode MCAP found under data/episodes/piper_bimanual"
            )
        latest = max(candidates, key=lambda path: path.stat().st_mtime)
        sys.argv.extend(["--episode", str(latest)])
    runpy.run_path(
        str(Path(__file__).with_name("19_marvin_homing_replay.py")),
        run_name="__main__",
    )


if __name__ == "__main__":
    main()
