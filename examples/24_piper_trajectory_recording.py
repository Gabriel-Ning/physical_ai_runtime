#!/usr/bin/env python3
"""Run the validated bimanual JTC recording flow with the Piper profile."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def _has_option(name: str) -> bool:
    return any(arg == name or arg.startswith(f"{name}=") for arg in sys.argv[1:])


def main() -> None:
    if not _has_option("--profile"):
        sys.argv.extend(["--profile", "piper_bimanual.yaml"])
    if not _has_option("--task"):
        sys.argv.extend(["--task", "piper_bimanual_jtc_swing"])
    runpy.run_path(
        str(Path(__file__).with_name("20_marvin_trajectory_recording.py")),
        run_name="__main__",
    )


if __name__ == "__main__":
    main()
