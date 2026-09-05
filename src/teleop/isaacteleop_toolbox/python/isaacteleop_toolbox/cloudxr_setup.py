"""Command-line setup for offline IsaacTeleop CloudXR host-client assets."""

from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path

from .cloudxr_host_client import prepare_workspace_cloudxr


def main() -> int:
    workspace_default = os.environ.get("CLOUDXR_DIR")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cloudxr-dir",
        type=Path,
        default=Path(workspace_default) if workspace_default else None,
        help=(
            "CloudXR data directory (default: $CLOUDXR_DIR; "
            "required when that variable is unset)"
        ),
    )
    args = parser.parse_args()
    if args.cloudxr_dir is None:
        parser.error(
            "--cloudxr-dir is required outside the Physical AI Runtime Pixi environment"
        )
    logging.basicConfig(level=logging.INFO, format="[cloudxr-setup] %(message)s")
    static_dir = prepare_workspace_cloudxr(args.cloudxr_dir)
    print(f"CloudXR host-client assets ready: {static_dir}")
    return 0
