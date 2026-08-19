#!/usr/bin/env python3
"""apps/replay.py: Production Multi-Modal Episode Replay Application.

Performs strict 1:1 native-rate trajectory replay from recorded MCAP datasets
on real or simulated robot embodiments.

Usage:
  # Replay latest recorded episode at 1:1 native rate:
  pixi run replay --profile piper_bimanual.yaml

  # Replay a specific MCAP episode file:
  pixi run replay --profile piper_bimanual.yaml \
      --episode data/episodes/piper_policy/episode_000000/episode_000000.partial_0.mcap
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

import rmi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RMI Multi-Modal Episode Replayer (1:1 Native Rate)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="piper_bimanual.yaml",
        help="Embodiment profile YAML filename or path",
    )
    parser.add_argument(
        "--episode",
        type=str,
        default="",
        help="Path to .mcap file or episode directory. Empty auto-selects the newest episode.",
    )
    return parser.parse_args()


def find_latest_mcap(episodes_root: Path) -> Path | None:
    """Scan episodes_root for the newest .mcap file."""
    if not episodes_root.exists():
        return None
    mcaps = sorted(episodes_root.glob("**/*.mcap"), key=lambda p: p.stat().st_mtime, reverse=True)
    return mcaps[0] if mcaps else None


def load_mcap_joint_stream(mcap_path: Path) -> list[tuple[float, dict[str, float]]]:
    """Extract (timestamp_s, {joint_name: position}) records from MCAP."""
    records: list[tuple[float, dict[str, float]]] = []

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            if channel.topic in {"/joint_states", "/robot/joint_states"}:
                msg = deserialize_message(message.data, JointState)
                if msg.name and msg.position:
                    pos_dict = dict(zip(msg.name, msg.position))
                    t_s = message.log_time / 1e9
                    records.append((t_s, pos_dict))

    if not records:
        raise RuntimeError(f"No valid JointState messages found in {mcap_path}")

    # Normalize timestamps relative to first frame
    t0 = records[0][0]
    return [(t - t0, pos_dict) for t, pos_dict in records]


def main() -> None:
    args = parse_args()

    # 1. Resolve episode path
    if args.episode:
        ep_path = Path(args.episode).resolve()
        if ep_path.is_dir():
            mcaps = list(ep_path.glob("*.mcap"))
            if not mcaps:
                print(f"[!] No .mcap file found in {ep_path}")
                sys.exit(1)
            ep_path = mcaps[0]
    else:
        episodes_dir = Path("data/episodes").resolve()
        latest = find_latest_mcap(episodes_dir)
        if not latest:
            print(f"[!] No recorded episodes found in {episodes_dir}. Please specify --episode.")
            sys.exit(1)
        ep_path = latest

    print("=" * 68)
    print("  RMI Multi-Modal Episode Replayer (1:1 Native Rate)")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Target Episode     : {ep_path.name}")
    print(f"  Full Path          : {ep_path}")
    print("=" * 68)

    # 2. Parse MCAP stream
    print("\n[1/2] Parsing MCAP Joint States...")
    stream = load_mcap_joint_stream(ep_path)
    total_frames = len(stream)
    duration_s = stream[-1][0]
    print(f"  [✓] Loaded {total_frames} joint state frames ({duration_s:.2f}s duration).")

    # 3. Initialize RMI Context & Session
    ctx = rmi.Context.from_profile(args.profile)
    robot = ctx.profile.name

    policy = ctx.make_agent("Policy", frequency=100.0)

    print(f"\n[2/2] Executing 1:1 Native Replay ({duration_s:.2f}s)...")
    with policy.run(robot) as session:
        start_time = time.monotonic()

        for idx, (rel_t, joint_dict) in enumerate(stream):
            # 1:1 timeline pacing
            target_wall_time = start_time + rel_t
            now = time.monotonic()
            if target_wall_time > now:
                time.sleep(target_wall_time - now)

            # Dispatch joint references to matching embodiment parts
            for part_name, part_spec in ctx.profile.parts.items():
                part_joints = part_spec.joint_names
                if all(j in joint_dict for j in part_joints):
                    part_positions = [joint_dict[j] for j in part_joints]
                    session.act(
                        rmi.Action(part=part_name, command="joint_reference", value=part_positions)
                    )

            if (idx + 1) % 30 == 0 or idx == total_frames - 1:
                pct = ((idx + 1) / total_frames) * 100.0
                curr_t = time.monotonic() - start_time
                print(
                    f"    [{pct:5.1f}% | {curr_t:4.1f}s/{duration_s:.1f}s] Frame {idx+1:4d}/{total_frames:4d}",
                    end="\r",
                    flush=True,
                )

    print("\n\n" + "=" * 68)
    print("  [✓] Episode 1:1 Native Replay Completed Successfully.")
    print("=" * 68)


if __name__ == "__main__":
    main()
