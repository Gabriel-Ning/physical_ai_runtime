#!/usr/bin/env python3
"""Home Marvin, then replay a recorded bimanual episode as a MEMORY source."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
import rmi
from rclpy.utilities import remove_ros_args


def latest_episode() -> Path:
    candidates = list(Path("data/episodes/marvin_bimanual").glob("episode_*/*.mcap"))
    if not candidates:
        raise FileNotFoundError(
            "no Marvin episode MCAP found under data/episodes/marvin_bimanual"
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


class SmoothHomingPlanner:
    """Build profile-configured arm trajectories and gripper commands."""

    def __init__(self, profile: rmi.EmbodimentConfig) -> None:
        homing = profile.raw_data.get("homing", {})
        self.duration_s = float(homing.get("duration_s", 0.0))
        self._targets = homing.get("joint_positions", {})
        self._parts = profile.parts
        if self.duration_s <= 0.0 or not isinstance(self._targets, dict):
            raise ValueError(
                "profile.homing requires positive duration_s and joint_positions"
            )

    def plan(self, observation: rmi.Observation) -> dict[str, object]:
        current = dict(
            zip(
                observation.data["joint_names"],
                observation.data["joint_positions"],
                strict=False,
            )
        )
        plans: dict[str, object] = {}
        for part_name, raw_target in self._targets.items():
            part = self._parts.get(part_name)
            if part is None:
                raise ValueError(f"homing references unknown part {part_name!r}")
            target = [float(value) for value in raw_target]
            if len(target) != len(part.joint_names):
                raise ValueError(
                    f"homing target for {part_name} has {len(target)} values; "
                    f"expected {len(part.joint_names)}"
                )
            missing = [name for name in part.joint_names if name not in current]
            if missing:
                raise RuntimeError(
                    f"joint state is missing {part_name} joints: {missing}"
                )
            if part.part_type == "parallel_gripper":
                plans[part_name] = target
                continue
            zeros = [0.0] * len(target)
            plans[part_name] = rmi.PlanResult(
                valid=True,
                joint_names=list(part.joint_names),
                points=[
                    rmi.PlanPoint(
                        positions=[current[name] for name in part.joint_names],
                        velocities=zeros,
                        accelerations=zeros,
                        time_from_start_s=0.0,
                    ),
                    rmi.PlanPoint(
                        positions=target,
                        velocities=zeros,
                        accelerations=zeros,
                        time_from_start_s=self.duration_s,
                    ),
                ],
            )
        return plans


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="marvin_bimanual.yaml")
    parser.add_argument(
        "--episode",
        type=Path,
        help="Episode directory or MCAP file (default: latest Marvin episode)",
    )
    parser.add_argument(
        "--source",
        choices=("auto", "joint_states", "commands"),
        default="auto",
        help="auto uses realized joint states when teleop intervention was recorded",
    )
    parser.add_argument(
        "--max-rate-hz",
        type=float,
        default=30.0,
        help="maximum command rate; higher-rate recordings are downsampled",
    )
    args = parser.parse_args(remove_ros_args()[1:])
    if args.max_rate_hz <= 0.0:
        parser.error("--max-rate-hz must be positive")
    episode = args.episode or latest_episode()

    rclpy.init()
    try:
        with rmi.Context.from_profile(args.profile, timeout_sec=15.0) as context:
            robot = context.robot
            homing_planner = SmoothHomingPlanner(context.profile)
            planner_node = context.make_node("Planner", homing_planner)
            left_replay = rmi.EpisodeReplayPolicy.from_mcap(
                episode, context.profile, side="left", source=args.source
            )
            right_replay = rmi.EpisodeReplayPolicy.from_mcap(
                episode, context.profile, side="right", source=args.source
            )
            replay_node = context.make_node("Replay")

            context.wait_until_ready(timeout=30.0, require_execution_manager=True)
            print(f"[HOMING] Executing homing over {homing_planner.duration_s:.1f}s...")
            plans = homing_planner.plan(robot["dual_manipulator"].get_observation())
            with planner_node.activate():
                planner_node["dual_manipulator"].execute(
                    plans, timeout=homing_planner.duration_s + 5.0
                )
            print("[HOMING] Completed.")

            if left_replay.info.frame_count != right_replay.info.frame_count:
                raise RuntimeError(
                    "left and right replay timelines have different lengths"
                )
            source_rate_hz = min(left_replay.info.rate_hz, right_replay.info.rate_hz)
            frame_stride = max(1, math.ceil(source_rate_hz / args.max_rate_hz))
            rate_hz = source_rate_hz / frame_stride
            print(
                f"[REPLAY] MEMORY source replaying {left_replay.info.frame_count} "
                f"{left_replay.info.source} frames from {episode} at {rate_hz:.1f} Hz "
                f"(source {source_rate_hz:.1f} Hz, stride {frame_stride})"
            )
            period_s = 1.0 / rate_hz
            pacer = rmi.ReplayPacer.from_node(context.node)
            pacer.start()
            frame_index = 0
            with replay_node.activate():
                while rclpy.ok() and not left_replay.done and not right_replay.done:
                    pacer.wait_until(frame_index * period_s)
                    observation = robot["dual_manipulator"].get_observation()
                    actions = []
                    for _ in range(frame_stride):
                        if left_replay.done or right_replay.done:
                            break
                        actions = left_replay.select_action(observation)
                        actions += right_replay.select_action(observation)
                    if not actions:
                        break
                    replay_node["dual_manipulator"].submit(actions)
                    frame_index += 1
            print("[DONE] Marvin bimanual episode replay completed")
    except KeyboardInterrupt:
        print("\n[STOP] Replay interrupted")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
