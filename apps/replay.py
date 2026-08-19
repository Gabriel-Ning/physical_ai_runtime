#!/usr/bin/env python3
"""apps/replay.py: Production Multi-Modal Episode Replay Application.

Performs strict 1:1 native-rate Action trajectory replay from recorded MCAP datasets
on real or simulated robot embodiments, with smooth JTC pre-alignment.

Workflow:
1. Phase 1 (JTC Smooth Homing): Uses the Planner Agent (JointTrajectoryController)
   to smoothly drive the robot from its live current pose to the episode's initial
   Action configuration via quintic spline (eliminating startup jitter).
2. Phase 2 (1:1 Native Action Replay): Switches to Policy Agent (JointSpacePosition/Impedance
   Controller & ForwardCommandController) to replay the multi-modal Action commands
   at their exact recorded 1:1 Action rate.

Usage:
  # Replay latest recorded episode at 1:1 native rate:
  pixi run replay --profile piper_bimanual.yaml

  # Replay a specific MCAP episode:
  pixi run replay --profile piper_bimanual.yaml \
      --mcap-file data/episodes/fr3_pika_policy/episode_000004/episode_000004.mcap
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory

import rmi
from rmi.planning import PlanPoint, PlanResult


class ProgressBar:
    """ASCII progress bar with live throughput stats."""

    def __init__(self, total_steps: int, prefix: str = "Replaying Action", width: int = 35) -> None:
        self.total_steps = max(total_steps, 1)
        self.prefix = prefix
        self.width = width
        self.start_time = time.monotonic()
        self.last_update = 0.0

    def update(self, current_step: int, extra_info: str = "") -> None:
        now = time.monotonic()
        if now - self.last_update < 0.05 and current_step < self.total_steps:
            return
        self.last_update = now

        progress = min(current_step / self.total_steps, 1.0)
        filled = int(self.width * progress)
        bar = "█" * filled + "░" * (self.width - filled)
        elapsed = now - self.start_time
        fps = current_step / max(elapsed, 1e-6)

        info_str = f" | {extra_info}" if extra_info else ""
        msg = f"\r  {self.prefix} [{bar}] {current_step:4d}/{self.total_steps} ({progress * 100:5.1f}%) | {elapsed:5.1f}s ({fps:4.1f} Hz){info_str}"
        print(msg, end="", flush=True)

    def finish(self) -> None:
        self.update(self.total_steps)
        print()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RMI Multi-Modal Episode Replayer with JTC Homing Pre-Alignment",
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
        "--mcap-file",
        dest="episode",
        type=str,
        default="",
        help="Path to .mcap file or episode directory. Empty auto-selects the newest episode.",
    )
    parser.add_argument(
        "--homing-duration",
        type=float,
        default=None,
        help="Duration in seconds for smooth JTC homing alignment to start pose (default: from profile)",
    )
    return parser.parse_args()


def find_latest_mcap(episodes_root: Path) -> Path | None:
    """Scan episodes_root for the newest .mcap file."""
    if not episodes_root.exists():
        return None
    mcaps = sorted(episodes_root.glob("**/*.mcap"), key=lambda p: p.stat().st_mtime, reverse=True)
    return mcaps[0] if mcaps else None


def load_mcap_trajectory(
    mcap_path: Path, profile: rmi.EmbodimentConfig
) -> tuple[list[tuple[float, dict[str, list[float]]]], float, dict[str, list[float]]]:
    """Load Action command reference trajectory (Action-First) or fallback to /joint_states.

    Returns:
        timeline: list of (rel_time_s, {part_name: positions_list})
        native_hz: calculated action stream frequency
        start_poses: {part_name: initial_positions_list}
    """
    # Build topic-to-part mapping from profile
    part_to_action_topic: dict[str, str] = {}
    for part_name, part_spec in profile.parts.items():
        if "arm" in part_name:
            part_to_action_topic[part_name] = f"/execution/{part_name}/joint_reference"
        elif "gripper" in part_name:
            part_to_action_topic[part_name] = f"/execution/{part_name}/joint_reference"
        elif part_name == "end_effector":
            part_to_action_topic[part_name] = "/execution/end_effector/joint_reference"

    # Also check generic fallback topics
    if "arm" in profile.parts and "arm" not in part_to_action_topic:
        part_to_action_topic["arm"] = "/execution/arm/joint_reference"

    topic_to_part = {v: k for k, v in part_to_action_topic.items()}

    # Collect streams
    action_events: list[tuple[int, str, list[float]]] = []
    state_events: list[tuple[int, dict[str, float]]] = []

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            if channel.topic in topic_to_part:
                part_name = topic_to_part[channel.topic]
                try:
                    jt = deserialize_message(message.data, RosJointTrajectory)
                    if jt.points:
                        pos = list(jt.points[-1].positions)
                        action_events.append((message.log_time, part_name, pos))
                except Exception:
                    try:
                        arr = deserialize_message(message.data, Float64MultiArray)
                        if arr.data:
                            action_events.append((message.log_time, part_name, list(arr.data)))
                    except Exception:
                        pass

            elif channel.topic in {"/joint_states", "/robot/joint_states"}:
                try:
                    js = deserialize_message(message.data, JointState)
                    if js.name and js.position:
                        name_to_pos = dict(zip(js.name, js.position))
                        state_events.append((message.log_time, name_to_pos))
                except Exception:
                    pass

    # Action-First resolution
    timeline: list[tuple[float, dict[str, list[float]]]] = []
    start_poses: dict[str, list[float]] = {}

    if action_events:
        # Sort chronologically
        action_events.sort(key=lambda x: x[0])
        t0 = action_events[0][0]

        # Partition events by part to compute accurate per-arm frequency
        part_counts: dict[str, int] = {}
        for _, part_name, _ in action_events:
            part_counts[part_name] = part_counts.get(part_name, 0) + 1

        primary_part = max(part_counts.keys(), key=lambda k: part_counts[k])
        total_primary_frames = part_counts[primary_part]
        duration_s = (action_events[-1][0] - t0) / 1e9
        native_hz = round(total_primary_frames / max(duration_s, 0.01), 1)

        counts_summary = ", ".join(f"{k}: {v} frames" for k, v in part_counts.items())
        print(f"  [✓] Action Stream Breakdown: {counts_summary}")
        print(f"  [✓] Primary Action Frequency: {native_hz:.1f} Hz (Duration: {duration_s:.2f}s)")

        # Synchronize multi-part state over timeline
        current_part_poses: dict[str, list[float]] = {}
        for log_time, part_name, pos in action_events:
            rel_t = (log_time - t0) / 1e9
            current_part_poses[part_name] = pos
            if part_name not in start_poses:
                start_poses[part_name] = list(pos)
            timeline.append((rel_t, dict(current_part_poses)))

    elif state_events:
        print(f"  [!] No Action topics found in MCAP; falling back to {len(state_events)} /joint_states frames.")
        state_events.sort(key=lambda x: x[0])
        t0 = state_events[0][0]

        for log_time, name_to_pos in state_events:
            rel_t = (log_time - t0) / 1e9
            frame_parts: dict[str, list[float]] = {}
            for part_name, part_spec in profile.parts.items():
                part_joints = part_spec.joint_names
                if all(j in name_to_pos for j in part_joints):
                    pos = [name_to_pos[j] for j in part_joints]
                    frame_parts[part_name] = pos
                    if part_name not in start_poses:
                        start_poses[part_name] = list(pos)
            timeline.append((rel_t, frame_parts))

        duration_s = timeline[-1][0]
        native_hz = 50.0
        if duration_s > 0.01 and len(timeline) > 1:
            native_hz = round((len(timeline) - 1) / duration_s, 1)
    else:
        raise RuntimeError(f"No valid Joint references or JointStates found in {mcap_path}")

    return timeline, native_hz, start_poses


def generate_quintic_transition(
    joint_names: list[str],
    q_start: list[float],
    q_goal: list[float],
    duration_s: float = 3.0,
    steps: int = 50,
) -> PlanResult:
    """Generate smooth quintic polynomial trajectory plan from q_start to q_goal."""
    points = []
    steps = max(steps, 10)
    for i in range(steps + 1):
        s = i / steps
        # Quintic polynomial factor: h(s) = 10*s^3 - 15*s^4 + 6*s^5
        h = 10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5)
        vel_scale = (30.0 * (s**2) - 60.0 * (s**3) + 30.0 * (s**4)) / max(duration_s, 0.1)

        pos = [q_start[j] + h * (q_goal[j] - q_start[j]) for j in range(len(q_start))]
        vel = [vel_scale * (q_goal[j] - q_start[j]) for j in range(len(q_start))]
        t = s * duration_s
        points.append(PlanPoint(positions=pos, velocities=vel, time_from_start_s=t))

    return PlanResult(valid=True, joint_names=joint_names, points=points)


def main() -> None:
    args = parse_args()

    # 1. Connect RMI Context & Topology
    ctx = rmi.Context.from_profile(args.profile)
    rec_cfg = ctx.profile.raw_data.get("recorder", {})
    homing_duration_s = (
        args.homing_duration if args.homing_duration is not None else rec_cfg.get("homing_duration_s", 3.0)
    )

    # 2. Resolve episode path
    if args.episode:
        ep_path = Path(args.episode).resolve()
        if ep_path.is_dir():
            mcaps = sorted(ep_path.glob("*.mcap"), key=lambda p: p.stat().st_mtime, reverse=True)
            if not mcaps:
                print(f"[!] No .mcap file found in {ep_path}")
                sys.exit(1)
            ep_path = mcaps[0]
    else:
        episodes_dir = Path(rec_cfg.get("root_dir", "data/episodes")).resolve()
        latest = find_latest_mcap(episodes_dir)
        if not latest:
            print(f"[!] No recorded episodes found in {episodes_dir}. Please specify --episode.")
            sys.exit(1)
        ep_path = latest

    print("=" * 72)
    print("  RMI Multi-Modal Episode Replayer (JTC Homing + 1:1 Action Replay)")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Target Episode     : {ep_path.name}")
    print(f"  Full Path          : {ep_path}")
    print(f"  Homing Duration    : {homing_duration_s:.1f} s (profile: {rec_cfg.get('homing_duration_s', 'n/a')}s)")
    print("=" * 72)

    # 3. Wait for Hardware Readiness
    print("\n[1/4] Waiting for hardware and controller readiness...")
    ctx.wait_until_ready(timeout=6.0)
    robot = ctx.robot

    # 3. Parse MCAP Action Stream (Action-First)
    print("\n[2/4] Loading Recorded Action Trajectory from MCAP...")
    timeline, native_hz, start_poses = load_mcap_trajectory(ep_path, ctx.profile)
    total_frames = len(timeline)
    duration_s = timeline[-1][0]
    print(
        f"  [✓] Parsed {total_frames} Action events ({duration_s:.2f}s duration @ {native_hz:.1f} Hz native Action rate)."
    )

    # 4. Phase 1: JTC Smooth Transition to Initial Pose
    print("\n[3/4] Phase 1: Smooth JTC Homing to Episode Start Pose...")
    current_obs = robot.get_observation()
    name_to_pos = dict(zip(current_obs.joint_names, current_obs.joint_positions))

    planner_agent = ctx.make_agent("Planner")
    with planner_agent.run(robot) as planner_session:
        for part_name, q_start in start_poses.items():
            if part_name not in ctx.profile.parts:
                continue
            part_joints = list(ctx.profile.parts[part_name].joint_names)
            if len(part_joints) < 2:
                # Single-joint grippers handled in replay directly
                continue

            if all(j in name_to_pos for j in part_joints):
                q_curr = [name_to_pos[j] for j in part_joints]
                max_delta = max(abs(c - s) for c, s in zip(q_curr, q_start[:len(part_joints)]))

                if max_delta > 0.005:
                    print(
                        f"  -> Part '{part_name}': delta = {max_delta:.4f} rad. Executing smooth JTC homing ({homing_duration_s:.1f}s)..."
                    )
                    homing_plan = generate_quintic_transition(
                        part_joints,
                        q_curr,
                        q_start[:len(part_joints)],
                        duration_s=homing_duration_s,
                    )
                    execution = planner_session.execute(part_name, homing_plan)
                    execution.wait(timeout=homing_duration_s + 5.0)
                    if execution.done and not execution.canceled:
                        print(f"  [✓] Part '{part_name}' settled smoothly at episode start pose.")
                    else:
                        print(f"  [!] Part '{part_name}' homing state: {execution.state.name}")
                else:
                    print(f"  [✓] Part '{part_name}' already aligned (delta = {max_delta:.4f} rad).")

    # 5. Phase 2: High-Rate 1:1 Policy Replay Loop
    print(f"\n[4/4] Phase 2: Activating JSIC Controller & Replaying Actions ({total_frames} frames @ {native_hz:.1f} Hz)...")
    policy_agent = ctx.make_agent("Policy", frequency=native_hz)

    try:
        with policy_agent.run(robot) as session:
            pbar = ProgressBar(total_frames, prefix="Replaying Action")
            start_time = time.monotonic()

            for idx, (rel_t, part_poses) in enumerate(timeline):
                target_wall_time = start_time + rel_t
                now = time.monotonic()
                if target_wall_time > now:
                    time.sleep(target_wall_time - now)

                # Dispatch joint positions for all parts in this frame
                for part_name, pos in part_poses.items():
                    session.act(
                        rmi.Action(part=part_name, command="joint_reference", value=pos)
                    )

                pbar.update(idx + 1)

            pbar.finish()
            print("\n[✓] Episode Action trajectory replay completed successfully!")

    except KeyboardInterrupt:
        print("\n\n[!] Replay interrupted by operator (Ctrl+C).")
    finally:
        ctx.close()


if __name__ == "__main__":
    main()
