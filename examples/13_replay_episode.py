#!/usr/bin/env python3
"""13_replay_episode.py: Offline Unindexed Episode Dataset Replay Agent.

Demonstrates **1:1 Native Trajectory Replay with Smooth JTC Homing**:
1. Loads recorded demonstration from an unindexed MCAP dataset file (e.g. data/episodes/...).
2. Phase 1 (JTC Smooth Homing): Uses the Planner Agent (JointTrajectoryController) to smoothly
   interpolate and drive the robot from its current live pose to the episode's initial configuration
   with zero velocity/acceleration shock (eliminating start-up jitter).
3. Phase 2 (Online Replay): Hands over control to the Policy Agent (JointSpaceImpedanceController
   & ForwardCommandController) to replay the multi-modal trajectory at its exact recorded 1:1 native rate.

Usage:
  # Replay default recorded episode:
  python examples/13_replay_episode.py --profile fr3_pika_single_arm.yaml

  # Replay specific episode file:
  python examples/13_replay_episode.py --profile fr3_pika_single_arm.yaml \
      --mcap-file data/episodes/fr3_pika_policy/episode_000000/episode_000000.mcap
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

import rmi
from motion_planner_core.contracts import PlanPoint, PlanResult


class ProgressBar:
    """ASCII progress bar with live throughput stats."""

    def __init__(self, total_steps: int, prefix: str = "Replaying", width: int = 35) -> None:
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
        sys.stdout.write(msg)
        sys.stdout.flush()

    def finish(self) -> None:
        self.update(self.total_steps)
        print()


def load_mcap_trajectory(
    mcap_path: Path,
    arm_joint_names: tuple[str, ...],
    gripper_joint_name: str | None = None,
) -> tuple[list[list[float]], list[list[float]], float]:
    """Parse unindexed MCAP file and extract Action references (preferred) or feedback joint states."""
    from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory
    from std_msgs.msg import Float64MultiArray

    if not mcap_path.is_file():
        raise FileNotFoundError(f"MCAP file not found: {mcap_path}")

    # Action reference streams (high-level policy commands)
    arm_action_traj: list[list[float]] = []
    arm_action_ts: list[int] = []
    grip_action_traj: list[list[float]] = []
    grip_action_ts: list[int] = []

    # Sensor feedback streams (100Hz encoder states)
    arm_state_traj: list[list[float]] = []
    arm_state_ts: list[int] = []
    grip_state_traj: list[list[float]] = []

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(
            topics=[
                "/execution/arm/joint_reference",
                "/execution/end_effector/joint_reference",
                "/joint_states",
            ]
        ):
            if channel.topic == "/execution/arm/joint_reference":
                try:
                    jt = deserialize_message(message.data, RosJointTrajectory)
                    if jt.points:
                        pos = list(jt.points[-1].positions)
                        if len(pos) >= len(arm_joint_names):
                            arm_action_traj.append(pos[: len(arm_joint_names)])
                            arm_action_ts.append(message.log_time)
                except Exception:
                    pass

            elif channel.topic == "/execution/end_effector/joint_reference":
                try:
                    arr = deserialize_message(message.data, Float64MultiArray)
                    if arr.data:
                        grip_action_traj.append([float(arr.data[0])])
                        grip_action_ts.append(message.log_time)
                except Exception:
                    pass

            elif channel.topic == "/joint_states":
                js = deserialize_message(message.data, JointState)
                name_to_pos = dict(zip(js.name, js.position))
                arm_q = [name_to_pos[j] for j in arm_joint_names if j in name_to_pos]
                if len(arm_q) == len(arm_joint_names):
                    arm_state_traj.append(arm_q)
                    arm_state_ts.append(message.log_time)
                    if gripper_joint_name and gripper_joint_name in name_to_pos:
                        grip_state_traj.append([name_to_pos[gripper_joint_name]])
                    else:
                        grip_state_traj.append([])

    # Prefer high-level Action command references over raw sensor feedback
    if len(arm_action_traj) > 0:
        print(f"  [✓] Loaded {len(arm_action_traj)} recorded Action references (/execution/arm/joint_reference)")
        arm_traj = arm_action_traj
        grip_traj = grip_action_traj if len(grip_action_traj) > 0 else grip_state_traj
        timestamps_ns = arm_action_ts
    else:
        print(f"  [!] No arm action commands found in dataset; falling back to {len(arm_state_traj)} /joint_states feedback frames")
        arm_traj = arm_state_traj
        grip_traj = grip_state_traj
        timestamps_ns = arm_state_ts

    # Compute native rate from recorded duration
    native_hz = 30.0
    if len(timestamps_ns) >= 2:
        duration_s = (timestamps_ns[-1] - timestamps_ns[0]) * 1e-9
        if duration_s > 0.01:
            native_hz = round((len(timestamps_ns) - 1) / duration_s, 1)

    return arm_traj, grip_traj, native_hz


def generate_quintic_transition(
    joint_names: list[str],
    q_start: list[float],
    q_goal: list[float],
    duration_s: float = 4.0,
    steps: int = 50,
) -> PlanResult:
    """Generate a smooth quintic polynomial trajectory from q_start to q_goal.

    Position profile: s(t) = 10*t^3 - 15*t^4 + 6*t^5  (zero pos/vel/acc shock)
    """
    points = []
    steps = max(steps, 10)
    for i in range(steps + 1):
        s = i / steps
        # Quintic polynomial interpolation factor
        h = 10 * (s**3) - 15 * (s**4) + 6 * (s**5)
        # Velocity scaling factor
        vel_scale = (30 * (s**2) - 60 * (s**3) + 30 * (s**4)) / max(duration_s, 0.1)

        pos = [q_start[j] + h * (q_goal[j] - q_start[j]) for j in range(len(q_start))]
        vel = [vel_scale * (q_goal[j] - q_start[j]) for j in range(len(q_start))]
        t = s * duration_s
        points.append(PlanPoint(positions=pos, velocities=vel, time_from_start_s=t))

    return PlanResult(valid=True, joint_names=joint_names, points=points)


def main() -> None:
    parser = argparse.ArgumentParser(description="Unified Episode Replay Agent with JTC Pre-Alignment")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile name",
    )
    parser.add_argument(
        "--mcap-file",
        type=str,
        default="data/episodes/fr3_pika_policy/episode_000000/episode_000000.mcap",
        help="Path to unindexed MCAP episode file",
    )
    parser.add_argument(
        "--homing-duration",
        type=float,
        default=5.0,
        help="Duration in seconds for JTC initial pose alignment",
    )
    args = parser.parse_args()

    mcap_path = Path(args.mcap_file)
    if not mcap_path.is_file():
        # Fallback to search under workspace
        candidates = list(Path("data/episodes").glob("**/episode_*.mcap"))
        if candidates:
            mcap_path = candidates[0]
            print(f"[!] Target MCAP not found at {args.mcap_file}, using found candidate: {mcap_path}")
        else:
            print(f"[X] Error: No episode MCAP file found at {args.mcap_file}")
            sys.exit(1)

    print("\n=======================================================")
    print("  RMI Demo 13: Episode Dataset Replay Agent")
    print(f"  Profile:    {args.profile}")
    print(f"  MCAP File:  {mcap_path}")
    print("=======================================================\n")

    # 1. Initialize RMI Context & Introspect Hardware
    print("[1/4] Initializing RMI Context & Resolving Embodiment Topology...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot

    arm_part = "arm" if "arm" in ctx.profile.parts else next(iter(ctx.profile.parts.keys()))
    arm_joints = tuple(ctx.profile.parts[arm_part].joint_names)
    gripper_part = "end_effector" if "end_effector" in ctx.profile.parts else None
    gripper_joint = (
        ctx.profile.parts[gripper_part].joint_names[0]
        if gripper_part and ctx.profile.parts[gripper_part].joint_names
        else None
    )

    print(f"  [✓] Arm Part:     {arm_part!r} ({len(arm_joints)} joints: {arm_joints})")
    print(f"  [✓] Gripper Part: {gripper_part!r} (Joint: {gripper_joint!r})")

    # 2. Parse Unindexed Episode MCAP
    print(f"\n[2/4] Loading Recorded Trajectory from {mcap_path.name}...")
    t0 = time.time()
    arm_traj, grip_traj, native_hz = load_mcap_trajectory(mcap_path, arm_joints, gripper_joint)
    num_frames = len(arm_traj)
    if num_frames == 0:
        print("[X] Error: No valid joint frames extracted from MCAP file.")
        ctx.close()
        sys.exit(1)

    load_duration = time.time() - t0
    total_duration_s = num_frames / native_hz
    print(f"  [✓] Parsed {num_frames} frames in {load_duration:.2f}s (Native Rate: {native_hz} Hz, Duration: {total_duration_s:.1f}s)")
    start_arm_q = arm_traj[0]
    print(f"  Start Arm Pose: {[round(x, 4) for x in start_arm_q]}")

    # 3. Phase 1: JTC Smooth Transition to Initial Pose (Eliminate Startup Jitter)
    print("\n[3/4] Phase 1: Smooth JTC Homing to Episode Start Pose...")
    current_obs = robot.get_observation()
    current_arm_q = list(current_obs.joint_positions[: len(arm_joints)])
    max_discrepancy = max(abs(c - s) for c, s in zip(current_arm_q, start_arm_q))
    print(f"  Live Arm Pose:  {[round(x, 4) for x in current_arm_q]}")
    print(f"  Max Joint Delta: {max_discrepancy:.4f} rad")

    if max_discrepancy > 0.005:
        print(f"  -> Arm is not at start pose. Executing smooth JTC alignment ({args.homing_duration}s)...")
        homing_plan = generate_quintic_transition(
            list(arm_joints),
            current_arm_q,
            start_arm_q,
            duration_s=args.homing_duration,
        )

        planner_agent = ctx.make_agent("Planner")
        with planner_agent.run(robot) as planner_session:
            execution = planner_session.execute(arm_part, homing_plan)
            execution.wait(timeout=args.homing_duration + 5.0)
            if execution.done and not execution.canceled:
                print("  [✓] JTC Homing complete! Arm settled smoothly at episode start pose.")
            else:
                print(f"  [!] Homing status: {execution.state.name}")
    else:
        print("  [✓] Arm is already within 0.005 rad of start pose. Skipping homing.")

    # 4. Phase 2: High-Rate Policy Replay Loop
    print(f"\n[4/4] Phase 2: Activating JSIC Controller & Replaying ({num_frames} steps at {native_hz} Hz)...")
    policy_agent = ctx.make_agent("Policy", frequency=native_hz)

    try:
        with policy_agent.run(robot) as session:
            print(f"  [Replay Session Active] Provider = Policy | Generation = {session.generation_for(arm_part)}")
            pbar = ProgressBar(num_frames, prefix="Replaying Episode")

            # 1:1 Pure replay directly starting from frame 0
            for step in range(num_frames):
                arm_q = arm_traj[step]
                session.act(rmi.Action(part=arm_part, command="joint_reference", value=arm_q))

                if gripper_part and step < len(grip_traj) and grip_traj[step]:
                    session.act(
                        rmi.Action(
                            part=gripper_part,
                            command="joint_reference",
                            value=grip_traj[step],
                        )
                    )

                pbar.update(step + 1)
                session.wait()

            pbar.finish()
            print("\n[✓] Episode trajectory replay completed successfully!")

    except KeyboardInterrupt:
        print("\n[!] Replay interrupted by user (Ctrl+C). Holding safe position.")

    finally:
        ctx.close()


if __name__ == "__main__":
    main()
