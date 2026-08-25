#!/usr/bin/env python3
# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""apps/record.py: Production Multi-Modal Robot Dataset Recorder Application.

A lightweight, high-performance dataset recording client built entirely on the
Robot Middleware Interface (RMI) SDK and distributed C++ Episode Recorder.

Usage:
  # Record dataset using profile defaults:
  pixi run record --profile piper_bimanual.yaml

  # Record specific task with custom episode count:
  pixi run record --profile piper_bimanual.yaml --task "cup_sorting" --episodes 15
"""

from __future__ import annotations

import argparse
import os
import shutil
import threading
import time
from contextlib import ExitStack
from dataclasses import fields
from pathlib import Path
from typing import Any

import rmi
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


def set_teleop_preempt(
    node: Any, teleoperators: dict[str, Any], preempt_active: bool
) -> bool:
    """Set every leader mode, requiring all transitions to succeed."""
    if not teleoperators:
        return True
    try:
        context = getattr(node, "context", None)
        if context is None or not context.ok():
            return False
    except Exception:  # noqa: BLE001 - RCL context may already be torn down.
        return False
    failures: list[str] = []
    for name, cfg in teleoperators.items():
        srv_name = cfg.get("preempt_service")
        if not srv_name:
            failures.append(f"{name}: missing preempt_service")
            continue
        client = None
        try:
            client = node.create_client(SetBool, srv_name)
            if not client.wait_for_service(timeout_sec=3.0):
                failures.append(f"{name}: {srv_name} unavailable")
                continue
            future = client.call_async(SetBool.Request(data=preempt_active))
            t_end = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < t_end:
                time.sleep(0.01)
            if not future.done():
                failures.append(f"{name}: {srv_name} timed out")
                continue
            response = future.result()
            if response is None or not response.success:
                failures.append(
                    f"{name}: {getattr(response, 'message', 'no response')}"
                )
                continue
            mode_label = (
                "ACTIVE (0-G Float)" if preempt_active else "RELEASED (Shadow/Passive)"
            )
            print(f"  [✓] {name} Preempt {mode_label}: {response.message}")
        except Exception as exc:  # noqa: BLE001 - service failures are aggregated.
            failures.append(f"{name}: {exc.__class__.__name__}")
        finally:
            if client is not None:
                try:
                    node.destroy_client(client)
                except Exception:  # noqa: BLE001, S110 - best-effort during shutdown.
                    pass
    if failures:
        print("  [!] Leader mode transition failed: " + "; ".join(failures))
        return False
    return True


def verify_cameras(
    node: Any, cameras_cfg: dict[str, Any], timeout_sec: float = 3.0
) -> None:
    """Verify all camera streams defined in Profile are actively publishing frames."""
    if not cameras_cfg:
        return

    results: dict[str, bool] = {cam_id: False for cam_id in cameras_cfg}
    frames_info: dict[str, str] = {}
    subscriptions = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def _make_cb(c_id: str):
        def _cb(msg: Any):
            if not results[c_id]:
                results[c_id] = True
                w, h = getattr(msg, "width", 0), getattr(msg, "height", 0)
                enc = getattr(msg, "encoding", None) or getattr(msg, "format", "raw")
                nbytes = len(getattr(msg, "data", b""))
                frames_info[c_id] = (
                    f"{w}x{h}, {enc}" if w and h else f"{enc}, {nbytes} B"
                )

        return _cb

    for cam_id, cfg in cameras_cfg.items():
        topic = cfg.get("ros_topic")
        if topic:
            encoding = str(cfg.get("encoding", "rgb8")).lower()
            msg_type = (
                CompressedImage if encoding in ("jpeg", "jpg", "mjpeg") else Image
            )
            subscriptions.append(
                node.create_subscription(msg_type, topic, _make_cb(cam_id), qos)
            )

    t_end = time.monotonic() + timeout_sec
    while time.monotonic() < t_end and not all(results.values()):
        time.sleep(0.1)

    for sub in subscriptions:
        node.destroy_subscription(sub)

    print("\n  📸 Perception Camera Stream Verification:")
    for cam_id, cfg in cameras_cfg.items():
        topic = cfg.get("ros_topic", "")
        status = (
            "STREAMING (Ready)"
            if results.get(cam_id)
            else "NO FRAMES (Check workstation_stack)"
        )
        icon = "[✓]" if results.get(cam_id) else "[!]"
        detail = frames_info.get(cam_id, "live stream")
        print(f"    {icon} {cam_id:<16}: {topic} ({detail}) -> {status}")


def smooth_homing(
    ctx: rmi.Context,
    robot: rmi.Robot,
    home_pose: list[float],
    teleoperators: dict[str, Any],
    duration_s: float = 2.5,
    rate_hz: float = 50.0,
) -> None:
    """Smooth quintic spline staging motion to home poses."""
    obs = robot.get_observation()
    name_to_pos = dict(zip(obs.joint_names, obs.joint_positions))

    # Collect arm and gripper start configurations
    parts_to_home = {}
    for name, cfg in teleoperators.items():
        arm_p, grip_p = cfg["arm_part"], cfg["gripper_part"]
        arm_spec, grip_spec = (
            ctx.profile.parts.get(arm_p),
            ctx.profile.parts.get(grip_p),
        )

        if arm_spec and all(j in name_to_pos for j in arm_spec.joint_names):
            parts_to_home[arm_p] = (
                [name_to_pos[j] for j in arm_spec.joint_names],
                list(home_pose),
            )
        if grip_spec and all(j in name_to_pos for j in grip_spec.joint_names):
            parts_to_home[grip_p] = (
                [name_to_pos[j] for j in grip_spec.joint_names],
                [0.020],
            )

    steps = int(max(duration_s * rate_hz, 10))
    dt = duration_s / steps

    agent = ctx.make_agent("Policy", frequency=rate_hz)
    with agent.run(robot) as session:
        t_start = time.monotonic()
        for i in range(steps + 1):
            s = i / steps
            h = 10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5)

            for part_name, (q_start, q_goal) in parts_to_home.items():
                interp = [
                    q_start[j] + h * (q_goal[j] - q_start[j])
                    for j in range(len(q_goal))
                ]
                session.act(
                    rmi.Action(part=part_name, command="joint_reference", value=interp)
                )

            elapsed = time.monotonic() - t_start
            target_t = (i + 1) * dt
            if target_t > elapsed:
                time.sleep(target_t - elapsed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Production Multi-Modal Dataset Recorder Client",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="piper_bimanual.yaml",
        help="Embodiment profile YAML filename or path",
    )
    parser.add_argument(
        "--task",
        type=str,
        default=None,
        help="Task name (default: from profile.recorder)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help="Target number of successful episodes (default: from profile.recorder)",
    )
    parser.add_argument(
        "--operator",
        type=str,
        default="operator",
        help="Operator ID / annotator identifier",
    )
    parser.add_argument(
        "--skip-camera-check",
        action="store_true",
        help="Skip camera perception stream warmup check",
    )
    return parser.parse_args()


def _normalize_task_name(value: str) -> str:
    """Return one task label that is also safe as a dataset directory name."""
    task = value.strip()
    if not task:
        raise ValueError("task name must not be empty")
    if task in {".", ".."} or "/" in task or "\\" in task or "\0" in task:
        raise ValueError(
            "task name must be one directory component (no '/', '\\', '.' or '..')"
        )
    return task


def _task_recorder_config(
    recorder_values: dict[str, Any], task: str, operator: str
) -> Any:
    """Build a recorder config whose directory and episode task stay aligned."""
    from episode_recorder import RecorderConfig

    valid_names = {item.name for item in fields(RecorderConfig)}
    values = {
        key: value for key, value in recorder_values.items() if key in valid_names
    }
    values.update(
        {
            "experiment_name": task,
            "task": task,
            "operator_name": operator,
            "max_episode_duration": float(recorder_values.get("max_duration_s", 0.0)),
        }
    )
    return RecorderConfig(**values)


def _finalized_episode_directory(scope: Any) -> Path:
    status = getattr(scope, "final_status", None)
    episode_path = getattr(status, "episode_path", "") if status is not None else ""
    if not episode_path:
        raise RuntimeError("recorder finalized episode without an episode_path")
    path = Path(episode_path)
    return path if path.is_dir() else path.parent


def _relay(session: rmi.Session, part: str):
    def _callback(msg: JointTrajectory) -> None:
        if session.active_for(part):
            session.act(rmi.Action(part=part, command="joint_reference", value=msg))

    return _callback


def main() -> None:
    args = parse_args()

    # 1. Connect Context strictly from Profile
    print("[1/4] Connecting to robot embodiment runtime...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=6.0)

    # 2. Extract Profile Parameters
    rec_cfg = ctx.profile.raw_data.get("recorder", {})
    task_name = _normalize_task_name(
        args.task or rec_cfg.get("task", "bimanual_manipulation")
    )
    target_episodes = (
        args.episodes if args.episodes is not None else rec_cfg.get("episodes", 10)
    )
    rate_hz = float(rec_cfg.get("rate_hz", 50.0))
    max_duration_s = float(rec_cfg.get("max_duration_s", 60.0))
    homing_duration_s = float(rec_cfg.get("homing_duration_s", 2.5))
    home_pose = list(rec_cfg.get("home_pose", [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]))

    teleoperators = ctx.profile.raw_data.get("teleoperators", {})
    cameras_cfg = ctx.profile.raw_data.get("sensors", {}).get("cameras", {})

    print("=" * 72)
    print("  RMI Production Multi-Modal Dataset Recorder Client")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Task Description   : '{task_name}'")
    print(f"  Target Episodes    : {target_episodes}")
    print(f"  Recording Rate     : {rate_hz:.1f} Hz")
    print(
        f"  Dataset Directory  : {Path(rec_cfg.get('root_dir', 'data/episodes')) / task_name}"
    )
    print("=" * 72)

    # 3. Perception Warmup
    if not args.skip_camera_check and cameras_cfg:
        verify_cameras(ctx.node, cameras_cfg, timeout_sec=3.0)

    # 4. Activate MCAP Recorder Backend
    recorder = ctx.make_recorder(
        config=_task_recorder_config(rec_cfg, task_name, args.operator),
        autostart=True,
    )
    recorder.activate()

    robot = ctx.robot
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    saved_episodes: list[str] = []

    try:
        current_ep_idx = 1
        while current_ep_idx <= target_episodes:
            print("\n" + "=" * 72)
            print(f"  [EPISODE {current_ep_idx}/{target_episodes}] Task: '{task_name}'")
            print("=" * 72)

            # STEP 1: Staging Reset (Leader mirrors Follower in Shadow mode)
            print("  >> Moving robot arm(s) smoothly to Staging Home Pose...")
            smooth_homing(
                ctx,
                robot,
                home_pose,
                teleoperators,
                duration_s=homing_duration_s,
                rate_hz=rate_hz,
            )
            print("  [✓] Staging Home Pose reached. Master & Slave aligned.")

            # STEP 2: Ready Gate (Simulating Preempt Engagement via Keyboard Enter)
            print("\n" + "-" * 72)
            input(
                f"  >> [READY] Press [ENTER] to ENGAGE Preempt & START recording Episode {current_ep_idx}... "
            )
            print("-" * 72)

            metadata = {
                "task": task_name,
                "operator": args.operator,
                "profile": args.profile,
                "rate_hz": rate_hz,
                "episode_index": current_ep_idx,
            }

            stop_event = threading.Event()
            stop_thread = threading.Thread(
                target=lambda: (input(), stop_event.set()), daemon=True
            )

            last_recorded_path = ""
            start_time = time.monotonic()
            dt = 1.0 / rate_hz

            # STEP 3: Lockstep Recording & Active Teleop Streaming
            with recorder.episode(task=task_name, metadata=metadata) as ep:
                with ExitStack() as stack:
                    for name, cfg in teleoperators.items():
                        agent = ctx.make_agent(cfg["target_agent"], frequency=rate_hz)
                        session = stack.enter_context(
                            agent.run(
                                robot,
                                parts=[cfg["arm_part"], cfg["gripper_part"]],
                                preempt=True,
                            )
                        )
                        arm_sub = ctx.node.create_subscription(
                            JointTrajectory,
                            cfg["arm_source"],
                            _relay(session, cfg["arm_part"]),
                            qos,
                        )
                        gripper_sub = ctx.node.create_subscription(
                            JointTrajectory,
                            cfg["gripper_source"],
                            _relay(session, cfg["gripper_part"]),
                            qos,
                        )
                        stack.callback(ctx.node.destroy_subscription, gripper_sub)
                        stack.callback(ctx.node.destroy_subscription, arm_sub)

                    # Engage 0-G float on physical teleop devices
                    if not set_teleop_preempt(ctx.node, teleoperators, True):
                        set_teleop_preempt(ctx.node, teleoperators, False)
                        raise RuntimeError(
                            "teleoperation was not engaged; every leader must confirm 0-G mode"
                        )

                    print(
                        "\n  🔴 RECORDING ACTIVE! Manipulate master arms to demonstrate task."
                    )
                    print("  >> Press [ENTER] in console when episode is COMPLETE.\n")
                    stop_thread.start()

                    step_count = 0
                    while not stop_event.is_set():
                        t_loop = time.monotonic()
                        step_count += 1
                        elapsed = time.monotonic() - start_time

                        if elapsed >= max_duration_s:
                            print(f"\n  [!] Max duration {max_duration_s}s reached.")
                            break

                        if step_count % int(max(1, rate_hz / 5)) == 0:
                            print(
                                f"    Recording: {elapsed:5.1f}s | {step_count:5d} frames captured  (Press [ENTER] to Finish)",
                                end="\r",
                                flush=True,
                            )

                        t_spent = time.monotonic() - t_loop
                        if t_spent < dt:
                            time.sleep(dt - t_spent)

                    # STEP 4: Release Preempt back to Shadow fallback mode
                    set_teleop_preempt(ctx.node, teleoperators, False)

            last_recorded_path = str(_finalized_episode_directory(ep))

            print(
                f"\n  [✓] Demonstration concluded ({elapsed:.1f}s, {step_count} frames)."
            )

            # STEP 5: Parallel Homing Reset & Quality Gate
            homing_thread = threading.Thread(
                target=smooth_homing,
                args=(ctx, robot, home_pose, teleoperators),
                kwargs={"duration_s": homing_duration_s, "rate_hz": rate_hz},
            )
            homing_thread.start()

            while True:
                choice = (
                    input(
                        "\n  >> Episode Action: [S]ave (Default / Enter) | [D]iscard & Retry | [R]eplay | [Q]uit : "
                    )
                    .strip()
                    .lower()
                )

                if choice in {"", "s", "save"}:
                    saved_episodes.append(last_recorded_path)
                    print(
                        f"  [✓] Episode {current_ep_idx} COMMITTED & SAVED to: {last_recorded_path}"
                    )
                    current_ep_idx += 1
                    homing_thread.join()
                    break
                elif choice in {"d", "discard"}:
                    print(
                        f"  [!] Episode {current_ep_idx} DISCARDED. Cleaning temporary files..."
                    )
                    if last_recorded_path:
                        ep_dir = Path(last_recorded_path)
                        if ep_dir.is_dir() and "episode_" in ep_dir.name:
                            shutil.rmtree(ep_dir, ignore_errors=True)
                    homing_thread.join()
                    break
                elif choice in {"r", "replay"}:
                    homing_thread.join()
                    print(
                        "\n  >> Replaying demonstration on robot (Leader mirrors in Shadow mode)..."
                    )
                    os.system(
                        f"python apps/replay.py --profile '{args.profile}' --mcap-file '{last_recorded_path}'"
                    )
                    print("  [✓] Replay inspection finished.")
                elif choice in {"q", "quit"}:
                    print("  [!] Stopping recording session.")
                    homing_thread.join()
                    current_ep_idx = target_episodes + 1
                    break

    except (KeyboardInterrupt, EOFError):
        print("\n\n  [!] Recording session interrupted by operator.")
    finally:
        set_teleop_preempt(ctx.node, teleoperators, False)
        ctx.close()
        print("\n" + "=" * 72)
        print(
            f"  RECORDING SESSION COMPLETE. Total Saved Episodes: {len(saved_episodes)}"
        )
        print("=" * 72)


if __name__ == "__main__":
    main()
