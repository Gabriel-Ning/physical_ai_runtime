#!/usr/bin/env python3
# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""apps/record.py: Production Multi-Modal Robot Dataset Recorder Application.

A lightweight, high-performance dataset recording client built entirely on the
Robot Middleware Interface (RMI) SDK and distributed C++ Episode Recorder.

Usage:
  # Record dataset using profile defaults:
  pixi run -e runtime python apps/record.py --profile piper_bimanual.yaml

  # Record specific task with custom episode count:
  pixi run -e runtime python apps/record.py --profile piper_bimanual.yaml \
    --task "cup_sorting" --episodes 15
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import threading
import time
import uuid
from collections import Counter
from contextlib import ExitStack
from dataclasses import fields
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import rmi
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

CAMERA_STALL_TIMEOUT_SEC = 1.0
RECORD_CAMERA_TOPICS = {
    "orbbec": "/observation/orbbec/color/image_raw",
    "left_wrist_cam": "/observation/left_hand_realsense/color/image_raw",
    "right_wrist_cam": "/observation/right_hand_realsense/color/image_raw",
}


class CameraFrameGuard:
    """Keep camera subscriptions alive and report missing or stalled streams."""

    def __init__(self, node: Any, cameras_cfg: dict[str, Any]) -> None:
        self._node = node
        self._cameras_cfg = cameras_cfg
        self._last_frame_s: dict[str, float] = {}
        self._frames_info: dict[str, str] = {}
        self._subscriptions: list[Any] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for cam_id, cfg in self._cameras_cfg.items():
            topic = cfg.get("ros_topic")
            if not topic:
                continue
            encoding = str(cfg.get("encoding", "rgb8")).lower()
            msg_type = (
                CompressedImage if encoding in ("jpeg", "jpg", "mjpeg") else Image
            )
            self._subscriptions.append(
                self._node.create_subscription(
                    msg_type, topic, self._make_cb(cam_id), qos
                )
            )

    def close(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()

    def require_initial_frames(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and self._missing():
            time.sleep(0.05)
        self._print_status()
        missing = self._missing()
        if missing:
            raise RuntimeError(
                "相机首帧门禁失败；不会启动录制。未收到有效图像帧: "
                + ", ".join(missing)
            )

    def stalled(self, timeout_sec: float) -> list[str]:
        now = time.monotonic()
        with self._lock:
            ages = {
                cam_id: now - self._last_frame_s.get(cam_id, 0.0)
                for cam_id in self._cameras_cfg
            }
        return [
            self._describe(cam_id)
            for cam_id, age in ages.items()
            if age > timeout_sec
        ]

    def _make_cb(self, cam_id: str):
        def _cb(msg: Any) -> None:
            w, h = getattr(msg, "width", 0), getattr(msg, "height", 0)
            enc = getattr(msg, "encoding", None) or getattr(msg, "format", "raw")
            nbytes = len(getattr(msg, "data", b""))
            if not nbytes:
                return
            with self._lock:
                self._last_frame_s[cam_id] = time.monotonic()
                self._frames_info[cam_id] = (
                    f"{w}x{h}, {enc}" if w and h else f"{enc}, {nbytes} B"
                )

        return _cb

    def _missing(self) -> list[str]:
        with self._lock:
            return [
                self._describe(cam_id)
                for cam_id in self._cameras_cfg
                if cam_id not in self._last_frame_s
            ]

    def _describe(self, cam_id: str) -> str:
        topic = self._cameras_cfg[cam_id].get("ros_topic", "<missing ros_topic>")
        return f"{cam_id} ({topic})"

    def _print_status(self) -> None:
        print("\n  📸 Perception Camera Stream Verification:")
        with self._lock:
            ready = set(self._last_frame_s)
            frames_info = dict(self._frames_info)
        for cam_id in self._cameras_cfg:
            status = "STREAMING (Ready)" if cam_id in ready else "NO FRAMES"
            icon = "[✓]" if cam_id in ready else "[!]"
            print(
                f"    {icon} {self._describe(cam_id)} "
                f"({frames_info.get(cam_id, 'no valid frame')}) -> {status}"
            )


def set_teleop_preempt(
    node: Any, teleoperators: dict[str, Any], preempt_active: bool
) -> None:
    """Toggle hardware preemption (0-G Float vs Fallback mode) on teleoperation devices."""
    for name, cfg in teleoperators.items():
        srv_name = cfg.get("preempt_service")
        if not srv_name:
            continue
        client = node.create_client(SetBool, srv_name)
        if client.wait_for_service(timeout_sec=0.3):
            future = client.call_async(SetBool.Request(data=preempt_active))
            t_end = time.monotonic() + 0.5
            while not future.done() and time.monotonic() < t_end:
                time.sleep(0.01)
            if future.done() and future.result().success:
                mode_label = (
                    "ACTIVE (0-G Float)"
                    if preempt_active
                    else "RELEASED (Shadow/Passive)"
                )
                print(f"  [✓] {name} Preempt {mode_label}: {future.result().message}")


def verify_cameras(
    node: Any, cameras_cfg: dict[str, Any], timeout_sec: float = 3.0
) -> None:
    """Require one valid frame from every camera declared by the profile."""
    if not cameras_cfg:
        return
    guard = CameraFrameGuard(node, cameras_cfg)
    guard.start()
    try:
        guard.require_initial_frames(timeout_sec)
    finally:
        guard.close()


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
    for cfg in teleoperators.values():
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
        "--max-duration-s",
        type=float,
        default=None,
        help="Override profile recorder.max_duration_s for each episode",
    )
    parser.add_argument(
        "--operator",
        type=str,
        default="operator",
        help="Operator ID / annotator identifier",
    )
    parser.add_argument(
        "--dataset-version",
        default=None,
        help="Dataset version stored in every episode manifest (default: profile)",
    )
    parser.add_argument(
        "--config-id",
        default=None,
        help="Frozen experiment config ID stored in every episode manifest",
    )
    parser.add_argument(
        "--camera-setup",
        default=None,
        help="Frozen camera setup ID stored in every episode manifest",
    )
    args = parser.parse_args()
    if args.max_duration_s is not None and args.max_duration_s <= 0.0:
        parser.error("--max-duration-s 必须大于 0")
    return args


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


def _trial_id(task: str) -> str:
    """Return a sortable, globally unique ID for one attempted demonstration."""
    timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    return f"{task}_{timestamp}_{uuid.uuid4().hex[:8]}"


def _write_demonstration_label(
    episode_dir: Path,
    trial_id: str,
    operator: str,
    quality: str,
    stable_success: bool,
) -> None:
    """Persist the operator's BC quality decision next to the finalized MCAP."""
    payload = {
        "schema_version": "1.0",
        "trial_id": trial_id,
        "demonstration_quality": quality,
        "stable_corner_grasp_success": stable_success,
        "label_source": "operator_at_collection",
        "human_verified": True,
        "operator": operator,
        "labelled_at_utc": datetime.now(UTC).isoformat(),
    }
    (episode_dir / "demonstration_label.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _verify_finalized_episode(
    episode_dir: Path, required_camera_topics: set[str]
) -> dict[str, int]:
    """Verify finalized files and read the MCAP through EOF before acceptance."""
    checksum_file = episode_dir / "checksums.sha256"
    if not checksum_file.is_file():
        raise RuntimeError(f"finalized episode 缺少 checksums.sha256: {episode_dir}")
    for raw_line in checksum_file.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip():
            continue
        parts = raw_line.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise RuntimeError(f"checksums.sha256 格式错误: {checksum_file}")
        expected, filename = parts
        relative = Path(filename.lstrip("*"))
        if relative.is_absolute() or ".." in relative.parts:
            raise RuntimeError(f"checksums.sha256 包含不安全路径: {relative}")
        target = episode_dir / relative
        if not target.is_file():
            raise RuntimeError(f"checksum 文件不存在: {target}")
        digest = hashlib.sha256()
        with target.open("rb") as stream:
            for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(chunk)
        if digest.hexdigest() != expected.lower():
            raise RuntimeError(f"checksum mismatch: {target.name}")

    mcaps = sorted(episode_dir.glob("*.mcap"))
    if len(mcaps) != 1:
        raise RuntimeError(f"finalized episode 应恰有一个 MCAP: {episode_dir}")
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(episode_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    actual_counts: Counter[str] = Counter()
    try:
        while reader.has_next():
            topic, _serialized, _timestamp_ns = reader.read_next()
            actual_counts[topic] += 1
    except Exception as exc:
        raise RuntimeError(f"MCAP 未能完整读取到 EOF: {mcaps[0]}") from exc

    metadata_path = episode_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeError(f"finalized episode 缺少 metadata.yaml: {episode_dir}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    expected_counts = {
        item.get("topic_metadata", {}).get("name"): int(item.get("message_count", 0))
        for item in metadata.get("rosbag2_bagfile_information", {}).get(
            "topics_with_message_count", []
        )
    }
    problems = []
    for topic in sorted(required_camera_topics):
        actual = actual_counts[topic]
        expected = expected_counts.get(topic)
        if actual <= 0 or expected is None or actual != expected:
            problems.append(f"{topic}: actual={actual}, metadata={expected}")
    if problems:
        raise RuntimeError("相机 topic 计数校验失败: " + "; ".join(problems))
    return dict(actual_counts)


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


def _select_recording_cameras(cameras_cfg: dict[str, Any]) -> dict[str, Any]:
    """Return the fixed Orbbec-plus-wrists recording camera contract."""
    selected: dict[str, Any] = {}
    problems: list[str] = []
    for camera_id, expected_topic in RECORD_CAMERA_TOPICS.items():
        config = cameras_cfg.get(camera_id)
        actual_topic = config.get("ros_topic") if isinstance(config, dict) else None
        if actual_topic != expected_topic:
            problems.append(
                f"{camera_id}: expected={expected_topic}, actual={actual_topic}"
            )
            continue
        selected[camera_id] = dict(config)
    if problems:
        raise RuntimeError(
            "录制 profile 必须提供固定的奥比中光和左右腕部相机: "
            + "; ".join(problems)
        )
    return selected


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
    try:
        ctx.wait_until_ready(timeout=6.0)
    except BaseException:
        # from_profile() owns an executor thread.  If readiness fails before
        # the main recording try/finally is entered, leaving that thread alive
        # makes the interpreter abort during shutdown.
        ctx.close()
        raise

    # 2. Extract Profile Parameters
    rec_cfg = ctx.profile.raw_data.get("recorder", {})
    task_name = _normalize_task_name(
        args.task or rec_cfg.get("task", "bimanual_manipulation")
    )
    target_episodes = (
        args.episodes if args.episodes is not None else rec_cfg.get("episodes", 10)
    )
    rate_hz = float(rec_cfg.get("rate_hz", 50.0))
    max_duration_s = float(
        args.max_duration_s
        if args.max_duration_s is not None
        else rec_cfg.get("max_duration_s", 60.0)
    )
    homing_duration_s = float(rec_cfg.get("homing_duration_s", 2.5))
    home_pose = list(rec_cfg.get("home_pose", [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]))
    dataset_version = str(
        args.dataset_version or rec_cfg.get("dataset_version", "unversioned")
    )
    config_id = str(args.config_id or rec_cfg.get("config_id", "unfrozen"))
    camera_setup = str(
        args.camera_setup or rec_cfg.get("camera_setup", "unspecified")
    )

    teleoperators = ctx.profile.raw_data.get("teleoperators", {})
    cameras_cfg = _select_recording_cameras(
        ctx.profile.raw_data.get("sensors", {}).get("cameras", {})
    )
    rec_cfg["max_duration_s"] = max_duration_s

    print("=" * 72)
    print("  RMI Production Multi-Modal Dataset Recorder Client")
    print(f"  Embodiment Profile : {args.profile}")
    print("  Camera Input       : Orbbec + left/right wrist")
    print(f"  Task Description   : '{task_name}'")
    print(f"  Target Episodes    : {target_episodes}")
    print(f"  Dataset Version    : {dataset_version}")
    print(f"  Config ID          : {config_id}")
    print(f"  Camera Setup       : {camera_setup}")
    print(f"  Recording Rate     : {rate_hz:.1f} Hz")
    print(
        f"  Dataset Directory  : {Path(rec_cfg.get('root_dir', 'data/episodes')) / task_name}"
    )
    print("=" * 72)

    # 3. Perception Warmup and continuous liveness monitor
    camera_guard: CameraFrameGuard | None = None
    if cameras_cfg:
        camera_guard = CameraFrameGuard(ctx.node, cameras_cfg)
        camera_guard.start()
        try:
            camera_guard.require_initial_frames(timeout_sec=3.0)
        except Exception:
            camera_guard.close()
            ctx.close()
            raise

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

            trial_id = _trial_id(task_name)
            metadata = {
                "trial_id": trial_id,
                "task": task_name,
                "operator": args.operator,
                "profile": args.profile,
                "rate_hz": rate_hz,
                "episode_index": current_ep_idx,
                "dataset_version": dataset_version,
                "config_id": config_id,
                "camera_setup": camera_setup,
            }
            print(f"  Trial ID           : {trial_id}")

            stop_event = threading.Event()
            timed_capture = args.max_duration_s is not None
            stop_thread = None
            if not timed_capture:
                stop_thread = threading.Thread(
                    target=lambda event=stop_event: (input(), event.set()), daemon=True
                )

            last_recorded_path = ""
            start_time = time.monotonic()
            dt = 1.0 / rate_hz
            camera_stall: list[str] = []

            # STEP 3: Lockstep Recording & Active Teleop Streaming
            with (
                recorder.episode(task=task_name, metadata=metadata) as ep,
                ExitStack() as stack,
            ):
                for cfg in teleoperators.values():
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
                set_teleop_preempt(ctx.node, teleoperators, True)

                print(
                    "\n  🔴 RECORDING ACTIVE! Manipulate master arms to demonstrate task."
                )
                if timed_capture:
                    print(f"  >> Automatic stop after {max_duration_s:.1f} seconds.\n")
                else:
                    print("  >> Press [ENTER] in console when episode is COMPLETE.\n")
                    assert stop_thread is not None
                    stop_thread.start()

                step_count = 0
                while not stop_event.is_set():
                    t_loop = time.monotonic()
                    step_count += 1
                    elapsed = time.monotonic() - start_time

                    if camera_guard is not None:
                        camera_stall = camera_guard.stalled(
                            CAMERA_STALL_TIMEOUT_SEC
                        )
                        if camera_stall:
                            print(
                                "\n  [!] Camera stream stalled; stopping and discarding "
                                "this episode: "
                                + ", ".join(camera_stall)
                            )
                            ep.discard()
                            break

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

            if camera_stall:
                print(
                    "  [!] Recording session stopped. Restore all camera streams "
                    "before starting a new session."
                )
                break

            episode_dir = _finalized_episode_directory(ep)
            last_recorded_path = str(episode_dir)

            required_camera_topics = {
                str(config["ros_topic"])
                for config in cameras_cfg.values()
                if config.get("ros_topic")
            }
            verified_counts = _verify_finalized_episode(
                episode_dir, required_camera_topics
            )
            print(
                "  [✓] Episode integrity verified: "
                f"{len(verified_counts)} topics, MCAP readable to EOF."
            )

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
                        "\n  >> Episode Action: [S]ave VERIFIED SUCCESS | "
                        "[D]iscard/label failure outside initial BC | [R]eplay | [Q]uit : "
                    )
                    .strip()
                    .lower()
                )

                if choice in {"s", "save"}:
                    _write_demonstration_label(
                        Path(last_recorded_path),
                        trial_id,
                        args.operator,
                        "accepted_success",
                        True,
                    )
                    saved_episodes.append(last_recorded_path)
                    print(
                        f"  [✓] Episode {current_ep_idx} COMMITTED & SAVED to: {last_recorded_path}"
                    )
                    current_ep_idx += 1
                    homing_thread.join()
                    break
                elif choice in {"d", "discard"}:
                    print(
                        f"  [!] Episode {current_ep_idx} rejected from initial BC; "
                        "preserving it in the failure set..."
                    )
                    if last_recorded_path:
                        ep_dir = Path(last_recorded_path)
                        if ep_dir.is_dir() and "episode_" in ep_dir.name:
                            _write_demonstration_label(
                                ep_dir,
                                trial_id,
                                args.operator,
                                "rejected_failure",
                                False,
                            )
                            failure_root = ep_dir.parent.with_name(
                                f"{ep_dir.parent.name}_failures"
                            )
                            failure_root.mkdir(parents=True, exist_ok=True)
                            destination = failure_root / ep_dir.name
                            if destination.exists():
                                destination = failure_root / (
                                    f"{ep_dir.name}_{uuid.uuid4().hex[:8]}"
                                )
                            shutil.move(str(ep_dir), str(destination))
                            print(f"  [i] Failure episode preserved at: {destination}")
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
        if camera_guard is not None:
            camera_guard.close()
        set_teleop_preempt(ctx.node, teleoperators, False)
        ctx.close()
        print("\n" + "=" * 72)
        print(
            f"  RECORDING SESSION COMPLETE. Total Saved Episodes: {len(saved_episodes)}"
        )
        print("=" * 72)


if __name__ == "__main__":
    main()
