#!/usr/bin/env python3
# Copyright 2026 Physical AI Runtime contributors
# SPDX-License-Identifier: Apache-2.0
"""apps/record.py: Profile-driven multi-modal episode recorder.

Does not start RT or workstation. Bring those up first, then:

  python apps/record.py --profile <profile.yaml>

Profiles:
  piper_bimanual.yaml              MuJoCo default (head + two wrist cams from RT)
  site/piper_bimanual_real.yaml    Real cell (workstation Orbbec + RealSense)
  fr3_pika_single_arm.yaml / marvin_bimanual.yaml
                                   Same app; physical clutch owns engage

Piper has no device clutch. ENTER calls every workstation ``preempt_service``
in parallel (leaders publish clutch). ENTER again ends the episode.

MuJoCo smoke (local RT cameras; no real camera drivers):

  pixi run rt-piper backend:=mujoco
  pixi run workstation-piper use_sim_time:=true \\
    with_orbbec:=false with_realsense:=false with_leaders:=true
  python apps/record.py --profile piper_bimanual.yaml --use-sim-time

Real cell (RT already up on the RT host):

  pixi run workstation-piper with_leaders:=true
  python apps/record.py --profile site/piper_bimanual_real.yaml
"""

from __future__ import annotations

import argparse
import os
import select
import shutil
import sys
import threading
import time
from dataclasses import fields
from pathlib import Path
from typing import Any

import rmi
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool


def _profile_raw(profile: Any) -> dict[str, Any]:
    raw = getattr(profile, "raw_data", None)
    if raw is None and isinstance(profile, dict):
        raw = profile
    return raw if isinstance(raw, dict) else {}


def _workstation_share(profile: Any) -> Path | None:
    em = _profile_raw(profile).get("execution_manager_config")
    if not isinstance(em, dict):
        return None
    package = em.get("package")
    if not package:
        return None
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory(str(package)))
        if installed.is_dir():
            return installed
    except Exception:  # noqa: BLE001 - fall back to the source tree.
        pass
    repo = Path(__file__).resolve().parents[1]
    for package_xml in repo.glob("src/**/package.xml"):
        if f"<name>{package}</name>" not in package_xml.read_text(encoding="utf-8"):
            continue
        return package_xml.parent
    return None


def _part_for_joints(groups: dict[str, Any], joint_names: list[str]) -> str:
    wanted = list(joint_names)
    for name, group in groups.items():
        if isinstance(group, dict) and list(group.get("joint_names") or []) == wanted:
            return str(name)
    return ""


def _teleoperators_from_workstation(
    leaders: dict[str, Any], em: dict[str, Any]
) -> dict[str, Any]:
    groups = em.get("groups") or {}
    sources = em.get("sources") or {}
    devices: dict[str, Any] = {}
    for node_name, block in leaders.items():
        if not isinstance(block, dict):
            continue
        params = block.get("ros__parameters")
        if not isinstance(params, dict) or not params.get("joint_reference_topic"):
            continue
        clutch = params.get("clutch_topic")
        arm_part = _part_for_joints(groups, list(params.get("joint_names") or []))
        grip_name = params.get("follower_gripper_joint_name") or params.get(
            "gripper_joint_name"
        )
        gripper_part = _part_for_joints(groups, [grip_name] if grip_name else [])
        target_node = ""
        for source_name, source in sources.items():
            if not isinstance(source, dict):
                continue
            if source.get("activation_topic") != clutch:
                continue
            inputs = source.get("inputs") or {}
            arm_in = inputs.get(arm_part) if arm_part else None
            if (
                isinstance(arm_in, dict)
                and arm_in.get("command_contract") == "joint_reference"
                and gripper_part in inputs
            ):
                target_node = str(source_name)
                break
        if not arm_part or not gripper_part or not target_node:
            continue
        preempt = params.get("preempt_service")
        if not isinstance(preempt, str) or not preempt.strip():
            preempt = f"/{node_name}/preempt"
        devices[node_name] = {
            "preempt_service": preempt.strip(),
            "arm_source": params["joint_reference_topic"],
            "gripper_source": params["gripper_reference_topic"],
            "arm_part": arm_part,
            "gripper_part": gripper_part,
            "target_node": target_node,
        }
    return devices


def load_teleoperators(profile: Any) -> dict[str, Any]:
    """Leader devices from workstation ``config/teleop/*.yaml`` (profile package)."""
    share = _workstation_share(profile)
    em_ref = _profile_raw(profile).get("execution_manager_config")
    em_file = em_ref.get("file") if isinstance(em_ref, dict) else None
    if share is None or not em_file:
        return {}
    em_path = share / str(em_file)
    teleop_dir = share / "config" / "teleop"
    if not em_path.is_file() or not teleop_dir.is_dir():
        return {}
    em = yaml.safe_load(em_path.read_text(encoding="utf-8")) or {}
    if not isinstance(em, dict):
        return {}
    devices: dict[str, Any] = {}
    for path in sorted(teleop_dir.glob("*.yaml")):
        leaders = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if isinstance(leaders, dict):
            devices.update(_teleoperators_from_workstation(leaders, em))
    return devices


def _device_cfgs(teleoperators: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        name: cfg
        for name, cfg in teleoperators.items()
        if isinstance(cfg, dict)
    }


def _node_context_ok(node: Any) -> bool:
    try:
        context = getattr(node, "context", None)
        return bool(context is not None and context.ok())
    except Exception:  # noqa: BLE001 - RCL context may already be torn down.
        return False


def verify_leader_preempt_services(
    node: Any, teleoperators: dict[str, Any], timeout_sec: float = 3.0
) -> bool:
    """Shadow standby has no joint stream; only ``~/preempt`` must be up."""
    ok = True
    for name, cfg in _device_cfgs(teleoperators).items():
        srv_name = cfg.get("preempt_service")
        if not srv_name:
            print(f"  [!] {name}: missing preempt_service")
            ok = False
            continue
        client = None
        try:
            client = node.create_client(SetBool, srv_name)
            ready = client.wait_for_service(timeout_sec=timeout_sec)
            print(
                f"  {'[✓]' if ready else '[!]'} {name:<14}: {srv_name} -> "
                f"{'READY' if ready else 'UNAVAILABLE (workstation with_leaders:=true)'}"
            )
            ok = ok and ready
        except Exception as exc:  # noqa: BLE001 - aggregated like set_teleop_preempt.
            print(f"  [!] {name}: {exc.__class__.__name__}")
            ok = False
        finally:
            if client is not None:
                try:
                    node.destroy_client(client)
                except Exception:  # noqa: BLE001, S110 - best-effort during shutdown.
                    pass
    return ok


def _call_set_bool_services(
    node: Any,
    named_services: list[tuple[str, str]],
    data: bool,
    *,
    verb: str,
) -> bool:
    if not named_services:
        return True
    if not _node_context_ok(node):
        return False
    clients: list[Any] = []
    futures: list[tuple[str, str, Any]] = []
    failures: list[str] = []
    try:
        for name, srv_name in named_services:
            if not srv_name:
                failures.append(f"{name}: missing service")
                continue
            client = node.create_client(SetBool, srv_name)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=3.0):
                failures.append(f"{name}: {srv_name} unavailable")
                continue
            futures.append(
                (name, srv_name, client.call_async(SetBool.Request(data=data)))
            )
        t_end = time.monotonic() + 5.0
        pending = list(futures)
        while pending and time.monotonic() < t_end:
            pending = [item for item in pending if not item[2].done()]
            if pending:
                time.sleep(0.01)
        for name, srv_name, future in futures:
            if not future.done():
                failures.append(f"{name}: {srv_name} timed out")
                continue
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001 - service failures are aggregated.
                failures.append(f"{name}: {exc.__class__.__name__}")
                continue
            if response is None or not response.success:
                failures.append(
                    f"{name}: {getattr(response, 'message', 'no response')}"
                )
                continue
            print(f"  [✓] {name} {verb}: {response.message}")
    finally:
        for client in clients:
            try:
                node.destroy_client(client)
            except Exception:  # noqa: BLE001, S110 - best-effort during shutdown.
                pass
    if failures:
        print("  [!] Leader mode transition failed: " + "; ".join(failures))
        return False
    return True


def set_teleop_preempt(
    node: Any, teleoperators: dict[str, Any], preempt_active: bool
) -> bool:
    """Call every leader ``~/preempt`` in parallel. Leaders publish clutch to EM."""
    cfgs = [
        (name, cfg.get("preempt_service"))
        for name, cfg in _device_cfgs(teleoperators).items()
    ]
    verb = (
        "Preempt ACTIVE (0-G Float)"
        if preempt_active
        else "Preempt RELEASED (Shadow/Passive)"
    )
    return _call_set_bool_services(node, cfgs, preempt_active, verb=verb)


def _drain_stdin() -> None:
    if not sys.stdin.isatty():
        return
    while select.select([sys.stdin], [], [], 0)[0]:
        if not sys.stdin.readline():
            break


def _wait_leader_teleop_active(
    ctx: Any, teleoperators: dict[str, Any], timeout_sec: float = 2.0
) -> set[str]:
    wanted = [
        str(cfg["target_node"])
        for cfg in _device_cfgs(teleoperators).values()
        if cfg.get("target_node")
    ]
    if not wanted:
        return set()
    nodes = {name: ctx.make_node(name) for name in wanted}
    deadline = time.monotonic() + timeout_sec
    active: set[str] = set()
    need = set(wanted)
    while time.monotonic() < deadline:
        active = {name for name, node in nodes.items() if node.has_control}
        if need <= active:
            return active
        time.sleep(0.02)
    return active


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


def _quintic_home_plan(
    joint_names: list[str],
    q_start: list[float],
    q_goal: list[float],
    duration_s: float,
    steps: int = 80,
) -> rmi.PlanResult:
    """Minimum-jerk quintic with matching velocities for JTC interpolation."""
    steps = max(steps, 10)
    duration_s = max(duration_s, 0.1)
    points: list[rmi.PlanPoint] = []
    for i in range(steps + 1):
        s = i / steps
        h = 10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5)
        vel_scale = (30.0 * (s**2) - 60.0 * (s**3) + 30.0 * (s**4)) / duration_s
        points.append(
            rmi.PlanPoint(
                positions=[
                    q_start[j] + h * (q_goal[j] - q_start[j]) for j in range(len(q_start))
                ],
                velocities=[
                    vel_scale * (q_goal[j] - q_start[j]) for j in range(len(q_start))
                ],
                time_from_start_s=s * duration_s,
            )
        )
    return rmi.PlanResult(valid=True, joint_names=joint_names, points=points)


def smooth_homing(
    ctx: rmi.Context,
    robot: rmi.Robot,
    home_pose: list[float] | dict[str, list[float]],
    teleoperators: dict[str, Any],
    duration_s: float = 5.0,
    rate_hz: float = 50.0,
) -> None:
    """Smooth JTC quintic staging motion to home poses."""
    del rate_hz  # JTC interpolates the quintic; recorder Hz is not the motion rate.
    obs = robot.get_observation()
    name_to_pos = dict(zip(obs.joint_names, obs.joint_positions))

    # Collect arm and gripper start configurations
    parts_to_home = {}
    if isinstance(home_pose, dict):
        for part_name, target in home_pose.items():
            part_spec = ctx.profile.parts.get(part_name)
            if part_spec and all(j in name_to_pos for j in part_spec.joint_names):
                parts_to_home[part_name] = (
                    [name_to_pos[j] for j in part_spec.joint_names],
                    list(target),
                )

    for cfg in _device_cfgs(teleoperators).values():
        arm_p, grip_p = cfg["arm_part"], cfg["gripper_part"]
        arm_spec, grip_spec = (
            ctx.profile.parts.get(arm_p),
            ctx.profile.parts.get(grip_p),
        )

        if (
            not isinstance(home_pose, dict)
            and arm_spec
            and all(j in name_to_pos for j in arm_spec.joint_names)
        ):
            parts_to_home[arm_p] = (
                [name_to_pos[j] for j in arm_spec.joint_names],
                list(home_pose),
            )
        if grip_spec and all(j in name_to_pos for j in grip_spec.joint_names):
            parts_to_home.setdefault(
                grip_p,
                (
                    [name_to_pos[j] for j in grip_spec.joint_names],
                    [0.020],
                ),
            )

    if not parts_to_home:
        return

    planner = ctx.make_node("TrajectoryPlanner")
    with planner.activate(preempt=True):
        executions = []
        for part_name, (q_start, q_goal) in parts_to_home.items():
            part_spec = ctx.profile.parts[part_name]
            if len(q_goal) < 2:
                executions.append(planner.execute(part_name, q_goal))
                continue
            plan = _quintic_home_plan(
                list(part_spec.joint_names),
                q_start,
                q_goal,
                duration_s=duration_s,
            )
            executions.append(planner.execute(part_name, plan))
        for execution in executions:
            execution.wait(timeout=duration_s + 5.0)


def _open_context(profile: str, *, use_sim_time: bool):
    """Build RMI Context; optionally pin the node clock to /clock."""
    if not use_sim_time:
        return rmi.Context.from_profile(profile)

    import rclpy
    from rclpy.parameter import Parameter

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("rmi_record")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    return rmi.Context.from_profile(profile, node=node)


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
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Stamp commands with /clock (required when workstation uses sim time). "
            "Wall-clock stamps are dropped by EM as future_command."
        ),
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


def main() -> None:
    args = parse_args()

    # 1. Connect Context strictly from Profile
    print("[1/4] Connecting to robot embodiment runtime...")
    print(f"  use_sim_time={args.use_sim_time}")
    ctx = _open_context(args.profile, use_sim_time=args.use_sim_time)
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
    homing_cfg = ctx.profile.raw_data.get("homing", {})
    homing_duration_s = float(homing_cfg.get("duration_s", 2.5))
    home_pose = homing_cfg.get(
        "joint_positions",
        homing_cfg.get("home_pose", [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]),
    )

    teleoperators = load_teleoperators(ctx.profile)
    cameras_cfg = ctx.profile.raw_data.get("sensors", {}).get("cameras", {})

    print("=" * 72)
    print("  RMI Production Multi-Modal Dataset Recorder Client")
    print(f"  Embodiment Profile : {args.profile}")
    print(f"  Sim clock          : {args.use_sim_time}")
    print(f"  Task Description   : '{task_name}'")
    print(f"  Target Episodes    : {target_episodes}")
    print(f"  Recording Rate     : {rate_hz:.1f} Hz")
    print(
        f"  Dataset Directory  : {Path(rec_cfg.get('root_dir', 'data/episodes')) / task_name}"
    )
    if _device_cfgs(teleoperators):
        services = [
            f"{name}:{cfg.get('preempt_service')}"
            for name, cfg in _device_cfgs(teleoperators).items()
        ]
        print(f"  Keyboard clutch    : ENTER toggles {services}")
    else:
        print("  Keyboard clutch    : none (device clutch / ENTER gates recording only)")
    print("=" * 72)

    # 3. Perception Warmup
    if not args.skip_camera_check and cameras_cfg:
        verify_cameras(ctx.node, cameras_cfg, timeout_sec=3.0)

    if _device_cfgs(teleoperators):
        print("\n[leaders] Verifying ~/preempt (shadow has no joint stream)...")
        verify_leader_preempt_services(ctx.node, teleoperators)

    # 4. Activate MCAP Recorder Backend
    recorder = ctx.make_recorder(
        config=_task_recorder_config(rec_cfg, task_name, args.operator),
        autostart=True,
    )
    recorder.activate()

    robot = ctx.robot
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

            # STEP 2: Keyboard gate. Piper: ENTER = parallel preempt. Else ENTER starts bag only.
            print("\n" + "-" * 72)
            _drain_stdin()
            if _device_cfgs(teleoperators):
                input(
                    f"  >> [READY] Press [ENTER] to ENGAGE both leaders & START Episode {current_ep_idx}... "
                )
            else:
                input(
                    f"  >> [READY] Press [ENTER] to START recording Episode {current_ep_idx}... "
                )
            _drain_stdin()
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
                target=lambda ev=stop_event: (input(), ev.set()), daemon=True
            )

            last_recorded_path = ""
            start_time = time.monotonic()
            dt = 1.0 / rate_hz

            # STEP 3: Record while leaders publish clutch + joints to EM.
            with recorder.episode(task=task_name, metadata=metadata) as ep:
                    if not set_teleop_preempt(ctx.node, teleoperators, True):
                        set_teleop_preempt(ctx.node, teleoperators, False)
                        raise RuntimeError(
                            "teleoperation was not engaged; every leader must confirm 0-G mode"
                        )
                    active = _wait_leader_teleop_active(ctx, teleoperators)
                    if _device_cfgs(teleoperators):
                        print(f"  [ACTIVE] {sorted(active) or list(_device_cfgs(teleoperators))}")

                    print(
                        "\n  🔴 RECORDING ACTIVE! Manipulate master arms to demonstrate task."
                    )
                    print("  >> Press [ENTER] to FINISH this episode (releases preempt).\n")
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

            _drain_stdin()
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
