#!/usr/bin/env python3
"""02_policy_camera.py: Policy loop with Multimodal Workstation Camera Facade.

Demonstrates **Multimodal AI Policy Execution**:
1. Attaches workstation-side Camera sensor facades (RGB/Depth image streams).
2. Creates an RMI Policy Agent bound to the robot and camera sensors.
3. Executes a synchronized control loop where each step receives aligned `(joint_states, camera_frames)`
   and dispatches policy action references.

Usage:
  # In terminal 1 (start RT fake hardware):
  ros2 launch franka_manipulation_controller_bringup controller_bringup.launch.py use_fake_hardware:=true

  # In terminal 2:
  python examples/02_policy_camera.py --profile fr3_pika_single_arm.yaml --ticks 20
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import rmi
from rmi.sensing import Camera


def main() -> None:
    parser = argparse.ArgumentParser(description="Multimodal Policy with Camera Stream")
    parser.add_argument(
        "--profile",
        type=str,
        default="fr3_pika_single_arm.yaml",
        help="Embodiment profile path (e.g. fr3_pika_single_arm.yaml)",
    )
    parser.add_argument(
        "--camera",
        type=str,
        default="",
        help="Name of camera declared in profile (defaults to first profile camera, e.g. 'wrist_d405')",
    )
    parser.add_argument(
        "--camera-topic",
        type=str,
        default="",
        help="Optional custom ROS Image topic override",
    )
    parser.add_argument(
        "--ticks",
        type=int,
        default=30,
        help="Number of control loop iterations",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Policy control rate (Hz)",
    )
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f"  RMI Demo 02: Multimodal Policy with Camera Facade")
    print(f"  Profile: {args.profile}")
    print(f"  Rate: {args.rate_hz} Hz ({args.ticks} ticks)")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context & Auto-Detect Camera
    print("[1/3] Initializing Context & Attaching Camera Sensor...")
    ctx = rmi.Context.from_profile(args.profile)
    ctx.wait_until_ready(timeout=5.0)
    robot = ctx.robot

    # Select Camera
    selected_camera_name = args.camera
    if not selected_camera_name and hasattr(ctx.profile, "cameras") and ctx.profile.cameras:
        selected_camera_name = next(iter(ctx.profile.cameras.keys()))

    camera: Any | None = None
    sensors = []
    try:
        from sensor_msgs.msg import Image
        if selected_camera_name and selected_camera_name in ctx.profile.cameras:
            camera = ctx.make_camera(selected_camera_name)
            topic_str = ctx.profile.cameras[selected_camera_name].ros_topic
        elif args.camera_topic:
            camera = ctx.make_sensor(
                "custom_cam",
                topic=args.camera_topic,
                message_type=Image,
                history_size=10,
            )
            topic_str = args.camera_topic
        else:
            topic_str = "None"

        if camera is not None:
            try:
                camera.wait_until_ready(timeout=1.5)
            except Exception:
                pass

            if camera.is_ready():
                sensors.append(camera)
                print(f"  [✓] Camera facade active: {camera.name!r} on {topic_str!r}")
            else:
                print(f"  [!] Camera {camera.name!r} topic idle; running in joint-only policy mode")
    except Exception as exc:
        print(f"  [!] Camera facade note: {exc}")

    # 2. Build Policy Agent with sensor bindings
    print("\n[2/3] Constructing Policy Agent with sensor bindings...")
    arm_part = "arm" if "arm" in ctx.profile.parts else next(iter(ctx.profile.parts.keys()))
    arm_joints = ctx.profile.parts[arm_part].joint_names if arm_part in ctx.profile.parts else ()
    agent = ctx.make_agent("Policy", frequency=args.rate_hz, sensors=sensors)

    # 3. Synchronized Multimodal Execution Loop
    print(f"\n[3/3] Starting multimodal control loop ({args.ticks} ticks at {args.rate_hz} Hz)...")
    dt = 1.0 / args.rate_hz

    with agent.run(robot) as session:
        initial_obs = session.observe()
        home_arm_q = list(initial_obs.joint_positions[:len(arm_joints)])
        print(f"  [Policy Session Active] Generation = {session.generation_for(arm_part)}")

        for tick in range(1, args.ticks + 1):
            t = tick * dt

            # Acquire synchronized multi-modal observation (robot states + camera frames)
            obs = session.observe()

            cam_info = "N/A"
            if camera is not None and camera.is_ready():
                sample = camera.latest
                img = sample.value
                w, h = getattr(img, "width", 0), getattr(img, "height", 0)
                cam_info = f"seq={sample.sequence} t_src={sample.source_time_s:.2f}s ({w}x{h})"

            # Compute policy reference (sinusoidal holding wave)
            target_q = list(home_arm_q)
            if target_q and len(target_q) >= 4:
                target_q[0] += 0.03 * math.sin(1.5 * t)
                target_q[3] += 0.02 * math.cos(1.5 * t)

            # Send reference action
            session.act(rmi.Action(
                part=arm_part,
                command="joint_reference",
                value=target_q,
            ))

            if tick % int(args.rate_hz // 2 or 1) == 0:
                print(
                    f"  [Tick {tick:3d}/{args.ticks}] "
                    f"Joints: {[round(x, 3) for x in obs.joint_positions[:3]]}... | "
                    f"Camera: {cam_info}"
                )

            session.wait()

    if camera is not None:
        camera.close()
    ctx.close()
    print("\n[✓] Multimodal policy execution completed successfully.")


if __name__ == "__main__":
    main()
