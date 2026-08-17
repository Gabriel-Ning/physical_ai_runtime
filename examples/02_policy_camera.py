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
        "--camera-topic",
        type=str,
        default="/camera/head/color/image_raw",
        help="ROS Image topic for workstation camera",
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
        default=20.0,
        help="Policy control rate (Hz)",
    )
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f"  RMI Demo 02: Multimodal Policy with Camera Facade")
    print(f"  Profile: {args.profile}")
    print(f"  Camera Topic: {args.camera_topic}")
    print(f"  Rate: {args.rate_hz} Hz ({args.ticks} ticks)")
    print(f"=======================================================\n")

    # 1. Initialize RMI Context
    print("[1/3] Initializing Context & Workstation Camera...")
    ctx = rmi.Context.from_profile(args.profile)
    robot = ctx.robot
    robot.wait_until_ready(timeout=5.0)

    # Optional: Attach Camera Sensor facade via Context
    camera: Any | None = None
    sensors = []
    try:
        from sensor_msgs.msg import Image
        if "head_cam" in ctx.profile.cameras:
            camera = ctx.make_camera("head_cam")
        else:
            camera = ctx.make_sensor(
                "head_cam",
                topic=args.camera_topic,
                message_type=Image,
                history_size=10,
            )

        # If no real camera node is active, emit a demo frame so multimodal synchronization is demonstrated
        if not camera.is_ready():
            demo_pub = ctx.node.create_publisher(Image, args.camera_topic, 10)
            img = Image()
            img.header.stamp = ctx.node.get_clock().now().to_msg()
            img.header.frame_id = "camera_optical_frame"
            img.height = 240
            img.width = 320
            img.encoding = "rgb8"
            img.data = [128] * (240 * 320 * 3)
            demo_pub.publish(img)
            try:
                camera.wait_until_ready(timeout=1.0)
            except Exception:
                pass

        if camera.is_ready():
            sensors.append(camera)
            print(f"  [✓] Camera facade active on {args.camera_topic!r}")
        else:
            print(f"  [!] Camera facade offline; running joint-only policy mode")
    except Exception as exc:
        print(f"  [!] Camera facade note: {exc}")

    # 2. Build Policy Agent with sensor bindings
    print("[2/3] Constructing Policy Agent with sensor bindings...")
    arm_part = "arm" if "arm" in ctx.profile.parts else next(iter(ctx.profile.parts.keys()))
    agent = ctx.make_agent("Policy", frequency=args.rate_hz, sensors=sensors)

    # 3. Synchronized Multimodal Execution Loop
    print(f"[3/3] Starting multimodal control loop...")
    dt = 1.0 / args.rate_hz

    with agent.run(robot) as session:
        initial_obs = session.observe()
        home_q = list(initial_obs.joint_positions)
        print(f"  [Policy Session Active] Generation = {session.generation_for(arm_part)}")

        for tick in range(1, args.ticks + 1):
            t = tick * dt

            # Acquire synchronized multi-modal observation (robot states + camera frames)
            obs = session.observe()

            cam_info = "N/A"
            if camera is not None and camera.is_ready():
                sample = camera.latest
                cam_info = f"seq={sample.sequence} t_src={sample.source_time_s:.2f}s"

            # Compute policy reference (sinusoidal holding wave)
            target_q = list(home_q)
            if target_q:
                target_q[0] += 0.05 * math.sin(1.5 * t)
                target_q[3] += 0.03 * math.cos(1.5 * t)

            # Send reference action
            session.act(rmi.Action(
                part=arm_part,
                command="joint_reference",
                value=target_q,
            ))

            if tick % int(args.rate_hz // 2 or 1) == 0:
                print(
                    f"  [Tick {tick:3d}/{args.ticks}] "
                    f"Joints: {[round(x, 2) for x in obs.joint_positions[:3]]}... | "
                    f"Camera: {cam_info}"
                )

            session.wait()

    if camera is not None:
        camera.close()
    ctx.close()
    print("\n[✓] Multimodal policy execution completed successfully.")


if __name__ == "__main__":
    main()
