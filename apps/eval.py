"""Evaluate a LeRobot policy through the standard RMI application loop."""

from __future__ import annotations

import argparse
import time

import rmi

from policy_inference.lerobot import LeRobotPolicy, ros_image_to_numpy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RMI LeRobot policy evaluation")
    parser.add_argument("--profile", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--resource", required=True)
    parser.add_argument("--node-name", default="Policy")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--policy-type", default=None)
    parser.add_argument("--max-stream-skew", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with rmi.Context.from_profile(args.profile) as context:
        layout = context.profile.policy_layout(args.node_name)
        for camera_name in dict.fromkeys(layout.camera_sources.values()):
            context.make_camera(
                camera_name,
                converter=ros_image_to_numpy,
                history_size=1,
            )

        policy = LeRobotPolicy(
            layout,
            args.checkpoint,
            task=args.task,
            device=args.device,
            expected_policy_type=args.policy_type,
            max_stream_skew_s=args.max_stream_skew,
        )
        policy_node = context.make_node(args.node_name, policy)
        period_s = 1.0 / layout.frequency
        next_tick = time.monotonic()
        try:
            context.wait_until_ready(
                timeout=30.0,
                check_cameras=bool(layout.camera_sources),
                require_execution_manager=True,
            )
            with policy_node.activate():
                while True:
                    next_tick += period_s
                    observation = context.robot[args.resource].get_observation()
                    actions = policy.select_action(observation)
                    policy_node[args.resource].submit(actions)

                    sleep_s = next_tick - time.monotonic()
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)
                    else:
                        next_tick = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            policy.close()


if __name__ == "__main__":
    main()
