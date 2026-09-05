#!/usr/bin/env python3
"""Relay both Piper leaders through EM and record one bimanual episode."""

from __future__ import annotations

import argparse
import threading
import time
from contextlib import ExitStack
from typing import Any

import rmi
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

SIDES = {
    "left": {
        "node": "TeleopJoint_Left",
        "arm": "left_arm",
        "gripper": "left_gripper",
    },
    "right": {
        "node": "TeleopJoint_Right",
        "arm": "right_arm",
        "gripper": "right_gripper",
    },
}


def _topic(side: str, endpoint: str) -> str:
    return f"/action_sources/piper_leader_{side}/{endpoint}/joint_reference"


def _relay(node: rmi.Node, part: str):
    def callback(message: JointTrajectory) -> None:
        if node.is_active:
            node.submit(rmi.Action(part=part, command="joint_reference", value=message))

    return callback


def _set_preempt(node: Any, services: list[str], active: bool) -> None:
    """Transition both leaders; fail the gate unless every service confirms."""
    for service in services:
        client = node.create_client(SetBool, service)
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f"preempt service unavailable: {service}")
            future = client.call_async(SetBool.Request(data=active))
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            response = future.result() if future.done() else None
            if response is None or not response.success:
                message = getattr(response, "message", "timeout")
                raise RuntimeError(f"{service} failed: {message}")
        finally:
            node.destroy_client(client)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="piper_bimanual.yaml")
    parser.add_argument("--task", default="piper_bimanual_leader_teleop")
    args = parser.parse_args()

    stop = threading.Event()
    with rmi.Context.from_profile(args.profile, timeout_sec=15.0) as context:
        context.wait_until_ready(timeout=30.0, require_execution_manager=True)
        recorder = context.make_recorder(autostart=True)
        recorder.wait_ready(timeout_s=10.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        teleoperators = context.profile.raw_data.get("teleoperators", {})
        preempt_services = [
            config["preempt_service"] for config in teleoperators.values()
        ]
        if len(preempt_services) != 2:
            raise RuntimeError("profile requires two Piper preempt services")

        with ExitStack() as stack:
            for side, config in SIDES.items():
                node = context.make_node(config["node"])
                stack.enter_context(node.activate(preempt=True))
                context.node.create_subscription(
                    JointTrajectory,
                    _topic(side, "arm"),
                    _relay(node, config["arm"]),
                    qos,
                )
                context.node.create_subscription(
                    JointTrajectory,
                    _topic(side, "end_effector"),
                    _relay(node, config["gripper"]),
                    qos,
                )

            input(
                "[READY] Press Enter to PREEMPT both leaders and START recording... "
            )
            try:
                _set_preempt(context.node, preempt_services, True)
                with recorder.episode(
                    task=args.task, metadata={"profile": args.profile}
                ) as episode:

                    def wait_for_stop() -> None:
                        input("[RECORDING] Press Enter to RELEASE and save... ")
                        stop.set()

                    threading.Thread(target=wait_for_stop, daemon=True).start()
                    while not stop.is_set():
                        time.sleep(0.1)
            finally:
                try:
                    _set_preempt(context.node, preempt_services, False)
                except RuntimeError as error:
                    print(f"[RELEASE WARNING] {error}")

        if not episode.validated:
            raise RuntimeError("recorder did not return a validated episode")
        print(f"[VALID] {getattr(episode.final_status, 'episode_path', '')}")


if __name__ == "__main__":
    main()
