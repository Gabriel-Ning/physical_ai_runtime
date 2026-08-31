#!/usr/bin/env python3
"""Franka JSPC policy with transparent gamepad teleop preemption and MCAP recording."""

from __future__ import annotations

import argparse
import math
import threading
import time

import rclpy
import rmi

FRANKA_ARM_JOINTS = (
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
)


class SmoothHomingPlanner:
    """Generate the profile-configured quintic arm trajectory for JTC."""

    def __init__(self, profile: rmi.EmbodimentConfig) -> None:
        homing = profile.raw_data.get("homing", {})
        self.duration_s = float(homing.get("duration_s", 0.0))
        self._targets = homing.get("joint_positions", {})
        self._parts = profile.parts
        if (
            self.duration_s <= 0.0
            or not isinstance(self._targets, dict)
            or not self._targets
        ):
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
        for part_name, target in self._targets.items():
            part = self._parts.get(part_name)
            if part is None:
                raise ValueError(
                    f"profile.homing references unknown part {part_name!r}"
                )
            if len(target) != len(part.joint_names):
                raise ValueError(
                    f"profile.homing.joint_positions.{part_name} has {len(target)} "
                    f"values; expected {len(part.joint_names)}"
                )
            missing = [name for name in part.joint_names if name not in current]
            if missing:
                raise RuntimeError(
                    f"joint state is missing {part_name} joints: {missing}"
                )
            if part.part_type == "parallel_gripper":
                plans[part_name] = [float(value) for value in target]
                continue
            start = [current[name] for name in part.joint_names]
            goal = [float(value) for value in target]
            zeros = [0.0] * len(goal)
            plans[part_name] = rmi.PlanResult(
                valid=True,
                joint_names=list(part.joint_names),
                points=[
                    rmi.PlanPoint(
                        positions=start,
                        velocities=zeros,
                        accelerations=zeros,
                        time_from_start_s=0.0,
                    ),
                    rmi.PlanPoint(
                        positions=goal,
                        velocities=zeros,
                        accelerations=zeros,
                        time_from_start_s=self.duration_s,
                    ),
                ],
            )
        return plans


class CirclePolicy:
    """Select observation-relative single-arm JSPC circular actions for Franka."""

    def __init__(
        self,
        *,
        rate_hz: float = 100.0,
        amplitude: float = 0.08,
        period_s: float = 4.0,
    ) -> None:
        self._amplitude = amplitude
        self._period_s = period_s
        self._rate_hz = rate_hz
        self._started_at = time.monotonic()

    def select_action(self, observation: rmi.Observation) -> rmi.Action:
        positions = dict(
            zip(
                observation.data["joint_names"],
                observation.data["joint_positions"],
                strict=False,
            )
        )
        target = [positions.get(joint, 0.0) for joint in FRANKA_ARM_JOINTS]
        elapsed_s = time.monotonic() - self._started_at
        phase = 2.0 * math.pi * elapsed_s / self._period_s
        step = self._amplitude * 2.0 * math.pi / self._period_s / self._rate_hz
        target[4] -= step * math.sin(phase)
        target[5] += step * math.cos(phase)
        return rmi.Action(part="arm", command="joint_reference", value=target)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Franka single-arm policy with EM-managed gamepad teleop preemption"
    )
    parser.add_argument("--profile", default="fr3_pika_single_arm.yaml")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--amplitude", type=float, default=0.08)
    parser.add_argument("--period", type=float, default=4.0)
    parser.add_argument("--task", default="franka_gamepad_takeover")
    args = parser.parse_args()
    if args.rate_hz <= 0.0 or args.amplitude <= 0.0 or args.period <= 0.0:
        parser.error("rate-hz, amplitude, and period must be positive")

    rclpy.init()
    policy = CirclePolicy(
        rate_hz=args.rate_hz,
        amplitude=args.amplitude,
        period_s=args.period,
    )
    try:
        with rmi.Context.from_profile(args.profile, timeout_sec=15.0) as context:
            robot = context.robot
            policy_node = context.make_node("DummyPolicy", policy)
            homing_planner = SmoothHomingPlanner(context.profile)
            planner_node = context.make_node("Planner", homing_planner)
            teleop_node = context.make_node("GamepadTeleop")

            context.wait_until_ready(
                timeout=30.0,
                require_execution_manager=True,
            )
            print(f"[READY] Dummy JSPC policy active at {args.rate_hz:.1f}Hz.")
            print("[READY] Hold L1/LB for teleop; release it to resume policy.")
            print(
                f"[READY] EM sources: default={policy_node.name}, "
                f"preempt={teleop_node.name}"
            )

            recorder = context.make_recorder(autostart=True)
            recorder.wait_ready(timeout_s=10.0)

            record_requested = threading.Event()
            stop_requested = threading.Event()

            print(
                f"\n[HOMING] Executing JTC homing over "
                f"{homing_planner.duration_s:.1f}s..."
            )
            homing_plans = homing_planner.plan(robot["manipulator"].get_observation())
            with planner_node.activate():
                planner_node["manipulator"].execute(
                    homing_plans,
                    timeout=homing_planner.duration_s + 5.0,
                )
            print("[HOMING] JTC homing completed.")

            policy_node.activate()

            def user_input_thread() -> None:
                input(
                    "\n[CONTROL] Policy running. Press Enter to START recording episode... "
                )
                record_requested.set()
                input(
                    "\n[RECORDING] Recording active. Press Enter to STOP and save episode... "
                )
                stop_requested.set()

            threading.Thread(target=user_input_thread, daemon=True).start()

            period_s = 1.0 / args.rate_hz
            intervention_active = False
            episode_scope: rmi.EpisodeScope | None = None
            next_tick = time.monotonic()

            while rclpy.ok() and not stop_requested.is_set():
                next_tick += period_s
                observation = robot["arm"].get_observation()

                if teleop_node.is_active and not intervention_active:
                    print(
                        f"\n[INTERVENTION] {teleop_node.name} took over control "
                        "(human gamepad clutch active)"
                    )
                    intervention_active = True
                elif intervention_active and policy_node.is_active:
                    print(
                        f"\n[RESUMED] {policy_node.name} regained control "
                        "(human released clutch)"
                    )
                    intervention_active = False

                action = policy.select_action(observation)
                policy_node["arm"].submit(action)

                if record_requested.is_set() and episode_scope is None:
                    pending_episode = recorder.episode(
                        task=args.task,
                        metadata={"profile": args.profile, "rate_hz": args.rate_hz},
                    )
                    try:
                        pending_episode.__enter__()
                    except RuntimeError as error:
                        record_requested.clear()
                        print(f"\n[RECORDING ERROR] {error}")
                        print("[CONTROL] Policy continues; recording was not started.")
                    else:
                        episode_scope = pending_episode

                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()

            if episode_scope is not None:
                episode_scope.__exit__(None, None, None)
                episode_path = getattr(episode_scope.final_status, "episode_path", "")
                if not episode_scope.validated:
                    raise RuntimeError("recorder did not return a validated episode")
                print(f"\n[VALID] Episode passed validation: {episode_path}")
    except KeyboardInterrupt:
        print("\n[STOP] Example interrupted")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
