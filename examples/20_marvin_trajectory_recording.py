#!/usr/bin/env python3
"""Home Marvin, then record a two-cycle bimanual JTC trajectory."""

from __future__ import annotations

import argparse
import math

import rclpy
import rmi


class SmoothHomingPlanner:
    """Build profile-configured arm trajectories and gripper commands."""

    def __init__(self, profile: rmi.EmbodimentConfig) -> None:
        homing = profile.raw_data.get("homing", {})
        self.duration_s = float(homing.get("duration_s", 0.0))
        self.targets = homing.get("joint_positions", {})
        self.parts = profile.parts
        if self.duration_s <= 0.0 or not isinstance(self.targets, dict):
            raise ValueError(
                "profile.homing requires positive duration_s and joint_positions"
            )

    def plan(self, observation: rmi.Observation) -> dict[str, object]:
        current = _joint_positions(observation)
        plans: dict[str, object] = {}
        for part_name, raw_target in self.targets.items():
            part = self.parts.get(part_name)
            if part is None:
                raise ValueError(f"homing references unknown part {part_name!r}")
            target = [float(value) for value in raw_target]
            _validate_target(part_name, part.joint_names, target, current)
            if part.part_type == "parallel_gripper":
                plans[part_name] = target
                continue
            zeros = [0.0] * len(target)
            plans[part_name] = rmi.PlanResult(
                valid=True,
                joint_names=list(part.joint_names),
                points=[
                    rmi.PlanPoint(
                        positions=[current[name] for name in part.joint_names],
                        velocities=zeros,
                        accelerations=zeros,
                        time_from_start_s=0.0,
                    ),
                    rmi.PlanPoint(
                        positions=target,
                        velocities=zeros,
                        accelerations=zeros,
                        time_from_start_s=self.duration_s,
                    ),
                ],
            )
        return plans


class BimanualSwingPlanner:
    """Build mirrored sinusoidal Joint-1 trajectories around the home pose."""

    def __init__(
        self,
        homing_planner: SmoothHomingPlanner,
        *,
        amplitude_rad: float,
        cycles: int,
        period_s: float,
    ) -> None:
        self._homing = homing_planner
        self.amplitude_rad = amplitude_rad
        self.cycles = cycles
        self.period_s = period_s
        self.duration_s = cycles * period_s

    def plan(self, observation: rmi.Observation) -> dict[str, object]:
        current = _joint_positions(observation)
        plans: dict[str, object] = {}
        for part_name, raw_target in self._homing.targets.items():
            part = self._homing.parts[part_name]
            target = [float(value) for value in raw_target]
            _validate_target(part_name, part.joint_names, target, current)
            if part.part_type == "parallel_gripper":
                plans[part_name] = target
                continue

            direction = 1.0 if part_name == "left_arm" else -1.0
            base = [current[name] for name in part.joint_names]
            zeros = [0.0] * len(target)
            points = []
            for index in range(self.cycles * 4 + 1):
                time_s = index * self.period_s / 4.0
                phase = 2.0 * math.pi * time_s / self.period_s
                positions = list(base)
                positions[0] += direction * self.amplitude_rad * math.sin(phase)
                points.append(
                    rmi.PlanPoint(
                        positions=positions,
                        velocities=list(zeros),
                        accelerations=list(zeros),
                        time_from_start_s=time_s,
                    )
                )
            plans[part_name] = rmi.PlanResult(
                valid=True,
                joint_names=list(part.joint_names),
                points=points,
            )
        return plans


def _joint_positions(observation: rmi.Observation) -> dict[str, float]:
    return dict(
        zip(
            observation.data["joint_names"],
            observation.data["joint_positions"],
            strict=False,
        )
    )


def _validate_target(
    part_name: str,
    joint_names: tuple[str, ...] | list[str],
    target: list[float],
    current: dict[str, float],
) -> None:
    if len(target) != len(joint_names):
        raise ValueError(
            f"homing target for {part_name} has {len(target)} values; "
            f"expected {len(joint_names)}"
        )
    missing = [name for name in joint_names if name not in current]
    if missing:
        raise RuntimeError(f"joint state is missing {part_name} joints: {missing}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="marvin_bimanual.yaml")
    parser.add_argument("--amplitude-deg", type=float, default=15.0)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--period", type=float, default=4.0)
    parser.add_argument("--task", default="marvin_bimanual_jtc_swing")
    args = parser.parse_args()
    if args.amplitude_deg <= 0.0 or args.cycles <= 0 or args.period <= 0.0:
        parser.error("amplitude-deg, cycles, and period must be positive")

    rclpy.init()
    try:
        with rmi.Context.from_profile(args.profile, timeout_sec=15.0) as context:
            robot = context.robot
            homing_planner = SmoothHomingPlanner(context.profile)
            swing_planner = BimanualSwingPlanner(
                homing_planner,
                amplitude_rad=math.radians(args.amplitude_deg),
                cycles=args.cycles,
                period_s=args.period,
            )
            planner_node = context.make_node("Planner", swing_planner)
            context.wait_until_ready(timeout=30.0, require_execution_manager=True)

            observation = robot["dual_manipulator"].get_observation()
            print(f"[HOMING] Executing homing over {homing_planner.duration_s:.1f}s...")
            with planner_node.activate():
                planner_node["dual_manipulator"].execute(
                    homing_planner.plan(observation),
                    timeout=homing_planner.duration_s + 5.0,
                )
            print("[HOMING] Completed.")

            input(
                f"[READY] Press Enter to record {args.cycles} mirrored Joint-1 "
                f"cycles at ±{args.amplitude_deg:.1f} deg... "
            )
            recorder = context.make_recorder(autostart=True)
            recorder.wait_ready(timeout_s=10.0)
            swing_plan = swing_planner.plan(robot["dual_manipulator"].get_observation())
            with recorder.episode(
                task=args.task,
                metadata={
                    "profile": args.profile,
                    "amplitude_deg": args.amplitude_deg,
                    "cycles": args.cycles,
                    "period_s": args.period,
                    "command_type": "joint_trajectory",
                },
            ) as episode:
                print(
                    f"[RECORDING] Executing {swing_planner.duration_s:.1f}s JTC "
                    "trajectory..."
                )
                with planner_node.activate():
                    planner_node["dual_manipulator"].execute(
                        swing_plan,
                        timeout=swing_planner.duration_s + 5.0,
                    )

            episode_path = getattr(episode.final_status, "episode_path", "")
            if not episode.validated:
                raise RuntimeError("recorder did not return a validated episode")
            print(f"[VALID] Episode passed validation: {episode_path}")
    except KeyboardInterrupt:
        print("\n[STOP] Example interrupted")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
