#!/usr/bin/env python3
"""Franka JSPC policy with transparent gamepad teleop preemption and MCAP recording."""

from __future__ import annotations

import argparse
import math
import threading
import time
from collections.abc import Callable

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
    """Select single-arm JSPC actions around a rebaselinable joint center.

    Starts from profile.homing. On clutch release, rebaseline the center from
    the current observation (example 17 style) so resume does not jump back to
    the pre-teleop pose. Phase advances on ``clock_now`` (sim clock under
    MuJoCo). Large ``/clock`` jumps are clamped so they do not become
    discontinuous joint targets. Command stamps still come from the RMI node
    clock separately.
    """

    def __init__(
        self,
        *,
        center: list[float],
        rate_hz: float = 100.0,
        amplitude: float = 0.5,
        period_s: float = 3.0,
        hold_only: bool = False,
        clock_now: Callable[[], float] | None = None,
        max_phase_dt_s: float | None = None,
    ) -> None:
        if len(center) != len(FRANKA_ARM_JOINTS):
            raise ValueError(
                f"center requires {len(FRANKA_ARM_JOINTS)} joints, got {len(center)}"
            )
        self._center_positions = [float(value) for value in center]
        self._amplitude = amplitude
        self._period_s = period_s
        self._rate_hz = rate_hz
        self._hold_only = hold_only or amplitude <= 0.0
        self._clock_now = clock_now or time.monotonic
        # Absorb /clock stalls without skipping phase (2 control periods default).
        self._max_phase_dt_s = (
            float(max_phase_dt_s)
            if max_phase_dt_s is not None
            else max(0.05, 2.0 / max(rate_hz, 1.0))
        )
        self._phase_elapsed_s = 0.0
        self._last_clock_s: float | None = None

    def reset_center(self, observation: rmi.Observation) -> None:
        """Rebaseline circle center to current joints and restart soft-start."""
        positions = dict(
            zip(
                observation.data["joint_names"],
                observation.data["joint_positions"],
                strict=False,
            )
        )
        missing = [name for name in FRANKA_ARM_JOINTS if name not in positions]
        if missing:
            raise RuntimeError(f"joint state is missing arm joints: {missing}")
        self._center_positions = [float(positions[name]) for name in FRANKA_ARM_JOINTS]
        self._phase_elapsed_s = 0.0
        self._last_clock_s = None

    def select_action(self, observation: rmi.Observation) -> rmi.Action:
        del observation  # Absolute targets around the last rebaselined center.
        now_s = float(self._clock_now())
        if self._last_clock_s is None:
            self._last_clock_s = now_s
        else:
            dt_s = now_s - self._last_clock_s
            self._last_clock_s = now_s
            if dt_s < 0.0:
                dt_s = 0.0
            elif dt_s > self._max_phase_dt_s:
                dt_s = self._max_phase_dt_s
            self._phase_elapsed_s += dt_s

        if self._hold_only:
            return rmi.Action(
                part="arm",
                command="joint_reference",
                value=list(self._center_positions),
            )

        elapsed_s = self._phase_elapsed_s
        # Soft start: hold center, then ramp amplitude over one period.
        if elapsed_s < 2.0:
            return rmi.Action(
                part="arm",
                command="joint_reference",
                value=list(self._center_positions),
            )
        ramp = min(1.0, (elapsed_s - 2.0) / self._period_s)
        phase = 2.0 * math.pi * (elapsed_s - 2.0) / self._period_s
        amp = self._amplitude * ramp

        target = list(self._center_positions)
        # Multi-joint elliptical stress around center (stay inside FR3 limits).
        target[0] += amp * math.sin(phase)
        target[1] += amp * 0.6 * math.cos(phase)
        target[2] += amp * 0.45 * math.sin(phase + 0.5 * math.pi)
        target[3] += amp * 0.55 * math.sin(phase + math.pi)
        target[4] += amp * 0.35 * math.cos(phase + 0.25 * math.pi)
        return rmi.Action(part="arm", command="joint_reference", value=target)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Franka single-arm policy with EM-managed gamepad teleop preemption"
    )
    parser.add_argument("--profile", default="fr3_pika_single_arm.yaml")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.5,
        help=(
            "Circular motion amplitude in radians (default: 0.5, multi-joint stress). "
            "Peak joint rate ≈ amplitude * 2π / period; keep that ≲ 1.5 rad/s. "
            "Use 0 with --hold-only."
        ),
    )
    parser.add_argument(
        "--period",
        type=float,
        default=3.0,
        help="Circle period in seconds (default: 3.0). Longer = gentler for a given amplitude.",
    )
    parser.add_argument(
        "--hold-only",
        action="store_true",
        help="Command constant profile.homing (no circle) to isolate JSPC tracking.",
    )
    parser.add_argument("--task", default="franka_gamepad_takeover")
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Drive RMI stamps/policy phase from /clock (default: on for MuJoCo). "
            "Pass --no-use-sim-time on real hardware."
        ),
    )
    parser.add_argument(
        "--no-cam",
        action="store_true",
        help="Use camera-free recording configuration (apps/recording/franka_manipulation_no_cam.yaml)",
    )
    parser.add_argument(
        "--no-record",
        action="store_true",
        help=(
            "Skip episode_recorder entirely (policy motion only). "
            "Required when workstation is launched with with_recorder:=false."
        ),
    )
    parser.add_argument(
        "--record-config",
        type=str,
        default="",
        help="Override recording configuration contract YAML",
    )
    args = parser.parse_args()
    if args.rate_hz <= 0.0 or args.amplitude < 0.0 or args.period <= 0.0:
        parser.error("rate-hz and period must be positive; amplitude must be >= 0")
    if not args.hold_only and args.amplitude == 0.0:
        parser.error("amplitude 0 requires --hold-only")

    from rclpy.parameter import Parameter

    rclpy.init()
    node = rclpy.create_node(
        "fr3_pika_single_arm_rmi_client",
        parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, bool(args.use_sim_time))
        ],
    )

    try:
        with rmi.Context.from_profile(
            args.profile, node=node, timeout_sec=15.0
        ) as context:
            robot = context.robot
            homing_planner = SmoothHomingPlanner(context.profile)
            arm_home = list(homing_planner._targets["arm"])

            def policy_clock_s() -> float:
                if args.use_sim_time:
                    return node.get_clock().now().nanoseconds * 1e-9
                return time.monotonic()

            policy = CirclePolicy(
                center=arm_home,
                rate_hz=args.rate_hz,
                amplitude=args.amplitude,
                period_s=args.period,
                hold_only=args.hold_only,
                clock_now=policy_clock_s,
            )
            policy_node = context.make_node("DummyPolicy", policy)
            planner_node = context.make_node("TrajectoryPlanner", homing_planner)
            teleop_node = context.make_node("TeleopTwist")

            context.wait_until_ready(
                timeout=30.0,
                require_execution_manager=True,
                check_cameras=not args.no_cam,
            )

            print(f"\n[READY] Dummy JSPC policy active at {args.rate_hz:.1f}Hz.")
            clock_desc = (
                "/clock (phase clamped; loop paced in sim time)"
                if args.use_sim_time
                else "wall clock"
            )
            print(
                f"[READY] use_sim_time={args.use_sim_time} "
                f"(stamps + policy phase follow {clock_desc})"
            )
            peak_rate = (
                0.0
                if args.hold_only
                else args.amplitude * 2.0 * math.pi / args.period
            )
            print(
                f"[READY] policy center=homing {['%.3f' % value for value in arm_home]} "
                f"hold_only={args.hold_only} amplitude={args.amplitude} "
                f"period={args.period} peak_|q̇|≈{peak_rate:.2f} rad/s"
            )
            if not args.hold_only and peak_rate > 1.5:
                print(
                    f"[WARN] peak_|q̇|≈{peak_rate:.2f} rad/s is aggressive for MuJoCo "
                    "JSPC; raise --period or lower --amplitude."
                )
            if args.use_sim_time:
                print(
                    "[TIP] MuJoCo RTF: prefer "
                    "`pixi run rt-franka backend:=mujoco headless:=true ...` "
                    "and/or `--no-cam` for motion stress; GUI + RGB-D cameras "
                    "are the usual cause of RTF < 1."
                )
            print("[READY] Hold L1/LB for teleop; release it to resume policy.")
            print(
                f"[READY] EM sources: default={policy_node.name}, "
                f"preempt={teleop_node.name}"
            )

            record_requested = threading.Event()
            stop_requested = threading.Event()

            # Trajectory duration is sim time under MuJoCo; RMI waits on wall clock.
            # Allow slow RTF (cameras) without false TimeoutError / cancel.
            homing_timeout_s = homing_planner.duration_s + 5.0
            if args.use_sim_time:
                homing_timeout_s = max(homing_timeout_s * 10.0, 120.0)

            # Homing before recorder camera subscriptions: episode_recorder DDS
            # load can stall EM heartbeats and trip jtc_guard mid-trajectory.
            print(
                f"\n[HOMING] Executing JTC homing over "
                f"{homing_planner.duration_s:.1f}s"
                f"{' sim' if args.use_sim_time else ''} "
                f"(wall wait ≤ {homing_timeout_s:.0f}s)..."
            )
            homing_plans = homing_planner.plan(robot["manipulator"].get_observation())
            with planner_node.activate():
                planner_node["manipulator"].execute(
                    homing_plans,
                    timeout=homing_timeout_s,
                )
            print("[HOMING] JTC homing completed.")

            recorder = None
            if args.no_record:
                print("[READY] --no-record: skipping episode_recorder; policy only.")
            else:
                rec_cfg = args.record_config
                if not rec_cfg and args.no_cam:
                    rec_cfg = "apps/recording/franka_manipulation_no_cam.yaml"
                recorder = context.make_recorder(config=rec_cfg or None, autostart=True)
                try:
                    recorder.wait_ready(timeout_s=10.0)
                except Exception as exc:
                    if "cannot prepare from episode state" in str(exc):
                        print(
                            f"[RECOVER] Recovering recorder from previous state: {exc}"
                        )
                        from rmi.recording import _run_sync

                        try:
                            _run_sync(recorder._client.discard())
                        except Exception:
                            pass
                        recorder.wait_ready(timeout_s=10.0)
                    else:
                        raise

            policy_node.activate()

            def user_input_thread() -> None:
                if args.no_record:
                    input("\n[CONTROL] Policy running. Press Enter to STOP... ")
                    stop_requested.set()
                    return
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
            next_tick_wall = time.monotonic()
            next_tick_sim = policy_clock_s()

            try:
                while rclpy.ok() and not stop_requested.is_set():
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
                            "(human released clutch); rebasing motion at current pose"
                        )
                        intervention_active = False
                        policy.reset_center(observation)

                    action = policy.select_action(observation)
                    policy_node["arm"].submit(action)

                    if (
                        recorder is not None
                        and record_requested.is_set()
                        and episode_scope is None
                    ):
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

                    # Pace in the same time base as phase/stamps so RTF!=1 does
                    # not compress the DummyPolicy trajectory into sim time.
                    if args.use_sim_time:
                        next_tick_sim += period_s
                        while rclpy.ok() and not stop_requested.is_set():
                            now_sim = policy_clock_s()
                            if now_sim >= next_tick_sim:
                                if now_sim - next_tick_sim > period_s:
                                    next_tick_sim = now_sim
                                break
                            time.sleep(0.001)
                    else:
                        next_tick_wall += period_s
                        sleep_s = next_tick_wall - time.monotonic()
                        if sleep_s > 0.0:
                            time.sleep(sleep_s)
                        else:
                            next_tick_wall = time.monotonic()
            finally:
                if episode_scope is not None:
                    episode_scope.__exit__(None, None, None)
                    episode_path = getattr(episode_scope.final_status, "episode_path", "")
                    if episode_scope.validated:
                        print(f"\n[VALID] Episode passed validation: {episode_path}")
                    else:
                        print(f"\n[SAVED] Episode saved to {episode_path}")
    except KeyboardInterrupt:
        print("\n[STOP] Example interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
