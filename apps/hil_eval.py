"""HIL / teleop-intervention evaluation loop (experimental).

Standard policy-only eval lives in ``apps/eval.py``. This entrypoint adds:

- EM authority shadow tracing while teleop owns the lease
- HandoverBlender when teleop releases back to policy
- Optional HIL-SAC LearnerService transition streaming / weight hot-reload

Not required for ordinary VLA / FastWAM rollouts.
"""

from __future__ import annotations

import argparse
import os
import time
from typing import Any

import numpy as np

from dataclasses import replace

import rmi
from rmi.errors import SourceAuthorityError
from rmi import ros_image_to_numpy

from policy_inference.lerobot import LeRobotPolicy
from policy_inference.lerobot.bridges import HILTransitionCollector
from policy_inference.lerobot.engines import LeRobotLearnerClient
from policy_inference.lerobot.geometry import normalize_quat_wxyz


def _nlerp_quat(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
    res = (1.0 - t) * q0 + t * q1
    norm = float(np.linalg.norm(res))
    return (res / norm) if norm > 1e-10 else q1


class HandoverBlender:
    """Blend from the live handover state into the newly planned policy chunk."""

    def __init__(self, blend_steps: int = 10) -> None:
        self.blend_steps = max(1, blend_steps)
        self._active = False
        self._current_step = 0
        self._anchor_pos: np.ndarray | None = None
        self._anchor_quat: np.ndarray | None = None
        self._anchor_joints: dict[str, float] = {}

    def trigger(self, observation: Any, layout: Any) -> None:
        self._active = True
        self._current_step = 0
        if layout.pose_part and layout.pose_part in observation.sensors:
            tcp = observation.sensors[layout.pose_part].value
            self._anchor_pos = np.asarray(tcp.position_xyz, dtype=np.float64).copy()
            self._anchor_quat = normalize_quat_wxyz(tcp.orientation_wxyz)
        else:
            self._anchor_pos = None
            self._anchor_quat = None
        if observation.joint_names and observation.joint_positions:
            self._anchor_joints = dict(
                zip(observation.joint_names, map(float, observation.joint_positions))
            )
        else:
            self._anchor_joints = {}

    @property
    def is_blending(self) -> bool:
        return self._active

    def blend(self, actions: Any, layout: Any) -> Any:
        if not self._active or actions is None:
            return actions

        self._current_step += 1
        alpha = min(1.0, float(self._current_step) / float(self.blend_steps))
        blended: list[rmi.Action] = []
        for act in actions:
            if (
                act.command == "pose_reference"
                and self._anchor_pos is not None
                and self._anchor_quat is not None
            ):
                tgt_pos = np.asarray(act.value["position"], dtype=np.float64)
                tgt_quat = normalize_quat_wxyz(act.value["orientation"])
                b_pos = (1.0 - alpha) * self._anchor_pos + alpha * tgt_pos
                b_quat = _nlerp_quat(self._anchor_quat, tgt_quat, alpha)
                blended.append(
                    replace(
                        act,
                        value={
                            "position": b_pos.tolist(),
                            "orientation": b_quat.tolist(),
                        },
                    )
                )
            elif act.command == "joint_reference" and self._anchor_joints:
                tgt_val = np.asarray(act.value, dtype=np.float64)
                group = next(
                    (g for g in layout.joints.groups if g.part == act.part), None
                )
                joint_names = (
                    getattr(group, "joint_names", None) if group is not None else None
                )
                if joint_names is not None and len(joint_names) == len(tgt_val):
                    curr_val = np.array(
                        [self._anchor_joints.get(j, 0.0) for j in joint_names],
                        dtype=np.float64,
                    )
                    b_val = (1.0 - alpha) * curr_val + alpha * tgt_val
                    blended.append(
                        replace(act, value=b_val)
                    )
                else:
                    blended.append(act)
            else:
                blended.append(act)

        if self._current_step >= self.blend_steps:
            self._active = False
        return blended


def _encode_obs(policy: LeRobotPolicy, observation: Any) -> dict[str, Any]:
    return policy._encoder.encode(observation)


def _require_consistent_tf_env() -> None:
    import rclpy
    import tf2_ros

    def _prefix(path: str) -> str:
        marker = "/lib/python"
        return path.split(marker, 1)[0] if marker in path else path

    rclpy_prefix = _prefix(rclpy.__file__ or "")
    tf2_prefix = _prefix(tf2_ros.__file__ or "")
    if rclpy_prefix != tf2_prefix:
        raise SystemExit(
            "ROS env mismatch: rclpy and tf2_ros come from different prefixes.\n"
            f"  rclpy:   {rclpy.__file__}\n"
            f"  tf2_ros: {tf2_ros.__file__}\n"
            "Use a single Pixi env and source install/setup.bash."
        )


def _parse_rename_map(items: list[str] | None) -> dict[str, str] | None:
    if not items:
        return None
    mapping: dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(
                f"rename map entry must be profile=checkpoint, got {item!r}"
            )
        src, dst = item.split("=", 1)
        src, dst = src.strip(), dst.strip()
        if not src or not dst:
            raise ValueError(f"rename map entry is empty: {item!r}")
        mapping[src] = dst
    return mapping


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RMI LeRobot HIL / teleop-intervention evaluation"
    )
    parser.add_argument("--profile", required=True)
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--task", required=True)
    parser.add_argument("--resource", default="manipulator")
    parser.add_argument("--node-name", default="CartesianPolicy")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--policy-type", default=None)
    parser.add_argument(
        "--inference",
        choices=("sync", "async", "rtc", "remote"),
        default="sync",
    )
    parser.add_argument(
        "--remote-address",
        default=os.environ.get("LEROBOT_SERVER_ADDRESS", "localhost:50051"),
    )
    parser.add_argument("--rtc-queue-threshold", type=int, default=30)
    parser.add_argument(
        "--learner-address",
        default=None,
        help="gRPC LearnerService address for HIL-SAC (required for transition sync)",
    )
    parser.add_argument("--hil-sync-interval", type=int, default=50)
    parser.add_argument("--hil-buffer-size", type=int, default=1000)
    parser.add_argument("--max-stream-skew", type=float, default=0.5)
    parser.add_argument("--action-space", choices=("abs", "rel"), default=None)
    parser.add_argument("--rename-map", action="append", default=None)
    parser.add_argument("--gripper-max-width", type=float, default=0.045)
    parser.add_argument("--position-scale", type=float, default=0.05)
    parser.add_argument("--orientation-scale", type=float, default=0.5)
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--handover-blend-steps", type=int, default=10)
    parser.add_argument("--preempt", action="store_true")
    args = parser.parse_args()
    if args.inference in ("sync", "async", "rtc") and not args.checkpoint:
        parser.error(f"--checkpoint is required when --inference is {args.inference}")
    return args


def _open_context(profile: str, *, use_sim_time: bool):
    if not use_sim_time:
        return rmi.Context.from_profile(profile)
    import rclpy
    from rclpy.parameter import Parameter

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("rmi_hil_eval")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    return rmi.Context.from_profile(profile, node=node)


def _setup(
    context: Any,
    args: argparse.Namespace,
    rename_map: dict[str, str] | None,
) -> tuple[Any, LeRobotPolicy, Any, LeRobotLearnerClient | None, HILTransitionCollector | None]:
    layout = context.profile.policy_layout(args.node_name)
    print(
        f"[hil_eval] node={args.node_name} control_mode={layout.control_mode} "
        f"action_space={args.action_space or layout.action_space} "
        f"action_dim={layout.action_dimension}"
    )
    for camera_name in dict.fromkeys(layout.camera_sources.values()):
        context.make_camera(
            camera_name, converter=ros_image_to_numpy, history_size=1
        )
    if layout.control_mode == "cartesian":
        _require_consistent_tf_env()
        context.make_tcp_pose(str(layout.pose_part))
        print(f"[hil_eval] TCP pose sensor: sensors[{layout.pose_part!r}]")

    common = dict(
        task=args.task,
        device=args.device,
        expected_policy_type=args.policy_type,
        max_stream_skew_s=args.max_stream_skew,
        rename_map=rename_map,
        action_space=args.action_space,
        gripper_max_width=args.gripper_max_width,
        position_scale=args.position_scale,
        orientation_scale=args.orientation_scale,
    )
    if args.inference == "remote":
        policy = LeRobotPolicy(
            layout,
            checkpoint=args.checkpoint,
            inference="remote",
            remote_grpc_address=args.remote_address,
            **common,
        )
    else:
        policy = LeRobotPolicy(
            layout,
            checkpoint=args.checkpoint,
            inference=args.inference,
            rtc_queue_threshold=args.rtc_queue_threshold,
            **common,
        )
    print(f"[hil_eval] inference={policy.inference}")
    policy_node = context.make_node(args.node_name, policy)

    learner_client: LeRobotLearnerClient | None = None
    hil_collector: HILTransitionCollector | None = None
    if args.learner_address:
        print(f"[hil_eval] LearnerService at {args.learner_address}")
        learner_client = LeRobotLearnerClient(args.learner_address)
        learner_client.connect()
        hil_collector = HILTransitionCollector(max_buffer_size=args.hil_buffer_size)
    return layout, policy, policy_node, learner_client, hil_collector


def _log_step(
    *,
    step: int,
    layout: Any,
    policy: LeRobotPolicy,
    observation: Any,
    actions: Any,
    prev_target: list[float] | None,
    is_intervened: bool = False,
) -> list[float] | None:
    raw = policy.last_raw_action
    log_this = (
        step <= 8
        or policy.last_was_replan
        or step % max(1, int(layout.frequency)) == 0
    )
    tag = "[infer (shadow)]" if is_intervened else "[infer]"
    if not log_this:
        return prev_target
    if raw is None:
        print(f"{tag} step={step} raw=None")
        return prev_target
    dxyz = raw[:3]
    dnorm = float(sum(v * v for v in dxyz) ** 0.5)
    raw_s = ",".join(f"{v:.4f}" for v in raw.tolist())
    print(
        f"{tag} step={step} replan={policy.last_was_replan} "
        f"chunk_i={policy.last_chunk_index} q_left={policy.last_queue_remaining} "
        f"||dxyz||={dnorm:.5f} raw=[{raw_s}]"
    )
    pose_part = layout.pose_part
    if pose_part and actions is not None and pose_part in observation.sensors:
        tcp = observation.sensors[pose_part].value.position_xyz
        pose_act = next((a for a in actions if a.command == "pose_reference"), None)
        if pose_act is not None:
            tgt = pose_act.value["position"]
            err_n = float(sum((tgt[i] - tcp[i]) ** 2 for i in range(3)) ** 0.5)
            print(
                f"         tcp=[{tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}] "
                f"tgt=[{tgt[0]:.4f},{tgt[1]:.4f},{tgt[2]:.4f}] "
                f"||tgt-tcp||={err_n:.5f}"
            )
            prev_target = list(tgt)
    return prev_target


def _run_loop(
    context: Any,
    *,
    resource: str,
    layout: Any,
    policy: LeRobotPolicy,
    policy_node: Any,
    preempt: bool,
    learner_client: LeRobotLearnerClient | None = None,
    hil_collector: HILTransitionCollector | None = None,
    hil_sync_interval: int = 50,
    handover_blend_steps: int = 10,
) -> None:
    period_s = 1.0 / layout.frequency
    next_tick = time.monotonic()
    context.wait_until_ready(
        timeout=30.0,
        check_cameras=bool(layout.camera_sources),
        require_execution_manager=True,
    )
    if hil_collector is not None:
        try:
            init_obs = context.robot[resource].get_observation()
            hil_collector.start_episode(_encode_obs(policy, init_obs))
        except Exception as exc:
            print(f"[hil] warning encoding initial observation: {exc}")

    blender = HandoverBlender(blend_steps=handover_blend_steps)
    was_active = True
    with policy_node.activate(preempt=preempt):
        step = 0
        prev_target: list[float] | None = None
        while True:
            next_tick += period_s
            observation = context.robot[resource].get_observation()
            step += 1
            is_active = policy_node.has_control
            is_intervened = not is_active

            if was_active and not is_active:
                print(f"[hil_eval] step={step} teleop preempted policy (shadow)")
                was_active = False
            elif not was_active and is_active:
                print(
                    f"[hil_eval] step={step} teleop released; reset + handover blend "
                    f"({blender.blend_steps} steps)"
                )
                try:
                    policy.reset()
                except Exception as exc:
                    print(f"[hil_eval] warning reset: {exc}")
                blender.trigger(observation, layout)
                was_active = True

            try:
                actions = (policy_node.select_action(observation) if is_active
                           else policy.select_action(observation))
            except SourceAuthorityError:
                actions = None
            if actions is not None and blender.is_blending:
                actions = blender.blend(actions, layout)
            if actions is not None and is_active:
                try:
                    policy_node.submit(actions)
                except SourceAuthorityError:
                    pass

            prev_target = _log_step(
                step=step,
                layout=layout,
                policy=policy,
                observation=observation,
                actions=actions,
                prev_target=prev_target,
                is_intervened=is_intervened,
            )

            if hil_collector is not None and learner_client is not None:
                raw_act = policy.last_raw_action
                if raw_act is not None:
                    try:
                        hil_collector.record_step(
                            executed_action=raw_act,
                            reward=0.0,
                            next_state=_encode_obs(policy, observation),
                            done=False,
                            is_intervened=is_intervened,
                            info={
                                "policy_action": raw_act,
                                "authority": "teleop" if is_intervened else "policy",
                            },
                        )
                    except Exception as exc:
                        print(f"[hil] encode error: {exc}")
                if step % hil_sync_interval == 0:
                    transitions = hil_collector.flush()
                    if transitions:
                        try:
                            learner_client.send_transitions(transitions)
                            print(
                                f"[hil] streamed {len(transitions)} transitions "
                                f"(step {step})"
                            )
                        except Exception as exc:
                            print(f"[hil] send error: {exc}")
                    try:
                        new_weights = learner_client.fetch_policy_parameters()
                        if new_weights:
                            policy.update_weights(new_weights)
                            print(f"[hil] hot-reloaded weights (step {step})")
                    except Exception as exc:
                        print(f"[hil] parameter sync failed: {exc}")

            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()


def main() -> None:
    args = parse_args()
    rename_map = _parse_rename_map(args.rename_map)
    with _open_context(args.profile, use_sim_time=args.use_sim_time) as context:
        print(f"[hil_eval] use_sim_time={args.use_sim_time}")
        layout, policy, policy_node, learner_client, hil_collector = _setup(
            context, args, rename_map
        )
        try:
            _run_loop(
                context,
                resource=args.resource,
                layout=layout,
                policy=policy,
                policy_node=policy_node,
                preempt=args.preempt,
                learner_client=learner_client,
                hil_collector=hil_collector,
                hil_sync_interval=args.hil_sync_interval,
                handover_blend_steps=args.handover_blend_steps,
            )
        except KeyboardInterrupt:
            pass
        finally:
            if hil_collector is not None and learner_client is not None:
                remaining = hil_collector.flush()
                if remaining:
                    try:
                        learner_client.send_transitions(remaining)
                    except Exception as exc:
                        print(f"[hil] flush error: {exc}")
                learner_client.close()
            policy.close()


if __name__ == "__main__":
    main()
