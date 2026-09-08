"""Evaluate a LeRobot policy through the standard RMI application loop.

```text
context.make_camera(...)
context.make_tcp_pose(pose_part)          # cartesian: TF is mandatory
obs = robot[resource].get_observation()
actions = node.select_action(obs)
node.submit(actions)
```

Teleop handback joins the latest shadow chunk from measured state under the new lease.
For HIL-SAC collection and learning, see ``apps/hil_eval.py``.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any

_SRC_DIR = str(Path(__file__).resolve().parent.parent / 'src')
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

import rmi
from rmi import ros_image_to_numpy
from rmi.errors import SourceAuthorityError

from policy_inference.lerobot import LeRobotPolicy
from policy_inference.lerobot.handover import ChunkHandover


def _require_consistent_tf_env() -> None:
    """Fail fast when tf2_ros / rclpy come from different Pixi prefixes."""
    import rclpy
    import tf2_ros

    def _prefix(path: str) -> str:
        marker = '/lib/python'
        return path.split(marker, 1)[0] if marker in path else path

    rclpy_prefix = _prefix(rclpy.__file__ or '')
    tf2_prefix = _prefix(tf2_ros.__file__ or '')
    if rclpy_prefix != tf2_prefix:
        raise SystemExit(
            'ROS env mismatch: rclpy and tf2_ros come from different prefixes.\n'
            f'  rclpy:   {rclpy.__file__}\n'
            f'  tf2_ros: {tf2_ros.__file__}\n'
            'Use a single Pixi env and source the workspace overlay:\n'
            '  pixi install && pixi shell\n'
            '  source install/setup.bash\n'
            '  python apps/eval.py --profile fr3_pika_single_arm.yaml '
            "--checkpoint /path/to/pretrained_model --task '…'\n"
            "Do not import tf2_ros from another env's site-packages."
        )


def _parse_rename_map(items: list[str] | None) -> dict[str, str] | None:
    if not items:
        return None
    mapping: dict[str, str] = {}
    for item in items:
        if '=' not in item:
            raise ValueError(f'rename map entry must be profile=checkpoint, got {item!r}')
        src, dst = item.split('=', 1)
        src, dst = src.strip(), dst.strip()
        if not src or not dst:
            raise ValueError(f'rename map entry is empty: {item!r}')
        mapping[src] = dst
    return mapping


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='RMI LeRobot policy evaluation')
    parser.add_argument('--profile', required=True)
    parser.add_argument(
        '--checkpoint',
        default=None,
        help='LeRobot pretrained_model path (required for local sync/async; optional for remote/hold)',
    )
    parser.add_argument('--task', required=True, help='Task / language goal string')
    parser.add_argument(
        '--resource',
        default='manipulator',
        help='Robot observation resource / compound group',
    )
    parser.add_argument(
        '--node-name',
        default='CartesianPolicy',
        help='Profile POLICY node: JointPolicy, CartesianPolicy, or Policy',
    )
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--policy-type', default=None)
    parser.add_argument(
        '--hold',
        action='store_true',
        help=(
            'Hold current joint positions (no checkpoint). For joint-layout '
            'nodes only; validates EM claim and controller routing.'
        ),
    )
    parser.add_argument(
        '--inference',
        choices=('sync', 'async', 'rtc', 'remote'),
        default='sync',
        help=(
            'Inference backend: sync / async(rtc) = local LeRobot engines; '
            'remote = LeRobot gRPC policy_server'
        ),
    )
    parser.add_argument(
        '--remote-address',
        default=os.environ.get('LEROBOT_SERVER_ADDRESS', 'localhost:50051'),
        help='gRPC address of remote LeRobot policy_server (default: localhost:50051)',
    )
    parser.add_argument(
        '--rtc-queue-threshold',
        type=int,
        default=30,
        help='Action queue threshold for RTC async engine (default: 30)',
    )
    parser.add_argument('--max-stream-skew', type=float, default=0.5)
    parser.add_argument(
        '--action-space',
        choices=('abs', 'rel'),
        default=None,
        help='Override features.action.space; rel integrates against live TCP pose',
    )
    parser.add_argument(
        '--rename-map',
        action='append',
        default=None,
        help='Camera feature rename profile=checkpoint (repeatable)',
    )
    parser.add_argument(
        '--gripper-max-width',
        type=float,
        default=0.045,
        help=(
            'Gripper full-open width in meters. Cartesian LIBERO: map [-1,1]→'
            '[max,0]. Joint + --normalize-gripper: map Aloha/RoboTwin [0,1]↔'
            '[0,max] (Piper native gripper uses 0.04).'
        ),
    )
    parser.add_argument(
        '--normalize-gripper',
        action='store_true',
        help=(
            'For joint policies: scale gripper obs meters→[0,1] and action '
            '[0,1]→meters using --gripper-max-width (RoboTwin/Aloha pi05).'
        ),
    )
    parser.add_argument(
        '--position-scale',
        type=float,
        default=0.05,
        help=(
            'Scale unnormalized EE xyz (LIBERO ~[-1,1] controller units) to meters '
            'before integrating; 0.05 ≈ OSC max translation'
        ),
    )
    parser.add_argument(
        '--orientation-scale',
        type=float,
        default=0.5,
        help='Scale unnormalized axis-angle before integrating (default 0.5)',
    )
    parser.add_argument(
        '--use-sim-time',
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            'Stamp commands with /clock (required when workstation uses sim time). '
            'Wall-clock stamps are dropped by EM as future_command.'
        ),
    )
    parser.add_argument(
        '--handover-duration',
        type=float,
        default=1.0,
        help='Seconds for the transition into the selected shadow chunk point.',
    )
    parser.add_argument(
        '--handover-max-age',
        type=float,
        default=2.0,
        help='Maximum age in seconds of the chunk observation used for joining.',
    )
    parser.add_argument(
        '--episodes',
        type=int,
        default=1,
        help='Number of evaluation episodes to run (default: 1)',
    )
    parser.add_argument(
        '--max-steps',
        type=int,
        default=300,
        help='Maximum steps per episode before truncation (default: 300)',
    )
    parser.add_argument(
        '--reset-mode',
        choices=('auto', 'sim', 'homing', 'interactive', 'none'),
        default='auto',
        help='Reset strategy between episodes: auto, sim (MuJoCo ResetWorld), homing, interactive, or none',
    )
    parser.add_argument(
        '--keyframe',
        type=int,
        default=None,
        help='Optional keyframe ID to reset to in MuJoCo simulation',
    )
    parser.add_argument(
        '--recover-faults',
        action='store_true',
        help=(
            'Clear leftover EM FAULT from a previous eval/source drop '
            '(Ctrl+C, crashed client). Required after '
            'external_source_lost_requires_explicit_recovery; not the default.'
        ),
    )
    args = parser.parse_args()
    if args.hold:
        return args
    if args.inference in ('sync', 'async', 'rtc') and not args.checkpoint:
        parser.error(f'--checkpoint is required when --inference is {args.inference}')
    return args


class _HoldJointPolicy:
    """Hold measured joints; used for EM / controller smoke without a checkpoint."""

    inference = 'hold'
    last_raw_action = None
    last_was_replan = False
    last_chunk_index = -1
    last_queue_remaining = 0
    last_chunk_raw = None
    prev_chunk0_raw = None

    def __init__(self, layout: Any) -> None:
        if layout.control_mode != 'joint':
            raise SystemExit('--hold requires a joint-layout POLICY node')
        from policy_inference.lerobot.bridges import JointActionDecoder

        self._decoder = JointActionDecoder(layout)
        self._held: Any = None
        self._last_actions = None

    def reset(self) -> None:
        self._held = None
        self._last_actions = None
        self.last_raw_action = None
        self.last_was_replan = False

    def select_action(self, observation: Any) -> Any:
        import numpy as np

        positions = observation.data.get('joint_positions')
        if not positions:
            return None
        values = np.asarray(positions, dtype=np.float64).reshape(-1)
        if self._held is None:
            self._held = values.copy()
            self.last_was_replan = True
        else:
            self.last_was_replan = False
        self.last_raw_action = self._held
        self._last_actions = self._decoder.decode(self._held)
        return self._last_actions

    def take_action_chunk(self):
        chunk = () if self._last_actions is None else (self._last_actions,)
        self._last_actions = None
        return chunk

    def observe(self, observation):
        pass

    def close(self) -> None:
        pass


def _open_context(profile: str, *, use_sim_time: bool):
    """Build RMI Context; optionally pin the node clock to /clock."""
    if not use_sim_time:
        return rmi.Context.from_profile(profile)

    import rclpy
    from rclpy.parameter import Parameter

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('rmi_eval')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    return rmi.Context.from_profile(profile, node=node)


def _setup(
    context: Any,
    args: argparse.Namespace,
    rename_map: dict[str, str] | None,
) -> tuple[Any, Any, Any]:
    layout = context.profile.policy_layout(args.node_name)
    action_space = args.action_space
    if action_space is None and layout.control_mode == 'cartesian':
        # Profile features.action.space is shared with joint layouts (often abs).
        # LIBERO / pi05 cartesian checkpoints are EE deltas around the live TCP.
        action_space = 'rel'
        print(
            '[eval] cartesian default action_space=rel '
            '(profile has abs; pass --action-space abs to override)',
            flush=True,
        )
    print(
        f'[eval] node={args.node_name} control_mode={layout.control_mode} '
        f'action_space={action_space or layout.action_space} '
        f'action_dim={layout.action_dimension} '
        f'position_scale={args.position_scale} '
        f'orientation_scale={args.orientation_scale} '
        f'normalize_gripper={args.normalize_gripper} '
        f'gripper_max_width={args.gripper_max_width}'
    )
    for camera_name in dict.fromkeys(layout.camera_sources.values()):
        context.make_camera(
            camera_name,
            converter=ros_image_to_numpy,
            history_size=1,
        )
    if layout.control_mode == 'cartesian':
        _require_consistent_tf_env()
        context.make_tcp_pose(str(layout.pose_part))
        print(f'[eval] TCP pose sensor: sensors[{layout.pose_part!r}]')

    if args.hold:
        print('[eval] hold mode (no neural checkpoint)')
        policy = _HoldJointPolicy(layout)
        policy_node = context.make_node(args.node_name, policy)
        return layout, policy, policy_node

    if args.inference == 'remote':
        print(f'[eval] remote policy_server at {args.remote_address}')
        policy = LeRobotPolicy(
            layout,
            checkpoint=args.checkpoint,
            task=args.task,
            device=args.device,
            expected_policy_type=args.policy_type,
            max_stream_skew_s=args.max_stream_skew,
            rename_map=rename_map,
            action_space=action_space,
            gripper_max_width=args.gripper_max_width,
            normalize_gripper=args.normalize_gripper,
            position_scale=args.position_scale,
            orientation_scale=args.orientation_scale,
            inference='remote',
            remote_grpc_address=args.remote_address,
        )
    else:
        policy = LeRobotPolicy(
            layout,
            checkpoint=args.checkpoint,
            task=args.task,
            device=args.device,
            expected_policy_type=args.policy_type,
            max_stream_skew_s=args.max_stream_skew,
            rename_map=rename_map,
            action_space=action_space,
            gripper_max_width=args.gripper_max_width,
            normalize_gripper=args.normalize_gripper,
            position_scale=args.position_scale,
            orientation_scale=args.orientation_scale,
            inference=args.inference,
            rtc_queue_threshold=args.rtc_queue_threshold,
        )
    print(f'[eval] inference={policy.inference}')
    policy_node = context.make_node(args.node_name, policy)
    return layout, policy, policy_node


def _log_step(
    *,
    step: int,
    layout: Any,
    policy: LeRobotPolicy,
    observation: Any,
    actions: Any,
    prev_target: list[float] | None,
) -> list[float] | None:
    raw = policy.last_raw_action
    log_this = step <= 8 or policy.last_was_replan or step % max(1, int(layout.frequency)) == 0
    if not log_this:
        return prev_target

    if raw is None:
        print(f'[infer] step={step} raw=None (no submit)', flush=True)
        return prev_target

    dxyz = raw[:3]
    dnorm = float(sum(v * v for v in dxyz) ** 0.5)
    raw_s = ','.join(f'{v:.4f}' for v in raw.tolist())
    print(
        f'[infer] step={step} replan={policy.last_was_replan} '
        f'chunk_i={policy.last_chunk_index} '
        f'q_left={policy.last_queue_remaining} '
        f'||dxyz||={dnorm:.5f} raw=[{raw_s}]',
        flush=True,
    )
    if policy.last_was_replan and policy.last_chunk_raw is not None:
        norms = [
            float(sum(float(v) ** 2 for v in row[:3]) ** 0.5) for row in policy.last_chunk_raw
        ]
        print(
            f'         chunk||dxyz||={[f"{n:.5f}" for n in norms]} '
            f'(idx0 is postprocessed; rest may be pre-post)',
            flush=True,
        )
        if policy.prev_chunk0_raw is not None:
            prev = policy.prev_chunk0_raw
            dd = raw - prev
            print(
                f'         ||chunk0-prev_chunk0||='
                f'{float(sum(float(v) ** 2 for v in dd) ** 0.5):.5f} '
                f'dxyz={[f"{v:.4f}" for v in dd[:3].tolist()]}',
                flush=True,
            )

    pose_part = layout.pose_part
    if pose_part and actions is not None and pose_part in observation.sensors:
        tcp = observation.sensors[pose_part].value.position_xyz
        pose_act = next(
            (a for a in actions if a.command == 'pose_reference'),
            None,
        )
        if pose_act is not None:
            tgt = pose_act.value['position']
            err = [tgt[i] - tcp[i] for i in range(3)]
            err_n = float(sum(v * v for v in err) ** 0.5)
            jump = None
            if prev_target is not None:
                jump = float(sum((tgt[i] - prev_target[i]) ** 2 for i in range(3)) ** 0.5)
            print(
                f'         tcp=[{tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}] '
                f'tgt=[{tgt[0]:.4f},{tgt[1]:.4f},{tgt[2]:.4f}] '
                f'||tgt-tcp||={err_n:.5f}'
                + (f' ||tgt-prev_tgt||={jump:.5f}' if jump is not None else ''),
                flush=True,
            )
            prev_target = list(tgt)

    if actions is not None:
        for act in actions:
            if act.command != 'pose_reference':
                print(f'         submit {act.part}.{act.command}={act.value}')
    return prev_target


def _teleop_nodes(context: Any) -> list[Any]:
    """Mirror examples 16/17: watch TELEOP nodes for clutch/preempt intervention."""
    nodes: list[Any] = []
    for name, cfg in context.profile.nodes.items():
        if str(getattr(cfg, 'source_role', '')).upper() != 'TELEOP':
            continue
        nodes.append(context.make_node(name))
    return nodes


class _EvalPolicy:
    """Shadow inference and chunk playback have independent consumption cursors."""

    def __init__(
        self,
        policy: Any,
        layout: Any,
        clock: Any,
        *,
        duration=1.0,
        max_age=2.0,
        age_clock=None,
    ):
        import math

        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('handover max age must be finite and positive')
        self.policy, self.layout, self.clock = policy, layout, clock
        self.age_clock = age_clock or clock
        self.duration, self.max_age = duration, max_age
        self.handover = ChunkHandover(layout, duration=duration)
        self._need_join = True
        self._join_fail = 'no_chunk'
        self._chunk_at = None
        self._playing_chunk_at = None

    def reset(self):
        self.policy.reset()  # Episode boundary only.
        self.handover = ChunkHandover(self.layout, duration=self.duration)
        self._chunk_at = self._playing_chunk_at = None
        self._need_join = True
        self._join_fail = 'no_chunk'

    def _infer(self, observation):
        started = self.age_clock()
        actions = self.policy.select_action(observation)
        if actions is not None and (self._chunk_at is None or self.policy.last_was_replan):
            self._chunk_at = started
        return actions

    def shadow_step(self, observation):
        self.handover.observe(observation, self.clock())
        return self._infer(observation)

    def on_control_acquired(self, observation):
        # No inference/reset here. RMI calls this before binding new commands.
        self._need_join = True

    def _start_join(self, observation):
        now = self.clock()
        frames = self.policy.take_action_chunk()
        chunk_at = self._chunk_at
        if not frames and self.handover.frames:
            # A takeover may happen entirely between polling ticks while a
            # transferred chunk is playing. Its unleased remainder is reusable.
            start = max(self.handover.join_index, self.handover.cursor - 1)
            frames = self.handover.frames[start:]
            chunk_at = self._playing_chunk_at
        if not frames:
            self._join_fail = 'no_chunk'
            return False
        if chunk_at is None:
            self._join_fail = 'no_chunk_time'
            return False
        age = self.age_clock() - chunk_at
        if not 0 <= age <= self.max_age:
            self._join_fail = f'stale_chunk age={age:.3f}s'
            return False
        if not self.handover.start(frames, observation, now):
            self._join_fail = self.handover.reason
            return False
        self._playing_chunk_at = chunk_at
        self._need_join = False
        print(
            f'[eval] chunk handover: join={self.handover.join_index}/{len(frames) - 1}, '
            f'duration={self.handover.bridge_duration:.2f}s, age={age:.3f}s',
            flush=True,
        )
        return True

    def select_action(self, observation):
        self.handover.observe(observation, self.clock())
        if self._need_join and not self._start_join(observation):
            # Missing/stale/infeasible prior chunk: infer once and try to join
            # that result. If join still fails, submit the inferred head —
            # dropping it leaves the cloud inferring while the robot holds.
            self.handover.frames = ()
            self.handover.started = None
            self._chunk_at = None
            actions = self._infer(observation)
            if self._start_join(observation):
                pass
            else:
                self._need_join = False
                if actions is not None:
                    print(
                        f'[eval] chunk handover skipped ({self._join_fail}); '
                        'submitting inferred action',
                        flush=True,
                    )
                return actions
        if self.handover.started is not None:
            self.policy.observe(observation)  # Feed RTC without popping actions.
            actions = self.handover.select_action(self.clock())
            if actions is not None:
                return actions
        return self._infer(observation)


def _run_loop(
    context: Any,
    *,
    resource: str,
    layout: Any,
    policy: Any,
    policy_node: Any,
    check_cameras: bool,
    handover_duration: float = 1.0,
    handover_max_age: float = 2.0,
    use_sim_time: bool = False,
    episodes: int = 1,
    max_steps: int = 300,
    reset_mode: str = 'auto',
    keyframe: int | None = None,
    recover_faults: bool = False,
) -> None:
    period_s = 1.0 / layout.frequency
    if recover_faults:
        print('[eval] recovering leftover EM FAULT (previous source drop)...', flush=True)
    context.wait_until_ready(
        timeout=30.0,
        check_cameras=check_cameras,
        require_execution_manager=True,
        recover_faults=recover_faults,
    )

    teleop_nodes = _teleop_nodes(context)
    print(
        f'[eval] EM sources: default={policy_node.name}, '
        f'preempt={[node.name for node in teleop_nodes] or "(none)"}'
    )
    print(
        '[eval] Policy remains in the candidate pool and runs in shadow while preempted. '
        'New leases join the latest usable shadow chunk; old commands are never relabelled.'
    )

    def clock_now_s() -> float:
        if use_sim_time:
            return context.node.get_clock().now().nanoseconds * 1e-9
        return time.monotonic()

    # Like examples 16/17: one activation for the whole session.
    producer = _EvalPolicy(
        policy,
        layout,
        clock_now_s,
        duration=handover_duration,
        max_age=handover_max_age,
        age_clock=time.monotonic,
    )
    original_producer = policy_node.producer
    policy_node.producer = producer
    activation = None
    episode_results: list[dict[str, Any]] = []
    try:
        for ep in range(1, episodes + 1):
            print(f'\n[eval] ========== Starting Episode {ep}/{episodes} ==========', flush=True)
            if reset_mode != 'none':
                print(
                    f'[eval] Resetting scene/embodiment (mode={reset_mode})...',
                    flush=True,
                )
                reset_ok = context.reset(mode=reset_mode, keyframe=keyframe)
                if not reset_ok:
                    print(
                        '[eval] WARNING: Reset handler reported failure or timed out.',
                        flush=True,
                    )
                else:
                    print('[eval] Reset finished.', flush=True)
            if activation is None:
                activation = policy_node.activate()
                print(f'[eval] {policy_node.name} activated.', flush=True)

            producer.reset()
            print('[eval] Control loop started (waiting for first submitted action).', flush=True)

            step = 0
            empty_polls = 0
            prev_target: list[float] | None = None
            next_tick_wall = time.monotonic()
            next_tick_sim = clock_now_s()
            intervention_active = False

            while step < max_steps:
                active_teleops = [node for node in teleop_nodes if node.has_control]
                if active_teleops and not intervention_active:
                    names = ','.join(node.name for node in active_teleops)
                    print(
                        f'\n[eval] INTERVENTION: {names} took over control '
                        '(teleop preempt active); EM gates Policy'
                    )
                    intervention_active = True
                actions = None
                controlling = policy_node.has_control
                if controlling:
                    try:
                        # Fetch only after observing control; RMI captures the lease
                        # before preparing the transition or the next chunk reference.
                        observation = context.robot[resource].get_observation()
                        actions = policy_node.select_action(observation)
                        if actions is not None:
                            policy_node.submit(actions)
                            step += 1
                            if intervention_active:
                                print(
                                    f'\n[eval] RESUMED: {policy_node.name} joined its shadow chunk from current state'
                                )
                                intervention_active = False
                                prev_target = None
                    except SourceAuthorityError:
                        actions = None
                else:
                    observation = context.robot[resource].get_observation()
                    actions = producer.shadow_step(observation)  # Shadow only; never submit.
                if actions is None and step == 0:
                    empty_polls += 1
                    if empty_polls in {1, 5, 15} or empty_polls % 30 == 0:
                        print(
                            f'[eval] still waiting for first action '
                            f'(has_control={controlling} polls={empty_polls})',
                            flush=True,
                        )
                if actions is not None:
                    prev_target = _log_step(
                        step=step,
                        layout=layout,
                        policy=policy,
                        observation=observation,
                        actions=actions,
                        prev_target=prev_target,
                    )

                if use_sim_time:
                    next_tick_sim += period_s
                    while True:
                        now_sim = clock_now_s()
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

            outcome = 'truncated' if step >= max_steps else 'completed'
            print(f'[eval] Episode {ep} finished: {outcome} ({step}/{max_steps} steps)')
            episode_results.append({'episode': ep, 'steps': step, 'outcome': outcome})
    finally:
        try:
            if activation is not None:
                activation.close()
        finally:
            policy_node.producer = original_producer

    print('\n[eval] ================= Evaluation Summary =================')
    print(f'[eval] Total episodes: {len(episode_results)}')
    avg_steps = sum(r['steps'] for r in episode_results) / max(1, len(episode_results))
    print(f'[eval] Average steps per episode: {avg_steps:.1f}')


def main() -> None:
    args = parse_args()
    rename_map = _parse_rename_map(args.rename_map)

    with _open_context(args.profile, use_sim_time=args.use_sim_time) as context:
        print(f'[eval] use_sim_time={args.use_sim_time}')
        layout, policy, policy_node = _setup(context, args, rename_map)
        check_cameras = bool(layout.camera_sources) and not args.hold
        try:
            _run_loop(
                context,
                handover_duration=args.handover_duration,
                handover_max_age=args.handover_max_age,
                resource=args.resource,
                layout=layout,
                policy=policy,
                policy_node=policy_node,
                check_cameras=check_cameras,
                use_sim_time=args.use_sim_time,
                episodes=args.episodes,
                max_steps=args.max_steps,
                reset_mode=args.reset_mode,
                keyframe=args.keyframe,
                recover_faults=args.recover_faults,
            )
        except KeyboardInterrupt:
            pass
        finally:
            policy.close()


if __name__ == '__main__':
    main()
