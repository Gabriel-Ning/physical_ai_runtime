from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from rmi import Action
from rmi.errors import SourceAuthorityError

spec = importlib.util.spec_from_file_location('eval_app', Path(__file__).parents[1] / 'eval.py')
eval_app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(eval_app)


def observation(q=0.0):
    return NS(joint_names=['j'], joint_positions=[q], joint_velocities=[0.0], sensors={})


def layout(frequency=10.0):
    return NS(
        frequency=frequency,
        pose_part=None,
        gripper_parts=(),
        joints=NS(groups=[NS(part='arm', joint_names=['j'])]),
    )


def frame(q):
    return (Action('arm', 'joint_reference', [q]),)


class Policy:
    last_was_replan = True

    def __init__(self):
        self.resets = self.calls = self.observes = 0
        self.chunk = ()

    def reset(self):
        self.resets += 1
        self.chunk = ()

    def select_action(self, obs):
        self.calls += 1
        self.chunk = tuple(frame(q) for q in (0.2, 0.01, 0.02, 0.03))
        return self.chunk[0]

    def take_action_chunk(self):
        result, self.chunk = self.chunk, ()
        return result

    def observe(self, obs):
        self.observes += 1


def test_new_lease_joins_shadow_chunk_without_inference_or_reset():
    clock, policy = [0.0], Policy()
    producer = eval_app._EvalPolicy(policy, layout(), lambda: clock[0])
    producer.shadow_step(observation())
    producer.on_control_acquired(observation())
    action = producer.select_action(observation())
    assert action[0].value == [0.0]
    assert producer.handover.join_index == 1
    assert policy.calls == 1 and policy.resets == 0
    assert not policy.chunk  # Tail transferred, so it cannot be replayed by the backend.
    clock[0] = producer.handover.bridge_duration
    assert producer.select_action(observation())[0].value == pytest.approx([0.01])
    clock[0] += 0.1
    assert producer.select_action(observation())[0].value == [0.02]
    clock[0] += 0.1
    assert producer.select_action(observation())[0].value == [0.03]
    assert policy.calls == 1  # Independent playback cursor; no target replacement.
    clock[0] += 0.1
    producer.select_action(observation())
    assert policy.calls == 2  # Natural inference resumes only after the transferred tail.


def test_missed_preemption_rejoins_retained_unleased_remainder():
    clock, policy = [0.0], Policy()
    producer = eval_app._EvalPolicy(policy, layout(), lambda: clock[0])
    producer.shadow_step(observation())
    producer.on_control_acquired(observation())
    producer.select_action(observation())
    clock[0] = 0.1
    producer.on_control_acquired(observation(0.02))
    assert producer.select_action(observation(0.02))[0].value == pytest.approx([0.02])
    assert policy.calls == 1 and policy.resets == 0


def test_stale_chunk_falls_back_without_reset():
    clock, policy = [0.0], Policy()
    producer = eval_app._EvalPolicy(policy, layout(), lambda: clock[0], max_age=1.0)
    producer.shadow_step(observation())
    clock[0] = 2.0
    producer.on_control_acquired(observation())
    producer.select_action(observation())
    assert policy.calls == 2 and policy.resets == 0


def test_infeasible_join_submits_inferred_action():
    class FarPolicy(Policy):
        def select_action(self, obs):
            self.calls += 1
            self.chunk = tuple(frame(q) for q in (2.0, 2.1, 2.2))
            return self.chunk[0]

    clock, policy = [0.0], FarPolicy()
    producer = eval_app._EvalPolicy(policy, layout(), lambda: clock[0])
    action = producer.select_action(observation())
    assert action[0].value == [2.0]
    assert policy.calls == 1 and producer._need_join is False


def test_infeasible_join_submits_inferred_action():
    class FarPolicy(Policy):
        def select_action(self, obs):
            self.calls += 1
            self.chunk = tuple(frame(q) for q in (2.0, 2.1, 2.2))
            return self.chunk[0]

    clock, policy = [0.0], FarPolicy()
    producer = eval_app._EvalPolicy(policy, layout(), lambda: clock[0])
    action = producer.select_action(observation())
    assert action[0].value == [2.0]
    assert policy.calls == 1 and policy.resets == 0
    assert producer._need_join is False
    assert producer.handover.started is None


def test_shadow_never_submits_or_counts_steps_and_rejected_actions_are_dropped(monkeypatch):
    policy = Policy()
    phases = iter([False, True, False, True, True])

    class Node:
        name = 'Policy'
        producer = policy
        activations = attempts = 0
        closed = False

        @property
        def has_control(self):
            return next(phases)

        def activate(self, **kwargs):
            assert not kwargs
            self.activations += 1
            return NS(close=lambda: setattr(self, 'closed', True))

        def select_action(self, obs):
            if self.attempts < 2:
                self.producer.on_control_acquired(obs)
            return self.producer.select_action(obs)

        def submit(self, actions):
            self.attempts += 1
            if self.attempts == 1:
                raise SourceAuthorityError('preempted during selection')

    node = Node()
    context = NS(
        profile=NS(nodes={}),
        robot={'arm': NS(get_observation=observation)},
        wait_until_ready=lambda **_: None,
    )
    monkeypatch.setattr(eval_app, '_log_step', lambda **_: None)
    eval_app._run_loop(
        context,
        resource='arm',
        layout=layout(1000.0),
        policy=policy,
        policy_node=node,
        check_cameras=False,
        reset_mode='none',
        max_steps=2,
    )
    assert policy.calls == 2  # Both shadow ticks run inference; joining reuses them.
    assert policy.resets == 1  # Episode boundary only.
    assert node.attempts == 3
    assert node.activations == 1 and node.closed
    assert node.producer is policy
