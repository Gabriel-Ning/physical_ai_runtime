import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "apps"))

from act_piper import ActPiperNode
from diffusion_piper import _build_parser as build_diffusion_parser
from smolvla_piper import _build_parser as build_smolvla_parser


def test_smolvla_parser_supplies_shared_control_arguments():
    args = build_smolvla_parser().parse_args(
        ["--checkpoint", "/tmp/checkpoint", "--task", "pick_corner"]
    )

    assert args.hold_grippers_open is False
    assert args.open_gripper_position == 0.020
    assert args.layout_ids is None


def test_diffusion_parser_supplies_shared_control_arguments():
    args = build_diffusion_parser().parse_args(
        ["--checkpoint", "/tmp/checkpoint"]
    )

    assert args.hold_grippers_open is False
    assert args.open_gripper_position == 0.020
    assert args.layout_ids is None
    assert args.action_steps is None


def test_control_bridge_feeds_observation_history_while_executing_chunk():
    class Node:
        def create_subscription(self, *_args, **_kwargs):
            return object()

    class Runner:
        image_keys = ()

        def __init__(self):
            self.observations = []
            self.reset_count = 0

        def observe(self, observation):
            self.observations.append(observation)

        def reset_history(self):
            self.reset_count += 1

    args = SimpleNamespace(
        state_topic="/joint_states",
        top_camera_topic="/top",
        left_wrist_topic="/left",
        right_wrist_topic="/right",
        max_input_age_s=1.0,
        hold_grippers_open=False,
        open_gripper_position=0.02,
    )
    runner = Runner()
    bridge = ActPiperNode(Node(), runner, args)
    observation = {"marker": "latest"}
    bridge.inputs = SimpleNamespace(snapshot=lambda: (observation, 0.0))
    bridge._running = True
    bridge.session = SimpleNamespace(active=True)
    bridge.actions.append(np.zeros(14, dtype=np.float32))
    published = []
    bridge._publish_action = lambda action, **kwargs: published.append(action)

    bridge._tick()
    assert runner.observations == [observation]
    assert len(published) == 1

    bridge.clear_session()
    bridge.close()
    assert runner.reset_count == 1
