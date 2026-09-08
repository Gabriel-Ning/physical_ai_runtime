"""Unit tests for RmiEnv and ResidualRmiEnv Gymnasium adapters."""

import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from policy_inference.lerobot.env import ResidualRmiEnv, RmiEnv
from rmi import JointGroup, JointLayout, PolicyLayout
from rmi.node import NodeActivation
from rmi.selection import AuthoritySnapshot, SourceRole, ResourceAuthority


def _make_dummy_layout() -> PolicyLayout:
    groups = (
        JointGroup("arm", "joint_reference", ("j1", "j2", "j3", "j4", "j5", "j6"), 0, 6),
        JointGroup("gripper", "joint_reference", ("f1", "f2"), 6, 8),
    )
    joints = JointLayout(groups=groups, joint_names=("j1", "j2", "j3", "j4", "j5", "j6", "f1", "f2"))
    return PolicyLayout(
        profile_name="test_robot",
        profile_hash="hash123",
        joints=joints,
        state_topic="/joint_states",
        action_topics={"arm": "/execution/arm/joint_reference", "gripper": "/execution/gripper/joint_reference"},
        camera_sources={"observation.images.agentview": "agentview"},
        camera_topics={"observation.images.agentview": "/agentview/image_raw"},
        camera_shapes={"observation.images.agentview": (480, 640, 3)},
        frequency=30.0,
        control_mode="cartesian",
        action_space="rel",
        pose_part="arm",
        gripper_parts=("gripper",),
    )


class DummyRobotObservation:
    def __init__(self) -> None:
        now = time.monotonic()
        self.receive_time_s = now
        self.joint_names = ["j1", "j2", "j3", "j4", "j5", "j6", "f1", "f2"]
        self.joint_positions = [0.0] * 8
        self.joint_velocities = [0.0] * 8

        cam_sample = MagicMock()
        cam_sample.value = np.zeros((480, 640, 3), dtype=np.uint8)
        cam_sample.receive_time_s = now

        mock_pose = MagicMock()
        mock_pose.position_xyz = (0.3, 0.0, 0.5)
        mock_pose.orientation_wxyz = (1.0, 0.0, 0.0, 0.0)
        mock_pose.quaternion_xyzw = (0.0, 0.0, 0.0, 1.0)
        pose_sample = MagicMock(value=mock_pose, receive_time_s=now)

        self.sensors = {
            "agentview": cam_sample,
            "arm": pose_sample,
        }
        self.data = {"joint_positions": self.joint_positions}


class DummyContext:
    def __init__(self) -> None:
        self.reset = MagicMock(return_value=True)
        self.robot = MagicMock()
        self.robot.__getitem__.return_value.get_observation.return_value = DummyRobotObservation()
        self.robot.get_observation.return_value = DummyRobotObservation()
        self.authority_client = MagicMock(spec=["describe_authority"])
        self.authority_client.describe_authority.return_value = AuthoritySnapshot({})

    def make_node(self, name: str) -> MagicMock:
        node = MagicMock(spec=["activate", "deactivate", "submit"])
        node.activate.side_effect = lambda: NodeActivation(node)
        return node


def test_rmi_env_reset_and_step() -> None:
    context = DummyContext()
    layout = _make_dummy_layout()

    env = RmiEnv(
        context,
        layout,
        max_steps=10,
        control_freq=1000.0,  # High freq for fast test
    )

    obs, info = env.reset()
    assert isinstance(obs, dict)
    assert "agentview" in obs
    assert "j1.pos" in obs
    assert info["is_intervention"] is False
    assert context.reset.called

    action = np.zeros(8, dtype=np.float32)
    next_obs, reward, terminated, truncated, step_info = env.step(action)

    assert isinstance(next_obs, dict)
    assert reward == 0.0
    assert terminated is False
    assert truncated is False
    assert step_info["step"] == 1
    assert step_info["is_intervention"] is False
    assert env.policy_node.submit.called
    env.close()


def test_rmi_env_max_steps_truncation() -> None:
    context = DummyContext()
    layout = _make_dummy_layout()

    env = RmiEnv(context, layout, max_steps=3, control_freq=1000.0)
    env.reset()

    action = np.zeros(8, dtype=np.float32)
    _, _, term1, trunc1, _ = env.step(action)
    assert not trunc1 and not term1

    _, _, term2, trunc2, _ = env.step(action)
    assert not trunc2 and not term2

    _, _, term3, trunc3, info3 = env.step(action)
    assert trunc3 is True
    assert term3 is False
    assert info3["timeout"] is True
    env.close()


def test_rmi_env_task_success_termination() -> None:
    context = DummyContext()
    layout = _make_dummy_layout()

    mock_task = MagicMock()
    mock_task.check_success.return_value = True

    env = RmiEnv(context, layout, task=mock_task, max_steps=10, control_freq=1000.0)
    env.reset()

    action = np.zeros(8, dtype=np.float32)
    _, reward, terminated, truncated, info = env.step(action)

    assert reward == 1.0
    assert terminated is True
    assert truncated is False
    assert info["success"] is True
    env.close()


def test_rmi_env_intervention_detection() -> None:
    context = DummyContext()
    layout = _make_dummy_layout()

    # Simulate Teleop claiming the arm
    context.authority_client.describe_authority.return_value = AuthoritySnapshot({
        "arm": {"authority_state": ResourceAuthority.OWNED, "source_role": SourceRole.TELEOP},
    })

    env = RmiEnv(context, layout, max_steps=10, control_freq=1000.0)
    env.reset()

    action = np.zeros(8, dtype=np.float32)
    _, _, _, _, info = env.step(action)
    assert info["is_intervention"] is True
    env.close()


def test_residual_rmi_env_action_fusion() -> None:
    context = DummyContext()
    layout = _make_dummy_layout()

    base_env = RmiEnv(context, layout, max_steps=10, control_freq=1000.0)

    # Base policy outputs a constant action [0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.5, 0.5]
    mock_base_policy = MagicMock()
    base_action = np.array([0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.5, 0.5], dtype=np.float32)
    mock_base_policy.select_action.return_value = base_action

    residual_env = ResidualRmiEnv(
        base_env,
        mock_base_policy,
        residual_scale=0.1,
    )

    obs, info = residual_env.reset()
    assert mock_base_policy.reset.called

    # Residual action is [1.0, -1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    res_action = np.array([1.0, -1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    next_obs, reward, term, trunc, step_info = residual_env.step(res_action)

    # Expected fused = base + 0.1 * res = [0.2 + 0.1, 0.2 - 0.1, 0.2 + 0.05, 0, 0, 0, 0.5, 0.5]
    expected_fused = np.array([0.3, 0.1, 0.25, 0.0, 0.0, 0.0, 0.5, 0.5], dtype=np.float32)
    np.testing.assert_allclose(step_info["executed_action"], expected_fused, rtol=1e-5)
    np.testing.assert_allclose(step_info["base_action"], base_action, rtol=1e-5)
    np.testing.assert_allclose(step_info["residual_action"], res_action, rtol=1e-5)

    residual_env.close()


def test_env_close_releases_activation_once():
    env = RmiEnv(DummyContext(), _make_dummy_layout())
    env.reset()
    env.reset()
    env.policy_node.activate.assert_called_once()
    env.close()
    env.close()
    env.policy_node.deactivate.assert_called_once()


@pytest.mark.parametrize("resource,state,role", [
    ("other_arm", ResourceAuthority.OWNED, SourceRole.TELEOP),
    ("arm", ResourceAuthority.UNOWNED, SourceRole.TELEOP),
    ("arm", ResourceAuthority.OWNED, SourceRole.POLICY),
])
def test_intervention_ignores_unrelated_or_unowned_resources(resource, state, role):
    context = DummyContext()
    context.authority_client.describe_authority.return_value = AuthoritySnapshot({
        resource: {"authority_state": state, "source_role": role},
    })
    env = RmiEnv(context, _make_dummy_layout())
    assert env._check_intervention() is False
