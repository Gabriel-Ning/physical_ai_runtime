import asyncio
from pathlib import Path
from types import SimpleNamespace
from typing import ClassVar

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from rmi import ActionProviderClient, EmbodimentConfig
from trajectory_msgs.msg import JointTrajectory

PROFILE = (
    Path(__file__).parents[3]
    / "interfaces"
    / "rmi"
    / "config"
    / "embodiment_profiles"
    / "marvin_bimanual.yaml"
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.timers = []

    def create_publisher(self, message_type, endpoint, qos):
        publisher = FakePublisher()
        self.publishers.append((message_type, endpoint, qos, publisher))
        return publisher

    def create_timer(self, period_sec, callback):
        timer = SimpleNamespace(period_sec=period_sec, callback=callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=123, nanosec=456)
            )
        )


class FakeActionClient:
    instances: ClassVar[list] = []

    def __init__(self, node, action_type, endpoint):
        del node, action_type
        self.endpoint = endpoint
        self.goals = []
        self.__class__.instances.append(self)

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        future = asyncio.get_running_loop().create_future()
        handle = SimpleNamespace(accepted=True)
        handle.get_result_async = self.get_result_async
        handle.cancel_goal_async = self.cancel_goal_async
        future.set_result(handle)
        return future

    def get_result_async(self):
        future = asyncio.get_running_loop().create_future()
        future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result="complete")
        )
        return future

    def cancel_goal_async(self):
        future = asyncio.get_running_loop().create_future()
        future.set_result(SimpleNamespace(goals_canceling=[True]))
        return future


def test_policy_provider_publishes_to_em_gateway_not_controller_endpoint():
    node = FakeNode()
    provider = ActionProviderClient.from_profile(
        EmbodimentConfig.from_yaml(PROFILE), "Policy", node
    )
    reference = JointTrajectory()

    provider.send("left_arm", "joint_reference", reference)

    endpoints = [entry[1] for entry in node.publishers]
    assert "/execution/left_arm/joint_reference" in endpoints
    publisher = next(entry[3] for entry in node.publishers if entry[1] == endpoints[0])
    assert reference in publisher.messages


def test_policy_provider_converts_ros_free_values_to_stamped_native_message():
    node = FakeNode()
    provider = ActionProviderClient.from_profile(
        EmbodimentConfig.from_yaml(PROFILE), "Policy", node
    )

    provider.send_joint_reference(
        "left_arm",
        [f"Joint{i}_L" for i in range(1, 8)],
        [[0.0] * 7, [0.1] * 7],
        [0.02, 0.08],
    )

    message = next(
        publisher.messages[-1]
        for _, endpoint, _, publisher in node.publishers
        if endpoint == "/execution/left_arm/joint_reference"
    )
    assert isinstance(message, JointTrajectory)
    assert message.header.stamp.sec == 123
    assert len(message.points) == 2
    assert message.points[1].time_from_start.nanosec == 80_000_000


def test_teleop_cartesian_and_twist_are_separate_providers():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    cartesian = ActionProviderClient.from_profile(
        profile, "TeleopCartesian_Left", node
    )
    twist_provider = ActionProviderClient.from_profile(
        profile, "TeleopTwist_Left", node
    )
    twist = TwistStamped()

    twist_provider.send("left_arm", "twist_reference", twist)

    assert cartesian.commands["left_arm"] == frozenset({"pose_reference"})
    assert twist_provider.commands["left_arm"] == frozenset({"twist_reference"})
    assert any(twist in publisher.messages for *_, publisher in node.publishers)


def test_planner_provider_uses_em_trajectory_action():
    FakeActionClient.instances.clear()
    node = FakeNode()
    provider = ActionProviderClient.from_profile(
        EmbodimentConfig.from_yaml(PROFILE),
        "Planner",
        node,
        action_client_factory=FakeActionClient,
    )

    asyncio.run(provider.send("left_arm", "joint_trajectory", JointTrajectory()))

    assert FakeActionClient.instances[0].endpoint == (
        "/execution/left_arm/follow_joint_trajectory"
    )
    assert "/execution/left_arm/trajectory_guard_heartbeat" in [
        entry[1] for entry in node.publishers
    ]


def test_planner_provider_waits_for_terminal_trajectory_result():
    FakeActionClient.instances.clear()
    provider = ActionProviderClient.from_profile(
        EmbodimentConfig.from_yaml(PROFILE),
        "Planner",
        FakeNode(),
        action_client_factory=FakeActionClient,
    )
    trajectory = {
        "points": [
            {"positions": [0.0] * 7, "time_from_start_s": 0.1},
            {"positions": [0.1] * 7, "time_from_start_s": 0.2},
        ]
    }

    result = asyncio.run(
        provider.execute_joint_trajectory(
            "left_arm",
            trajectory,
            [f"Joint{i}_L" for i in range(1, 8)],
            timeout_sec=1.0,
        )
    )

    assert result == "complete"


def test_provider_rejects_another_providers_part_or_command():
    provider = ActionProviderClient.from_profile(
        EmbodimentConfig.from_yaml(PROFILE), "TeleopCartesian_Left", FakeNode()
    )

    with pytest.raises(KeyError, match="does not control"):
        provider.send("right_arm", "pose_reference", JointTrajectory())
    with pytest.raises(KeyError, match="no 'joint_reference' source"):
        provider.send("left_arm", "joint_reference", JointTrajectory())
