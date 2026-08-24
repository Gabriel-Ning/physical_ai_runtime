import asyncio
from pathlib import Path
from types import SimpleNamespace

from execution_manager_interfaces.action import LeasedFollowJointTrajectory
from execution_manager_interfaces.msg import LeasedJointReference, LeasedPoseReference
from rmi import ActionProviderClient, EmbodimentConfig
from rmi.selection import EndpointBinding, LeaseGrant
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PROFILE = Path(__file__).parents[4] / "apps" / "profiles" / "marvin_bimanual.yaml"


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.destroyed_publishers = []

    def create_publisher(self, message_type, endpoint, qos):
        publisher = FakePublisher()
        self.publishers.append((message_type, endpoint, qos, publisher))
        return publisher

    def destroy_publisher(self, publisher):
        self.destroyed_publishers.append(publisher)

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=123, nanosec=456)
            )
        )


class FakeActionClient:
    instance = None

    def __init__(self, node, action_type, endpoint):
        del node
        assert action_type is LeasedFollowJointTrajectory
        self.endpoint = endpoint
        self.goals = []
        self.__class__.instance = self

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, **kwargs):
        del kwargs
        self.goals.append(goal)
        future = asyncio.get_running_loop().create_future()
        future.set_result(SimpleNamespace(accepted=True))
        return future


def _grant(command="joint_reference", *, action=False):
    endpoint = (
        "/action_sources/planner/left_arm/follow_joint_trajectory"
        if action
        else "/action_sources/policy/left_arm/joint_reference"
    )
    return LeaseGrant(
        "lease-1",
        {
            ("left_arm", command): EndpointBinding(
                "left_arm", command, endpoint, action
            )
        },
    )


def test_streaming_command_is_top_level_stamped_lease_envelope():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    client = ActionProviderClient(
        "policy", node, {"left_arm": "joint_reference"}, profile=profile
    )
    client.bind(_grant())
    trajectory = JointTrajectory()
    trajectory.joint_names = list(profile.parts["left_arm"].joint_names)
    trajectory.points = [JointTrajectoryPoint(positions=[0.0] * 7)]

    client.send("left_arm", "joint_reference", trajectory)

    envelope = node.publishers[0][3].messages[0]
    assert isinstance(envelope, LeasedJointReference)
    assert envelope.lease_id == "lease-1"
    assert envelope.header.stamp.sec == 123
    assert envelope.command.header.stamp.sec == 123
    assert envelope.command.joint_names == trajectory.joint_names


def test_streaming_joint_reference_accepts_bare_position_list():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    client = ActionProviderClient(
        "policy", node, {"left_arm": "joint_reference"}, profile=profile
    )
    client.bind(_grant())

    client.send("left_arm", "joint_reference", [0.1] * 7)

    command = node.publishers[0][3].messages[0].command
    assert command.joint_names == list(profile.parts["left_arm"].joint_names)
    assert list(command.points[0].positions) == [0.1] * 7


def test_unbind_destroys_session_publishers():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    client = ActionProviderClient(
        "policy", node, {"left_arm": "joint_reference"}, profile=profile
    )
    client.bind(_grant())
    client.send("left_arm", "joint_reference", [0.1] * 7)
    publisher = node.publishers[0][3]

    client.unbind()

    assert node.destroyed_publishers == [publisher]
    assert client._publishers == {}


def test_streaming_pose_reference_accepts_single_cartesian_state():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    client = ActionProviderClient(
        "teleop", node, {"left_arm": "pose_reference"}, profile=profile
    )
    client.bind(_grant("pose_reference"))
    pose = SimpleNamespace(
        position_xyz=(0.1, 0.2, 0.3),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    client.send("left_arm", "pose_reference", pose)

    envelope = node.publishers[0][3].messages[0]
    assert isinstance(envelope, LeasedPoseReference)
    assert envelope.lease_id == "lease-1"
    assert len(envelope.command.points) == 1
    assert envelope.command.points[0].point.pose.position.z == 0.3
    assert envelope.command.points[0].point.pose.orientation.w == 1.0


def test_unbound_client_cannot_publish():
    client = ActionProviderClient(
        "policy", FakeNode(), {"left_arm": "joint_reference"}
    )
    try:
        client.send("left_arm", "joint_reference", JointTrajectory())
    except RuntimeError as error:
        assert "no active lease" in str(error)
    else:
        raise AssertionError("unbound client accepted a command")


def test_fork_has_independent_lease_binding():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    client = ActionProviderClient(
        "policy", node, {"left_arm": "joint_reference"}, profile=profile
    )
    first = client.fork()
    second = client.fork()
    first.bind(_grant())
    second.bind(LeaseGrant("lease-2", _grant().endpoints))

    first.send("left_arm", "joint_reference", [0.1] * 7)
    second.send("left_arm", "joint_reference", [0.2] * 7)

    assert node.publishers[0][3].messages[0].lease_id == "lease-1"
    assert node.publishers[1][3].messages[0].lease_id == "lease-2"


def test_none_action_client_factory_defaults_to_rclpy_action_client():
    from rclpy.action import ActionClient

    client = ActionProviderClient(
        "planner",
        FakeNode(),
        {"left_arm": "joint_trajectory"},
        action_client_factory=None,
    )
    assert client._action_client_factory is ActionClient


def test_jtc_goal_carries_same_lease_and_resource():
    node = FakeNode()
    profile = EmbodimentConfig.from_yaml(PROFILE)
    client = ActionProviderClient(
        "planner",
        node,
        {"left_arm": "joint_trajectory"},
        profile=profile,
        action_client_factory=FakeActionClient,
    )
    client.bind(_grant("joint_trajectory", action=True))
    trajectory = SimpleNamespace(
        points=[SimpleNamespace(positions=[0.0] * 7, time_from_start_s=0.1)],
        valid=True,
    )

    asyncio.run(
        client.start_joint_trajectory(
            "left_arm", trajectory, list(profile.parts["left_arm"].joint_names)
        )
    )

    goal = FakeActionClient.instance.goals[0]
    assert goal.lease_id == "lease-1"
    assert goal.resource == "left_arm"
    assert goal.header.stamp.sec == 123
