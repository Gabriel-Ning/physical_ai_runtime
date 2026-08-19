import asyncio
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import CartesianTrajectory
from rmi import (
    ControllerClientError,
    ControllerConfig,
    ForwardCommandControllerClient,
    GripperControllerClient,
    JointSpaceReferenceControllerClient,
    JointTrajectoryControllerClient,
    TaskSpaceReferenceControllerClient,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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


class FakeGoalHandle:
    accepted = True

    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1
        future = asyncio.get_running_loop().create_future()
        future.set_result(SimpleNamespace(goals_canceling=[self]))
        return future

    def get_result_async(self):
        future = asyncio.get_running_loop().create_future()
        future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result="done")
        )
        return future


class FakeActionClient:
    instance = None

    def __init__(self, node, action_type, endpoint):
        self.node = node
        self.ready = True
        self.goals = []
        self.goal_handle = FakeGoalHandle()
        self.heartbeat_at_dispatch = False
        FakeActionClient.instance = self

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.heartbeat_at_dispatch = any(
            publisher.messages and publisher.messages[-1].data
            for _message_type, _endpoint, _qos, publisher in self.node.publishers
        )
        self.goals.append(goal)
        future = asyncio.get_running_loop().create_future()
        future.set_result(self.goal_handle)
        return future


def _config(*, actions=None, topics=None, implementation="example/Controller"):
    return ControllerConfig(
        name="arm_controller",
        implementation=implementation,
        command_interface="position",
        ros_actions=actions or {},
        ros_topics=topics or {},
    )


def test_joint_trajectory_send_and_cancel():
    client = JointTrajectoryControllerClient(
        FakeNode(),
        _config(actions={"follow_joint_trajectory": "/arm/follow"}),
        action_client_factory=FakeActionClient,
    )
    trajectory = JointTrajectory()

    handle = asyncio.run(client.send(trajectory))
    asyncio.run(client.cancel())

    assert handle is FakeActionClient.instance.goal_handle
    assert FakeActionClient.instance.goals[0].trajectory is trajectory
    assert handle.cancel_count == 1


def test_joint_trajectory_cancel_ignores_already_terminal_goal():
    """EM handover after a succeeded trajectory must not fail on cancel."""
    client = JointTrajectoryControllerClient(
        FakeNode(),
        _config(actions={"follow_joint_trajectory": "/arm/follow"}),
        action_client_factory=FakeActionClient,
    )

    class TerminalHandle(FakeGoalHandle):
        def cancel_goal_async(self):
            self.cancel_count += 1
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(goals_canceling=[]))
            return future

    client._goal_handle = TerminalHandle()
    asyncio.run(client.cancel())
    assert client._goal_handle is None


def test_joint_trajectory_execute_waits_for_terminal_success():
    client = JointTrajectoryControllerClient(
        FakeNode(),
        _config(actions={"follow_joint_trajectory": "/arm/follow"}),
        action_client_factory=FakeActionClient,
    )

    result = asyncio.run(client.execute(JointTrajectory()))

    assert result == "done"


def test_joint_trajectory_guard_is_armed_only_for_accepted_goal_lifetime():
    node = FakeNode()
    client = JointTrajectoryControllerClient(
        node,
        _config(
            actions={"follow_joint_trajectory": "/arm/follow"},
            topics={"trajectory_guard_heartbeat": "/arm_guard/heartbeat"},
        ),
        action_client_factory=FakeActionClient,
    )

    handle = asyncio.run(client.send(JointTrajectory()))
    node.timers[0].callback()
    asyncio.run(client.wait_for_result(handle))
    node.timers[0].callback()

    assert node.publishers[0][1] == "/arm_guard/heartbeat"
    assert FakeActionClient.instance.heartbeat_at_dispatch
    assert [message.data for message in node.publishers[0][3].messages] == [
        True,
        True,
        False,
    ]


def test_joint_trajectory_wait_timeout_keeps_guard_armed():
    node = FakeNode()
    client = JointTrajectoryControllerClient(
        node,
        _config(
            actions={"follow_joint_trajectory": "/arm/follow"},
            topics={"trajectory_guard_heartbeat": "/arm_guard/heartbeat"},
        ),
        timeout_sec=0.001,
        action_client_factory=FakeActionClient,
    )
    handle = asyncio.run(client.send(JointTrajectory()))

    def pending_result():
        return asyncio.get_running_loop().create_future()

    handle.get_result_async = pending_result
    with pytest.raises(TimeoutError, match="wait for trajectory result"):
        asyncio.run(client.wait_for_result(handle, timeout_sec=0.001))
    node.timers[0].callback()

    assert [message.data for message in node.publishers[0][3].messages] == [True, True]


def test_joint_trajectory_cancel_disarms_guard_after_cancel_response():
    node = FakeNode()
    client = JointTrajectoryControllerClient(
        node,
        _config(
            actions={"follow_joint_trajectory": "/arm/follow"},
            topics={"trajectory_guard_heartbeat": "/arm_guard/heartbeat"},
        ),
        action_client_factory=FakeActionClient,
    )

    asyncio.run(client.send(JointTrajectory()))
    asyncio.run(client.cancel())
    node.timers[0].callback()

    assert [message.data for message in node.publishers[0][3].messages] == [True, False]


def test_joint_trajectory_cancel_failure_falls_back_to_guard_timeout():
    node = FakeNode()
    client = JointTrajectoryControllerClient(
        node,
        _config(
            actions={"follow_joint_trajectory": "/arm/follow"},
            topics={"trajectory_guard_heartbeat": "/arm_guard/heartbeat"},
        ),
        action_client_factory=FakeActionClient,
    )
    handle = asyncio.run(client.send(JointTrajectory()))

    def failed_cancel():
        future = asyncio.get_running_loop().create_future()
        future.set_exception(RuntimeError("cancel transport failed"))
        return future

    handle.cancel_goal_async = failed_cancel
    with pytest.raises(RuntimeError, match="cancel transport failed"):
        asyncio.run(client.cancel())
    node.timers[0].callback()

    # No explicit false is sent: heartbeat silence makes the independent RT
    # guard time out and cancel the JTC goal locally.
    assert [message.data for message in node.publishers[0][3].messages] == [True]


def test_joint_trajectory_execute_reports_aborted_terminal_result():
    client = JointTrajectoryControllerClient(
        FakeNode(),
        _config(actions={"follow_joint_trajectory": "/arm/follow"}),
        action_client_factory=FakeActionClient,
    )
    handle = FakeActionClient.instance.goal_handle

    def aborted_result():
        future = asyncio.get_running_loop().create_future()
        future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=None)
        )
        return future

    handle.get_result_async = aborted_result
    with pytest.raises(ControllerClientError, match="aborted"):
        asyncio.run(client.execute(JointTrajectory()))


def test_rejected_trajectory_goal_is_reported():
    client = JointTrajectoryControllerClient(
        FakeNode(),
        _config(actions={"follow_joint_trajectory": "/arm/follow"}),
        action_client_factory=FakeActionClient,
    )
    FakeActionClient.instance.goal_handle.accepted = False

    with pytest.raises(ControllerClientError, match="rejected"):
        asyncio.run(client.send(JointTrajectory()))


def test_rejected_trajectory_goal_explicitly_disarms_guard():
    node = FakeNode()
    client = JointTrajectoryControllerClient(
        node,
        _config(
            actions={"follow_joint_trajectory": "/arm/follow"},
            topics={"trajectory_guard_heartbeat": "/arm_guard/heartbeat"},
        ),
        action_client_factory=FakeActionClient,
    )
    FakeActionClient.instance.goal_handle.accepted = False

    with pytest.raises(ControllerClientError, match="rejected"):
        asyncio.run(client.send(JointTrajectory()))

    assert [message.data for message in node.publishers[0][3].messages] == [True, False]


def test_uncertain_trajectory_dispatch_falls_back_to_guard_timeout():
    class FailedDispatchActionClient(FakeActionClient):
        def send_goal_async(self, goal):
            del goal
            raise RuntimeError("dispatch transport failed")

    node = FakeNode()
    client = JointTrajectoryControllerClient(
        node,
        _config(
            actions={"follow_joint_trajectory": "/arm/follow"},
            topics={"trajectory_guard_heartbeat": "/arm_guard/heartbeat"},
        ),
        action_client_factory=FailedDispatchActionClient,
    )

    with pytest.raises(RuntimeError, match="dispatch transport failed"):
        asyncio.run(client.send(JointTrajectory()))
    node.timers[0].callback()

    # Dispatch may have reached RT, so no false is sent; silence requests the
    # conservative local cancel path.
    assert [message.data for message in node.publishers[0][3].messages] == [True]


def test_joint_reference_publishes_joint_trajectory():
    node = FakeNode()
    client = JointSpaceReferenceControllerClient(
        node, _config(topics={"joint_reference": "/arm/reference"})
    )
    message = JointTrajectory()

    client.send(message)

    assert node.publishers[0][1] == "/arm/reference"
    assert node.publishers[0][3].messages == [message]


def test_task_reference_supports_independent_pose_track():
    node = FakeNode()
    client = TaskSpaceReferenceControllerClient(
        node, _config(topics={"pose_reference": "/arm/pose"})
    )
    message = CartesianTrajectory()

    client.send_pose(message)

    assert node.publishers[0][1] == "/arm/pose"
    assert node.publishers[0][3].messages == [message]
    with pytest.raises(ControllerClientError, match="twist_reference"):
        client.send_twist(SimpleNamespace())


def test_gripper_sends_named_joint_state_and_cancels():
    client = GripperControllerClient(
        FakeNode(),
        _config(actions={"gripper_command": "/gripper/gripper_cmd"}),
        action_client_factory=FakeActionClient,
    )
    command = JointState(name=["finger_joint"], position=[0.04])

    handle = asyncio.run(client.send(command))
    asyncio.run(client.cancel())

    assert FakeActionClient.instance.goals[0].command is command
    assert handle.cancel_count == 1


def test_gripper_rejects_unnamed_or_misaligned_command():
    client = GripperControllerClient(
        FakeNode(),
        _config(actions={"gripper_command": "/gripper/gripper_cmd"}),
        action_client_factory=FakeActionClient,
    )

    with pytest.raises(ValueError, match="aligned"):
        asyncio.run(client.send(JointState(position=[0.04])))


def test_forward_controller_adapts_latest_joint_trajectory_point():
    node = FakeNode()
    client = ForwardCommandControllerClient(
        node,
        _config(
            implementation="forward_command_controller/ForwardCommandController",
            topics={"joint_reference": "/gripper/commands"},
        ),
    )
    reference = JointTrajectory()
    reference.joint_names = ["finger_joint"]
    first = JointTrajectoryPoint()
    first.positions = [0.01]
    last = JointTrajectoryPoint()
    last.positions = [0.04]
    reference.points = [first, last]

    client.send(reference)

    assert node.publishers[0][1] == "/gripper/commands"
    assert list(node.publishers[0][3].messages[0].data) == [0.04]
