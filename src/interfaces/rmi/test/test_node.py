from types import SimpleNamespace
from typing import Any, ClassVar

import pytest
from action_msgs.msg import GoalStatus
from execution_manager_interfaces.msg import ResourceAuthority
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from rmi import Action, EmbodimentConfig, Node, Robot, SourceState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers: list[tuple[type, str, Any, FakePublisher]] = []
        self.subscriptions: list[tuple[Any, str, Any]] = []
        self._current_time = SimpleNamespace(sec=100, nanosec=500000)

    def create_publisher(self, message_type, endpoint, qos):
        del qos
        publisher = FakePublisher()
        self.publishers.append((message_type, endpoint, publisher))
        return publisher

    def create_subscription(self, message_type, endpoint, callback, qos):
        del message_type, qos
        self.subscriptions.append((endpoint, callback))
        return SimpleNamespace(endpoint=endpoint, callback=callback)

    def destroy_subscription(self, subscription):
        del subscription

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: self._current_time)
        )


class FakeAuthority:
    def __init__(self):
        self.allocations = {}
        self.claims = []
        self.releases = []
        self.sources = {}

    def get_sources(self):
        return self.sources

    def set_source_activation(self, name, session, *, active, preempt=False):
        if active:
            self.claims.append((name, session, preempt))
            self.sources[name] = {"state": 3, "session_id": session, "lease_id": "node-lease"}
        else:
            self.releases.append(session)
            self.sources[name] = {"state": 0, "session_id": session, "lease_id": ""}


    def get_allocations(self):
        return self.allocations

    def claim(
        self,
        source_role,
        source_instance,
        resources,
        *,
        preempt=False,
        metadata=None,
    ):
        self.claims.append((source_role, source_instance, resources, preempt, metadata))
        return SimpleNamespace(lease_id="node-lease")

    def release(self, lease_id):
        self.releases.append(lease_id)


class FakeFuture:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value

    def exception(self):
        return None


class FakeGoalHandle:
    accepted = True

    def get_result_async(self):
        return FakeFuture(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result="ok")
        )

    def cancel_goal_async(self):
        return FakeFuture(SimpleNamespace())


class FakeActionClient:
    instances: ClassVar[list["FakeActionClient"]] = []

    def __init__(self, node, action_type, endpoint):
        del node, action_type
        self.endpoint = endpoint
        self.goals = []
        self.__class__.instances.append(self)

    def wait_for_server(self, timeout_sec):
        return timeout_sec > 0.0

    def send_goal_async(self, goal, feedback_callback=None):
        del feedback_callback
        self.goals.append(goal)
        return FakeFuture(FakeGoalHandle())

    def destroy(self):
        pass


def _sample_profile_dict():
    return {
        "metadata": {"name": "test_robot"},
        "compound_groups": {
            "manipulator": {"included_groups": ["arm", "gripper"]},
        },
        "groups": {
            "arm": {
                "type": "arm",
                "joint_names": ["joint1", "joint2"],
                "base_frame": "arm_base",
                "tcp_frame": "arm_tcp",
                "controller_manager": "/controller_manager",
                "default_controller": "joint_space_reference",
                "controllers": {
                    "joint_space_reference": {
                        "name": "arm_jspc",
                        "ros_topics": {
                            "joint_reference": "/execution/arm/joint_reference",
                            "twist_reference": "/execution/arm/twist_reference",
                            "pose_reference": "/execution/arm/pose_reference",
                        },
                        "ros_actions": {
                            "follow_joint_trajectory": "/execution/arm/follow_joint_trajectory",
                        },
                    },
                },
            },
            "gripper": {
                "type": "gripper",
                "joint_names": ["gripper_joint"],
                "controller_manager": "/controller_manager",
                "default_controller": "gripper_jspc",
                "controllers": {
                    "gripper_jspc": {
                        "name": "gripper_jspc",
                        "ros_topics": {
                            "joint_reference": "/execution/gripper/joint_reference",
                        },
                    },
                },
            },
        },
        "nodes": {
            "Policy": {
                "source_role": "POLICY",
                "resources": {
                    "arm": "joint_reference",
                    "gripper": "joint_reference",
                },
                "frequency": 30.0,
            },
            "TeleopTwist": {
                "source_role": "TELEOP",
                "resources": {
                    "arm": "twist_reference",
                },
                "frequency": 100.0,
            },
            "TeleopPose": {
                "source_role": "TELEOP",
                "resources": {
                    "arm": "pose_reference",
                },
            },
            "Planner": {
                "source_role": "PLANNER",
                "resources": {"arm": "joint_trajectory"},
            },
        },
        "sources": {
            "Policy": {
                "source_role": "POLICY",
                "inputs": {
                    "arm": {
                        "topic": "/action_sources/policy/arm/joint_reference",
                        "command_contract": "joint_reference",
                    },
                    "gripper": {
                        "topic": "/action_sources/policy/gripper/joint_reference",
                        "command_contract": "joint_reference",
                    },
                },
            },
            "TeleopTwist": {
                "source_role": "TELEOP",
                "inputs": {
                    "arm": {
                        "topic": "/action_sources/teleop/arm/twist_reference",
                        "command_contract": "twist_reference",
                    },
                },
            },
            "TeleopPose": {
                "source_role": "TELEOP",
                "inputs": {
                    "arm": {
                        "topic": "/action_sources/teleop/arm/pose_reference",
                        "command_contract": "pose_reference",
                    },
                },
            },
            "Planner": {
                "source_role": "PLANNER",
                "preempt": True,
                "inputs": {
                    "arm": {
                        "action": "/execution_manager/ingress/planner/arm/follow_joint_trajectory",
                        "command_contract": "joint_trajectory",
                    }
                },
            },
        },
    }


def test_node_init_binds_profile_nodes_and_sources():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())

    assert node.name == "Policy"
    assert node.config.name == "Policy"
    assert node.config.source_role == "POLICY"
    assert node.config.resources == {
        "arm": "joint_reference",
        "gripper": "joint_reference",
    }
    assert node.config.frequency == 30.0


def test_robot_compound_resource_reorders_joint_observation():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    robot = Robot(profile, FakeAuthority())
    state = JointState()
    state.name = ["gripper_joint", "joint2", "joint1"]
    state.position = [0.04, 0.2, 0.1]
    state.velocity = [0.0, 2.0, 1.0]
    state.effort = [0.0, 20.0, 10.0]
    robot.update_joint_state(state, receive_time_s=1.0)

    resource = robot["manipulator"]
    observation = resource.get_observation()

    assert resource.parts == ("arm", "gripper")
    assert resource.joint_names == ("joint1", "joint2", "gripper_joint")
    assert observation.joint_names == ["joint1", "joint2", "gripper_joint"]
    assert observation.joint_positions == [0.1, 0.2, 0.04]
    assert observation.joint_velocities == [1.0, 2.0, 0.0]


def test_node_compound_resource_splits_flat_joint_vector():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    node["manipulator"].submit(node["manipulator"].select_action(selector=lambda _: [0.1, 0.2, 0.04]))

    assert [entry[1] for entry in fake_node.publishers] == [
        "/execution_manager/ingress/policy/arm/joint_reference",
        "/execution_manager/ingress/policy/gripper/joint_reference",
    ]
    assert list(fake_node.publishers[0][2].messages[0].command.points[0].positions) == [
        0.1,
        0.2,
    ]
    assert list(fake_node.publishers[1][2].messages[0].command.points[0].positions) == [0.04]


def test_node_resource_accepts_no_policy_action():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    node["manipulator"].submit(None)

    assert fake_node.publishers == []


def test_node_compound_resource_executes_all_action_parts():
    data = _sample_profile_dict()
    data["groups"]["arm2"] = {
        **data["groups"]["arm"],
        "joint_names": ["joint3", "joint4"],
        "controllers": {
            "joint_space_reference": {
                "name": "arm2_jspc",
                "ros_topics": {
                    "joint_reference": "/execution/arm2/joint_reference",
                },
                "ros_actions": {
                    "follow_joint_trajectory": (
                        "/execution/arm2/follow_joint_trajectory"
                    ),
                },
            }
        },
    }
    data["compound_groups"]["dual_arm"] = {"included_groups": ["arm", "arm2"]}
    data["nodes"]["Planner"]["resources"]["arm2"] = "joint_trajectory"
    data["sources"]["Planner"]["inputs"]["arm2"] = {
        "action": "/execution_manager/ingress/planner/arm2/follow_joint_trajectory",
        "command_contract": "joint_trajectory",
    }
    profile = EmbodimentConfig.from_dict(data)
    FakeActionClient.instances.clear()
    node = Node(
        "Planner",
        FakeNode(),
        profile,
        FakeAuthority(),
        action_client_factory=FakeActionClient,
    )
    node.activate()
    plans = {
        part: SimpleNamespace(
            valid=True,
            points=[SimpleNamespace(positions=positions, time_from_start_s=1.0)],
        )
        for part, positions in {
            "arm": [0.1, 0.2],
            "arm2": [0.3, 0.4],
        }.items()
    }

    results = node["dual_arm"].execute(plans)

    assert results == {"arm": "ok", "arm2": "ok"}
    assert len(FakeActionClient.instances) == 2


def test_execute_builds_parallel_gripper_command():
    data = _sample_profile_dict()
    data["groups"]["gripper"]["controllers"]["gripper_action"] = {
        "name": "gripper_action",
        "ros_actions": {
            "gripper_command": "/execution/gripper/gripper_command",
        },
    }
    data["nodes"]["Planner"]["resources"]["gripper"] = "gripper_command"
    data["sources"]["Planner"]["inputs"]["gripper"] = {
        "action": "/execution_manager/ingress/planner/gripper/gripper_command",
        "command_contract": "gripper_command",
    }
    profile = EmbodimentConfig.from_dict(data)
    FakeActionClient.instances.clear()
    node = Node(
        "Planner",
        FakeNode(),
        profile,
        FakeAuthority(),
        action_client_factory=FakeActionClient,
    )
    node.activate()

    node.execute("gripper", [0.045])

    command = FakeActionClient.instances[0].goals[0].command
    assert command.name == ["gripper_joint"]
    assert list(command.position) == [0.045]


def test_external_node_handle_does_not_require_submit_binding():
    data = _sample_profile_dict()
    del data["sources"]["Policy"]
    profile = EmbodimentConfig.from_dict(data)
    fake_node = FakeNode()

    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()
    assert node.config.inputs == {}
    with pytest.raises(RuntimeError, match="no in-process ingress binding for 'arm'"):
        node.submit(node.select_action(selector=lambda _: Action(part="arm", command="joint_reference", value=[0.0, 0.0])))


def test_source_state_is_em_owned_and_has_control_is_derived():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), profile, authority)
    assert node.state is SourceState.INACTIVE
    for value, expected in enumerate(SourceState):
        authority.sources["Policy"] = {"state": value}
        assert node.state is expected
        assert node.has_control == (expected is SourceState.CONTROLLING)


def test_submit_none_or_empty_is_noop():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    node.submit(None)
    node.submit([])
    node.submit(())
    assert len(fake_node.publishers) == 0


def test_activate_and_deactivate_use_source_session_not_lease():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), profile, authority)
    with node.activate(preempt=True):
        name, session, preempt = authority.claims[0]
        assert name == "Policy" and preempt
        authority.sources["Policy"]["lease_id"] = "restored-lease"
    assert authority.releases == [session]


def test_submit_without_activation_and_while_waiting_fails():
    node = Node("Policy", FakeNode(), EmbodimentConfig.from_dict(_sample_profile_dict()), FakeAuthority())
    with pytest.raises(RuntimeError, match="not activated"):
        node.submit(None)
    node.activate()
    node._authority.sources["Policy"]["state"] = 1
    with pytest.raises(RuntimeError, match="control"):
        node.submit(Action("arm", "joint_reference", [0., 0.]))


def test_select_action_result_keeps_epoch_during_preemption():
    authority, fake = FakeAuthority(), FakeNode()
    class Producer:
        def select_action(self, observation):
            authority.sources["Policy"]["lease_id"] = "next-lease"
            return Action("arm", "joint_reference", [0., 0.])
    node = Node("Policy", fake, EmbodimentConfig.from_dict(_sample_profile_dict()), authority, Producer())
    node.activate()
    action = node.select_action(None)
    with pytest.raises(RuntimeError, match="revoked"):
        node.submit(action)
    assert not fake.publishers


def test_execute_uses_leased_action_and_returns_handle():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    FakeActionClient.instances.clear()
    node = Node(
        "Planner",
        FakeNode(),
        profile,
        FakeAuthority(),
        action_client_factory=FakeActionClient,
    )
    node.activate()
    plan = SimpleNamespace(
        valid=True,
        points=[SimpleNamespace(positions=[0.1, 0.2], time_from_start_s=1.0)],
    )

    execution = node.execute("arm", plan)
    assert execution.state.name == "ACCEPTED"
    assert FakeActionClient.instances[0].endpoint.endswith(
        "/planner/arm/follow_joint_trajectory"
    )
    assert list(FakeActionClient.instances[0].goals[0].trajectory.joint_names) == [
        "joint1",
        "joint2",
    ]
    assert execution.wait() == "ok"
    assert execution.state.name == "SUCCEEDED"


def test_submit_joint_reference():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    action = Action(part="arm", command="joint_reference", value=[0.25, -0.4])
    node.submit(node.select_action(selector=lambda _: action))

    assert len(fake_node.publishers) == 1
    msg_type, topic, pub = fake_node.publishers[0]
    assert msg_type.__name__ == "LeasedJointReference"
    assert topic == "/execution_manager/ingress/policy/arm/joint_reference"
    assert len(pub.messages) == 1
    msg = pub.messages[0].command
    assert list(msg.joint_names) == ["joint1", "joint2"]
    assert list(msg.points[0].positions) == [0.25, -0.4]
    assert msg.header.stamp.sec == 100
    assert msg.header.stamp.nanosec == 500000


def test_admission_stamp_is_taken_after_producer():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    def slow(_):
        fake_node._current_time = SimpleNamespace(sec=100, nanosec=800000000)
        return Action(part="arm", command="joint_reference", value=[0.25, -0.4])

    node.submit(node.select_action(selector=slow))
    stamp = fake_node.publishers[0][2].messages[0].header.stamp
    assert stamp.sec == 100
    assert stamp.nanosec == 800000000


def test_submit_twist_reference():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("TeleopTwist", fake_node, profile, FakeAuthority())
    node.activate()

    action = Action(
        part="arm",
        command="twist_reference",
        value=[0.1, 0.2, 0.3, 0.0, 0.0, 0.5],
    )
    node.submit(node.select_action(selector=lambda _: action))

    assert len(fake_node.publishers) == 1
    msg_type, topic, pub = fake_node.publishers[0]
    assert msg_type.__name__ == "LeasedTwistReference"
    assert topic == "/execution_manager/ingress/teleop/arm/twist_reference"
    assert len(pub.messages) == 1
    msg = pub.messages[0].command
    assert msg.header.frame_id == "arm_base"
    assert msg.twist.linear.x == 0.1
    assert msg.twist.linear.y == 0.2
    assert msg.twist.angular.z == 0.5
    assert msg.header.stamp.sec == 100


def test_submit_pose_reference():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("TeleopPose", fake_node, profile, FakeAuthority())
    node.activate()

    pose_dict = {
        "position": [0.3, 0.0, 0.5],
        "orientation": [1.0, 0.0, 0.0, 0.0],
    }
    action = Action(part="arm", command="pose_reference", value=pose_dict)
    node.submit(node.select_action(selector=lambda _: action))

    assert len(fake_node.publishers) == 1
    msg_type, topic, pub = fake_node.publishers[0]
    assert msg_type.__name__ == "LeasedPoseReference"
    assert topic == "/execution_manager/ingress/teleop/arm/pose_reference"
    assert len(pub.messages) == 1
    msg = pub.messages[0].command
    assert msg.header.frame_id == "arm_base"
    assert msg.tracked_frame == "arm_tcp"
    assert msg.points[0].point.pose.position.x == 0.3


def test_submit_batch_uniform_timestamp():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    actions = [
        Action(part="arm", command="joint_reference", value=[0.1, 0.2]),
        Action(part="gripper", command="joint_reference", value=[0.05]),
    ]
    node.submit(node.select_action(selector=lambda _: actions))

    assert len(fake_node.publishers) == 2
    pub_arm = fake_node.publishers[0][2]
    pub_grip = fake_node.publishers[1][2]
    assert len(pub_arm.messages) == 1
    assert len(pub_grip.messages) == 1

    stamp_arm = pub_arm.messages[0].header.stamp
    stamp_grip = pub_grip.messages[0].header.stamp
    assert stamp_arm.sec == stamp_grip.sec
    assert stamp_arm.nanosec == stamp_grip.nanosec


def test_submit_atomic_validation_rollback_on_error():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    # First action is valid, second action has invalid NaN values
    actions = [
        Action(part="arm", command="joint_reference", value=[0.1, 0.2]),
        Action(part="gripper", command="joint_reference", value=[float("nan")]),
    ]

    with pytest.raises(ValueError, match="NaN or infinity"):
        node.submit(node.select_action(selector=lambda _: actions))

    # All-or-nothing: Nothing should be published to ROS
    for _, _, pub in fake_node.publishers:
        assert len(pub.messages) == 0


def test_submit_undeclared_resource_raises():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    with pytest.raises(KeyError, match="does not provide 'base'"):
        node.submit(node.select_action(selector=lambda _: Action(part="base", command="joint_reference", value=[0.0])))


def test_submit_command_contract_mismatch_raises():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    # Policy expects joint_reference for arm, not twist_reference
    with pytest.raises(
        ValueError, match="requires 'joint_reference', got 'twist_reference'"
    ):
        node.submit(
            node.select_action(selector=lambda _: Action(
                part="arm",
                command="twist_reference",
                value=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            ))
        )


def test_publisher_cached_and_reused():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())
    node.activate()

    node.submit(node.select_action(selector=lambda _: Action(part="arm", command="joint_reference", value=[0.1, 0.2])))
    node.submit(node.select_action(selector=lambda _: Action(part="arm", command="joint_reference", value=[0.2, 0.3])))

    assert len(fake_node.publishers) == 1
    pub = fake_node.publishers[0][2]
    assert len(pub.messages) == 2


def test_format_jtc_guard_diagnostic_lines_heartbeat_timeout():
    from rmi.errors import format_jtc_guard_diagnostic_lines

    lines = format_jtc_guard_diagnostic_lines(
        part_name="arm",
        latest={
            "fault_code": "NONE",
            "guard_state": "IDLE",
            "heartbeat_timeout_ms": "500",
            "time_source": "ros_sim_time",
            "fault_sequence": "1",
            "heartbeat_topic": "/execution/arm/trajectory_guard_heartbeat",
        },
        last_fault={
            "fault_code": "TRAJECTORY_HEARTBEAT_TIMEOUT",
            "guard_state": "CANCELING",
            "heartbeat_timeout_ms": "500",
            "time_source": "ros_sim_time",
            "fault_sequence": "1",
            "heartbeat_topic": "/execution/arm/trajectory_guard_heartbeat",
        },
        status_name="franka_arm_jtc_guard: liveness",
    )
    assert any("likely_cause=" in line and "heartbeat timeout" in line for line in lines)
    assert any("TRAJECTORY_HEARTBEAT_TIMEOUT" in line for line in lines)


def test_format_jtc_guard_diagnostic_lines_without_guard_fault():
    from rmi.errors import format_jtc_guard_diagnostic_lines

    lines = format_jtc_guard_diagnostic_lines(
        part_name="arm",
        latest={
            "fault_code": "NONE",
            "guard_state": "IDLE",
            "heartbeat_timeout_ms": "500",
            "time_source": "ros_sim_time",
            "fault_sequence": "0",
            "heartbeat_topic": "/execution/arm/trajectory_guard_heartbeat",
        },
        last_fault=None,
        status_name="franka_arm_jtc_guard: liveness",
    )
    assert any("without jtc_guard fault" in line for line in lines)


def test_execute_aborted_externally_includes_jtc_guard_heartbeat_cause():
    data = _sample_profile_dict()
    data["groups"]["arm"]["controllers"]["joint_trajectory"] = {
        "name": "arm_jtc",
        "ros_actions": {
            "follow_joint_trajectory": "/execution/arm/follow_joint_trajectory",
        },
        "ros_topics": {
            "trajectory_guard_heartbeat": "/execution/arm/trajectory_guard_heartbeat",
        },
    }
    data["groups"]["arm"]["default_controller"] = "joint_trajectory"
    profile = EmbodimentConfig.from_dict(data)

    class AbortedGoalHandle(FakeGoalHandle):
        def get_result_async(self):
            return FakeFuture(
                SimpleNamespace(
                    status=GoalStatus.STATUS_ABORTED,
                    result=SimpleNamespace(error_code=0, error_string=""),
                )
            )

    class AbortingActionClient(FakeActionClient):
        def send_goal_async(self, goal, feedback_callback=None):
            del feedback_callback
            self.goals.append(goal)
            # Guard buffer already subscribed; publish a fault before wait returns.
            assert fake_node.subscriptions
            _endpoint, callback = fake_node.subscriptions[-1]
            callback(
                SimpleNamespace(
                    status=[
                        SimpleNamespace(
                            name="franka_arm_jtc_guard: liveness",
                            values=[
                                SimpleNamespace(
                                    key="fault_code",
                                    value="TRAJECTORY_HEARTBEAT_TIMEOUT",
                                ),
                                SimpleNamespace(key="guard_state", value="CANCELING"),
                                SimpleNamespace(
                                    key="heartbeat_timeout_ms", value="500"
                                ),
                                SimpleNamespace(
                                    key="time_source", value="ros_sim_time"
                                ),
                                SimpleNamespace(key="fault_sequence", value="3"),
                                SimpleNamespace(
                                    key="heartbeat_topic",
                                    value="/execution/arm/trajectory_guard_heartbeat",
                                ),
                                SimpleNamespace(
                                    key="jtc_action",
                                    value="/execution/arm/follow_joint_trajectory",
                                ),
                            ],
                        )
                    ]
                )
            )
            return FakeFuture(AbortedGoalHandle())

    AbortingActionClient.instances.clear()
    fake_node = FakeNode()
    node = Node(
        "Planner",
        fake_node,
        profile,
        FakeAuthority(),
        action_client_factory=AbortingActionClient,
    )
    node.activate()
    plan = SimpleNamespace(
        valid=True,
        points=[SimpleNamespace(positions=[0.1, 0.2], time_from_start_s=1.0)],
    )

    with pytest.raises(RuntimeError, match="cause=.*heartbeat timeout") as exc_info:
        node["arm"].execute(plan)

    text = str(exc_info.value)
    assert text.startswith("execution failed for arm: ABORTED")
    assert "cause=" in text
    assert "TRAJECTORY_HEARTBEAT_TIMEOUT" in text
    from rmi.errors import ExecutionError

    assert isinstance(exc_info.value, ExecutionError)
    assert "heartbeat timeout" in exc_info.value.cause



def test_pending_activation_succeeds_but_submit_does_not_claim():
    class PendingAuthority(FakeAuthority):
        def set_source_activation(self, name, session, *, active, preempt=False):
            super().set_source_activation(name, session, active=active, preempt=preempt)
            if active:
                self.sources[name]["state"] = 1
                self.sources[name]["lease_id"] = ""
    authority = PendingAuthority()
    node = Node("Policy", FakeNode(), EmbodimentConfig.from_dict(_sample_profile_dict()), authority)
    node.activate()
    assert node.state is SourceState.WAITING
    assert not node.has_control
    with pytest.raises(RuntimeError, match="control"):
        node.submit(Action("arm", "joint_reference", [0., 0.]))
    assert len(authority.claims) == 1
    node.deactivate()
    assert node.state is SourceState.INACTIVE


def test_release_failure_retains_session_for_retry():
    node = Node("Policy", FakeNode(), EmbodimentConfig.from_dict(_sample_profile_dict()), FakeAuthority())
    node.activate()
    session = node._session_id
    original = node._authority.set_source_activation
    def fail(*args, **kwargs):
        raise RuntimeError("release unconfirmed")
    node._authority.set_source_activation = fail
    with pytest.raises(RuntimeError, match="unconfirmed"):
        node.deactivate()
    assert node._session_id == session
    node._authority.set_source_activation = original
    node.deactivate()
    assert node._session_id is None


def test_missed_preemption_event_still_runs_acquisition_hook_for_new_lease():
    calls = []
    class Producer:
        def on_control_acquired(self, observation):
            calls.append(observation)
        def select_action(self, observation):
            return Action("arm", "joint_reference", [0., 0.])
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), EmbodimentConfig.from_dict(_sample_profile_dict()), authority, Producer())
    node.activate()
    node.submit(node.select_action("first observation"))
    authority.sources["Policy"]["lease_id"] = "new-lease"
    node.submit(node.select_action("new observation"))
    node.submit(node.select_action("next tick"))
    assert calls == ["first observation", "new observation"]


def test_unprepared_actions_are_rejected_even_when_controlling():
    fake = FakeNode()
    node = Node("Policy", fake, EmbodimentConfig.from_dict(_sample_profile_dict()), FakeAuthority())
    node.activate()
    with pytest.raises(RuntimeError, match="revoked"):
        node.submit(Action("arm", "joint_reference", [0., 0.]))
    with pytest.raises(TypeError, match="select_action"):
        node["manipulator"].submit([0., 0., 0.])
    assert not fake.publishers


def test_manual_selector_captures_epoch_before_computation():
    fake, authority = FakeNode(), FakeAuthority()
    node = Node("Policy", fake, EmbodimentConfig.from_dict(_sample_profile_dict()), authority)
    node.activate()
    def compute(_):
        authority.sources["Policy"]["lease_id"] = "replacement-lease"
        return [0., 0., 0.]
    actions = node["manipulator"].select_action(selector=compute)
    with pytest.raises(RuntimeError, match="revoked"):
        node.submit(actions)
    assert not fake.publishers


def test_mixed_prepared_and_unprepared_batch_publishes_nothing():
    fake = FakeNode()
    node = Node("Policy", fake, EmbodimentConfig.from_dict(_sample_profile_dict()), FakeAuthority())
    node.activate()
    prepared = node.select_action(selector=lambda _: Action("arm", "joint_reference", [0., 0.]))
    with pytest.raises(RuntimeError, match="revoked"):
        node.submit([prepared, Action("gripper", "joint_reference", [0.])])
    assert not fake.publishers


def test_selection_cannot_relabel_an_already_prepared_action():
    node = Node("Policy", FakeNode(), EmbodimentConfig.from_dict(_sample_profile_dict()), FakeAuthority())
    node.activate()
    action = node.select_action(selector=lambda _: Action("arm", "joint_reference", [0., 0.]))
    node._authority.sources["Policy"]["lease_id"] = "new-lease"
    with pytest.raises(RuntimeError, match="rebind"):
        node.select_action(selector=lambda _: action)


def test_plain_action_endpoint_is_rejected_in_profile():
    profile = _sample_profile_dict()
    profile["sources"]["Planner"]["inputs"]["arm"]["action"] = "/action_sources/planner/arm/follow_joint_trajectory"
    with pytest.raises(ValueError, match="leased ingress"):
        EmbodimentConfig.from_dict(profile)
