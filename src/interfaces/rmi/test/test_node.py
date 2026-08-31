from types import SimpleNamespace
from typing import Any, ClassVar

import pytest
from action_msgs.msg import GoalStatus
from execution_manager_interfaces.msg import ResourceAuthority
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from rmi import Action, EmbodimentConfig, Node, NodeStatus, Robot
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
        self._current_time = SimpleNamespace(sec=100, nanosec=500000)

    def create_publisher(self, message_type, endpoint, qos):
        del qos
        publisher = FakePublisher()
        self.publishers.append((message_type, endpoint, publisher))
        return publisher

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: self._current_time)
        )


class FakeAuthority:
    def __init__(self):
        self.allocations = {}
        self.claims = []
        self.releases = []

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
                        "action": "/action_sources/planner/arm/follow_joint_trajectory",
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

    node["manipulator"].submit([0.1, 0.2, 0.04])

    assert [entry[1] for entry in fake_node.publishers] == [
        "/action_sources/policy/arm/joint_reference",
        "/action_sources/policy/gripper/joint_reference",
    ]
    assert list(fake_node.publishers[0][2].messages[0].points[0].positions) == [
        0.1,
        0.2,
    ]
    assert list(fake_node.publishers[1][2].messages[0].points[0].positions) == [0.04]


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
        "action": "/action_sources/planner/arm2/follow_joint_trajectory",
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
        "action": "/action_sources/planner/gripper/gripper_command",
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
    assert node.config.inputs == {}
    with pytest.raises(RuntimeError, match="no in-process ingress binding for 'arm'"):
        node.submit(Action(part="arm", command="joint_reference", value=[0.0, 0.0]))


def test_node_status_aggregates_authority_without_exposing_resources():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), profile, authority)

    assert node.status is NodeStatus.INACTIVE
    assert not node.is_active

    authority.allocations = {
        "arm": {
            "authority_state": ResourceAuthority.OWNED,
            "source_instance": "Policy",
        }
    }
    assert node.status is NodeStatus.PARTIAL
    assert node.is_active

    authority.allocations["gripper"] = {
        "authority_state": ResourceAuthority.OWNED,
        "source_instance": "Policy",
    }
    assert node.status is NodeStatus.ACTIVE

    authority.allocations["arm"]["authority_state"] = ResourceAuthority.FAULT
    assert node.status is NodeStatus.FAULT


def test_submit_none_or_empty_is_noop():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())

    node.submit(None)
    node.submit([])
    node.submit(())
    assert len(fake_node.publishers) == 0


def test_activate_scopes_preemptive_authority_without_exposing_lease():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), profile, authority)

    with node.activate(preempt=True) as active_node:
        assert active_node is node
        assert authority.claims == [
            (
                "POLICY",
                "Policy",
                {"arm": "joint_reference", "gripper": "joint_reference"},
                True,
                {"activation": "rmi_node_scope"},
            )
        ]
        assert authority.releases == []

    assert authority.releases == ["node-lease"]


def test_close_releases_active_node_authority():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), profile, authority)

    node.activate(preempt=True)
    node.close()

    assert authority.releases == ["node-lease"]


def test_deactivate_releases_lease_restored_for_same_source():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    authority = FakeAuthority()
    node = Node("Policy", FakeNode(), profile, authority)

    node.activate()
    authority.allocations = {
        "arm": {
            "authority_state": ResourceAuthority.OWNED,
            "source_instance": "Policy",
            "lease_id": "restored-lease",
        },
        "gripper": {
            "authority_state": ResourceAuthority.OWNED,
            "source_instance": "Policy",
            "lease_id": "restored-lease",
        },
    }
    node.deactivate()

    assert authority.releases == ["node-lease", "restored-lease"]


def test_execute_uses_plain_profile_action_and_returns_handle():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    FakeActionClient.instances.clear()
    node = Node(
        "Planner",
        FakeNode(),
        profile,
        FakeAuthority(),
        action_client_factory=FakeActionClient,
    )
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

    action = Action(part="arm", command="joint_reference", value=[0.25, -0.4])
    node.submit(action)

    assert len(fake_node.publishers) == 1
    msg_type, topic, pub = fake_node.publishers[0]
    assert msg_type is JointTrajectory
    assert topic == "/action_sources/policy/arm/joint_reference"
    assert len(pub.messages) == 1
    msg = pub.messages[0]
    assert list(msg.joint_names) == ["joint1", "joint2"]
    assert list(msg.points[0].positions) == [0.25, -0.4]
    assert msg.header.stamp.sec == 100
    assert msg.header.stamp.nanosec == 500000


def test_submit_twist_reference():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("TeleopTwist", fake_node, profile, FakeAuthority())

    action = Action(
        part="arm",
        command="twist_reference",
        value=[0.1, 0.2, 0.3, 0.0, 0.0, 0.5],
    )
    node.submit(action)

    assert len(fake_node.publishers) == 1
    msg_type, topic, pub = fake_node.publishers[0]
    assert msg_type is TwistStamped
    assert topic == "/action_sources/teleop/arm/twist_reference"
    assert len(pub.messages) == 1
    msg = pub.messages[0]
    assert msg.header.frame_id == "arm_base"
    assert msg.twist.linear.x == 0.1
    assert msg.twist.linear.y == 0.2
    assert msg.twist.angular.z == 0.5
    assert msg.header.stamp.sec == 100


def test_submit_pose_reference():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("TeleopPose", fake_node, profile, FakeAuthority())

    pose_dict = {
        "position": [0.3, 0.0, 0.5],
        "orientation": [1.0, 0.0, 0.0, 0.0],
    }
    action = Action(part="arm", command="pose_reference", value=pose_dict)
    node.submit(action)

    assert len(fake_node.publishers) == 1
    msg_type, topic, pub = fake_node.publishers[0]
    assert msg_type is CartesianTrajectory
    assert topic == "/action_sources/teleop/arm/pose_reference"
    assert len(pub.messages) == 1
    msg = pub.messages[0]
    assert msg.header.frame_id == "arm_base"
    assert msg.tracked_frame == "arm_tcp"
    assert msg.points[0].point.pose.position.x == 0.3


def test_submit_batch_uniform_timestamp():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())

    actions = [
        Action(part="arm", command="joint_reference", value=[0.1, 0.2]),
        Action(part="gripper", command="joint_reference", value=[0.05]),
    ]
    node.submit(actions)

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

    # First action is valid, second action has invalid NaN values
    actions = [
        Action(part="arm", command="joint_reference", value=[0.1, 0.2]),
        Action(part="gripper", command="joint_reference", value=[float("nan")]),
    ]

    with pytest.raises(ValueError, match="NaN or infinity"):
        node.submit(actions)

    # All-or-nothing: Nothing should be published to ROS
    for _, _, pub in fake_node.publishers:
        assert len(pub.messages) == 0


def test_submit_undeclared_resource_raises():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())

    with pytest.raises(KeyError, match="does not provide 'base'"):
        node.submit(Action(part="base", command="joint_reference", value=[0.0]))


def test_submit_command_contract_mismatch_raises():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())

    # Policy expects joint_reference for arm, not twist_reference
    with pytest.raises(
        ValueError, match="requires 'joint_reference', got 'twist_reference'"
    ):
        node.submit(
            Action(
                part="arm",
                command="twist_reference",
                value=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        )


def test_publisher_cached_and_reused():
    profile = EmbodimentConfig.from_dict(_sample_profile_dict())
    fake_node = FakeNode()
    node = Node("Policy", fake_node, profile, FakeAuthority())

    node.submit(Action(part="arm", command="joint_reference", value=[0.1, 0.2]))
    node.submit(Action(part="arm", command="joint_reference", value=[0.2, 0.3]))

    assert len(fake_node.publishers) == 1
    pub = fake_node.publishers[0][2]
    assert len(pub.messages) == 2
