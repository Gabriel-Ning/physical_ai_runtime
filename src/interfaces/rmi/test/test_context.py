from types import SimpleNamespace

import pytest
from execution_manager_interfaces.msg import AuthorityEvent, ResourceAuthority
from rmi import Context, EmbodimentConfig
from rmi.selection import AuthoritySnapshot
from sensor_msgs.msg import JointState


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.errors = []

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                nanoseconds=1_000_000_000,
                to_msg=lambda: SimpleNamespace(sec=1, nanosec=0),
            )
        )

    def get_logger(self):
        return SimpleNamespace(error=self.errors.append)


class FaultedAuthority:
    def __init__(self):
        self.recovery_calls = 0
        self.faulted = True

    def require_execution_manager(self, *, timeout_sec=None):
        del timeout_sec

    def describe_authority(self):
        state = ResourceAuthority.FAULT if self.faulted else ResourceAuthority.UNOWNED
        return AuthoritySnapshot({"arm": {"authority_state": state}})

    def clear_fault(self, resources):
        assert resources == {"arm": "joint_reference"}
        self.recovery_calls += 1
        self.faulted = False
        return self.describe_authority()

    def get_allocations(self):
        return self.describe_authority().resources

    def get_events(self):
        event = AuthorityEvent()
        event.type = AuthorityEvent.TRANSITION_FAILED
        event.event_id = "event-42"
        event.lease_id = "lease-7"
        event.source_role = 1
        event.source_instance = "DummyPolicy"
        event.resources = ["arm"]
        event.reason = "switch_controller returned ok=false"
        return [event]

    def close(self):
        pass


def _profile():
    return EmbodimentConfig.from_dict(
        {
            "metadata": {"name": "test"},
            "groups": {
                "arm": {
                    "type": "arm",
                    "joint_names": ["j1"],
                    "controller_manager": "/controller_manager",
                    "default_controller": "joint_space_reference",
                    "controllers": {
                        "joint_space_reference": {
                            "name": "arm_jspc",
                            "ros_topics": {
                                "joint_reference": "/execution/arm/joint_reference"
                            },
                        }
                    },
                }
            },
            "nodes": {
                "Policy": {
                    "source_role": "POLICY",
                    "resources": {"arm": "joint_reference"},
                }
            },
        }
    )


def _faulted_context():
    authority = FaultedAuthority()
    context = Context(
        _profile(),
        FakeNode(),
        authority_client=authority,
        timeout_sec=0.1,
    )
    context.robot.update_joint_state(JointState(name=["j1"], position=[0.0]))
    return context, authority


def test_wait_until_ready_does_not_clear_fault_by_default():
    context, authority = _faulted_context()

    with pytest.raises(RuntimeError, match="explicit recovery is required"):
        context.wait_until_ready(timeout=0.1)

    assert authority.recovery_calls == 0
    assert len(context.node.errors) == 1
    assert "resource=arm" in context.node.errors[0]
    assert "switch_controller returned ok=false" in context.node.errors[0]
    assert "source=DummyPolicy" in context.node.errors[0]


def test_wait_until_ready_recovers_fault_only_when_explicitly_requested():
    context, authority = _faulted_context()

    context.wait_until_ready(timeout=0.1, recover_faults=True)

    assert authority.recovery_calls == 1
