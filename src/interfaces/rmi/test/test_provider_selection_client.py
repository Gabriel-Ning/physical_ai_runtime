from types import SimpleNamespace

import pytest
from execution_manager_interfaces.msg import (
    AuthorityEvent,
    AuthorityStatus,
    CommandEndpoint,
    ResourceAuthority,
)
from execution_manager_interfaces.srv import ClaimControl, ReleaseControl

from rmi.selection import (
    AUTHORITY_EVENTS_TOPIC,
    AUTHORITY_STATUS_TOPIC,
    CLAIM_SERVICE,
    RELEASE_SERVICE,
    ExecutionManagerUnavailableError,
    ExecutionManagerClient,
    SourceRole,
)


class DoneFuture:
    def __init__(self, response):
        self._response = response

    def done(self):
        return True

    def exception(self):
        return None

    def result(self):
        return self._response

    def cancel(self):
        return None


class FakeClient:
    def __init__(self, response, *, available=True):
        self.response = response
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return self.available and timeout_sec > 0.0

    def call_async(self, request):
        self.requests.append(request)
        return DoneFuture(self.response)


class FakeNode:
    def __init__(self, *, available=True):
        endpoint = CommandEndpoint()
        endpoint.resource = "arm"
        endpoint.command_contract = "joint_reference"
        endpoint.endpoint = "/action_sources/policy/arm/joint_reference"
        claim = ClaimControl.Response()
        claim.success = True
        claim.lease_id = "lease-1"
        claim.endpoints = [endpoint]
        release = ReleaseControl.Response()
        release.success = True
        self.clients = {
            CLAIM_SERVICE: FakeClient(claim, available=available),
            RELEASE_SERVICE: FakeClient(release, available=available),
        }
        self.subscriptions = []

    def create_client(self, service_type, endpoint):
        del service_type
        return self.clients[endpoint]

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type, topic=topic, callback=callback, qos=qos
        )
        self.subscriptions.append(subscription)
        return subscription


def test_typed_claim_returns_lease_and_static_endpoint():
    node = FakeNode()
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)

    grant = client.claim(
        "POLICY",
        "policy-v2",
        {"arm": "joint_reference"},
        metadata={"task": "pick"},
    )

    assert grant.lease_id == "lease-1"
    assert grant.endpoints[("arm", "joint_reference")].endpoint.endswith(
        "/policy/arm/joint_reference"
    )
    request = node.clients[CLAIM_SERVICE].requests[0]
    assert request.source_role == SourceRole.POLICY
    assert request.source_instance == "policy-v2"
    assert request.preempt is False
    assert [(item.key, item.value) for item in request.metadata] == [("task", "pick")]


def test_status_and_events_are_typed_and_lease_addressable():
    node = FakeNode()
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)
    status_sub = next(x for x in node.subscriptions if x.topic == AUTHORITY_STATUS_TOPIC)
    event_sub = next(x for x in node.subscriptions if x.topic == AUTHORITY_EVENTS_TOPIC)

    authority = ResourceAuthority()
    authority.resource = "arm"
    authority.authority_state = ResourceAuthority.OWNED
    authority.lease_id = "lease-1"
    authority.source_instance = "policy-v2"
    status = AuthorityStatus()
    status.resources = [authority]
    status_sub.callback(status)
    event = AuthorityEvent()
    event.lease_id = "lease-1"
    event_sub.callback(event)

    assert client.get_allocations()["arm"]["lease_id"] == "lease-1"
    assert client.get_events(lease_id="lease-1") == [event]


def test_stale_status_does_not_report_authority(monkeypatch):
    node = FakeNode()
    client = ExecutionManagerClient(
        None, node, timeout_sec=0.1, status_timeout_sec=1.0
    )
    status_sub = next(x for x in node.subscriptions if x.topic == AUTHORITY_STATUS_TOPIC)
    authority = ResourceAuthority()
    authority.resource = "arm"
    status = AuthorityStatus()
    status.resources = [authority]
    monkeypatch.setattr("rmi.selection.time.monotonic", lambda: 10.0)
    status_sub.callback(status)
    assert "arm" in client.get_allocations()
    monkeypatch.setattr("rmi.selection.time.monotonic", lambda: 11.1)
    assert client.get_allocations() == {}


def test_release_uses_exact_lease_identity():
    node = FakeNode()
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)
    client.release("lease-1")
    assert node.clients[RELEASE_SERVICE].requests[0].lease_id == "lease-1"


def test_missing_execution_manager_is_explicit():
    client = ExecutionManagerClient(None, FakeNode(available=False), timeout_sec=0.01)
    with pytest.raises(ExecutionManagerUnavailableError):
        client.require_execution_manager()
