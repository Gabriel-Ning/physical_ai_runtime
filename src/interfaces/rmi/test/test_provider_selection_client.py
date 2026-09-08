from types import SimpleNamespace

import pytest
from execution_manager_interfaces.msg import (
    AuthorityEvent,
    AuthorityStatus,
    ResourceAuthority,
)
from execution_manager_interfaces.srv import SetSourceActivation, RecoverResources
from rmi.selection import (
    AUTHORITY_EVENTS_TOPIC,
    AUTHORITY_STATUS_TOPIC,
    SOURCE_SERVICE,
    RECOVERY_SERVICE,
    AuthoritySnapshot,
    ExecutionManagerClient,
    ExecutionManagerUnavailableError,
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
        self.clients = {
            SOURCE_SERVICE: FakeClient(SetSourceActivation.Response(success=True), available=available),
            RECOVERY_SERVICE: FakeClient(RecoverResources.Response(success=True), available=available),
        }
        self.subscriptions = []

    def create_client(self, service_type, endpoint):
        del service_type
        return self.clients.setdefault(endpoint, FakeClient(SimpleNamespace(success=True, message="ok")))

    def create_publisher(self, message_type, endpoint, qos):
        return SimpleNamespace(publish=lambda message: None)

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type, topic=topic, callback=callback, qos=qos
        )
        self.subscriptions.append(subscription)
        return subscription


def test_only_source_protocol_is_exposed():
    node = FakeNode()
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)
    try:
        client.set_source_activation("Policy", "session", active=True)
        request = node.clients[SOURCE_SERVICE].requests[0]
        assert request.source_instance == "Policy"
        assert request.session_id == "session"
        assert request.active and not request.preempt
        assert not hasattr(client, "claim")
        assert not hasattr(client, "release")
    finally:
        client.close()


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


def test_missing_execution_manager_is_explicit():
    client = ExecutionManagerClient(None, FakeNode(available=False), timeout_sec=0.01)
    with pytest.raises(ExecutionManagerUnavailableError):
        client.require_execution_manager()


def test_describe_authority_reports_faults():
    node = FakeNode()
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)
    status_sub = next(x for x in node.subscriptions if x.topic == AUTHORITY_STATUS_TOPIC)
    authority = ResourceAuthority()
    authority.resource = "arm"
    authority.authority_state = ResourceAuthority.FAULT
    status = AuthorityStatus()
    status.resources = [authority]
    status_sub.callback(status)

    snapshot = client.describe_authority()
    assert isinstance(snapshot, AuthoritySnapshot)
    assert snapshot.faults == ("arm",)
    assert snapshot.state_name("arm") == "FAULT"
    assert snapshot.state_name("missing") == "UNKNOWN"


def test_recovery_does_not_claim_authority():
    node = FakeNode()
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)
    sub = next(x for x in node.subscriptions if x.topic == AUTHORITY_STATUS_TOPIC)
    recovery = node.clients[RECOVERY_SERVICE]
    original = recovery.call_async
    def recover(request):
        future = original(request)
        sub.callback(AuthorityStatus(resources=[ResourceAuthority(resource="arm", authority_state=ResourceAuthority.UNOWNED)]))
        return future
    recovery.call_async = recover
    try:
        snapshot = client.clear_fault({"arm": "joint_reference"})
        assert recovery.requests[0].resources == ["arm"]
        assert node.clients[SOURCE_SERVICE].requests == []
        assert snapshot.unowned == ("arm",)
    finally:
        client.close()


def test_recovery_rejection_is_reported():
    node = FakeNode()
    node.clients[RECOVERY_SERVICE].response = RecoverResources.Response(success=False, message="resources_owned")
    client = ExecutionManagerClient(None, node, timeout_sec=0.1)
    try:
        with pytest.raises(RuntimeError, match="resources_owned"):
            client.clear_fault({"arm": "joint_reference"})
    finally:
        client.close()


def test_candidate_registration_has_independent_heartbeat_membership():
    client = ExecutionManagerClient(None, FakeNode(), timeout_sec=0.1)
    try:
        client.set_source_activation("Policy", "session", active=True)
        assert "session" in client._sessions
        assert not client.get_sources()  # Registration is not an observed grant.
        client.set_source_activation("Policy", "session", active=False)
        assert not client._sessions
    finally:
        client.close()
