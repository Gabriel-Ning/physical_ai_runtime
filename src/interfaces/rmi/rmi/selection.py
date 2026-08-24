"""Typed client for the graph-wide C++ Execution Manager."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Protocol

from execution_manager_interfaces.msg import AuthorityEvent, AuthorityStatus, ResourceClaim
from execution_manager_interfaces.srv import ClaimControl, ReleaseControl
from diagnostic_msgs.msg import KeyValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

CLAIM_SERVICE = "/execution_manager/claim"
RELEASE_SERVICE = "/execution_manager/release"
AUTHORITY_STATUS_TOPIC = "/execution_manager/authority_status"
AUTHORITY_EVENTS_TOPIC = "/execution_manager/authority_events"


class SourceRole(IntEnum):
    POLICY = ClaimControl.Request.POLICY
    TELEOP = ClaimControl.Request.TELEOP
    PLANNER = ClaimControl.Request.PLANNER

    @classmethod
    def parse(cls, value: SourceRole | str | int) -> SourceRole:
        if isinstance(value, cls):
            return value
        if isinstance(value, str):
            try:
                return cls[value.strip().upper()]
            except KeyError as exc:
                raise ValueError(f"unknown source role {value!r}") from exc
        return cls(value)


@dataclass(frozen=True)
class EndpointBinding:
    resource: str
    command_contract: str
    endpoint: str
    is_action: bool


@dataclass(frozen=True)
class LeaseGrant:
    lease_id: str
    endpoints: dict[tuple[str, str], EndpointBinding]


class ExecutionManagerUnavailableError(RuntimeError):
    pass


class AuthorityClient(Protocol):
    def require_execution_manager(self, *, timeout_sec: float | None = None) -> None: ...

    def claim(
        self,
        source_role: SourceRole | str | int,
        source_instance: str,
        resources: dict[str, str],
        *,
        preempt: bool = False,
        metadata: dict[str, str] | None = None,
    ) -> LeaseGrant: ...

    def release(self, lease_id: str) -> None: ...

    def get_allocations(self) -> dict[str, dict[str, Any]]: ...

    def get_events(self, *, lease_id: str | None = None) -> list[AuthorityEvent]: ...

    def close(self) -> None: ...


class ExecutionManagerClient:
    """Synchronous Robot SDK facade; all authority remains in C++ EM."""

    def __init__(self, profile: Any, node: Any, *, timeout_sec: float = 5.0,
                 status_timeout_sec: float = 3.0) -> None:
        del profile
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self._node = node
        self._timeout_sec = timeout_sec
        if status_timeout_sec <= 0.0:
            raise ValueError("status_timeout_sec must be positive")
        self._status_timeout_sec = status_timeout_sec
        self._last_status_monotonic: float | None = None
        self._claim_client = node.create_client(ClaimControl, CLAIM_SERVICE)
        self._release_client = node.create_client(ReleaseControl, RELEASE_SERVICE)
        self._allocations: dict[str, dict[str, Any]] = {}
        self._events: deque[AuthorityEvent] = deque(maxlen=2048)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos = QoSProfile(depth=100)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        self._status_subscription = node.create_subscription(
            AuthorityStatus, AUTHORITY_STATUS_TOPIC, self._on_status, status_qos
        )
        self._event_subscription = node.create_subscription(
            AuthorityEvent, AUTHORITY_EVENTS_TOPIC, self._on_event, event_qos
        )

    def require_execution_manager(self, *, timeout_sec: float | None = None) -> None:
        timeout = self._timeout_sec if timeout_sec is None else timeout_sec
        if timeout <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if not self._claim_client.wait_for_service(timeout_sec=timeout):
            raise ExecutionManagerUnavailableError(
                f"Execution Manager is unavailable at {CLAIM_SERVICE}"
            )

    def claim(
        self,
        source_role: SourceRole | str | int,
        source_instance: str,
        resources: dict[str, str],
        *,
        preempt: bool = False,
        metadata: dict[str, str] | None = None,
    ) -> LeaseGrant:
        if not source_instance:
            raise ValueError("source_instance must not be empty")
        if not resources:
            raise ValueError("claim requires at least one resource")
        self.require_execution_manager()
        request = ClaimControl.Request()
        request.source_role = int(SourceRole.parse(source_role))
        request.source_instance = source_instance
        request.preempt = bool(preempt)
        for key, value in (metadata or {}).items():
            request.metadata.append(KeyValue(key=str(key), value=str(value)))
        for resource, command_contract in resources.items():
            item = ResourceClaim()
            item.resource = resource
            item.command_contract = command_contract
            request.resources.append(item)
        response = _wait_future(
            self._claim_client.call_async(request), self._timeout_sec, "control claim"
        )
        if response is None or not response.success:
            raise RuntimeError(
                "empty Execution Manager response"
                if response is None
                else response.message
            )
        bindings = {
            (item.resource, item.command_contract): EndpointBinding(
                resource=item.resource,
                command_contract=item.command_contract,
                endpoint=item.endpoint,
                is_action=item.is_action,
            )
            for item in response.endpoints
        }
        if set(bindings) != {(name, contract) for name, contract in resources.items()}:
            raise RuntimeError("Execution Manager returned incomplete endpoint bindings")
        return LeaseGrant(response.lease_id, bindings)

    def release(self, lease_id: str) -> None:
        if not lease_id:
            raise ValueError("lease_id must not be empty")
        if not self._release_client.wait_for_service(timeout_sec=self._timeout_sec):
            raise ExecutionManagerUnavailableError(
                f"Execution Manager is unavailable at {RELEASE_SERVICE}"
            )
        request = ReleaseControl.Request()
        request.lease_id = lease_id
        response = _wait_future(
            self._release_client.call_async(request), self._timeout_sec, "control release"
        )
        if response is None or not response.success:
            raise RuntimeError(
                "empty Execution Manager response"
                if response is None
                else response.message
            )

    def get_allocations(self) -> dict[str, dict[str, Any]]:
        if (
            self._last_status_monotonic is None
            or time.monotonic() - self._last_status_monotonic
            > self._status_timeout_sec
        ):
            return {}
        return {resource: dict(value) for resource, value in self._allocations.items()}

    def get_events(self, *, lease_id: str | None = None) -> list[AuthorityEvent]:
        if lease_id is None:
            return list(self._events)
        return [event for event in self._events if event.lease_id == lease_id]

    def close(self) -> None:
        if hasattr(self._node, "destroy_subscription"):
            self._node.destroy_subscription(self._status_subscription)
            self._node.destroy_subscription(self._event_subscription)
        if hasattr(self._node, "destroy_client"):
            self._node.destroy_client(self._claim_client)
            self._node.destroy_client(self._release_client)

    def _on_status(self, message: AuthorityStatus) -> None:
        self._last_status_monotonic = time.monotonic()
        self._allocations = {
            item.resource: {
                "authority_state": int(item.authority_state),
                "lease_id": item.lease_id,
                "source_role": int(item.source_role),
                "source_instance": item.source_instance,
                "command_contract": item.command_contract,
                "requested_controller": item.requested_controller,
                "observed_controllers": tuple(item.observed_controllers),
            }
            for item in message.resources
        }

    def _on_event(self, message: AuthorityEvent) -> None:
        self._events.append(message)


def _wait_future(future: Any, timeout: float | None, context: str) -> Any:
    deadline = None if timeout is None else time.monotonic() + timeout
    while not future.done():
        if deadline is not None and time.monotonic() >= deadline:
            future.cancel()
            raise TimeoutError(f"{context} timed out")
        time.sleep(0.01)
    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()


__all__ = [
    "AUTHORITY_EVENTS_TOPIC",
    "AUTHORITY_STATUS_TOPIC",
    "CLAIM_SERVICE",
    "RELEASE_SERVICE",
    "EndpointBinding",
    "ExecutionManagerUnavailableError",
    "LeaseGrant",
    "AuthorityClient",
    "ExecutionManagerClient",
    "SourceRole",
]
