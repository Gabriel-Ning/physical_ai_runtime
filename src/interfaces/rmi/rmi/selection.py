"""Typed client for the graph-wide C++ Execution Manager."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Protocol

from execution_manager_interfaces.msg import (
    AuthorityEvent,
    AuthorityStatus,
    ResourceAuthority,
    SourceLifecycleStatus,
)
from execution_manager_interfaces.srv import RecoverResources, SetSourceActivation
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .errors import ExecutionManagerUnavailableError

SOURCE_SERVICE = "/execution_manager/source_activation"
RECOVERY_SERVICE = "/execution_manager/recover_resources"
AUTHORITY_STATUS_TOPIC = "/execution_manager/authority_status"
AUTHORITY_EVENTS_TOPIC = "/execution_manager/authority_events"

_AUTHORITY_STATE_NAME = {
    int(ResourceAuthority.UNOWNED): "UNOWNED",
    int(ResourceAuthority.TRANSITIONING): "TRANSITIONING",
    int(ResourceAuthority.OWNED): "OWNED",
    int(ResourceAuthority.FAULT): "FAULT",
}


class SourceRole(IntEnum):
    POLICY = 1
    TELEOP = 2
    PLANNER = 3
    MEMORY = 4

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
class AuthoritySnapshot:
    """Read-only view of EM authority status for application startup checks."""

    resources: dict[str, dict[str, Any]]

    @property
    def faults(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, item in self.resources.items()
            if int(item.get("authority_state", ResourceAuthority.UNOWNED))
            == int(ResourceAuthority.FAULT)
        )

    @property
    def owned(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, item in self.resources.items()
            if int(item.get("authority_state", ResourceAuthority.UNOWNED))
            == int(ResourceAuthority.OWNED)
        )

    @property
    def unowned(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, item in self.resources.items()
            if int(item.get("authority_state", ResourceAuthority.UNOWNED))
            == int(ResourceAuthority.UNOWNED)
        )

    def state_name(self, resource: str) -> str:
        item = self.resources.get(resource)
        if item is None:
            return "UNKNOWN"
        return _AUTHORITY_STATE_NAME.get(
            int(item.get("authority_state", -1)), "UNKNOWN"
        )


class AuthorityClient(Protocol):
    def set_source_activation(
        self, name: str, session: str, *, active: bool, preempt: bool = False
    ) -> None: ...

    def get_sources(self) -> dict[str, dict[str, Any]]: ...

    def require_execution_manager(self, *, timeout_sec: float | None = None) -> None: ...

    def get_allocations(self) -> dict[str, dict[str, Any]]: ...

    def describe_authority(self) -> AuthoritySnapshot: ...

    def clear_fault(self, resources: dict[str, str]) -> AuthoritySnapshot: ...

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
        self._recovery_client = node.create_client(RecoverResources, RECOVERY_SERVICE)
        self._allocations: dict[str, dict[str, Any]] = {}
        self._sources: dict[str, dict[str, Any]] = {}
        self._source_status_time: float | None = None
        self._source_client = node.create_client(SetSourceActivation, SOURCE_SERVICE)
        self._heartbeat_pub = node.create_publisher(String, "/execution_manager/source_heartbeat", 10)
        self._sessions: set[str] = set()
        self._sessions_lock = threading.Lock()
        self._stop_heartbeat = threading.Event()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

        self._events: deque[AuthorityEvent] = deque(maxlen=2048)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos = QoSProfile(depth=100)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        # EM retains its recent audit trail so a client started after a fault
        # can still report the original transition failure reason.
        event_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._source_subscription = node.create_subscription(
            SourceLifecycleStatus, "/execution_manager/source_status", self._on_sources, status_qos
        )
        self._status_subscription = node.create_subscription(
            AuthorityStatus, AUTHORITY_STATUS_TOPIC, self._on_status, status_qos
        )
        self._event_subscription = node.create_subscription(
            AuthorityEvent, AUTHORITY_EVENTS_TOPIC, self._on_event, event_qos
        )

    def set_source_activation(self, name: str, session: str, *, active: bool, preempt: bool = False) -> None:
        if not self._source_client.wait_for_service(timeout_sec=self._timeout_sec):
            raise ExecutionManagerUnavailableError("source lifecycle service unavailable")
        request = SetSourceActivation.Request()
        request.source_instance, request.session_id = name, session
        request.active, request.preempt = active, preempt
        if active:
            with self._sessions_lock:
                self._sessions.add(session)
        try:
            response = _wait_future(self._source_client.call_async(request), max(12.0, self._timeout_sec), "source activation")
            if response is None or not response.success:
                raise RuntimeError("source activation failed" if response is None else response.message)
        except BaseException:
            if active:
                with self._sessions_lock:
                    self._sessions.discard(session)
            raise
        if not active:
            with self._sessions_lock:
                self._sessions.discard(session)
        if active and preempt:
            deadline = time.monotonic() + self._timeout_sec
            while time.monotonic() < deadline:
                record = self.get_sources().get(name, {})
                if record.get("session_id") == session and record.get("state") == 3:
                    break
                time.sleep(0.005)
            else:
                raise ExecutionManagerUnavailableError("grant acknowledged but source status unavailable")

    def get_sources(self) -> dict[str, dict[str, Any]]:
        if self._source_status_time is None:
            return {}
        if time.monotonic() - self._source_status_time > self._status_timeout_sec:
            raise ExecutionManagerUnavailableError("source lifecycle status expired")
        return {name: dict(value) for name, value in self._sources.items()}

    def _on_sources(self, message: SourceLifecycleStatus) -> None:
        self._sources = {item.source_instance: {
            "state": int(item.state), "mode": int(item.mode),
            "session_id": item.session_id, "lease_id": item.lease_id,
            "reason": item.reason,
        } for item in message.sources}
        self._source_status_time = time.monotonic()

    def _heartbeat_loop(self) -> None:
        while not self._stop_heartbeat.wait(0.5):
            with self._sessions_lock:
                sessions = tuple(self._sessions)
            for session in sessions:
                try:
                    self._heartbeat_pub.publish(String(data=session))
                except Exception:
                    # Shutdown may invalidate the ROS context before close().
                    if not self._node.context.ok():
                        return

    def require_execution_manager(self, *, timeout_sec: float | None = None) -> None:
        timeout = self._timeout_sec if timeout_sec is None else timeout_sec
        if timeout <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if not self._source_client.wait_for_service(timeout_sec=timeout):
            raise ExecutionManagerUnavailableError(
                f"Execution Manager is unavailable at {SOURCE_SERVICE}"
            )

    def get_allocations(self) -> dict[str, dict[str, Any]]:
        if (
            self._last_status_monotonic is None
            or time.monotonic() - self._last_status_monotonic
            > self._status_timeout_sec
        ):
            return {}
        return {resource: dict(value) for resource, value in self._allocations.items()}

    def describe_authority(self) -> AuthoritySnapshot:
        """Return the latest EM authority snapshot (empty if status is stale)."""
        return AuthoritySnapshot(self.get_allocations())

    def clear_fault(self, resources: dict[str, str]) -> AuthoritySnapshot:
        """Recover faulted resources without acquiring execution authority."""
        if not resources:
            raise ValueError("clear_fault requires at least one resource")
        if not self._recovery_client.wait_for_service(timeout_sec=self._timeout_sec):
            raise ExecutionManagerUnavailableError("resource recovery service unavailable")
        request = RecoverResources.Request(resources=list(resources))
        response = _wait_future(self._recovery_client.call_async(request),
                                self._timeout_sec, "resource recovery")
        if response is None or not response.success:
            raise RuntimeError("empty recovery response" if response is None else response.message)
        deadline = time.monotonic() + self._timeout_sec
        while time.monotonic() < deadline:
            snapshot = self.describe_authority()
            if all(name in snapshot.resources and name not in snapshot.faults for name in resources):
                return snapshot
            time.sleep(0.01)
        raise ExecutionManagerUnavailableError("recovery status was not confirmed")

    def get_events(self, *, lease_id: str | None = None) -> list[AuthorityEvent]:
        if lease_id is None:
            return list(self._events)
        return [event for event in self._events if event.lease_id == lease_id]

    def close(self) -> None:
        self._stop_heartbeat.set()
        self._heartbeat_thread.join(timeout=1.0)
        if hasattr(self._node, "destroy_publisher"):
            self._node.destroy_publisher(self._heartbeat_pub)
        if hasattr(self._node, "destroy_subscription"):
            self._node.destroy_subscription(self._source_subscription)
            self._node.destroy_subscription(self._status_subscription)
            self._node.destroy_subscription(self._event_subscription)
        if hasattr(self._node, "destroy_client"):
            self._node.destroy_client(self._source_client)
            self._node.destroy_client(self._recovery_client)

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
    "SOURCE_SERVICE",
    "RECOVERY_SERVICE",
    "AuthorityClient",
    "AuthoritySnapshot",
    "ExecutionManagerClient",
    "ExecutionManagerUnavailableError",
    "SourceRole",
]
