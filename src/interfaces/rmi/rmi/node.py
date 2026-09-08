"""Profile-bound action-producing node."""

from __future__ import annotations

import time
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import replace
from enum import Enum
from types import TracebackType
from typing import Any

from action_msgs.msg import GoalStatus
from execution_manager_interfaces.action import (
    LeasedFollowJointTrajectory,
    LeasedParallelGripperCommand,
)
from execution_manager_interfaces.msg import (
    LeasedJointReference,
    LeasedPoseReference,
    LeasedTwistReference,
    ResourceAuthority,
)
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from .command_messages import (
    cartesian_trajectory_from_spec,
    joint_trajectory_from_spec,
    reject_invalid_result,
    twist_stamped_from_spec,
)
from .contracts import Action
from .errors import (
    ActionTimeoutError,
    ExecutionError,
    NodeAlreadyActiveError,
    SourceAuthorityError,
    action_timeout_error,
    execution_failure_cause_and_details,
    format_jtc_guard_diagnostic_lines,
    goal_rejected_error,
)


def _part_jtc_guard_matchers(
    profile: Any, part_name: str
) -> tuple[str | None, str | None]:
    """Return (heartbeat_topic, jtc_action) declared for one part, if any."""
    parts = getattr(profile, "parts", None) or {}
    part = parts.get(part_name)
    if part is None:
        return None, None
    heartbeat: str | None = None
    action: str | None = None
    for controller in getattr(part, "controllers", {}).values():
        topics = getattr(controller, "ros_topics", {}) or {}
        actions = getattr(controller, "ros_actions", {}) or {}
        topic = topics.get("trajectory_guard_heartbeat")
        if topic:
            heartbeat = topic
        jtc = actions.get("follow_joint_trajectory")
        if jtc:
            action = jtc
    return heartbeat, action


def _jtc_guard_status_matches(
    *,
    status_name: str,
    values: Mapping[str, str],
    part_name: str,
    heartbeat_topic: str | None,
    jtc_action: str | None,
) -> bool:
    if "jtc_guard" not in status_name:
        return False
    topic = values.get("heartbeat_topic", "")
    action = values.get("jtc_action", "")
    if heartbeat_topic and topic == heartbeat_topic:
        return True
    if jtc_action and action == jtc_action:
        return True
    needle = f"/{part_name}/"
    return needle in topic or needle in action or part_name in status_name


class _JtcGuardDiagnosticBuffer:
    """Capture jtc_guard /diagnostics while a trajectory is in flight.

    Guards clear fault_code back to NONE quickly after canceling; remembering the
    last non-NONE fault is required to attribute ABORTED_EXTERNALLY.
    """

    def __init__(self, ros_node: Any, profile: Any, part_names: Sequence[str]) -> None:
        self._ros_node = ros_node
        self._part_names = tuple(part_names)
        self._matchers = {
            part: _part_jtc_guard_matchers(profile, part) for part in self._part_names
        }
        self._latest: dict[str, tuple[str, dict[str, str]]] = {}
        self._last_fault: dict[str, tuple[str, dict[str, str]]] = {}
        self._subscription = None
        create_subscription = getattr(ros_node, "create_subscription", None)
        if create_subscription is None:
            return
        try:
            from diagnostic_msgs.msg import DiagnosticArray
        except ImportError:
            return
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        try:
            self._subscription = create_subscription(
                DiagnosticArray, "/diagnostics", self._on_diagnostics, qos
            )
        except Exception:
            self._subscription = None

    def _on_diagnostics(self, message: Any) -> None:
        for status in getattr(message, "status", []) or []:
            name = str(getattr(status, "name", "") or "")
            values = {
                str(kv.key): str(kv.value)
                for kv in getattr(status, "values", []) or []
            }
            for part_name, (heartbeat, action) in self._matchers.items():
                matched = _jtc_guard_status_matches(
                    status_name=name,
                    values=values,
                    part_name=part_name,
                    heartbeat_topic=heartbeat,
                    jtc_action=action,
                )
                if not matched:
                    if heartbeat or action or "jtc_guard" not in name:
                        continue
                    if len(self._part_names) > 1 and part_name not in name:
                        continue
                self._latest[part_name] = (name, values)
                fault = values.get("fault_code", "NONE")
                if fault and fault != "NONE":
                    self._last_fault[part_name] = (name, dict(values))

    def lines_for(self, part_name: str) -> list[str]:
        latest_entry = self._latest.get(part_name)
        fault_entry = self._last_fault.get(part_name)
        return format_jtc_guard_diagnostic_lines(
            part_name=part_name,
            latest=latest_entry[1] if latest_entry else None,
            last_fault=fault_entry[1] if fault_entry else None,
            status_name=(
                fault_entry[0]
                if fault_entry
                else (latest_entry[0] if latest_entry else "")
            ),
        )

    def close(self) -> None:
        if self._subscription is None:
            return
        destroy = getattr(self._ros_node, "destroy_subscription", None)
        if destroy is not None:
            try:
                destroy(self._subscription)
            except Exception:
                pass
        self._subscription = None


class SourceState(str, Enum):
    """EM-owned source lifecycle; health and resource faults are separate."""
    INACTIVE = "INACTIVE"
    WAITING = "WAITING"
    ACQUIRING = "ACQUIRING"
    CONTROLLING = "CONTROLLING"
    RELEASING = "RELEASING"


class ExecutionState(str, Enum):
    ACCEPTED = "ACCEPTED"
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    ABORTED = "ABORTED"


class Execution:
    """Handle for one trajectory submitted through a Planner Node."""

    def __init__(self, part: str, goal_handle: Any, feedback: list[Any]) -> None:
        self.part = part
        self._goal_handle = goal_handle
        self._feedback = feedback
        self.state = ExecutionState.ACCEPTED
        self.result: Any | None = None
        self.done = False
        self.canceled = False

    @property
    def feedback(self) -> Any | None:
        return self._feedback[-1] if self._feedback else None

    def wait(self, timeout: float = 10.0) -> Any:
        wrapped = _wait_future(
            self._goal_handle.get_result_async(),
            timeout,
            f"trajectory result for {self.part!r}",
        )
        self.result = wrapped.result
        self.status = getattr(wrapped, "status", None)
        self.done = True
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.state = ExecutionState.SUCCEEDED
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self.state = ExecutionState.CANCELED
            self.canceled = True
        else:
            self.state = ExecutionState.ABORTED
        return self.result

    def cancel(self, timeout: float = 5.0) -> None:
        _wait_future(
            self._goal_handle.cancel_goal_async(),
            timeout,
            f"trajectory cancel for {self.part!r}",
        )


class NodeActivation:
    """Scoped EM authority for an application-controlled Node."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._closed = False

    def __enter__(self) -> Node:
        return self._node

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc, traceback
        self.close()

    def close(self) -> None:
        if not self._closed:
            self._node.deactivate()
            self._closed = True


class NodeResource:
    """Source-bound command view over one part or compound group."""

    def __init__(self, node: Node, name: str) -> None:
        self.node = node
        self.name = name
        self.parts = tuple(node._profile.get_part_names(name))
        if not self.parts:
            raise KeyError(f"unknown robot resource {name!r}")
        missing = [part for part in self.parts if part not in node.config.resources]
        if missing:
            raise KeyError(
                f"node {node.name!r} does not provide {name!r} parts: {missing}"
            )
        self.joint_names = tuple(node._profile.get_part_joints(name))

    def select_action(self, observation: Any = None, *, selector: Any = None) -> Any:
        """Select and split a joint vector under the node's captured epoch."""
        return self.node._select_action(observation, selector=selector, convert=self._actions)

    def submit(self, actions: Action | Sequence[Action] | None) -> None:
        self.node.submit(actions)

    def _actions(self, value: Any) -> Any:
        if value is None:
            return None
        if isinstance(value, Action):
            return value
        if (
            isinstance(value, Sequence)
            and value
            and all(isinstance(item, Action) for item in value)
        ):
            return value
        values = list(value)
        if len(values) != len(self.joint_names):
            raise ValueError(
                f"{self.name!r} requires {len(self.joint_names)} joint values, "
                f"got {len(values)}"
            )
        actions = []
        offset = 0
        for part_name in self.parts:
            command = self.node.config.resources[part_name]
            if command != "joint_reference":
                raise ValueError(
                    f"node {self.node.name!r} resource {part_name!r} uses "
                    f"{command!r}; flat joint vectors require 'joint_reference'"
                )
            width = len(self.node._profile.parts[part_name].joint_names)
            actions.append(
                Action(
                    part=part_name,
                    command=command,
                    value=values[offset : offset + width],
                )
            )
            offset += width
        return actions

    def execute(self, plans: Any, *, timeout: float = 10.0) -> dict[str, Any]:
        """Execute all action-backed parts in parallel and wait for completion."""
        by_part = plans if isinstance(plans, Mapping) else {self.parts[0]: plans}
        if set(by_part) != set(self.parts):
            raise ValueError(
                f"{self.name!r} execution requires plans for {list(self.parts)}, "
                f"got {list(by_part)}"
            )
        executions: list[Execution] = []
        guard_diag = _JtcGuardDiagnosticBuffer(
            self.node._node, self.node._profile, self.parts
        )
        try:
            for part_name in self.parts:
                executions.append(
                    self.node.execute(part_name, by_part[part_name], timeout=timeout)
                )
            results = {}
            for execution in executions:
                results[execution.part] = execution.wait(timeout=timeout)
                if execution.state is not ExecutionState.SUCCEEDED:
                    allocations = {}
                    if hasattr(self.node, "_authority") and hasattr(
                        self.node._authority, "get_allocations"
                    ):
                        try:
                            allocations = self.node._authority.get_allocations()
                        except Exception:
                            pass
                    cause, details = execution_failure_cause_and_details(
                        part=execution.part,
                        state=execution.state.value,
                        result=execution.result,
                        feedback=getattr(execution, "_feedback", []),
                        guard_lines=guard_diag.lines_for(execution.part),
                        allocations=allocations,
                    )
                    raise ExecutionError(
                        part=execution.part,
                        state=execution.state.value,
                        cause=cause,
                        details=details,
                    )
            return results
        except BaseException:
            for execution in executions:
                if not execution.done:
                    execution.cancel()
            raise
        finally:
            guard_diag.close()


class Node:
    """Profile-bound action submission and authority-status handle."""

    def __init__(
        self,
        name: str,
        node: Any,
        profile: Any,
        authority: Any,
        producer: Any = None,
        *,
        action_client_factory: Any = ActionClient,
    ) -> None:
        self.name = name
        self.producer = producer
        self.config = profile.nodes[name]
        self._node = node
        self._profile = profile
        self._authority = authority
        self._action_client_factory = action_client_factory
        self._publishers: dict[tuple[str, str], Any] = {}
        self._action_clients: dict[str, Any] = {}
        self._session_id: str | None = None
        self._selection_lease: str | None = None

    def __getitem__(self, name: str) -> NodeResource:
        return NodeResource(self, name)

    def activate(self, *, preempt: bool = False) -> NodeActivation:
        """Join EM's candidate pool, or explicitly preempt without joining it."""
        if self._session_id is not None:
            if self.state != SourceState.INACTIVE:
                raise NodeAlreadyActiveError(self.name)
            self.deactivate()
        session = uuid.uuid4().hex
        self._session_id = session
        try:
            self._authority.set_source_activation(self.name, session, active=True, preempt=preempt)
        except BaseException:
            record = self._authority.get_sources().get(self.name, {})
            if record.get("session_id") != session or record.get("state") != 4:
                self._session_id = None
            raise
        return NodeActivation(self)

    def deactivate(self) -> None:
        """Withdraw candidacy and confirm release; retain session on failure."""
        if self._session_id is None:
            return
        self._authority.set_source_activation(self.name, self._session_id, active=False)
        self._session_id = None
        self._selection_lease = None

    @property
    def state(self) -> SourceState:
        record = self._authority.get_sources().get(self.name)
        if record is None:
            return SourceState.INACTIVE
        return tuple(SourceState)[int(record["state"])]

    @property
    def has_control(self) -> bool:
        return self.state == SourceState.CONTROLLING

    def wait_for_control(self, timeout: float = 10.0) -> None:
        """Wait for an already registered candidate; never implicitly activate."""
        if self._session_id is None:
            raise SourceAuthorityError("source is not activated")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.has_control:
                self._current_lease()
                return
            record = self._authority.get_sources().get(self.name, {})
            if record.get("session_id") == self._session_id and self.state in {
                SourceState.INACTIVE, SourceState.RELEASING
            }:
                raise SourceAuthorityError(f"source cannot execute: {self.state.value}")
            time.sleep(0.01)
        raise SourceAuthorityError("timed out waiting for control")

    def _current_lease(self) -> str:
        if self._session_id is None:
            raise SourceAuthorityError("source is not activated")
        record = self._authority.get_sources().get(self.name, {})
        if (record.get("session_id") != self._session_id
                or record.get("state") != 3 or not record.get("lease_id")):
            raise SourceAuthorityError("source does not currently have control")
        return record["lease_id"]

    def select_action(self, observation: Any = None, *, selector: Any = None) -> Action | Sequence[Action] | None:
        """Capture authority before running the producer or an explicit selector.

        A selector receives observation and computes manual commands. It runs
        here, never in submit; do not pass a previously computed action through it.
        """
        return self._select_action(observation, selector=selector)

    def _select_action(self, observation: Any, *, selector: Any = None, convert: Any = None) -> Any:
        lease = self._current_lease()
        if selector is None:
            if self.producer is None:
                raise RuntimeError("select_action requires a producer or selector")
            if lease != self._selection_lease and hasattr(self.producer, "on_control_acquired"):
                self.producer.on_control_acquired(observation)
            selector = self.producer.select_action
        actions = selector(observation)
        self._selection_lease = lease
        if convert is not None:
            actions = convert(actions)
        if actions is None:
            return None
        # Lease is captured before the producer; admission stamp is taken after.
        # Remote inference is ~300ms and EM max_command_age_s is 0.25s — a
        # pre-inference stamp is rejected as stale_command and the arm never moves.
        stamp = self._node.get_clock().now().to_msg()
        def bind(action: Action) -> Action:
            if not isinstance(action, Action):
                raise TypeError("select_action must return Action values; use a resource view for joint vectors")
            if action._lease_id is not None:
                raise SourceAuthorityError("select_action cannot rebind a previously prepared action")
            return replace(action, _lease_id=lease, _source_instance=self.name, _prepared_stamp=stamp)
        return bind(actions) if isinstance(actions, Action) else [bind(a) for a in actions]

    def submit(self, actions: Action | Sequence[Action] | None) -> None:
        """Submit only under current authority; never activate or enter the pool."""
        lease = self._current_lease()
        if actions is None:
            return
        items = (actions,) if isinstance(actions, Action) else tuple(actions)
        for action in items:
            if not isinstance(action, Action):
                raise TypeError("submit requires actions returned by select_action")
            if action._prepared_stamp is None or (
                action._lease_id != lease or action._source_instance != self.name
            ):
                raise SourceAuthorityError("action belongs to a revoked execution epoch")
        # Every action retains the epoch and admission timestamp from selection.
        stamps = [a._prepared_stamp for a in items if a._prepared_stamp is not None]
        stamp = min(stamps, key=lambda value: (value.sec, value.nanosec)) if stamps else None
        self._submit(items, lease_id=lease, stamp=stamp)

    def _leased_publisher(self, part: str, command: str) -> tuple[Any, Any]:
        message_type = {
            "joint_reference": LeasedJointReference,
            "pose_reference": LeasedPoseReference,
            "twist_reference": LeasedTwistReference,
        }[command]
        key = (part, command)
        if key not in self._publishers:
            endpoint = f"/execution_manager/ingress/{self.config.source_role.lower()}/{part}/{command}"
            self._publishers[key] = self._node.create_publisher(
                message_type, endpoint,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            )
        return self._publishers[key], message_type

    def _submit(self, actions: Any, *, lease_id: str, stamp: Any = None) -> None:
        if actions is None:
            return
        items = (actions,) if isinstance(actions, Action) else tuple(actions)
        if not items:
            return

        # Validate the complete batch before the first ROS publish.
        prepared: list[tuple[Any, Any]] = []
        stamp = stamp or self._node.get_clock().now().to_msg()
        for action in items:
            try:
                command = self.config.resources[action.part]
            except KeyError as exc:
                raise KeyError(
                    f"node {self.name!r} does not provide {action.part!r}"
                ) from exc
            try:
                source_input = self.config.inputs[action.part]
            except KeyError as exc:
                raise RuntimeError(
                    f"node {self.name!r} has no in-process ingress binding for "
                    f"{action.part!r}; external nodes submit in their own process"
                ) from exc
            if action.command != command or source_input.command_contract != command:
                raise ValueError(
                    f"{action.part!r} requires {command!r}, got {action.command!r}"
                )
            reject_invalid_result(action.value)
            part = self._profile.parts[action.part]
            if command == "joint_reference":
                message = (
                    action.value
                    if isinstance(action.value, JointTrajectory)
                    else joint_trajectory_from_spec(
                        action.value, list(part.joint_names), False
                    )
                )
            elif command == "pose_reference":
                message = (
                    action.value
                    if isinstance(action.value, CartesianTrajectory)
                    else cartesian_trajectory_from_spec(
                        action.value,
                        part.base_frame or "base_link",
                        part.tcp_frame or "",
                    )
                )
            elif command == "twist_reference":
                message = (
                    action.value
                    if isinstance(action.value, TwistStamped)
                    else twist_stamped_from_spec(
                        action.value, part.base_frame or "base_link"
                    )
                )
            else:
                raise KeyError(f"unsupported node command {command!r}")

            if hasattr(message, "header"):
                message.header.stamp = stamp

            publisher, envelope_type = self._leased_publisher(action.part, command)
            envelope = envelope_type()
            envelope.header.stamp = stamp
            envelope.lease_id = lease_id
            envelope.command = message
            prepared.append((publisher, envelope))

        # ROS topic publication itself is not transactionally atomic.
        for publisher, message in prepared:
            publisher.publish(message)

    def execute(self, part: str, plan: Any, *, timeout: float = 5.0) -> Execution:
        """Submit one planner trajectory under the activated source scope."""
        lease = self._current_lease()
        stamp = self._node.get_clock().now().to_msg()
        try:
            command = self.config.resources[part]
            source_input = self.config.inputs[part]
        except KeyError as exc:
            raise KeyError(f"node {self.name!r} does not provide {part!r}") from exc
        if (
            command not in {"joint_trajectory", "gripper_command"}
            or not source_input.is_action
        ):
            raise ValueError(
                f"node {self.name!r} resource {part!r} is not an action contract"
            )
        reject_invalid_result(plan)
        part_config = self._profile.parts[part]
        if command == "joint_trajectory":
            action_type = LeasedFollowJointTrajectory
            goal = LeasedFollowJointTrajectory.Goal()
            goal.trajectory = joint_trajectory_from_spec(
                plan, list(part_config.joint_names), True
            )
        else:
            action_type = LeasedParallelGripperCommand
            goal = LeasedParallelGripperCommand.Goal()
            if isinstance(plan, JointState):
                goal.command = plan
            else:
                positions = (
                    [float(plan)] if isinstance(plan, (int, float)) else list(plan)
                )
                if len(positions) != len(part_config.joint_names):
                    raise ValueError(
                        f"{part!r} gripper command requires "
                        f"{len(part_config.joint_names)} position(s)"
                    )
                goal.command = JointState(
                    name=list(part_config.joint_names), position=positions
                )
        goal.header.stamp = stamp
        goal.lease_id = lease
        goal.resource = part
        suffix = "follow_joint_trajectory" if command == "joint_trajectory" else command
        endpoint = f"/execution_manager/ingress/{self.config.source_role.lower()}/{part}/{suffix}"
        client = self._action_clients.get(part)
        if client is None:
            client = self._action_client_factory(
                self._node,
                action_type,
                endpoint,
            )
            self._action_clients[part] = client
        if not client.wait_for_server(timeout_sec=timeout):
            raise ActionTimeoutError(
                f"planner action server unavailable: part={part!r} "
                f"endpoint={endpoint!r} wait_s={timeout}"
            )
        feedback: list[Any] = []
        future = client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback.append(message.feedback),
        )
        goal_handle = _wait_future(
            future,
            timeout,
            f"trajectory goal accept for {part!r} at {source_input.endpoint!r}",
        )
        if goal_handle is None or not goal_handle.accepted:
            raise goal_rejected_error(
                part=part,
                endpoint=source_input.endpoint,
                node_name=self.name,
                command=command,
                accepted=getattr(goal_handle, "accepted", None),
            )
        return Execution(part, goal_handle, feedback)

    def close(self) -> None:
        self.deactivate()
        if hasattr(self._node, "destroy_publisher"):
            for publisher in self._publishers.values():
                self._node.destroy_publisher(publisher)
        for client in self._action_clients.values():
            client.destroy()
        self._publishers.clear()
        self._action_clients.clear()


def _wait_future(future: Any, timeout: float, operation: str) -> Any:
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            future.cancel()
            raise action_timeout_error(operation, timeout)
        time.sleep(0.01)
    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()
