"""Profile-bound action-producing node."""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from enum import Enum
from types import TracebackType
from typing import Any

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from execution_manager_interfaces.msg import ResourceAuthority
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from .contracts import Action
from .provider import (
    _cartesian_trajectory_from_spec,
    _joint_trajectory_from_spec,
    _reject_invalid_result,
    _twist_stamped_from_spec,
)


class NodeStatus(str, Enum):
    """Resource-independent application view of one action node's authority."""

    INACTIVE = "INACTIVE"
    ACTIVE = "ACTIVE"
    PARTIAL = "PARTIAL"
    TRANSITIONING = "TRANSITIONING"
    FAULT = "FAULT"


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
            self._goal_handle.get_result_async(), timeout, "trajectory result"
        )
        self.result = wrapped.result
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
            self._goal_handle.cancel_goal_async(), timeout, "trajectory cancel"
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

    def submit(self, value: Any) -> None:
        """Split one ordered joint vector and publish one atomic action batch."""
        if isinstance(value, Action):
            self.node.submit(value)
            return
        if (
            isinstance(value, Sequence)
            and value
            and all(isinstance(item, Action) for item in value)
        ):
            self.node.submit(value)
            return
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
        self.node.submit(actions)

    def execute(self, plans: Any, *, timeout: float = 10.0) -> dict[str, Any]:
        """Execute all action-backed parts in parallel and wait for completion."""
        by_part = plans if isinstance(plans, Mapping) else {self.parts[0]: plans}
        if set(by_part) != set(self.parts):
            raise ValueError(
                f"{self.name!r} execution requires plans for {list(self.parts)}, "
                f"got {list(by_part)}"
            )
        executions: list[Execution] = []
        try:
            for part_name in self.parts:
                executions.append(
                    self.node.execute(part_name, by_part[part_name], timeout=timeout)
                )
            results = {}
            for execution in executions:
                results[execution.part] = execution.wait(timeout=timeout)
                if execution.state is not ExecutionState.SUCCEEDED:
                    details = []
                    if execution.result is not None:
                        error_code = getattr(execution.result, "error_code", None)
                        error_string = getattr(execution.result, "error_string", "")
                        if error_code is not None:
                            details.append(f"error_code={error_code}")
                        if error_string:
                            details.append(f"error_string={error_string!r}")
                    suffix = f" ({', '.join(details)})" if details else ""
                    raise RuntimeError(
                        f"execution failed for {execution.part}: "
                        f"{execution.state.value}{suffix}"
                    )
            return results
        except BaseException:
            for execution in executions:
                if not execution.done:
                    execution.cancel()
            raise


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
        self._activation_lease_id: str | None = None

    def __getitem__(self, name: str) -> NodeResource:
        return NodeResource(self, name)

    def activate(self, *, preempt: bool = False) -> NodeActivation:
        """Acquire scoped authority without exposing the underlying EM lease."""
        if self._activation_lease_id is not None:
            raise RuntimeError(f"node {self.name!r} is already active")
        grant = self._authority.claim(
            self.config.source_role,
            self.name,
            dict(self.config.resources),
            preempt=preempt,
            metadata={"activation": "rmi_node_scope"},
        )
        self._activation_lease_id = grant.lease_id
        return NodeActivation(self)

    def deactivate(self) -> None:
        """Release this node's scoped lease, including a lease restored by EM."""
        if self._activation_lease_id is None:
            return
        scoped_lease_id = self._activation_lease_id
        self._activation_lease_id = None
        restored_lease_ids: set[str] = set()
        allocations = self._authority.get_allocations()
        for resource in self.config.resources:
            allocation = allocations.get(resource, {})
            if allocation.get("source_instance") == self.name:
                lease_id = allocation.get("lease_id")
                if lease_id and lease_id != scoped_lease_id:
                    restored_lease_ids.add(lease_id)
        self._authority.release(scoped_lease_id)
        for lease_id in sorted(restored_lease_ids):
            self._authority.release(lease_id)

    @property
    def status(self) -> NodeStatus:
        """Aggregate EM authority without exposing resources to applications."""
        allocations = self._authority.get_allocations()
        states: list[int] = []
        owned = 0
        for resource in self.config.resources:
            allocation = allocations.get(resource, {})
            state = int(allocation.get("authority_state", ResourceAuthority.UNOWNED))
            states.append(state)
            if (
                state == int(ResourceAuthority.OWNED)
                and allocation.get("source_instance") == self.name
            ):
                owned += 1
        if any(state == int(ResourceAuthority.FAULT) for state in states):
            return NodeStatus.FAULT
        if any(state == int(ResourceAuthority.TRANSITIONING) for state in states):
            return NodeStatus.TRANSITIONING
        if owned == len(states) and states:
            return NodeStatus.ACTIVE
        if owned:
            return NodeStatus.PARTIAL
        return NodeStatus.INACTIVE

    @property
    def is_active(self) -> bool:
        return self.status in {NodeStatus.ACTIVE, NodeStatus.PARTIAL}

    def submit(self, actions: Action | Sequence[Action] | None) -> None:
        """Pre-validate one action batch and publish it with one timestamp."""
        if actions is None:
            return
        items = (actions,) if isinstance(actions, Action) else tuple(actions)
        if not items:
            return

        # Validate the complete batch before the first ROS publish.
        prepared: list[tuple[Any, Any]] = []
        stamp = self._node.get_clock().now().to_msg()
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
            _reject_invalid_result(action.value)
            part = self._profile.parts[action.part]
            if command == "joint_reference":
                message = (
                    action.value
                    if isinstance(action.value, JointTrajectory)
                    else _joint_trajectory_from_spec(
                        action.value, list(part.joint_names), False
                    )
                )
                message_type = JointTrajectory
            elif command == "pose_reference":
                message = (
                    action.value
                    if isinstance(action.value, CartesianTrajectory)
                    else _cartesian_trajectory_from_spec(
                        action.value,
                        part.base_frame or "base_link",
                        part.tcp_frame or "",
                    )
                )
                message_type = CartesianTrajectory
            elif command == "twist_reference":
                message = (
                    action.value
                    if isinstance(action.value, TwistStamped)
                    else _twist_stamped_from_spec(
                        action.value, part.base_frame or "base_link"
                    )
                )
                message_type = TwistStamped
            else:
                raise KeyError(f"unsupported node command {command!r}")

            if hasattr(message, "header"):
                message.header.stamp = stamp

            key = (action.part, command)
            if key not in self._publishers:
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
                self._publishers[key] = self._node.create_publisher(
                    message_type, source_input.endpoint, qos
                )
            prepared.append((self._publishers[key], message))

        # ROS topic publication itself is not transactionally atomic.
        for publisher, message in prepared:
            publisher.publish(message)

    def execute(self, part: str, plan: Any, *, timeout: float = 5.0) -> Execution:
        """Submit one planner trajectory; EM owns authority and handback."""
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
        _reject_invalid_result(plan)
        part_config = self._profile.parts[part]
        if command == "joint_trajectory":
            action_type = FollowJointTrajectory
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = _joint_trajectory_from_spec(
                plan, list(part_config.joint_names), True
            )
        else:
            action_type = ParallelGripperCommand
            goal = ParallelGripperCommand.Goal()
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
        client = self._action_clients.get(part)
        if client is None:
            client = self._action_client_factory(
                self._node,
                action_type,
                source_input.endpoint,
            )
            self._action_clients[part] = client
        if not client.wait_for_server(timeout_sec=timeout):
            raise TimeoutError(
                f"planner action unavailable at {source_input.endpoint!r}"
            )
        feedback: list[Any] = []
        future = client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback.append(message.feedback),
        )
        goal_handle = _wait_future(future, timeout, "trajectory goal")
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Execution Manager rejected trajectory")
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
            raise TimeoutError(f"{operation} timed out")
        time.sleep(0.01)
    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()
