"""Direct ros2_control controller clients and controller_manager client."""

from __future__ import annotations

import asyncio
import threading
from typing import Any

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from .config import ControllerConfig


class ControllerClientError(RuntimeError):
    """A controller endpoint was unavailable or rejected a command."""


class TrajectoryCanceledError(ControllerClientError):
    """A trajectory goal reached the terminal CANCELED status."""


def _command_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class JointTrajectoryControllerClient:
    """Cancelable action client for a JointTrajectoryController."""

    def __init__(
        self,
        node: Any,
        config: ControllerConfig,
        timeout_sec: float = 5.0,
        action_client_factory: Any = ActionClient,
    ) -> None:
        self.controller_name = config.name
        self._timeout_sec = _positive_timeout(timeout_sec)
        endpoint = _required_endpoint(
            config.ros_actions, "follow_joint_trajectory", config.name
        )
        self._client = action_client_factory(node, FollowJointTrajectory, endpoint)
        self._goal_handle: Any | None = None
        self._guard_lock = threading.Lock()
        self._goal_pending = False
        self._guard_active = False
        heartbeat_endpoint = config.ros_topics.get("trajectory_guard_heartbeat")
        self._guard_publisher = (
            node.create_publisher(Bool, heartbeat_endpoint, _command_qos())
            if heartbeat_endpoint
            else None
        )
        self._guard_timer = (
            node.create_timer(0.1, self._publish_guard_heartbeat)
            if self._guard_publisher is not None
            else None
        )

    async def send(
        self,
        trajectory: JointTrajectory,
        feedback_callback: Any | None = None,
    ) -> Any:
        """Send a trajectory and return its accepted ROS goal handle."""
        if not isinstance(trajectory, JointTrajectory):
            raise TypeError("trajectory must be a JointTrajectory")
        with self._guard_lock:
            if self._goal_pending or self._goal_handle is not None:
                raise ControllerClientError(
                    "a trajectory goal is already active for this controller"
                )
            self._goal_pending = True
        try:
            await _wait_until_ready(
                self._client.server_is_ready,
                self._timeout_sec,
                "follow_joint_trajectory action",
            )
        except (asyncio.CancelledError, Exception):
            with self._guard_lock:
                self._goal_pending = False
            raise

        # Pre-arm before dispatch. If the goal reaches RT but the acceptance
        # response is lost with the workstation, heartbeat silence still
        # drives the local guard through cancel-all-goals.
        with self._guard_lock:
            self._guard_active = True
            if self._guard_publisher is not None:
                self._guard_publisher.publish(Bool(data=True))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        try:
            send_future = (
                self._client.send_goal_async(goal)
                if feedback_callback is None
                else self._client.send_goal_async(
                    goal,
                    feedback_callback=feedback_callback,
                )
            )
            handle = await _bounded(
                send_future,
                self._timeout_sec,
                "send trajectory goal",
            )
        except (asyncio.CancelledError, Exception):
            # Dispatch outcome is unknown. Stop true heartbeats without
            # disarming so RT independently cancels any goal that arrived.
            with self._guard_lock:
                self._goal_pending = False
                self._guard_active = False
            raise
        if not handle.accepted:
            with self._guard_lock:
                self._goal_pending = False
                self._guard_active = False
                if self._guard_publisher is not None:
                    self._guard_publisher.publish(Bool(data=False))
            raise ControllerClientError("trajectory goal was rejected")
        with self._guard_lock:
            self._goal_pending = False
            self._goal_handle = handle
        return handle

    async def execute(
        self, trajectory: JointTrajectory, timeout_sec: float | None = None
    ) -> Any:
        """Execute until a terminal ROS action result is received."""
        handle = await self.send(trajectory)
        return await self.wait_for_result(handle, timeout_sec=timeout_sec)

    async def wait_for_result(
        self, handle: Any, timeout_sec: float | None = None
    ) -> Any:
        """Wait for a previously accepted trajectory goal's terminal result."""
        result_timeout = (
            self._timeout_sec if timeout_sec is None else _positive_timeout(timeout_sec)
        )
        wrapped = None
        try:
            wrapped = await _bounded(
                handle.get_result_async(),
                result_timeout,
                "wait for trajectory result",
            )
            status = getattr(wrapped, "status", None)
            if status == GoalStatus.STATUS_SUCCEEDED:
                return wrapped.result
            if status == GoalStatus.STATUS_CANCELED:
                raise TrajectoryCanceledError("trajectory goal was canceled")
            if status == GoalStatus.STATUS_ABORTED:
                raise ControllerClientError("trajectory goal was aborted")
            raise ControllerClientError(
                f"trajectory finished with unexpected status {status!r}"
            )
        except asyncio.CancelledError:
            self.abandon(handle)
            raise
        except TimeoutError:
            raise
        except Exception:
            self.abandon(handle)
            raise
        finally:
            # A local wait timeout is not a terminal action state. Keep the
            # guard alive so callers can continue waiting or explicitly cancel.
            if wrapped is not None:
                self.mark_terminal(handle)

    async def cancel(self) -> None:
        """Cancel the currently tracked goal, if one exists.

        Already-terminal goals (succeeded/aborted/canceled) often return an
        empty ``goals_canceling`` set. Treat that as a no-op so EM handovers
        after a completed trajectory do not fail.
        """
        with self._guard_lock:
            handle = self._goal_handle
        if handle is None:
            return
        try:
            response = await _bounded(
                handle.cancel_goal_async(),
                self._timeout_sec,
                "cancel trajectory goal",
            )
        except Exception:
            # Do not send false: silence intentionally lets the RT guard issue
            # its independent cancel if the workstation cancel path failed.
            self.abandon(handle)
            raise
        self.mark_terminal(handle)
        if not response.goals_canceling:
            # Goal was already terminal (or never cancelable); motion is stopped.
            return

    def mark_terminal(self, handle: Any) -> None:
        """Disarm the RT guard after a confirmed terminal/cancel transition."""
        with self._guard_lock:
            if self._goal_handle is handle:
                self._goal_handle = None
                self._guard_active = False
                if self._guard_publisher is not None:
                    self._guard_publisher.publish(Bool(data=False))

    def abandon(self, handle: Any) -> None:
        """Stop heartbeats without disarming, forcing the RT timeout path."""
        with self._guard_lock:
            if self._goal_handle is handle:
                self._guard_active = False

    def _publish_guard_heartbeat(self) -> None:
        with self._guard_lock:
            if self._guard_active and self._guard_publisher is not None:
                self._guard_publisher.publish(Bool(data=True))


class GripperControllerClient:
    """Cancelable action client for a ParallelGripperCommand controller."""

    def __init__(
        self,
        node: Any,
        config: ControllerConfig,
        timeout_sec: float = 5.0,
        action_client_factory: Any = ActionClient,
    ) -> None:
        self.controller_name = config.name
        self._timeout_sec = _positive_timeout(timeout_sec)
        endpoint = _required_endpoint(
            config.ros_actions, "gripper_command", config.name
        )
        self._client = action_client_factory(node, ParallelGripperCommand, endpoint)
        self._goal_handle: Any | None = None

    async def send(self, command: JointState) -> Any:
        """Send one named gripper position command."""
        if not isinstance(command, JointState):
            raise TypeError("command must be a JointState")
        if not command.name or len(command.name) != len(command.position):
            raise ValueError("gripper command requires aligned name and position")
        await _wait_until_ready(
            self._client.server_is_ready,
            self._timeout_sec,
            "gripper action",
        )
        goal = ParallelGripperCommand.Goal()
        goal.command = command
        handle = await _bounded(
            self._client.send_goal_async(goal), self._timeout_sec, "send gripper goal"
        )
        if not handle.accepted:
            raise ControllerClientError("gripper goal was rejected")
        self._goal_handle = handle
        return handle

    async def cancel(self) -> None:
        """Cancel the currently tracked gripper goal, if one exists.

        Empty ``goals_canceling`` means the goal is already terminal; EM
        handovers treat that as success.
        """
        if self._goal_handle is None:
            return
        handle = self._goal_handle
        self._goal_handle = None
        response = await _bounded(
            handle.cancel_goal_async(),
            self._timeout_sec,
            "cancel gripper goal",
        )
        if not response.goals_canceling:
            return


class JointSpaceReferenceControllerClient:
    """Publisher for bounded stamped JointTrajectory reference chunks."""

    def __init__(self, node: Any, config: ControllerConfig) -> None:
        self.controller_name = config.name
        endpoint = _required_endpoint(config.ros_topics, "joint_reference", config.name)
        self._publisher = node.create_publisher(
            JointTrajectory, endpoint, _command_qos()
        )

    def send(self, reference: JointTrajectory) -> None:
        """Publish one joint-space reference chunk."""
        if not isinstance(reference, JointTrajectory):
            raise TypeError("reference must be a JointTrajectory")
        self._publisher.publish(reference)


class ForwardCommandControllerClient:
    """Adapt native JointTrajectory references to forward position commands."""

    def __init__(self, node: Any, config: ControllerConfig) -> None:
        self.controller_name = config.name
        endpoint = _required_endpoint(config.ros_topics, "joint_reference", config.name)
        self._publisher = node.create_publisher(
            Float64MultiArray, endpoint, _command_qos()
        )

    def send(self, reference: JointTrajectory) -> None:
        """Publish the newest complete trajectory point."""
        if not isinstance(reference, JointTrajectory):
            raise TypeError("reference must be a JointTrajectory")
        if not reference.joint_names or not reference.points:
            raise ValueError("forward command requires joint names and points")
        positions = reference.points[-1].positions
        if len(positions) != len(reference.joint_names):
            raise ValueError("forward command positions must align with joint names")
        command = Float64MultiArray()
        command.data = list(positions)
        self._publisher.publish(command)


class TaskSpaceReferenceControllerClient:
    """Publisher for Cartesian trajectory and stamped twist references."""

    def __init__(self, node: Any, config: ControllerConfig) -> None:
        self.controller_name = config.name
        pose_endpoint = config.ros_topics.get("pose_reference")
        twist_endpoint = config.ros_topics.get("twist_reference")
        if not pose_endpoint and not twist_endpoint:
            raise ValueError(
                f"controller {config.name!r} requires pose_reference or twist_reference"
            )
        self._pose_publisher = (
            node.create_publisher(CartesianTrajectory, pose_endpoint, _command_qos())
            if pose_endpoint
            else None
        )
        self._twist_publisher = (
            node.create_publisher(TwistStamped, twist_endpoint, _command_qos())
            if twist_endpoint
            else None
        )

    def send_pose(self, reference: CartesianTrajectory) -> None:
        """Publish one Cartesian trajectory reference chunk."""
        if self._pose_publisher is None:
            raise ControllerClientError("pose_reference endpoint is not configured")
        if not isinstance(reference, CartesianTrajectory):
            raise TypeError("reference must be a CartesianTrajectory")
        self._pose_publisher.publish(reference)

    def send_twist(self, reference: TwistStamped) -> None:
        """Publish one stamped Cartesian twist reference."""
        if self._twist_publisher is None:
            raise ControllerClientError("twist_reference endpoint is not configured")
        if not isinstance(reference, TwistStamped):
            raise TypeError("reference must be a TwistStamped")
        self._twist_publisher.publish(reference)


def make_controller_client_factory(
    node: Any,
    timeout_sec: float = 5.0,
    action_client_factory: Any = ActionClient,
) -> Any:
    """Build a Robot factory that selects clients from endpoint contracts."""
    _positive_timeout(timeout_sec)

    def create(_part: str, contract: str, config: ControllerConfig) -> Any:
        if "follow_joint_trajectory" in config.ros_actions:
            return JointTrajectoryControllerClient(
                node, config, timeout_sec, action_client_factory
            )
        if "gripper_command" in config.ros_actions:
            return GripperControllerClient(
                node, config, timeout_sec, action_client_factory
            )
        if (
            config.implementation
            == "forward_command_controller/ForwardCommandController"
        ):
            return ForwardCommandControllerClient(node, config)
        if contract == "joint_space_reference":
            return JointSpaceReferenceControllerClient(node, config)
        if contract == "task_space_reference":
            return TaskSpaceReferenceControllerClient(node, config)
        raise ValueError(
            f"controller {config.name!r} has unsupported contract {contract!r}"
        )

    return create


def _required_endpoint(endpoints: dict[str, str], key: str, name: str) -> str:
    try:
        return endpoints[key]
    except KeyError as exc:
        raise ValueError(f"controller {name!r} is missing endpoint {key!r}") from exc


def _positive_timeout(timeout_sec: float) -> float:
    if timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive")
    return timeout_sec


async def _wait_until_ready(ready: Any, timeout_sec: float, endpoint_name: str) -> None:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_sec
    while not ready():
        remaining = deadline - loop.time()
        if remaining <= 0.0:
            raise TimeoutError(f"{endpoint_name} unavailable after {timeout_sec:g}s")
        await asyncio.sleep(min(0.05, remaining))


async def _bounded(awaitable: Any, timeout_sec: float, operation: str) -> Any:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_sec
    while not awaitable.done():
        remaining = deadline - loop.time()
        if remaining <= 0.0:
            awaitable.cancel()
            raise TimeoutError(f"{operation} timed out after {timeout_sec:g}s")
        await asyncio.sleep(min(0.01, remaining))
    exception = awaitable.exception()
    if exception is not None:
        raise exception
    return awaitable.result()
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CartesianTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from .config import ControllerConfig


class ControllerClientError(RuntimeError):
    """A controller endpoint was unavailable or rejected a command."""


class TrajectoryCanceledError(ControllerClientError):
    """A trajectory goal reached the terminal CANCELED status."""


class ControllerManagerError(RuntimeError):
    """A controller-manager request failed or returned an invalid state."""


class ControllerManagerClient:
    """Perform bounded STRICT switches and verify the resulting controller state.

    The owning ROS node must be spun by an executor while these coroutines run.
    """

    def __init__(self, node: Any, controller_manager: str, timeout_sec: float = 5.0):
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        manager = controller_manager.rstrip("/")
        if not manager:
            raise ValueError("controller_manager must not be empty")
        self._node = node
        self._manager = manager
        self._timeout_sec = timeout_sec
        self._switch_client = node.create_client(
            SwitchController, f"{manager}/switch_controller"
        )
        self._list_client = node.create_client(
            ListControllers, f"{manager}/list_controllers"
        )
        self._list_hw_client: Any | None = None
        self._set_hw_client: Any | None = None

    def _get_list_hw_client(self) -> Any:
        if self._list_hw_client is None:
            try:
                self._list_hw_client = self._node.create_client(
                    ListHardwareComponents, f"{self._manager}/list_hardware_components"
                )
            except Exception:
                pass
        return self._list_hw_client

    def _get_set_hw_client(self) -> Any:
        if self._set_hw_client is None:
            try:
                self._set_hw_client = self._node.create_client(
                    SetHardwareComponentState, f"{self._manager}/set_hardware_component_state"
                )
            except Exception:
                pass
        return self._set_hw_client

    async def get_hardware_diagnostics(self) -> list[str]:
        """Query hardware component states and return diagnostic descriptions for non-active components."""
        client = self._get_list_hw_client()
        if client is None:
            return []
        if not client.service_is_ready():
            try:
                await self._wait_for_service(client, "list_hardware_components")
            except Exception:
                return []
        try:
            listed = await self._call(
                client, ListHardwareComponents.Request(), "list_hardware_components"
            )
            diagnostics = []
            for comp in listed.component:
                state_label = comp.state.label if hasattr(comp.state, "label") else str(comp.state)
                if state_label != "active":
                    diagnostics.append(
                        f"Hardware Component '{comp.name}' (plugin: {comp.plugin_name}) is in '{state_label}' state (expected 'active')."
                    )
            return diagnostics
        except Exception:
            return []

    async def ensure_hardware_active(self) -> list[str]:
        """Automatically attempt to configure and activate any inactive/unconfigured hardware components."""
        list_client = self._get_list_hw_client()
        set_client = self._get_set_hw_client()
        if list_client is None or set_client is None:
            return []
        if not list_client.service_is_ready() or not set_client.service_is_ready():
            try:
                await asyncio.gather(
                    self._wait_for_service(list_client, "list_hardware_components"),
                    self._wait_for_service(set_client, "set_hardware_component_state"),
                )
            except Exception:
                return []

        activated: list[str] = []
        try:
            from lifecycle_msgs.msg import State

            listed = await self._call(
                list_client, ListHardwareComponents.Request(), "list_hardware_components"
            )
            for comp in listed.component:
                state_id = comp.state.id if hasattr(comp.state, "id") else 0
                state_label = comp.state.label if hasattr(comp.state, "label") else str(comp.state)
                if state_label == "unconfigured" or state_id == State.PRIMARY_STATE_UNCONFIGURED:
                    # Transition unconfigured -> inactive -> active
                    req_inact = SetHardwareComponentState.Request()
                    req_inact.name = comp.name
                    req_inact.target_state.id = State.PRIMARY_STATE_INACTIVE
                    req_inact.target_state.label = "inactive"
                    res = await self._call(set_client, req_inact, "set_hardware_component_state")
                    if res.ok:
                        req_act = SetHardwareComponentState.Request()
                        req_act.name = comp.name
                        req_act.target_state.id = State.PRIMARY_STATE_ACTIVE
                        req_act.target_state.label = "active"
                        res_act = await self._call(set_client, req_act, "set_hardware_component_state")
                        if res_act.ok:
                            activated.append(comp.name)
                elif state_label == "inactive" or state_id == State.PRIMARY_STATE_INACTIVE:
                    req_act = SetHardwareComponentState.Request()
                    req_act.name = comp.name
                    req_act.target_state.id = State.PRIMARY_STATE_ACTIVE
                    req_act.target_state.label = "active"
                    res_act = await self._call(set_client, req_act, "set_hardware_component_state")
                    if res_act.ok:
                        activated.append(comp.name)
        except Exception:
            pass
        return activated

    async def switch_controller(
        self, *, activate: tuple[str, ...], deactivate: tuple[str, ...]
    ) -> None:
        if not activate and not deactivate:
            raise ValueError("activate and deactivate must not both be empty")
        await asyncio.gather(
            self._wait_for_service(self._switch_client, "switch_controller"),
            self._wait_for_service(self._list_client, "list_controllers"),
        )

        listed = await self._call(
            self._list_client, ListControllers.Request(), "list_controllers"
        )
        loaded = {controller.name for controller in listed.controller}
        missing = [name for name in activate if name not in loaded]
        activate = tuple(name for name in activate if name in loaded)
        deactivate = tuple(name for name in deactivate if name in loaded)
        if not activate and not deactivate:
            if missing:
                return
            raise ValueError("activate and deactivate must not both be empty")

        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        response = await self._call(self._switch_client, request, "switch_controller")
        if not response.ok:
            hw_diags = await self.get_hardware_diagnostics()
            msg_lines = [
                "controller manager rejected STRICT switch "
                f"activate={list(activate)} deactivate={list(deactivate)}"
                + (f" skipped_unloaded={missing}" if missing else "")
            ]
            if hw_diags:
                msg_lines.append("\n  [Hardware Root Cause Diagnostics]:")
                for d in hw_diags:
                    msg_lines.append(f"    • {d}")
                msg_lines.append("\n  [Actionable Remediation]:")
                msg_lines.append("    1. Verify robot E-Stop is released and FCI/Desk is enabled.")
                msg_lines.append("    2. Re-enable hardware via: ros2 control set_hardware_component_state <ComponentName> active")
            raise ControllerManagerError("\n".join(msg_lines))

        listed = await self._call(
            self._list_client, ListControllers.Request(), "list_controllers"
        )
        states = {controller.name: controller.state for controller in listed.controller}
        invalid = [name for name in activate if states.get(name) != "active"]
        invalid.extend(name for name in deactivate if states.get(name) == "active")
        if invalid:
            raise ControllerManagerError(
                "controller state verification failed for: " + ", ".join(invalid)
            )

    async def active_controllers(self, candidates: tuple[str, ...]) -> tuple[str, ...]:
        """Return active controllers from a declared Part conflict set."""
        await self._wait_for_service(self._list_client, "list_controllers")
        listed = await self._call(
            self._list_client, ListControllers.Request(), "list_controllers"
        )
        candidate_set = set(candidates)
        return tuple(
            controller.name
            for controller in listed.controller
            if controller.name in candidate_set and controller.state == "active"
        )

    async def _wait_for_service(self, client: Any, service_name: str) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self._timeout_sec
        while not client.service_is_ready():
            remaining = deadline - loop.time()
            if remaining <= 0.0:
                raise TimeoutError(
                    f"{service_name} unavailable after {self._timeout_sec:g}s"
                )
            await asyncio.sleep(min(0.05, remaining))

    async def _call(self, client: Any, request: Any, service_name: str) -> Any:
        future = client.call_async(request)
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self._timeout_sec
        while not future.done():
            remaining = deadline - loop.time()
            if remaining <= 0.0:
                future.cancel()
                raise TimeoutError(
                    f"{service_name} timed out after {self._timeout_sec:g}s"
                )
            await asyncio.sleep(min(0.01, remaining))
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()
