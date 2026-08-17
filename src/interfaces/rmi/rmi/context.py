"""Application Context factories for one RMI process."""

from __future__ import annotations

import threading
from collections.abc import Mapping
from pathlib import Path
from types import TracebackType
from typing import TYPE_CHECKING, Any, Self

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState as RosJointState

from .provider import ActionProviderClient
from .agent import Agent, Robot
from .config import EmbodimentConfig
from .execution import LocalExecutionManager
from .sensing import Camera, Sensor, _node_now_s

if TYPE_CHECKING:
    from .planning import (
        CartesianStreamer,
        JointStreamer,
        PlannerCatalog,
        Planner,
        Resolver,
    )

class Context:
    """Shared ROS resources and factories for one RMI application process."""

    def __init__(
        self,
        profile: EmbodimentConfig,
        node: Any,
        *,
        spin_node: bool = False,
        timeout_sec: float = 5.0,
        action_client_factory: Any | None = None,
        state_topic: str = "/joint_states",
        planner_catalog: PlannerCatalog | None = None,
        owns_node: bool = False,
    ) -> None:
        self.profile = profile
        self.node = node
        self.execution = LocalExecutionManager(profile, node=node)
        # Node clock keeps robot-state receive times comparable with sensor
        # facades (and correct under use_sim_time).
        self.robot = Robot(
            profile,
            self.execution,
            clock=lambda: _node_now_s(node),
        )
        self._timeout_sec = timeout_sec
        self._action_client_factory = action_client_factory
        self._owns_node = owns_node
        self._closed = False
        self._agents: dict[str, Agent] = {}
        self._cameras: dict[str, Camera[Any]] = {}
        self._camera_history_sizes: dict[str, int] = {}
        self._sensors: dict[str, Sensor[Any]] = {}
        self._sensor_specs: dict[str, tuple[str, Any, int]] = {}
        self._recorder = None
        self._recorder_config = None
        self._planner_catalog = planner_catalog
        self._executor = None
        self._spin_thread = None
        self._state_subscription = node.create_subscription(
            RosJointState,
            state_topic,
            self.robot.update_joint_state,
            qos_profile_sensor_data,
        )
        if spin_node:
            self._start_executor()

    def _catalog(self):
        if self._planner_catalog is None:
            from .planning import PlannerCatalog

            self._planner_catalog = PlannerCatalog()
        return self._planner_catalog

    @classmethod
    def from_profile(
        cls,
        profile: EmbodimentConfig | Mapping[str, Any] | str | Path,
        *,
        node: Any | None = None,
        spin_node: bool = True,
        timeout_sec: float = 5.0,
        action_client_factory: Any | None = None,
        state_topic: str = "/joint_states",
        planner_catalog: PlannerCatalog | None = None,
    ) -> Context:
        config = _load_profile(profile)
        owns_node = False
        if node is None:
            import rclpy

            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node(f"{config.name}_rmi_client")
            owns_node = True
        return cls(
            config,
            node,
            spin_node=spin_node,
            timeout_sec=timeout_sec,
            action_client_factory=action_client_factory,
            state_topic=state_topic,
            planner_catalog=planner_catalog,
            owns_node=owns_node,
        )

    def register_planner(
        self,
        name: str,
        factory: Any,
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        """Register a lazy adapter factory for this application process."""
        self._catalog().register(
            name,
            factory,
            display_name=display_name,
            warmup_on_create=warmup_on_create,
        )

    def make_planner(self, name: str) -> Planner:
        """Construct or reuse a pure planner; this never grants control authority."""
        return self._catalog().make(name)

    def register_resolver(
        self,
        name: str,
        factory: Any,
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._catalog().register_resolver(
            name,
            factory,
            display_name=display_name,
            warmup_on_create=warmup_on_create,
        )

    def make_resolver(self, name: str) -> Resolver:
        return self._catalog().make_resolver(name)

    def register_joint_streamer(
        self,
        name: str,
        factory: Any,
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._catalog().register_joint_streamer(
            name,
            factory,
            display_name=display_name,
            warmup_on_create=warmup_on_create,
        )

    def make_joint_streamer(self, name: str) -> JointStreamer:
        return self._catalog().make_joint_streamer(name)

    def register_cartesian_streamer(
        self,
        name: str,
        factory: Any,
        *,
        display_name: str = "",
        warmup_on_create: bool = True,
    ) -> None:
        self._catalog().register_cartesian_streamer(
            name,
            factory,
            display_name=display_name,
            warmup_on_create=warmup_on_create,
        )

    def make_cartesian_streamer(self, name: str) -> CartesianStreamer:
        return self._catalog().make_cartesian_streamer(name)

    def available_planners(self) -> list[str]:
        return self._catalog().available()

    def make_agent(
        self,
        name: str,
        *,
        robot: Robot | None = None,
        sensors: list[Sensor[Any]] | tuple[Sensor[Any], ...] = (),
        frequency: float | None = None,
    ) -> Agent:
        """Construct or reuse an Agent with fixed observation sources."""
        target_robot = robot if robot is not None else self.robot
        requested_sensors = tuple(sensors)
        agent_config = self.profile.agents.get(name)
        provider = agent_config.provider if agent_config is not None else name
        configured_frequency = (
            agent_config.frequency if agent_config is not None else None
        )
        requested_frequency = configured_frequency if frequency is None else frequency
        if name in self._agents:
            agent = self._agents[name]
            if (
                agent._robot is not target_robot
                or agent.sensors != requested_sensors
                or agent.frequency != requested_frequency
            ):
                raise ValueError(
                    f"agent {name!r} already exists with different robot, sensors, "
                    "or frequency"
                )
            return agent
        client = ActionProviderClient.from_profile(
            self.profile,
            provider,
            self.node,
            self._timeout_sec,
            action_client_factory=self._action_client_factory,
        )
        self._agents[name] = Agent(
            name,
            client,
            self.profile,
            self.execution,
            provider=provider,
            frequency=requested_frequency,
            robot=target_robot,
            sensors=requested_sensors,
        )
        return self._agents[name]

    def make_camera(
        self,
        name: str,
        *,
        converter: Any | None = None,
        history_size: int = 8,
    ) -> Camera[Any]:
        if name in self._cameras:
            if converter is not None:
                raise ValueError(
                    f"camera {name!r} already exists; configure its converter once"
                )
            if history_size != self._camera_history_sizes[name]:
                raise ValueError(
                    f"camera {name!r} already exists with history_size="
                    f"{self._camera_history_sizes[name]}"
                )
            return self._cameras[name]
        try:
            config = self.profile.cameras[name]
        except KeyError as exc:
            raise KeyError(f"unknown profile camera {name!r}") from exc
        camera = Camera(
            config,
            self.node,
            converter=converter,
            history_size=history_size,
        )
        self._cameras[name] = camera
        self._camera_history_sizes[name] = history_size
        return camera

    def make_sensor(
        self,
        name: str,
        *,
        topic: str,
        message_type: Any,
        converter: Any | None = None,
        history_size: int = 8,
    ) -> Sensor[Any]:
        if name in self._sensors:
            existing_topic, existing_type, existing_history = self._sensor_specs[name]
            if (
                topic != existing_topic
                or message_type is not existing_type
                or history_size != existing_history
            ):
                raise ValueError(
                    f"sensor {name!r} already exists with different topic, "
                    "message_type, or history_size"
                )
            if converter is not None:
                raise ValueError(
                    f"sensor {name!r} already exists; configure its converter once"
                )
            return self._sensors[name]
        sensor = Sensor(
            name=name,
            node=self.node,
            topic=topic,
            message_type=message_type,
            converter=converter,
            history_size=history_size,
        )
        self._sensors[name] = sensor
        self._sensor_specs[name] = (topic, message_type, history_size)
        return sensor

    def make_recorder(
        self,
        *,
        config: Any | None = None,
        node_name: str = "/episode_recorder",
    ) -> Any:
        """Construct the synchronous facade for the independent recorder node."""
        if self._recorder is not None:
            if config is not None and config is not self._recorder_config:
                raise ValueError(
                    "recorder already exists; configure it once via make_recorder()"
                )
            return self._recorder
        from episode_recorder import Recorder, RecorderConfig, RosRecorderBackend

        from .recording import Recorder

        if config is None:
            values = dict(self.profile.recording)
            if values.get("profile") and not values.get("profile_dir"):
                from ament_index_python.packages import get_package_share_directory

                values["profile_dir"] = str(
                    Path(get_package_share_directory("episode_recorder"))
                    / "config"
                    / "profiles"
                )
            recorder_config = RecorderConfig(**values)
        else:
            recorder_config = config
        self._recorder_config = recorder_config
        self._recorder = Recorder(
            Recorder(recorder_config, RosRecorderBackend(self.node, node_name))
        )
        return self._recorder

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        for camera in self._cameras.values():
            camera.close()
        for sensor in self._sensors.values():
            sensor.close()
        self.execution.close()
        if hasattr(self.node, "destroy_subscription"):
            self.node.destroy_subscription(self._state_subscription)
        if self._owns_node and hasattr(self.node, "destroy_node"):
            self.node.destroy_node()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc, traceback
        self.close()

    def _start_executor(self) -> None:
        from rclpy.executors import MultiThreadedExecutor

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name=f"{self.profile.name}-rmi-executor",
            daemon=True,
        )
        self._spin_thread.start()



def _resolve_profile_path(profile_path: str | Path) -> Path:
    candidate = Path(profile_path)
    if candidate.is_file():
        return candidate
    # 1. Search in ROS 2 ament package share for rmi
    try:
        from ament_index_python.packages import get_package_share_directory
        share_candidate = (
            Path(get_package_share_directory("rmi"))
            / "config"
            / "embodiment_profiles"
            / candidate.name
        )
        if share_candidate.is_file():
            return share_candidate
    except Exception:
        pass
    # 2. Search relative to repository workspace source tree
    repo_src = (
        Path(__file__).resolve().parents[2]
        / "config"
        / "embodiment_profiles"
        / candidate.name
    )
    if repo_src.is_file():
        return repo_src
    return candidate


def _load_profile(
    profile: EmbodimentConfig | Mapping[str, Any] | str | Path,
) -> EmbodimentConfig:
    if isinstance(profile, EmbodimentConfig):
        return profile
    if isinstance(profile, Mapping):
        return EmbodimentConfig.from_dict(dict(profile))
    if isinstance(profile, (str, Path)):
        resolved = _resolve_profile_path(profile)
        return EmbodimentConfig.from_yaml(resolved)
    raise TypeError("profile must be EmbodimentConfig, mapping, or path")


