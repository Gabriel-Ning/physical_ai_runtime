"""Application Context factories for one RMI process."""

from __future__ import annotations

import threading
from collections.abc import Mapping
from pathlib import Path
from types import TracebackType
from typing import Any, Self

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState as RosJointState

from .config import EmbodimentConfig
from .node import Node
from .robot import Robot
from .selection import AuthorityClient, ExecutionManagerClient
from .sensing import Camera, Sensor, _node_now_s


class Context:
    """Shared ROS resources and factories for one RMI application process."""

    def __init__(
        self,
        profile: EmbodimentConfig,
        node: Any,
        *,
        spin_node: bool = False,
        timeout_sec: float = 5.0,
        state_topic: str = "/joint_states",
        provider_selector: Any | None = None,
        recorder_client_factory: Any | None = None,
        owns_node: bool = False,
    ) -> None:
        self.profile = profile
        self.node = node
        self.provider_selector: AuthorityClient = (
            provider_selector
            if provider_selector is not None
            else ExecutionManagerClient(profile, node, timeout_sec=timeout_sec)
        )
        # Node clock keeps robot-state receive times comparable with sensor
        # facades (and correct under use_sim_time).
        self.robot = Robot(
            profile,
            self.provider_selector,
            clock=lambda: _node_now_s(node),
        )
        self._timeout_sec = timeout_sec
        self._owns_node = owns_node
        self._closed = False
        self._execution_prepared = False
        self._nodes: dict[str, Node] = {}
        self._cameras: dict[str, Camera[Any]] = {}
        self._camera_history_sizes: dict[str, int] = {}
        self._sensors: dict[str, Sensor[Any]] = {}
        self._sensor_specs: dict[str, tuple[str, Any, int]] = {}
        self._recorder = None
        self._recorder_config = None
        self._recorder_client_factory = recorder_client_factory
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

    @classmethod
    def from_profile(
        cls,
        profile: EmbodimentConfig | Mapping[str, Any] | str | Path,
        *,
        node: Any | None = None,
        spin_node: bool = True,
        timeout_sec: float = 5.0,
        state_topic: str = "/joint_states",
        recorder_client_factory: Any | None = None,
    ) -> Context:
        """Create an application Context from an embodiment profile.

        If ``node`` is omitted, this object owns the created node and the
        process-global rclpy context; closing it calls ``rclpy.shutdown()``.
        Processes that host other ROS nodes must pass their own node.
        """
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
            state_topic=state_topic,
            provider_selector=ExecutionManagerClient(
                config, node, timeout_sec=timeout_sec
            ),
            recorder_client_factory=recorder_client_factory,
            owns_node=owns_node,
        )

    def is_ready(self, *, check_cameras: bool = False) -> bool:
        """Check observation readiness; this does not require the Execution Manager."""
        if not self.robot.is_ready():
            return False
        if check_cameras:
            for cam in self._cameras.values():
                if not cam.is_ready():
                    return False
        return True

    def wait_until_ready(
        self,
        timeout: float = 10.0,
        check_frequency: float = 50.0,
        *,
        check_hardware: bool = False,
        check_cameras: bool = False,
        require_execution_manager: bool = False,
        prepare_execution: bool = True,
    ) -> None:
        """Wait for RT state/health and optional sensors, without acquiring control.

        When ``prepare_execution`` is true (default), also runs one-shot application
        startup authority recovery: clear leftover EM FAULT after RT faults/restarts.
        Live transition failures are not auto-recovered during the run.
        """
        # 1. Wait for robot body and hardware readiness
        self.robot.wait_until_ready(
            timeout=timeout,
            check_frequency=check_frequency,
            check_hardware=check_hardware,
        )

        if require_execution_manager or prepare_execution:
            self.provider_selector.require_execution_manager(timeout_sec=timeout)
        if prepare_execution:
            self.prepare_execution(timeout_sec=timeout)

        # 2. If check_cameras is requested, instantiate declared cameras and wait for them
        if check_cameras and hasattr(self.profile, "cameras") and self.profile.cameras:
            for cam_name in self.profile.cameras:
                if cam_name not in self._cameras:
                    self.make_camera(cam_name)
            for cam in self._cameras.values():
                cam.wait_until_ready(timeout=timeout)

    def prepare_execution(self, *, timeout_sec: float | None = None) -> Any:
        """One-shot app-start authority check: clear FAULT, leave OWNED alone.

        Call at application startup (``wait_until_ready`` does this by default).
        Do not call mid-run to paper over live switch failures — those stay FAULT
        until the operator restarts the app or explicitly preempts.
        """
        import time

        from execution_manager_interfaces.msg import ResourceAuthority

        from .selection import AuthoritySnapshot

        timeout = self._timeout_sec if timeout_sec is None else timeout_sec
        if timeout <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if self._execution_prepared:
            describe = getattr(self.provider_selector, "describe_authority", None)
            if describe is not None:
                return describe()
            return AuthoritySnapshot({})

        self.provider_selector.require_execution_manager(timeout_sec=timeout)
        describe = getattr(self.provider_selector, "describe_authority", None)
        clear_fault = getattr(self.provider_selector, "clear_fault", None)
        if describe is None or clear_fault is None:
            self._execution_prepared = True
            return AuthoritySnapshot({})

        deadline = time.monotonic() + timeout
        snapshot = describe()
        while not snapshot.resources and time.monotonic() < deadline:
            time.sleep(0.01)
            snapshot = describe()

        resources = _profile_command_contracts(self.profile)
        faulted = {
            name: contract
            for name, contract in resources.items()
            if int(
                snapshot.resources.get(name, {}).get(
                    "authority_state", ResourceAuthority.UNOWNED
                )
            )
            == int(ResourceAuthority.FAULT)
        }
        if faulted:
            snapshot = clear_fault(faulted)
        self._execution_prepared = True
        return snapshot

    def make_node(
        self,
        name: str,
        producer: Any = None,
    ) -> Node:
        """Create or retrieve one action source handle, optionally linked to its producer."""
        if name in self._nodes:
            if producer is not None and self._nodes[name].producer is not producer:
                raise ValueError(
                    f"node {name!r} already exists with a different producer"
                )
            return self._nodes[name]
        result = Node(
            name,
            self.node,
            self.profile,
            self.provider_selector,
            producer,
        )
        self._nodes[name] = result
        return result

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
        self.robot._attach_sensor(camera)
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
        self.robot._attach_sensor(sensor)
        self._sensor_specs[name] = (topic, message_type, history_size)
        return sensor

    def make_recorder(
        self,
        *,
        config: Any | None = None,
        client: Any | None = None,
        node_name: str = "/episode_recorder",
        autostart: bool = True,
    ) -> Any:
        """Construct the MCAP episode-recorder client; never starts the server."""
        if self._recorder is not None:
            if config is not None and config is not self._recorder_config:
                raise ValueError(
                    "recorder already exists; configure it once via make_recorder()"
                )
            return self._recorder
        from .recording import EpisodeRecorder

        if client is None:
            factory = self._recorder_client_factory or _make_episode_recorder_client
            client = factory(
                self.node,
                node_name=node_name,
                config=config,
                profile_values=dict(self.profile.recording),
            )
        self._recorder_config = config
        self._recorder = EpisodeRecorder(
            client,
            autostart=autostart,
            node_name=node_name,
        )
        return self._recorder

    def make_replay_buffer(self, *, capacity: int = 100_000) -> Any:
        """Construct an in-memory (observation, action) buffer for gym / RL."""
        from .recording import MemoryReplayBuffer

        return MemoryReplayBuffer(capacity=capacity)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._recorder is not None and hasattr(self._recorder, "close"):
            try:
                self._recorder.close()
            except Exception:
                pass
            self._recorder = None
        for action_node in self._nodes.values():
            action_node.close()
        self._nodes.clear()
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.5)
            except Exception:
                pass
            self._executor = None
        if self._spin_thread is not None:
            try:
                self._spin_thread.join(timeout=0.5)
            except Exception:
                pass
            self._spin_thread = None
        for camera in list(self._cameras.values()):
            try:
                camera.close()
            except Exception:
                pass
        for sensor in list(self._sensors.values()):
            try:
                sensor.close()
            except Exception:
                pass
        try:
            self.provider_selector.close()
        except Exception:
            pass
        if hasattr(self.node, "destroy_subscription"):
            try:
                self.node.destroy_subscription(self._state_subscription)
            except Exception:
                pass
        if self._owns_node and hasattr(self.node, "destroy_node"):
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

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
    # 1. Search relative to repository workspace apps/profiles or apps/
    workspace_root = Path(__file__).resolve().parents[4]
    for apps_dir in ("apps/profiles", "apps/config", "apps"):
        app_candidate = workspace_root / apps_dir / candidate
        if app_candidate.is_file():
            return app_candidate

    # 2. Search in ROS 2 ament package share for rmi
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("rmi")) / "config"
        for sub in ("templates", "embodiment_profiles"):
            share_candidate = share_dir / sub / candidate.name
            if share_candidate.is_file():
                return share_candidate
    except Exception:
        pass

    # 3. Search relative to repository workspace source tree
    config_dir = Path(__file__).resolve().parents[1] / "config"
    for sub in ("templates", "embodiment_profiles"):
        repo_src = config_dir / sub / candidate.name
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


def _profile_command_contracts(profile: EmbodimentConfig) -> dict[str, str]:
    """Union of Node resource→contract maps declared in the application profile."""
    resources: dict[str, str] = {}
    for node in profile.nodes.values():
        resources.update(dict(node.resources))
    return resources


def _make_episode_recorder_client(
    node: Any,
    *,
    node_name: str,
    config: Any | None,
    profile_values: dict[str, Any],
) -> Any:
    """Build the installed episode_recorder SDK client for an existing server."""
    import inspect

    from episode_recorder import (
        Recorder as EpisodeRecorderClient,
    )
    from episode_recorder import (
        RecorderConfig,
        RosRecorderBackend,
    )

    recorder_config = config
    if recorder_config is None:
        valid_keys = inspect.signature(RecorderConfig.__init__).parameters
        values = {
            key: value
            for key, value in profile_values.items()
            if key != "self" and key in valid_keys
        }
        # ``profile`` is not a RecorderConfig field (launch uses it); resolve
        # before filtering so older profiles without ``config`` still work.
        if not (values.get("profile_dir") or values.get("contract_path")):
            legacy_profile = profile_values.get("profile")
            if legacy_profile:
                values["contract_path"] = str(
                    _resolve_recording_profile(str(legacy_profile))
                )
        recorder_config = RecorderConfig(**values)
    return EpisodeRecorderClient(
        recorder_config,
        RosRecorderBackend(node, node_name),
    )


def _resolve_recording_profile(profile: str) -> Path:
    """Resolve one recorder stream profile from ROS shares or this workspace."""
    filename = f"{profile}.yaml"
    install_candidates: list[Path] = []
    source_candidates: list[Path] = []

    try:
        from ament_index_python.packages import get_packages_with_prefixes

        for package, prefix in get_packages_with_prefixes().items():
            candidate = Path(prefix) / "share" / package / "recording" / filename
            if candidate.is_file():
                install_candidates.append(candidate.resolve())
    except (ImportError, OSError):
        pass

    if install_candidates:
        if len(install_candidates) > 1:
            rendered = ", ".join(str(path) for path in sorted(install_candidates))
            raise RuntimeError(
                f"recorder profile {profile!r} is ambiguous across ROS packages: "
                f"{rendered}"
            )
        return install_candidates[0]

    workspace_root = Path(__file__).resolve().parents[4]
    candidate = workspace_root / "apps" / "recording" / filename
    if candidate.is_file():
        source_candidates.append(candidate.resolve())

    if not source_candidates:
        raise FileNotFoundError(
            f"recorder profile {profile!r} was not found under ROS package shares "
            "or apps/recording"
        )
    if len(source_candidates) > 1:
        rendered = ", ".join(str(path) for path in sorted(source_candidates))
        raise RuntimeError(f"recorder profile {profile!r} is ambiguous: {rendered}")
    return source_candidates[0]
