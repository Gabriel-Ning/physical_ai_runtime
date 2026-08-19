"""ROS-independent configuration contracts for RMI."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ControllerConfig:
    """One ros2_control controller, keyed externally by its input contract."""

    name: str
    implementation: str
    command_interface: str
    ros_actions: dict[str, str] = field(default_factory=dict)
    ros_topics: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class PartConfig:
    """One controllable component of a robot embodiment."""

    name: str
    part_type: str
    joint_names: tuple[str, ...]
    controller_manager: str
    controllers: dict[str, ControllerConfig]
    default_controller: str
    parent: str | None = None
    base_frame: str | None = None
    flange_frame: str | None = None
    tcp_frame: str | None = None


@dataclass(frozen=True)
class CameraSensorConfig:
    """One camera observation stream declared by the unified profile."""

    name: str
    ros_topic: str
    encoding: str = "rgb8"
    fps: int = 30
    resolution: tuple[int, int] = (480, 640)
    qos_profile: str = "sensor_data"


@dataclass(frozen=True)
class AgentConfig:
    """Application scheduling defaults for one EM provider-backed Agent."""

    name: str
    provider: str
    frequency: float | None = None


@dataclass(frozen=True)
class EmbodimentConfig:
    """Validated, unified RMI profile with role-specific consumer views."""

    name: str
    parts: dict[str, PartConfig]
    groups: dict[str, tuple[str, ...]] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    host_roles: dict[str, dict[str, Any]] = field(default_factory=dict)
    execution: dict[str, Any] = field(default_factory=dict)
    cameras: dict[str, CameraSensorConfig] = field(default_factory=dict)
    agents: dict[str, AgentConfig] = field(default_factory=dict)
    recording: dict[str, Any] = field(default_factory=dict)
    features: dict[str, Any] = field(default_factory=dict)
    calibration: dict[str, Any] = field(default_factory=dict)
    streams: dict[str, Any] = field(default_factory=dict)
    raw_data: dict[str, Any] = field(default_factory=dict, repr=False)

    @property
    def embodiment_type(self) -> str:
        return str(self.metadata.get("embodiment_type", "single_arm"))

    @property
    def vendor(self) -> str:
        return str(self.metadata.get("vendor", "generic"))

    @property
    def version(self) -> str:
        return str(self.metadata.get("version", "1.0"))

    @property
    def features_def(self) -> dict[str, Any]:
        return self.features

    def host_role(self, role: str) -> dict[str, Any]:
        try:
            return self.host_roles[role]
        except KeyError as exc:
            raise KeyError(
                f"host_roles.{role} not declared in embodiment profile {self.name!r}"
            ) from exc

    def profile_hash(self) -> str:
        canonical = json.dumps(
            self.raw_data, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def get_part_joints(self, name: str) -> list[str]:
        if name in self.parts:
            return list(self.parts[name].joint_names)
        if name in self.groups:
            joints: list[str] = []
            for member in self.groups[name]:
                joints.extend(self.get_part_joints(member))
            return joints
        return []

    def get_action_endpoint(
        self, part: str, command: str, *, provider: str | None = None
    ) -> str | None:
        return self._source_endpoint(part, command, "action", provider=provider)

    def get_topic_endpoint(
        self, part: str, command: str, *, provider: str | None = None
    ) -> str | None:
        return self._source_endpoint(part, command, "topic", provider=provider)

    def _source_endpoint(
        self,
        part: str,
        command: str,
        endpoint_kind: str,
        *,
        provider: str | None = None,
    ) -> str | None:
        matches: list[str] = []
        for source in self.execution.get("sources", []):
            if source.get("part") != part or source.get("command") != command:
                continue
            if provider is not None and source.get("provider") != provider:
                continue
            endpoint = source.get(endpoint_kind)
            if isinstance(endpoint, str) and endpoint:
                matches.append(endpoint)
        if len(matches) > 1:
            raise ValueError(
                f"ambiguous {endpoint_kind} for part={part!r} command={command!r}; "
                "pass provider= to disambiguate"
            )
        return matches[0] if matches else None

    @classmethod
    def from_yaml(cls, path: str | Path) -> EmbodimentConfig:
        with Path(path).open(encoding="utf-8") as stream:
            return cls.from_dict(yaml.safe_load(stream))

    @classmethod
    def from_dict(cls, data: Any) -> EmbodimentConfig:
        root = _mapping(data, "profile")
        metadata = _mapping(root.get("metadata"), "metadata")
        name = _string(metadata.get("name"), "metadata.name")
        raw_parts = _mapping(root.get("groups"), "groups")
        if not raw_parts:
            raise ValueError("groups must not be empty")

        parts: dict[str, PartConfig] = {}
        for part_name, value in raw_parts.items():
            path = f"groups.{part_name}"
            raw_part = _mapping(value, path)
            raw_controllers = _mapping(
                raw_part.get("controllers"), f"{path}.controllers"
            )
            if not raw_controllers:
                raise ValueError(f"{path}.controllers must not be empty")

            controllers: dict[str, ControllerConfig] = {}
            for contract, controller_value in raw_controllers.items():
                controller_path = f"{path}.controllers.{contract}"
                controller = _mapping(controller_value, controller_path)
                actions = _endpoints(
                    controller.get("ros_actions", {}), f"{controller_path}.ros_actions"
                )
                topics = _endpoints(
                    controller.get("ros_topics", {}), f"{controller_path}.ros_topics"
                )
                if not actions and not topics:
                    raise ValueError(
                        f"{controller_path} must declare ros_actions or ros_topics"
                    )
                controllers[contract] = ControllerConfig(
                    name=_string(controller.get("name"), f"{controller_path}.name"),
                    implementation=_string(
                        controller.get("implementation"),
                        f"{controller_path}.implementation",
                    ),
                    command_interface=_string(
                        controller.get("command_interface"),
                        f"{controller_path}.command_interface",
                    ),
                    ros_actions=actions,
                    ros_topics=topics,
                )

            default = _string(
                raw_part.get("default_controller"), f"{path}.default_controller"
            )
            if default not in controllers:
                raise ValueError(
                    f"{path}.default_controller must be a controller contract"
                )
            joint_names = raw_part.get("joint_names")
            if not isinstance(joint_names, list) or any(
                not isinstance(joint, str) or not joint for joint in joint_names
            ):
                raise TypeError(
                    f"{path}.joint_names must be a list of non-empty strings"
                )

            parts[part_name] = PartConfig(
                name=part_name,
                part_type=_string(raw_part.get("type"), f"{path}.type"),
                joint_names=tuple(joint_names),
                controller_manager=_string(
                    raw_part.get("controller_manager"), f"{path}.controller_manager"
                ),
                controllers=controllers,
                default_controller=default,
                parent=_optional_string(
                    raw_part.get("parent_group"), f"{path}.parent_group"
                ),
                base_frame=_optional_string(
                    raw_part.get("base_frame"), f"{path}.base_frame"
                ),
                flange_frame=_optional_string(
                    raw_part.get("flange_frame"), f"{path}.flange_frame"
                ),
                tcp_frame=_optional_string(
                    raw_part.get("tcp_frame"), f"{path}.tcp_frame"
                ),
            )

        for part in parts.values():
            if part.parent is not None and part.parent not in parts:
                raise ValueError(
                    f"part {part.name!r} references unknown parent {part.parent!r}"
                )

        raw_groups = _mapping(root.get("compound_groups", {}), "compound_groups")
        groups: dict[str, tuple[str, ...]] = {}
        for group_name, value in raw_groups.items():
            group = _mapping(value, f"compound_groups.{group_name}")
            members = group.get("included_groups")
            if not isinstance(members, list) or not members:
                raise TypeError(
                    f"compound_groups.{group_name}.included_groups "
                    "must be a non-empty list"
                )
            unknown = [member for member in members if member not in parts]
            if unknown:
                raise ValueError(
                    f"compound_groups.{group_name} references unknown groups: {unknown}"
                )
            groups[group_name] = tuple(members)

        host_roles = _mapping(root.get("host_roles", {}), "host_roles")
        execution = _mapping(root.get("execution_manager", {}), "execution_manager")
        _validate_execution(execution, parts)
        agents = _agents(root.get("agents", {}), execution)
        recording = _mapping(root.get("recorder", {}), "recorder")
        features = _mapping(root.get("features", {}), "features")
        calibration = _mapping(root.get("calibration", {}), "calibration")
        streams = _mapping(root.get("streams", {}), "streams")

        sensors = _mapping(root.get("sensors", {}), "sensors")
        raw_cameras = _mapping(sensors.get("cameras", {}), "sensors.cameras")
        cameras: dict[str, CameraSensorConfig] = {}
        for camera_name, value in raw_cameras.items():
            camera = _mapping(value, f"sensors.cameras.{camera_name}")
            resolution = camera.get("resolution", [480, 640])
            if (
                not isinstance(resolution, list)
                or len(resolution) != 2
                or any(not isinstance(item, int) or item <= 0 for item in resolution)
            ):
                raise TypeError(
                    f"sensors.cameras.{camera_name}.resolution must contain two positive integers"
                )
            cameras[camera_name] = CameraSensorConfig(
                name=camera_name,
                ros_topic=_string(
                    camera.get("ros_topic"),
                    f"sensors.cameras.{camera_name}.ros_topic",
                ),
                encoding=str(camera.get("encoding", "rgb8")),
                fps=int(camera.get("fps", 30)),
                resolution=(resolution[0], resolution[1]),
                qos_profile=str(camera.get("qos_profile", "sensor_data")),
            )

        return cls(
            name=name,
            parts=parts,
            groups=groups,
            metadata=dict(metadata),
            host_roles=dict(host_roles),
            execution=dict(execution),
            cameras=cameras,
            agents=agents,
            recording=dict(recording),
            features=dict(features),
            calibration=dict(calibration),
            streams=dict(streams),
            raw_data=dict(root),
        )


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TypeError(f"{path} must be a mapping")
    return value


def _string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise TypeError(f"{path} must be a non-empty string")
    return value


def _optional_string(value: Any, path: str) -> str | None:
    if value is None:
        return None
    return _string(value, path)


def _endpoints(value: Any, path: str) -> dict[str, str]:
    endpoints = _mapping(value, path)
    for name, endpoint in endpoints.items():
        _string(name, f"{path} key")
        _string(endpoint, f"{path}.{name}")
    return dict(endpoints)


def _agents(value: Any, execution: dict[str, Any]) -> dict[str, AgentConfig]:
    raw_agents = _mapping(value, "agents")
    providers = execution.get("providers", {})
    agents: dict[str, AgentConfig] = {}
    for name, raw_value in raw_agents.items():
        path = f"agents.{name}"
        raw_agent = _mapping(raw_value, path)
        provider = _string(raw_agent.get("provider"), f"{path}.provider")
        if provider not in providers:
            raise ValueError(f"{path} references unknown provider {provider!r}")
        frequency = raw_agent.get("frequency")
        if frequency is not None and (
            not isinstance(frequency, (int, float))
            or isinstance(frequency, bool)
            or frequency <= 0.0
        ):
            raise ValueError(f"{path}.frequency must be positive")
        agents[name] = AgentConfig(
            name=name,
            provider=provider,
            frequency=float(frequency) if frequency is not None else None,
        )
    return agents


def _validate_execution(
    execution: dict[str, Any], parts: dict[str, PartConfig]
) -> None:
    """Validate the optional canonical EM deployment view."""
    if "ingress" in execution:
        raise ValueError(
            "execution_manager.ingress is removed; declare "
            "execution_manager.providers and execution_manager.sources"
        )
    has_deployment = "providers" in execution or "sources" in execution
    if not has_deployment:
        return

    providers = _mapping(execution.get("providers"), "execution_manager.providers")
    sources = execution.get("sources")
    if not providers:
        raise ValueError("execution_manager.providers must not be empty")
    if not isinstance(sources, list) or not sources:
        raise ValueError("execution_manager.sources must be a non-empty list")

    provider_parts: dict[str, set[str]] = {}
    for provider_name, value in providers.items():
        provider_path = f"execution_manager.providers.{provider_name}"
        provider = _mapping(value, provider_path)
        priority = provider.get("priority")
        if not isinstance(priority, int) or isinstance(priority, bool):
            raise TypeError(f"{provider_path}.priority must be an integer")
        controllers = _mapping(
            provider.get("controllers"), f"{provider_path}.controllers"
        )
        if not controllers:
            raise ValueError(f"{provider_path}.controllers must not be empty")
        provider_parts[provider_name] = set(controllers)
        for part_name, contract in controllers.items():
            if part_name not in parts:
                raise ValueError(
                    f"{provider_path} references unknown part {part_name!r}"
                )
            if contract not in parts[part_name].controllers:
                raise ValueError(
                    f"{provider_path}.controllers.{part_name} references unknown "
                    f"controller contract {contract!r}"
                )

    for index, value in enumerate(sources):
        source_path = f"execution_manager.sources[{index}]"
        source = _mapping(value, source_path)
        provider_name = _string(source.get("provider"), f"{source_path}.provider")
        part_name = _string(source.get("part"), f"{source_path}.part")
        command = _string(source.get("command"), f"{source_path}.command")
        if provider_name not in providers:
            raise ValueError(
                f"{source_path} references unknown provider {provider_name!r}"
            )
        if part_name not in provider_parts[provider_name]:
            raise ValueError(
                f"{source_path} part {part_name!r} is not controlled by "
                f"provider {provider_name!r}"
            )
        endpoint_key = "action" if command == "joint_trajectory" else "topic"
        _string(source.get(endpoint_key), f"{source_path}.{endpoint_key}")
