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
    """Application defaults for one dynamically claimed action source."""

    name: str
    source_role: str
    resources: dict[str, str]
    frequency: float | None = None


@dataclass(frozen=True)
class EmbodimentConfig:
    """Validated, unified RMI profile with role-specific consumer views."""

    name: str
    parts: dict[str, PartConfig]
    groups: dict[str, tuple[str, ...]] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    host_roles: dict[str, dict[str, Any]] = field(default_factory=dict)
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

    @classmethod
    def from_yaml(cls, path: str | Path) -> EmbodimentConfig:
        path = Path(path)
        with path.open(encoding="utf-8") as stream:
            return cls.from_dict(yaml.safe_load(stream), source_path=path)

    @classmethod
    def from_dict(
        cls, data: Any, *, source_path: Path | str | None = None
    ) -> EmbodimentConfig:
        root = _mapping(data, "profile")
        if "provider_selection" in root:
            raise ValueError(
                "provider_selection is removed; declare dynamic agents with "
                "source_role and resources"
            )
        resolved_source = Path(source_path) if source_path is not None else None
        root = _bind_execution_manager_routing(root, resolved_source)
        root = _bind_recorder_stream_contract(root, resolved_source)
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
        agents = _agents(root.get("agents", {}), parts)
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
            cameras=cameras,
            agents=agents,
            recording=dict(recording),
            features=dict(features),
            calibration=dict(calibration),
            streams=dict(streams),
            raw_data=dict(root),
        )


def _bind_execution_manager_routing(
    root: dict[str, Any], source_path: Path | None
) -> dict[str, Any]:
    """Load the Robot groups from the EM execution-capability projection."""
    reference = root.get("execution_manager_config")
    if reference is None:
        return root
    if root.get("groups"):
        raise ValueError(
            "groups belong to the Execution Manager capability file; "
            "do not duplicate them in the RMI profile"
        )
    em_path = _resolve_package_file(
        reference, source_path, "execution_manager_config"
    )
    with em_path.open(encoding="utf-8") as handle:
        em = _mapping(yaml.safe_load(handle), str(em_path))
    bound = dict(root)
    bound["groups"] = em["groups"]
    return bound


def _bind_recorder_stream_contract(
    root: dict[str, Any], source_path: Path | None
) -> dict[str, Any]:
    """Resolve recorder.config package/file to the bringup stream contract."""
    recording = root.get("recorder")
    if not isinstance(recording, dict) or "config" not in recording:
        return root
    path = _resolve_package_file(recording["config"], source_path, "recorder.config")
    bound = dict(root)
    recorder = dict(recording)
    recorder["contract_path"] = str(path)
    bound["recorder"] = recorder
    return bound


def _resolve_package_file(
    reference: Any, source_path: Path | None, field_name: str
) -> Path:
    if isinstance(reference, str):
        candidate = Path(reference)
        if candidate.is_file():
            return candidate
        if source_path is not None:
            relative = (source_path.parent / reference).resolve()
            if relative.is_file():
                return relative
        raise FileNotFoundError(f"{field_name} file not found: {reference}")

    spec = _mapping(reference, field_name)
    package = _string(spec.get("package"), f"{field_name}.package")
    file_name = _string(spec.get("file"), f"{field_name}.file")
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory(package)) / file_name
        if installed.is_file():
            return installed
    except Exception:
        pass
    repo = _repository_root(source_path)
    for package_xml in repo.glob("src/**/package.xml"):
        if f"<name>{package}</name>" not in package_xml.read_text(
            encoding="utf-8"
        ):
            continue
        candidate = package_xml.parent / file_name
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"{field_name} {package!r} {file_name!r} not found")


def _repository_root(source_path: Path | None) -> Path:
    start = source_path.resolve() if source_path is not None else Path(__file__).resolve()
    for parent in (start, *start.parents):
        if (parent / "apps" / "profiles").is_dir():
            return parent
    raise FileNotFoundError("repository root with apps/profiles not found")


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


def _agents(value: Any, parts: dict[str, PartConfig]) -> dict[str, AgentConfig]:
    raw_agents = _mapping(value, "agents")
    agents: dict[str, AgentConfig] = {}
    for name, raw_value in raw_agents.items():
        path = f"agents.{name}"
        raw_agent = _mapping(raw_value, path)
        role = _string(raw_agent.get("source_role"), f"{path}.source_role").upper()
        if role not in {"POLICY", "TELEOP", "PLANNER"}:
            raise ValueError(f"{path}.source_role must be POLICY, TELEOP, or PLANNER")
        resources = _mapping(raw_agent.get("resources"), f"{path}.resources")
        if not resources:
            raise ValueError(f"{path}.resources must not be empty")
        validated_resources: dict[str, str] = {}
        for resource, command in resources.items():
            command = _string(command, f"{path}.resources.{resource}")
            if resource not in parts:
                raise ValueError(f"{path} references unknown resource {resource!r}")
            available = {
                topic
                for controller in parts[resource].controllers.values()
                for topic in controller.ros_topics
            }
            if any(
                "follow_joint_trajectory" in controller.ros_actions
                for controller in parts[resource].controllers.values()
            ):
                available.add("joint_trajectory")
            if command not in available:
                raise ValueError(
                    f"{path}.resources.{resource} references unsupported command "
                    f"{command!r}"
                )
            validated_resources[resource] = command
        frequency = raw_agent.get("frequency")
        if frequency is not None and (
            not isinstance(frequency, (int, float))
            or isinstance(frequency, bool)
            or frequency <= 0.0
        ):
            raise ValueError(f"{path}.frequency must be positive")
        agents[name] = AgentConfig(
            name=name,
            source_role=role,
            resources=validated_resources,
            frequency=float(frequency) if frequency is not None else None,
        )
    return agents
