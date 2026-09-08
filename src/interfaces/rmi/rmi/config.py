"""ROS-independent configuration contracts for RMI."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ControllerConfig:
    """One ros2_control controller, keyed externally by its input contract."""

    name: str
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
class NodeInputConfig:
    """One typed Execution Manager ingress bound to an action node."""

    endpoint: str
    command_contract: str
    is_action: bool = False


@dataclass(frozen=True)
class NodeConfig:
    """Application defaults for one profile-declared action node."""

    name: str
    source_role: str
    resources: dict[str, str]
    frequency: float | None = None
    inputs: dict[str, NodeInputConfig] = field(default_factory=dict)


@dataclass(frozen=True)
class JointGroup:
    """One profile Part's contiguous slice in a Node joint vector."""

    part: str
    command: str
    joint_names: tuple[str, ...]
    start: int
    stop: int


@dataclass(frozen=True)
class JointLayout:
    """ROS-independent ordered joint layout resolved for one profile Node."""

    groups: tuple[JointGroup, ...]
    joint_names: tuple[str, ...]

    @property
    def dimension(self) -> int:
        return len(self.joint_names)

    def order_values(
        self, names: Sequence[str], values: Sequence[float]
    ) -> list[float]:
        """Reorder named values into the canonical Node joint order."""
        if len(names) != len(values):
            raise ValueError(
                f"joint name count {len(names)} does not match value count {len(values)}"
            )
        by_name = dict(zip(names, values, strict=True))
        if len(by_name) != len(names):
            raise ValueError("joint names must be unique")
        missing = [name for name in self.joint_names if name not in by_name]
        if missing:
            raise ValueError(f"joint values are missing required names: {missing}")
        return [float(by_name[name]) for name in self.joint_names]

    def split_values(
        self, values: Sequence[float]
    ) -> tuple[tuple[JointGroup, list[float]], ...]:
        """Split one canonical flat vector into its profile Part groups."""
        if len(values) != self.dimension:
            raise ValueError(
                f"joint vector dimension {len(values)} does not match layout "
                f"{self.dimension}"
            )
        return tuple(
            (group, [float(value) for value in values[group.start : group.stop]])
            for group in self.groups
        )


@dataclass(frozen=True)
class PolicyLayout:
    """Backend-neutral I/O layout for one Profile policy Node."""

    profile_name: str
    profile_hash: str
    joints: JointLayout
    state_topic: str
    action_topics: dict[str, str]
    camera_sources: dict[str, str]
    camera_topics: dict[str, str]
    camera_shapes: dict[str, tuple[int, int, int]]
    frequency: float
    control_mode: str = "joint"
    action_space: str = "abs"
    pose_part: str | None = None
    gripper_parts: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        mode = str(self.control_mode).lower()
        if mode not in {"joint", "cartesian"}:
            raise ValueError(
                f"control_mode must be 'joint' or 'cartesian', got {self.control_mode!r}"
            )
        space = str(self.action_space).lower()
        if space not in {"abs", "rel"}:
            raise ValueError(
                f"action_space must be 'abs' or 'rel', got {self.action_space!r}"
            )
        if mode == "cartesian" and not self.pose_part:
            raise ValueError("cartesian PolicyLayout requires pose_part")

    @property
    def state_feature_names(self) -> tuple[str, ...]:
        """Canonical scalar state features shared by data and policy runtimes."""
        return tuple(f"{name}.pos" for name in self.joints.joint_names)

    @property
    def action_dimension(self) -> int:
        """Policy action vector size (may differ from joint state size)."""
        if self.control_mode == "cartesian":
            gripper_dim = sum(
                len(group.joint_names)
                for group in self.joints.groups
                if group.part in self.gripper_parts
            )
            return 6 + gripper_dim
        return self.joints.dimension

    @property
    def action_feature_names(self) -> tuple[str, ...]:
        """Canonical scalar action features in policy output order."""
        if self.control_mode == "cartesian":
            names = ["ee_x", "ee_y", "ee_z", "ee_ax", "ee_ay", "ee_az"]
            for group in self.joints.groups:
                if group.part in self.gripper_parts:
                    names.extend(f"{name}.pos" for name in group.joint_names)
            return tuple(names)
        return self.state_feature_names

    @property
    def topics(self) -> tuple[str, ...]:
        return (
            self.state_topic,
            *self.action_topics.values(),
            *self.camera_topics.values(),
        )


@dataclass(frozen=True)
class EmbodimentConfig:
    """Validated, unified RMI profile with role-specific consumer views."""

    name: str
    parts: dict[str, PartConfig]
    groups: dict[str, tuple[str, ...]] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    host_roles: dict[str, dict[str, Any]] = field(default_factory=dict)
    cameras: dict[str, CameraSensorConfig] = field(default_factory=dict)
    nodes: dict[str, NodeConfig] = field(default_factory=dict)
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

    def get_part_names(self, name: str) -> list[str]:
        """Expand one physical part or compound group in declaration order."""
        if name in self.parts:
            return [name]
        if name in self.groups:
            parts: list[str] = []
            for member in self.groups[name]:
                parts.extend(self.get_part_names(member))
            return parts
        return []

    def joint_layout(self, node_name: str) -> JointLayout:
        """Resolve a Node's joint resources in Profile declaration order."""
        try:
            node = self.nodes[node_name]
        except KeyError as exc:
            raise KeyError(f"profile has no node {node_name!r}") from exc

        groups: list[JointGroup] = []
        joint_names: list[str] = []
        offset = 0
        for part_name, command in node.resources.items():
            part = self.parts[part_name]
            if not part.joint_names:
                raise ValueError(f"part {part_name!r} has no joints")
            stop = offset + len(part.joint_names)
            groups.append(
                JointGroup(part_name, command, part.joint_names, offset, stop)
            )
            joint_names.extend(part.joint_names)
            offset = stop
        if len(joint_names) != len(set(joint_names)):
            raise ValueError(f"node {node_name!r} joint names must be unique")
        return JointLayout(tuple(groups), tuple(joint_names))

    def policy_layout(self, node_name: str = "Policy") -> PolicyLayout:
        """Resolve the common policy layout consumed offline and at runtime."""
        joints = self.joint_layout(node_name)
        pose_parts = [
            group.part
            for group in joints.groups
            if group.command == "pose_reference"
        ]
        joint_parts = [
            group.part
            for group in joints.groups
            if group.command == "joint_reference"
        ]
        other = [
            f"{group.part}.{group.command}"
            for group in joints.groups
            if group.command not in {"joint_reference", "pose_reference"}
        ]
        if other:
            raise ValueError(
                "policy layout supports joint_reference and pose_reference only; "
                f"got {other}"
            )
        if pose_parts and len(pose_parts) != 1:
            raise ValueError(
                "cartesian policy layout requires exactly one pose_reference part; "
                f"got {pose_parts}"
            )
        if pose_parts:
            control_mode = "cartesian"
            pose_part = pose_parts[0]
            gripper_parts = tuple(joint_parts)
            if not gripper_parts:
                raise ValueError(
                    "cartesian policy layout requires at least one gripper "
                    "joint_reference part"
                )
        else:
            control_mode = "joint"
            pose_part = None
            gripper_parts = ()

        node = self.nodes[node_name]
        if node.frequency is None or node.frequency <= 0.0:
            raise ValueError(f"node {node_name!r} must declare a positive frequency")

        state_feature = self.features.get("observation", {}).get(
            "observation.state", {}
        )
        state_topic = str(state_feature.get("source", "/joint_states"))
        if not state_topic.startswith("/"):
            raise ValueError("observation.state.source must be an absolute ROS topic")

        action_feature = self.features.get("action", {}).get("action", {})
        action_space = str(action_feature.get("space", "abs")).lower()
        if action_space not in {"abs", "rel"}:
            raise ValueError(
                f"features.action.action.space must be abs or rel, got {action_space!r}"
            )

        action_topics: dict[str, str] = {}
        for group in joints.groups:
            matches = [
                controller.ros_topics[group.command]
                for controller in self.parts[group.part].controllers.values()
                if group.command in controller.ros_topics
            ]
            if len(matches) != 1:
                raise KeyError(
                    f"expected one command topic for {group.part}.{group.command}, "
                    f"found {len(matches)}"
                )
            action_topics[group.part] = matches[0]

        camera_sources: dict[str, str] = {}
        camera_topics: dict[str, str] = {}
        camera_shapes: dict[str, tuple[int, int, int]] = {}
        for feature_name, feature in self.features.get("observation", {}).items():
            if feature.get("type") != "image":
                continue
            source = str(feature.get("source", ""))
            prefix = "sensors.cameras."
            if not source.startswith(prefix):
                raise ValueError(
                    f"image feature {feature_name!r} has invalid source {source!r}"
                )
            camera_name = source.removeprefix(prefix)
            try:
                camera = self.cameras[camera_name]
            except KeyError as exc:
                raise KeyError(
                    f"image feature {feature_name!r} references unknown camera "
                    f"{camera_name!r}"
                ) from exc
            shape = feature.get("shape")
            if not isinstance(shape, list) or len(shape) != 3:
                raise ValueError(
                    f"image feature {feature_name!r} must declare [C,H,W] shape"
                )
            channels, height, width = (int(value) for value in shape)
            camera_sources[feature_name] = camera_name
            camera_topics[feature_name] = camera.ros_topic
            camera_shapes[feature_name] = (height, width, channels)

        layout = PolicyLayout(
            profile_name=self.name,
            profile_hash=self.profile_hash(),
            joints=joints,
            state_topic=state_topic,
            action_topics=action_topics,
            camera_sources=camera_sources,
            camera_topics=camera_topics,
            camera_shapes=camera_shapes,
            frequency=float(node.frequency),
            control_mode=control_mode,
            action_space=action_space,
            pose_part=pose_part,
            gripper_parts=gripper_parts,
        )
        declared_action = action_feature
        shape = declared_action.get("shape")
        if shape is not None and list(shape) != [layout.action_dimension]:
            # Shared profiles may declare joint action shape while evaluating a
            # CartesianPolicy node; only enforce when contracts align.
            declared_contract = str(declared_action.get("command_contract", ""))
            if control_mode == "joint" or declared_contract == "pose_reference":
                raise ValueError(
                    f"features.action.action.shape {shape!r} does not match resolved "
                    f"action dimension {layout.action_dimension}"
                )
        return layout


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
        if "agents" in root:
            raise ValueError("agents is removed; declare action producers under nodes")
        nodes = _nodes(root.get("nodes", {}), parts, root.get("sources", {}))
        recording = _mapping(root.get("recorder", {}), "recorder")
        features = _mapping(root.get("features", {}), "features")
        calibration = _mapping(root.get("calibration", {}), "calibration")
        streams = _mapping(root.get("streams", {}), "streams")

        sensors = _mapping(root.get("sensors", {}), "sensors")
        raw_cameras = _mapping(sensors.get("cameras", {}), "sensors.cameras")
        cameras: dict[str, CameraSensorConfig] = {}
        camera_topics: dict[str, str] = {}
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
            ros_topic = _string(
                camera.get("ros_topic"),
                f"sensors.cameras.{camera_name}.ros_topic",
            )
            existing_camera = camera_topics.get(ros_topic)
            if existing_camera is not None:
                raise ValueError(
                    f"camera topic {ros_topic!r} is assigned to both "
                    f"{existing_camera!r} and {camera_name!r}"
                )
            camera_topics[ros_topic] = camera_name
            cameras[camera_name] = CameraSensorConfig(
                name=camera_name,
                ros_topic=ros_topic,
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
            nodes=nodes,
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
    bound["sources"] = em.get("sources", {})
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


def _nodes(
    value: Any, parts: dict[str, PartConfig], source_value: Any
) -> dict[str, NodeConfig]:
    raw_nodes = _mapping(value, "nodes")
    sources = _mapping(source_value, "sources")
    nodes: dict[str, NodeConfig] = {}
    for name, raw_value in raw_nodes.items():
        path = f"nodes.{name}"
        raw_node = _mapping(raw_value, path)
        role = _string(raw_node.get("source_role"), f"{path}.source_role").upper()
        if role not in {"POLICY", "TELEOP", "PLANNER", "MEMORY"}:
            raise ValueError(
                f"{path}.source_role must be POLICY, TELEOP, PLANNER, or MEMORY"
            )
        resources = _mapping(raw_node.get("resources"), f"{path}.resources")
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
            if any(
                "gripper_command" in controller.ros_actions
                for controller in parts[resource].controllers.values()
            ):
                available.add("gripper_command")
            if command not in available:
                raise ValueError(
                    f"{path}.resources.{resource} references unsupported command "
                    f"{command!r}"
                )
            validated_resources[resource] = command
        frequency = raw_node.get("frequency")
        if frequency is not None and (
            not isinstance(frequency, (int, float))
            or isinstance(frequency, bool)
            or frequency <= 0.0
        ):
            raise ValueError(f"{path}.frequency must be positive")
        inputs: dict[str, NodeInputConfig] = {}
        raw_source = sources.get(name)
        if raw_source is not None:
            source = _mapping(raw_source, f"sources.{name}")
            source_role = _string(
                source.get("source_role"), f"sources.{name}.source_role"
            ).upper()
            if source_role != role:
                raise ValueError(
                    f"{path}.source_role {role!r} does not match "
                    f"sources.{name}.source_role {source_role!r}"
                )
            raw_inputs = _mapping(source.get("inputs", {}), f"sources.{name}.inputs")
            for resource, raw_input in raw_inputs.items():
                input_path = f"sources.{name}.inputs.{resource}"
                input_config = _mapping(raw_input, input_path)
                if resource not in validated_resources:
                    continue
                contract = _string(
                    input_config.get("command_contract"),
                    f"{input_path}.command_contract",
                )
                if contract != validated_resources[resource]:
                    raise ValueError(
                        f"{input_path}.command_contract {contract!r} does not match "
                        f"{path}.resources.{resource}"
                    )
                topic = input_config.get("topic")
                action = input_config.get("action")
                if (topic is None) == (action is None):
                    raise ValueError(
                        f"{input_path} must declare exactly one of topic or action"
                    )
                if action is not None:
                    suffix = "follow_joint_trajectory" if contract == "joint_trajectory" else contract
                    expected = f"/execution_manager/ingress/{role.lower()}/{resource}/{suffix}"
                    if action != expected:
                        raise ValueError(f"{input_path}.action must use leased ingress {expected!r}")
                inputs[resource] = NodeInputConfig(
                    endpoint=_string(
                        action if action is not None else topic,
                        f"{input_path}.{'action' if action is not None else 'topic'}",
                    ),
                    command_contract=contract,
                    is_action=action is not None,
                )
        nodes[name] = NodeConfig(
            name=name,
            source_role=role,
            resources=validated_resources,
            frequency=float(frequency) if frequency is not None else None,
            inputs=inputs,
        )
    return nodes
