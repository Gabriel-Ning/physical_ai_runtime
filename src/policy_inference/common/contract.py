"""Resolved policy I/O contract derived from an RMI embodiment profile."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ActionGroup:
    """One contiguous slice of a flat policy action."""

    part: str
    command: str
    joint_names: tuple[str, ...]
    start: int
    stop: int

    @property
    def feature_names(self) -> tuple[str, ...]:
        return tuple(f"{name}.pos" for name in self.joint_names)


@dataclass(frozen=True)
class PolicyIOContract:
    """Backend-neutral feature names and action layout for one RMI agent."""

    profile_name: str
    profile_hash: str
    observation_feature_names: tuple[str, ...]
    action_feature_names: tuple[str, ...]
    action_groups: tuple[ActionGroup, ...]
    camera_sources: Mapping[str, str]
    camera_topics: Mapping[str, str]
    camera_shapes: Mapping[str, tuple[int, int, int]]
    frequency: float

    @property
    def action_dim(self) -> int:
        return len(self.action_feature_names)

    @property
    def state_feature_names(self) -> tuple[str, ...]:
        """Ordered scalar state names; joint-reference policies mirror action order."""
        return self.action_feature_names

    @classmethod
    def from_profile(
        cls,
        profile: Any,
        *,
        agent_name: str = "Policy",
    ) -> PolicyIOContract:
        """Resolve names once; reject ambiguous contracts before model loading."""
        try:
            agent = profile.agents[agent_name]
        except KeyError as exc:
            raise KeyError(f"profile has no agent {agent_name!r}") from exc

        groups: list[ActionGroup] = []
        action_names: list[str] = []
        offset = 0
        for part, command in agent.resources.items():
            if command != "joint_reference":
                raise ValueError(
                    f"LeRobot runtime requires joint_reference resources; "
                    f"{part!r} uses {command!r}"
                )
            try:
                joint_names = tuple(profile.parts[part].joint_names)
            except KeyError as exc:
                raise KeyError(
                    f"agent resource references unknown part {part!r}"
                ) from exc
            if not joint_names:
                raise ValueError(f"part {part!r} has no joints")
            stop = offset + len(joint_names)
            group = ActionGroup(part, command, joint_names, offset, stop)
            groups.append(group)
            action_names.extend(group.feature_names)
            offset = stop

        if len(action_names) != len(set(action_names)):
            raise ValueError("policy action joint names must be unique")

        camera_sources: dict[str, str] = {}
        camera_topics: dict[str, str] = {}
        camera_shapes: dict[str, tuple[int, int, int]] = {}
        declared_observations = profile.features.get("observation", {})
        for feature_name, feature in declared_observations.items():
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
                camera = profile.cameras[camera_name]
            except KeyError as exc:
                raise KeyError(
                    f"image feature {feature_name!r} references unknown camera {camera_name!r}"
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
        observation_names = tuple(action_names) + tuple(camera_topics)
        frequency = agent.frequency
        if frequency is None or frequency <= 0.0:
            raise ValueError(f"agent {agent_name!r} must declare a positive frequency")

        declared_action = profile.features.get("action", {}).get("action", {})
        shape = declared_action.get("shape")
        if shape is not None and list(shape) != [len(action_names)]:
            raise ValueError(
                f"features.action.action.shape {shape!r} does not match "
                f"resolved action dimension {len(action_names)}"
            )

        return cls(
            profile_name=profile.name,
            profile_hash=profile.profile_hash(),
            observation_feature_names=observation_names,
            action_feature_names=tuple(action_names),
            action_groups=tuple(groups),
            camera_sources=camera_sources,
            camera_topics=camera_topics,
            camera_shapes=camera_shapes,
            frequency=float(frequency),
        )

    def split_action(
        self, action: Sequence[float]
    ) -> tuple[tuple[ActionGroup, list[float]], ...]:
        """Split one flat LeRobot action without copying more than each output slice."""
        if len(action) != self.action_dim:
            raise ValueError(
                f"action dimension {len(action)} does not match contract {self.action_dim}"
            )
        return tuple(
            (group, [float(value) for value in action[group.start : group.stop]])
            for group in self.action_groups
        )
