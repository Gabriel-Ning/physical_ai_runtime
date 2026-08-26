"""Resolve dataset semantics from the embodiment Profile used at runtime."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from policy_inference.common.contract import PolicyIOContract


@dataclass(frozen=True)
class DatasetContract:
    """Topic-to-feature mapping shared by MCAP conversion and policy inference."""

    policy: PolicyIOContract
    state_topic: str
    action_topics: Mapping[str, str]

    @classmethod
    def from_profile(
        cls, profile: Any, *, agent_name: str = "Policy"
    ) -> DatasetContract:
        policy = PolicyIOContract.from_profile(profile, agent_name=agent_name)
        state_feature = profile.features.get("observation", {}).get(
            "observation.state", {}
        )
        state_topic = str(state_feature.get("source", "/joint_states"))
        if not state_topic.startswith("/"):
            raise ValueError("observation.state.source must be an absolute ROS topic")

        action_topics: dict[str, str] = {}
        for group in policy.action_groups:
            part = profile.parts[group.part]
            matches = [
                controller.ros_topics[group.command]
                for controller in part.controllers.values()
                if group.command in controller.ros_topics
            ]
            if len(matches) != 1:
                raise KeyError(
                    f"expected one recorded action topic for {group.part}.{group.command}, "
                    f"found {len(matches)}"
                )
            action_topics[group.part] = matches[0]
        return cls(policy=policy, state_topic=state_topic, action_topics=action_topics)

    @property
    def topics(self) -> tuple[str, ...]:
        return (
            self.state_topic,
            *self.action_topics.values(),
            *self.policy.camera_topics.values(),
        )
