"""Controller topology clients (Part / RobotTopology)."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol

from .config import ControllerConfig, EmbodimentConfig, PartConfig


class ControllerClient(Protocol):
    """A direct client for one controller input contract."""

    @property
    def controller_name(self) -> str: ...


class ControllerSwitcherClient(Protocol):
    """Client for a remote ros2_control controller manager."""

    async def switch_controller(
        self, *, activate: tuple[str, ...], deactivate: tuple[str, ...]
    ) -> None: ...

    async def active_controllers(
        self, candidates: tuple[str, ...]
    ) -> tuple[str, ...]: ...


ControllerClientFactory = Callable[[str, str, ControllerConfig], ControllerClient]
SwitcherClientFactory = Callable[[str], ControllerSwitcherClient]


@dataclass
class Part:
    """A robot component and the controller clients that can command it."""

    name: str
    config: PartConfig
    controllers: Mapping[str, ControllerClient]
    controller_switcher: ControllerSwitcherClient
    active_controller: ControllerClient | None

    async def synchronize_controller_state(self) -> ControllerClient | None:
        """Refresh the active route from controller_manager when supported."""
        discover = getattr(self.controller_switcher, "active_controllers", None)
        if discover is None:
            return self.active_controller
        controllers_by_name = {
            client.controller_name: client for client in self.controllers.values()
        }
        active_names = await discover(tuple(controllers_by_name))
        if len(active_names) > 1:
            raise RuntimeError(
                f"Part {self.name!r} has multiple active conflicting controllers: "
                f"{active_names!r}"
            )
        self.active_controller = (
            controllers_by_name[active_names[0]] if active_names else None
        )
        return self.active_controller

    async def switch_controller(self, contract: str) -> ControllerClient:
        try:
            controller = self.controllers[contract]
        except KeyError as exc:
            raise KeyError(
                f"part {self.name!r} has no controller contract {contract!r}"
            ) from exc
        await self.synchronize_controller_state()
        if controller is self.active_controller:
            return controller
        deactivate = (
            (self.active_controller.controller_name,)
            if self.active_controller is not None
            else ()
        )
        await self.controller_switcher.switch_controller(
            activate=(controller.controller_name,),
            deactivate=deactivate,
        )
        self.active_controller = controller
        return controller

    async def deactivate_active_controller(self) -> None:
        """Restore a Part to the safe state with no active motion controller."""
        await self.synchronize_controller_state()
        if self.active_controller is None:
            return
        await self.controller_switcher.switch_controller(
            activate=(), deactivate=(self.active_controller.controller_name,)
        )
        self.active_controller = None


class RobotTopology:
    """Direct controller topology used by the execution layer."""

    def __init__(
        self,
        config: EmbodimentConfig,
        controller_client_factory: ControllerClientFactory,
        switcher_client_factory: SwitcherClientFactory,
    ) -> None:
        switchers: dict[str, ControllerSwitcherClient] = {}
        parts: dict[str, Part] = {}
        for part_name, part_config in config.parts.items():
            switcher = switchers.get(part_config.controller_manager)
            if switcher is None:
                switcher = switcher_client_factory(part_config.controller_manager)
                switchers[part_config.controller_manager] = switcher
            controllers = {
                contract: controller_client_factory(
                    part_name, contract, controller_config
                )
                for contract, controller_config in part_config.controllers.items()
            }
            parts[part_name] = Part(
                name=part_name,
                config=part_config,
                controllers=MappingProxyType(controllers),
                controller_switcher=switcher,
                active_controller=controllers[part_config.default_controller],
            )

        self.name = config.name
        self.parts: Mapping[str, Part] = MappingProxyType(parts)
        self.groups: Mapping[str, tuple[Part, ...]] = MappingProxyType(
            {
                group_name: tuple(parts[member] for member in members)
                for group_name, members in config.groups.items()
            }
        )

    @classmethod
    def from_profile(
        cls,
        profile: EmbodimentConfig | Mapping[str, Any] | str | Path,
        node: Any,
        timeout_sec: float = 5.0,
        *,
        action_client_factory: Any | None = None,
    ) -> RobotTopology:
        """Construct all direct controller clients from one embodiment profile."""
        from .controllers import ControllerManagerClient, make_controller_client_factory

        if isinstance(profile, EmbodimentConfig):
            config = profile
        elif isinstance(profile, Mapping):
            config = EmbodimentConfig.from_dict(dict(profile))
        elif isinstance(profile, (str, Path)):
            config = EmbodimentConfig.from_yaml(profile)
        else:
            raise TypeError("profile must be EmbodimentConfig, mapping, or path")

        controller_factory = (
            make_controller_client_factory(node, timeout_sec)
            if action_client_factory is None
            else make_controller_client_factory(
                node, timeout_sec, action_client_factory
            )
        )
        return cls(
            config,
            controller_factory,
            lambda manager: ControllerManagerClient(node, manager, timeout_sec),
        )

    def __getitem__(self, part_name: str) -> Part:
        return self.parts[part_name]
