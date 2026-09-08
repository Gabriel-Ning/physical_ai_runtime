"""The RMI <-> LeRobot data boundary, as one dual pair.

======================  =====================  ==========================
Direction               Contract               Implementations
======================  =====================  ==========================
RMI -> LeRobot          ``ObservationEncoder``  (single encoding)
LeRobot -> RMI          ``ActionDecoder``       ``JointActionDecoder``,
                                                ``CartesianActionDecoder``
======================  =====================  ==========================

``select_action`` only ever calls ``encode`` and ``decode``; which decoder is
built is decided once by the Profile ``control_mode``.
"""

from __future__ import annotations

from rmi import PolicyLayout

from .action import ActionDecoder, CartesianActionDecoder, JointActionDecoder
from .hil_collector import HILTransition, HILTransitionCollector
from .observation import ObservationEncoder

__all__ = [
    "ActionDecoder",
    "CartesianActionDecoder",
    "HILTransition",
    "HILTransitionCollector",
    "JointActionDecoder",
    "ObservationEncoder",
    "make_action_decoder",
]


def make_action_decoder(
    layout: PolicyLayout,
    *,
    action_space: str | None = None,
    gripper_max_width: float = 0.045,
    position_scale: float = 0.05,
    orientation_scale: float = 0.5,
    normalize_gripper: bool = False,
) -> ActionDecoder:
    """Select the decoder for a Profile's ``control_mode``."""
    if layout.control_mode == "cartesian":
        return CartesianActionDecoder(
            layout,
            action_space=action_space,
            gripper_max_width=gripper_max_width,
            position_scale=position_scale,
            orientation_scale=orientation_scale,
        )
    return JointActionDecoder(
        layout,
        normalize_gripper=normalize_gripper,
        gripper_max_width=gripper_max_width,
    )
