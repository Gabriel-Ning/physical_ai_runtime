"""ROS Manipulation Interface (RMI) Python SDK for Physical AI systems.

Layout::

    contracts.py     Action / Observation DTOs
    agent.py         Agent, Session, Robot, PlanExecution
    context.py       Context + make_* factories
    topology.py      Part / RobotTopology controller clients
    controllers.py   ros2_control + controller_manager clients
    provider.py      workstation ActionProviderClient
    execution.py     in-process ExecutionManager core
    sensing.py / planning.py / recording.py / replay.py
"""

from .agent import (
    Agent,
    PlanExecution,
    PlanExecutionState,
    Robot,
    Session,
)
from .config import (
    AgentConfig,
    CameraSensorConfig,
    ControllerConfig,
    EmbodimentConfig,
    PartConfig,
)
from .context import Context
from .contracts import Action, ControlDiagnostics, Observation
from .controllers import (
    ControllerClientError,
    ControllerManagerClient,
    ControllerManagerError,
    ForwardCommandControllerClient,
    GripperControllerClient,
    JointSpaceReferenceControllerClient,
    JointTrajectoryControllerClient,
    TaskSpaceReferenceControllerClient,
    TrajectoryCanceledError,
    make_controller_client_factory,
)
from .execution import (
    ActionChunk,
    Allocation,
    ArbitrationRejected,
    ExecutionEvent,
    ExecutionManager,
    HandoverError,
    LifecycleTransitionError,
    ProviderLifecycle,
    ProviderRegistration,
    ProviderState,
)
from .provider import ActionProviderClient
from .recording import EpisodeScope, Recorder
from .replay import (
    ActionTimestampRebaser,
    McapActionSource,
    RecordedAction,
    ReplayPlayer,
)
from .sensing import Camera, SampleBuffer, Sensor, TimestampedSample
from .topology import (
    ControllerClient,
    ControllerSwitcherClient,
    Part,
    RobotTopology,
)

# Planning symbols are lazily loaded on attribute access.
_PLANNING_EXPORTS = frozenset(
    {
        "CartesianStreamer",
        "JointStreamer",
        "Planner",
        "PlannerCatalog",
        "Resolver",
        "RobotStateSource",
    }
)


def __getattr__(name: str):
    if name in _PLANNING_EXPORTS:
        from . import planning as _planning

        return getattr(_planning, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "Action",
    "ActionChunk",
    "ActionProviderClient",
    "ActionTimestampRebaser",
    "Agent",
    "AgentConfig",
    "Allocation",
    "ArbitrationRejected",
    "Camera",
    "CameraSensorConfig",
    "CartesianStreamer",
    "Context",
    "ControlDiagnostics",
    "ControllerClient",
    "ControllerClientError",
    "ControllerConfig",
    "ControllerManagerClient",
    "ControllerManagerError",
    "ControllerSwitcherClient",
    "EmbodimentConfig",
    "EpisodeScope",
    "ExecutionEvent",
    "ExecutionManager",
    "ForwardCommandControllerClient",
    "GripperControllerClient",
    "HandoverError",
    "JointSpaceReferenceControllerClient",
    "JointStreamer",
    "JointTrajectoryControllerClient",
    "LifecycleTransitionError",
    "McapActionSource",
    "Observation",
    "Part",
    "PartConfig",
    "PlanExecution",
    "PlanExecutionState",
    "Planner",
    "PlannerCatalog",
    "ProviderLifecycle",
    "ProviderRegistration",
    "ProviderState",
    "RecordedAction",
    "Recorder",
    "ReplayPlayer",
    "Resolver",
    "Robot",
    "RobotStateSource",
    "RobotTopology",
    "SampleBuffer",
    "Sensor",
    "Session",
    "TaskSpaceReferenceControllerClient",
    "TimestampedSample",
    "TrajectoryCanceledError",
    "make_controller_client_factory",
]
