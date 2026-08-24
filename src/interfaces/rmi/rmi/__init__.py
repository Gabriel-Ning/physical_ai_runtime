"""ROS Manipulation Interface (RMI) Python SDK for Physical AI systems.

Layout::

    contracts.py         Action / Observation / planner DTOs
    config.py            Embodiment profile loader
    context.py           Context factories for one application process
    agent.py             Agent and PlanExecution handle
    session.py           Ownership scope (observe / act / execute)
    robot.py             Joint-state facade
    provider.py          Command client for /action_sources
    selection.py         authority protocol and Execution Manager client
    controllers.py       internal/test-only ros2_control diagnostics
    sensing.py           Camera / sensor facades
    recording.py         EpisodeRecorder (MCAP) and MemoryReplayBuffer (RL)
    replay.py            MCAP action replay pacing
"""

from .agent import (
    Agent,
    PlanExecution,
    PlanExecutionState,
)
from .config import (
    AgentConfig,
    CameraSensorConfig,
    ControllerConfig,
    EmbodimentConfig,
    PartConfig,
)
from .context import Context
from .contracts import (
    Action,
    ControlDiagnostics,
    JointHorizonPoint,
    JointHorizonResult,
    Observation,
    PlanPoint,
    PlanResult,
    PoseHorizonPoint,
    PoseHorizonResult,
    ResolveResult,
)
from .errors import ControllerClientError, TrajectoryCanceledError
from .provider import ActionProviderClient
from .recording import EpisodeRecorder, EpisodeScope, MemoryReplayBuffer
from .replay import (
    ActionTimestampRebaser,
    McapActionSource,
    RecordedAction,
    ReplayPlayer,
)
from .robot import Robot
from .selection import (
    EndpointBinding,
    ExecutionManagerUnavailableError,
    LeaseGrant,
    AuthorityClient,
    ExecutionManagerClient,
    SourceRole,
)
from .sensing import Camera, SampleBuffer, Sensor, TimestampedSample
from .session import Session

__all__ = [
    "Action",
    "ActionProviderClient",
    "ActionTimestampRebaser",
    "Agent",
    "AgentConfig",
    "Camera",
    "CameraSensorConfig",
    "Context",
    "ControlDiagnostics",
    "ControllerClientError",
    "ControllerConfig",
    "EmbodimentConfig",
    "EndpointBinding",
    "EpisodeRecorder",
    "EpisodeScope",
    "ExecutionManagerUnavailableError",
    "JointHorizonPoint",
    "JointHorizonResult",
    "LeaseGrant",
    "McapActionSource",
    "MemoryReplayBuffer",
    "Observation",
    "PartConfig",
    "PlanExecution",
    "PlanExecutionState",
    "PlanPoint",
    "PlanResult",
    "PoseHorizonPoint",
    "PoseHorizonResult",
    "AuthorityClient",
    "ExecutionManagerClient",
    "RecordedAction",
    "ReplayPlayer",
    "ResolveResult",
    "Robot",
    "SampleBuffer",
    "Sensor",
    "Session",
    "SourceRole",
    "TimestampedSample",
    "TrajectoryCanceledError",
]
