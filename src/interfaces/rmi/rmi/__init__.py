"""ROS Manipulation Interface (RMI) Python SDK for Physical AI systems.

Layout::

    contracts.py         Action / Observation / planner DTOs
    config.py            Embodiment profile loader
    context.py           Context factories for one application process
    node.py              Action producer binding and authority status
    robot.py             Joint-state facade
    provider.py          Command client for /action_sources
    selection.py         authority protocol and Execution Manager client
    controllers.py       internal/test-only ros2_control diagnostics
    sensing.py           Camera / sensor facades
    recording.py         EpisodeRecorder (MCAP) and MemoryReplayBuffer (RL)
    replay.py            MCAP action replay pacing
"""

from .config import (
    CameraSensorConfig,
    ControllerConfig,
    EmbodimentConfig,
    NodeConfig,
    NodeInputConfig,
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
from .node import (
    Execution,
    ExecutionState,
    Node,
    NodeActivation,
    NodeResource,
    NodeStatus,
)
from .recording import EpisodeRecorder, EpisodeScope, MemoryReplayBuffer
from .replay import (
    ActionTimestampRebaser,
    EpisodeReplayInfo,
    EpisodeReplayPolicy,
    McapActionSource,
    RecordedAction,
    ReplayClockJumpError,
    ReplayPacer,
    ReplayPlayer,
)
from .robot import Robot, RobotResource
from .selection import (
    AuthorityClient,
    AuthoritySnapshot,
    ExecutionManagerClient,
    ExecutionManagerUnavailableError,
)
from .sensing import Camera, SampleBuffer, Sensor, TimestampedSample

__all__ = [
    "Action",
    "ActionTimestampRebaser",
    "AuthorityClient",
    "AuthoritySnapshot",
    "Camera",
    "CameraSensorConfig",
    "Context",
    "ControlDiagnostics",
    "ControllerClientError",
    "ControllerConfig",
    "EmbodimentConfig",
    "EpisodeRecorder",
    "EpisodeReplayInfo",
    "EpisodeReplayPolicy",
    "EpisodeScope",
    "Execution",
    "ExecutionManagerClient",
    "ExecutionManagerUnavailableError",
    "ExecutionState",
    "JointHorizonPoint",
    "JointHorizonResult",
    "McapActionSource",
    "MemoryReplayBuffer",
    "Node",
    "NodeActivation",
    "NodeConfig",
    "NodeInputConfig",
    "NodeResource",
    "NodeStatus",
    "Observation",
    "PartConfig",
    "PlanPoint",
    "PlanResult",
    "PoseHorizonPoint",
    "PoseHorizonResult",
    "RecordedAction",
    "ReplayClockJumpError",
    "ReplayPacer",
    "ReplayPlayer",
    "ResolveResult",
    "Robot",
    "RobotResource",
    "SampleBuffer",
    "Sensor",
    "TimestampedSample",
    "TrajectoryCanceledError",
]
