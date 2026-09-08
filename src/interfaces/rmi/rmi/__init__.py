"""ROS Manipulation Interface (RMI) Python SDK for Physical AI systems.

Layout::

    contracts.py         Action / Observation / planner DTOs
    command_messages.py  RMI command validation and ROS payload encoding
    config.py            Embodiment profile loader
    context.py           Context factories for one application process
    converters.py        ROS message -> value converters for Camera / Sensor
    errors.py            Explicit cause-bearing RMI exceptions
    node.py              Action producer binding and authority status
    robot.py             Joint-state facade
    selection.py         authority protocol and Execution Manager client
    sensing.py           Camera / topic / TF pose sensor facades
    recording.py         EpisodeRecorder (MCAP) and MemoryReplayBuffer (RL)
    replay.py            MCAP action replay pacing
"""

from .config import (
    CameraSensorConfig,
    ControllerConfig,
    EmbodimentConfig,
    JointGroup,
    JointLayout,
    NodeConfig,
    NodeInputConfig,
    PartConfig,
    PolicyLayout,
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
from .converters import ros_image_to_numpy
from .errors import (
    ActionTimeoutError,
    ControllerClientError,
    ExecutionError,
    ExecutionManagerUnavailableError,
    GoalRejectedError,
    NodeAlreadyActiveError,
    RmiError,
    TrajectoryCanceledError,
)
from .node import (
    Execution,
    ExecutionState,
    Node,
    NodeActivation,
    NodeResource,
    SourceState,
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
from .reset import ResetHandler, ResetMode
from .robot import Robot, RobotResource
from .selection import (
    AuthorityClient,
    AuthoritySnapshot,
    ExecutionManagerClient,
)
from .sensing import Camera, PoseSample, SampleBuffer, Sensor, TfTcpPoseSensor, TimestampedSample

__all__ = [
    "Action",
    "ActionTimeoutError",
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
    "ExecutionError",
    "ExecutionManagerClient",
    "ExecutionManagerUnavailableError",
    "ExecutionState",
    "GoalRejectedError",
    "JointGroup",
    "JointHorizonPoint",
    "JointHorizonResult",
    "JointLayout",
    "McapActionSource",
    "MemoryReplayBuffer",
    "Node",
    "NodeActivation",
    "NodeAlreadyActiveError",
    "NodeConfig",
    "NodeInputConfig",
    "NodeResource",
    "SourceState",
    "Observation",
    "PartConfig",
    "PlanPoint",
    "PlanResult",
    "PolicyLayout",
    "PoseHorizonPoint",
    "PoseHorizonResult",
    "PoseSample",
    "RecordedAction",
    "ReplayClockJumpError",
    "ReplayPacer",
    "ReplayPlayer",
    "ResetHandler",
    "ResetMode",
    "ResolveResult",
    "RmiError",
    "Robot",
    "RobotResource",
    "SampleBuffer",
    "Sensor",
    "TfTcpPoseSensor",
    "TimestampedSample",
    "TrajectoryCanceledError",
    "ros_image_to_numpy",
]
