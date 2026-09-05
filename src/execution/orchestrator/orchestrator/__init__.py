"""Thin Behavior Tree runtime layered above RMI.

Orchestrator owns the BT tree loader, tick engine, blackboard, task
lifecycle (start/abort/pause/resume), node registry, and status reporting.
Every leaf node calls RMI (``RobotFacade.control()``/``.execute()``,
``RecorderFacade.episode()``) synchronously — see
docs/BEHAVIOR_TREE_ORCHESTRATOR_DESIGN.md for the full architecture.
"""

from .blackboard import Blackboard
from .registry import NodeRegistry, default_registry
from .runtime import BehaviorTreeRuntime, TaskPhase
from .server import OrchestratorServer
from .status import NodeSnapshot, TreeStatus
from .tree import Node, NodeContext, Status

__all__ = [
    "BehaviorTreeRuntime",
    "Blackboard",
    "Node",
    "NodeContext",
    "NodeRegistry",
    "NodeSnapshot",
    "OrchestratorServer",
    "Status",
    "TaskPhase",
    "TreeStatus",
    "default_registry",
]
