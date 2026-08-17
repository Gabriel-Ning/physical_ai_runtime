"""Synchronous execution-manager client used by the application facade."""

from .execution import LocalExecutionManager as ExecutionManagerClient

__all__ = ["ExecutionManagerClient"]
