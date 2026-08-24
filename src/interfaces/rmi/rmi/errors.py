"""Application-facing RMI execution errors."""


class ControllerClientError(RuntimeError):
    """The authoritative execution path rejected or could not run a command."""


class TrajectoryCanceledError(ControllerClientError):
    """A trajectory reached the terminal CANCELED state."""
