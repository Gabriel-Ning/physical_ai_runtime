"""Application-facing RMI errors with explicit ``cause=`` for fast debugging.

Centralizes execution / authority failure attribution so callers never see a bare
``RuntimeError`` without a reason.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

# ---------------------------------------------------------------------------
# Joint Trajectory Controller result codes (control_msgs FollowJointTrajectory)
# ---------------------------------------------------------------------------

JTC_ERROR_CAUSES: dict[int, str] = {
    0: (
        "ABORTED_EXTERNALLY: FollowJointTrajectory was canceled outside JTC "
        "(jtc_guard / EM cancel_actions / controller switch / preemption / client)"
    ),
    -1: "INVALID_GOAL: trajectory goal rejected by joint_trajectory_controller",
    -2: "INVALID_JOINTS: trajectory joint set does not match controller joints",
    -3: "OLD_HEADER_TIMESTAMP: trajectory header stamp is too old",
    -4: "PATH_TOLERANCE_VIOLATED: tracking error exceeded path tolerance",
    -5: "GOAL_TOLERANCE_VIOLATED: final pose missed goal tolerance",
}

# jtc_guard /diagnostics fault_code values while canceling a goal.
JTC_GUARD_FAULT_CAUSES: dict[str, str] = {
    "TRAJECTORY_HEARTBEAT_TIMEOUT": (
        "jtc_guard heartbeat timeout: no true trajectory_guard_heartbeat within "
        "heartbeat_timeout_s (guard canceled FollowJointTrajectory)"
    ),
    "TRAJECTORY_CANCEL_RESPONSE_TIMEOUT": (
        "jtc_guard cancel-response timeout: JTC did not acknowledge cancel in time"
    ),
    "TRAJECTORY_CANCEL_REJECTED": "jtc_guard cancel request was rejected by JTC",
    "TRAJECTORY_CANCEL_NO_ACTIVE_GOAL": (
        "jtc_guard tried to cancel but JTC reported no active goal"
    ),
    "TRAJECTORY_CANCEL_ACCEPTED": "jtc_guard cancel request was accepted by JTC",
}


def _require_cause(cause: str) -> str:
    text = str(cause).strip()
    if not text:
        raise ValueError("RMI errors require a non-empty cause")
    return text


def _format_message(
    headline: str, cause: str, details: Sequence[str] | None = None
) -> str:
    lines = [headline, f"  cause={cause}"]
    cleaned = [str(item) for item in (details or ()) if str(item).strip()]
    if cleaned:
        lines.append("  [Diagnostics]:")
        lines.extend(f"    - {item}" for item in cleaned)
    return "\n".join(lines)


class RmiError(RuntimeError):
    """Base RMI error. Subclasses always expose a non-empty ``cause`` string."""

    def __init__(
        self,
        cause: str,
        *,
        details: Sequence[str] | None = None,
        headline: str | None = None,
    ) -> None:
        self.cause = _require_cause(cause)
        self.details = [str(item) for item in (details or ()) if str(item).strip()]
        super().__init__(
            _format_message(
                headline or "RMI error",
                self.cause,
                self.details,
            )
        )


class ExecutionManagerUnavailableError(RmiError):
    """Execution Manager service/action graph is not reachable in time."""

    def __init__(
        self,
        cause: str = "Execution Manager unavailable",
        *,
        details: Sequence[str] | None = None,
    ) -> None:
        super().__init__(
            cause,
            details=details,
            headline="execution manager unavailable",
        )


class NodeAlreadyActiveError(RmiError):
    """``Node.activate()`` called while a scoped lease is already held."""

    def __init__(self, node_name: str) -> None:
        super().__init__(
            f"node {node_name!r} is already active",
            details=["call deactivate()/exit the activation scope before re-activate"],
            headline=f"node activation failed for {node_name!r}",
        )


class ActionTimeoutError(TimeoutError):
    """Wall-clock wait for an action goal/result/cancel timed out."""

    def __init__(self, cause: str, *, details: Sequence[str] | None = None) -> None:
        self.cause = _require_cause(cause)
        self.details = [str(item) for item in (details or ()) if str(item).strip()]
        super().__init__(
            _format_message("action wait timed out", self.cause, self.details)
        )


class ExecutionError(RmiError):
    """Trajectory/gripper execution failed with an explicit primary cause."""

    def __init__(
        self,
        *,
        part: str,
        state: str,
        cause: str,
        details: Sequence[str] | None = None,
    ) -> None:
        self.part = part
        self.state = state
        super().__init__(
            cause,
            details=details,
            headline=f"execution failed for {part}: {state}",
        )


class GoalRejectedError(ExecutionError):
    """EM / downstream action server rejected the goal before execution."""


class TrajectoryCanceledError(ExecutionError):
    """Trajectory reached the terminal CANCELED state (historical RMI alias)."""


class ControllerClientError(RmiError):
    """Authoritative execution path rejected or could not run a command.

    Kept for compatibility with the original ``errors.py`` surface.
    Prefer ``ExecutionError`` for new trajectory failures.
    """


def format_jtc_guard_diagnostic_lines(
    *,
    part_name: str,
    latest: Mapping[str, str] | None,
    last_fault: Mapping[str, str] | None,
    status_name: str = "",
) -> list[str]:
    """Human-readable abort attribution from captured jtc_guard diagnostics."""
    del part_name
    lines: list[str] = []
    fault_values = last_fault or latest
    if fault_values is None:
        lines.append(
            "jtc_guard: no /diagnostics sample during execute "
            "(cannot attribute ABORTED_EXTERNALLY)"
        )
        return lines

    fault_code = fault_values.get("fault_code", "UNKNOWN")
    cause = JTC_GUARD_FAULT_CAUSES.get(fault_code)
    if last_fault is not None and cause:
        lines.append(f"likely_cause={cause}")
    elif last_fault is None:
        lines.append(
            "likely_cause=external cancel without jtc_guard fault "
            "(EM cancel_actions / controller switch / goal preemption / client cancel)"
        )

    latest_values = latest or {}
    state = fault_values.get("guard_state", latest_values.get("guard_state", "?"))
    timeout_ms = fault_values.get(
        "heartbeat_timeout_ms", latest_values.get("heartbeat_timeout_ms", "?")
    )
    time_source = fault_values.get(
        "time_source", latest_values.get("time_source", "?")
    )
    sequence = fault_values.get(
        "fault_sequence", latest_values.get("fault_sequence", "?")
    )
    topic = fault_values.get(
        "heartbeat_topic", latest_values.get("heartbeat_topic", "?")
    )
    who = status_name or "jtc_guard"
    lines.append(
        f"{who}: fault_code={fault_code}, guard_state={state}, "
        f"heartbeat_timeout_ms={timeout_ms}, time_source={time_source}, "
        f"fault_sequence={sequence}, heartbeat_topic={topic}"
    )
    return lines


def execution_failure_cause_and_details(
    *,
    part: str,
    state: str,
    result: Any,
    feedback: Sequence[Any],
    guard_lines: Sequence[str],
    allocations: Mapping[str, Any],
) -> tuple[str, list[str]]:
    """Pick one primary cause, then supporting diagnostic lines."""
    details: list[str] = []
    error_code = getattr(result, "error_code", None) if result is not None else None
    error_string = getattr(result, "error_string", "") if result is not None else ""

    jtc_cause: str | None = None
    if error_code is not None:
        jtc_cause = JTC_ERROR_CAUSES.get(
            int(error_code), f"JTC error_code={error_code}"
        )
        details.append(f"error_code={error_code} ({jtc_cause})")
    if error_string:
        details.append(f"error_string={error_string!r}")

    alloc = allocations.get(part, {}) if allocations else {}
    if alloc:
        details.append(
            f"EM lease: state={alloc.get('authority_state')}, "
            f"owner={alloc.get('source_instance')!r}, "
            f"controllers={alloc.get('observed_controllers')}"
        )

    if feedback:
        fb = feedback[-1]
        err = getattr(fb, "error", None)
        err_pos = getattr(err, "positions", None)
        actual = getattr(fb, "actual", None)
        actual_pos = getattr(actual, "positions", None)
        desired = getattr(fb, "desired", None)
        desired_pos = getattr(desired, "positions", None)
        details.append(
            f"feedback: count={len(feedback)}, "
            f"latest_err={err_pos}, desired={desired_pos}, actual={actual_pos}"
        )
    else:
        details.append(
            "feedback: none received (goal ended before first controller feedback)"
        )

    details.extend(str(line) for line in guard_lines)

    guard_cause = None
    for line in guard_lines:
        if line.startswith("likely_cause="):
            guard_cause = line[len("likely_cause=") :]
            break

    # Priority: concrete JTC tracking faults > guard attribution > generic external.
    if error_code not in (None, 0) and jtc_cause is not None:
        cause = jtc_cause
    elif guard_cause and "without jtc_guard fault" not in guard_cause:
        cause = guard_cause
    elif error_code == 0 and guard_cause:
        cause = guard_cause
    elif error_code == 0 and jtc_cause is not None:
        cause = jtc_cause
    elif state == "CANCELED":
        cause = "trajectory canceled (upstream cancel or preemption)"
    elif jtc_cause is not None:
        cause = jtc_cause
    else:
        cause = (
            f"execution ended in {state} without JTC error_code; "
            "inspect diagnostics below"
        )
    return cause, details


def goal_rejected_error(
    *,
    part: str,
    endpoint: str,
    node_name: str,
    command: str,
    accepted: Any,
) -> GoalRejectedError:
    return GoalRejectedError(
        part=part,
        state="ABORTED",
        cause=(
            "Execution Manager rejected trajectory goal "
            f"(part={part!r}, endpoint={endpoint!r}, accepted={accepted})"
        ),
        details=[
            f"node={node_name!r}",
            f"command={command!r}",
            "check EM lease ownership, controller active state, and goal stamp/age",
        ],
    )


def action_timeout_error(operation: str, timeout_s: float) -> ActionTimeoutError:
    return ActionTimeoutError(
        f"{operation} timed out after {timeout_s:.1f}s wall wait",
        details=[
            "wall-clock wait only; under use_sim_time raise timeout for slow RTF",
        ],
    )


__all__ = [
    "ActionTimeoutError",
    "ControllerClientError",
    "ExecutionError",
    "ExecutionManagerUnavailableError",
    "GoalRejectedError",
    "JTC_ERROR_CAUSES",
    "JTC_GUARD_FAULT_CAUSES",
    "NodeAlreadyActiveError",
    "RmiError",
    "TrajectoryCanceledError",
    "action_timeout_error",
    "execution_failure_cause_and_details",
    "format_jtc_guard_diagnostic_lines",
    "goal_rejected_error",
]
