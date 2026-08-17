"""Shared key-value store for one running behavior tree.

The blackboard is the only state channel between sibling nodes (e.g. a
``Fallback`` recovery branch reading the ``recovery_target`` a failed
``RunPolicy`` leaf wrote). It intentionally has no RMI, EM, or ROS
knowledge — it is plain data.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any


class Blackboard:
    """Namespaced key-value store with attribute-style access."""

    def __init__(self, initial: dict[str, Any] | None = None) -> None:
        object.__setattr__(self, "_data", dict(initial or {}))

    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._data[key] = value

    def has(self, key: str) -> bool:
        return key in self._data

    def clear(self, key: str | None = None) -> None:
        if key is None:
            self._data.clear()
        else:
            self._data.pop(key, None)

    def snapshot(self) -> dict[str, Any]:
        """Return a shallow copy for status reporting."""
        return dict(self._data)

    def __getattr__(self, key: str) -> Any:
        # __getattr__ is only reached when normal attribute lookup fails,
        # so this never shadows _data itself.
        try:
            return self._data[key]
        except KeyError as exc:
            raise AttributeError(key) from exc

    def __setattr__(self, key: str, value: Any) -> None:
        self._data[key] = value

    def __contains__(self, key: str) -> bool:
        return key in self._data

    def __iter__(self) -> Iterator[str]:
        return iter(self._data)

    def __repr__(self) -> str:
        return f"Blackboard({self._data!r})"
